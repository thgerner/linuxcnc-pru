;//----------------------------------------------------------------------//
;// Description: pru.stepdir.p                                           //
;// PRU code implementing edge step/dir generation task                  //
;//                                                                      //
;// Author(s): Charles Steinkuehler                                      //
;// License: GNU GPL Version 2.0 or (at your option) any later version.  //
;//                                                                      //
;// Major Changes:                                                       //
;// 2023-Nov    Thomas Gerner                                            //
;//             Derivered from step/dir                                  //
;// 2013-May    Charles Steinkuehler                                     //
;//             Split into several files                                 //
;//             Altered main loop to support a linked list of tasks      //
;//             Added support for GPIO pins in addition to PRU outputs   //
;// 2012-Dec-27 Charles Steinkuehler                                     //
;//             Initial version                                          //
;//----------------------------------------------------------------------//
;// This file is part of LinuxCNC HAL                                    //
;//                                                                      //
;// Copyright (C) 2013  Charles Steinkuehler                             //
;//                     <charles AT steinkuehler DOT net>                //
;//                                                                      //
;// This program is free software; you can redistribute it and/or        //
;// modify it under the terms of the GNU General Public License          //
;// as published by the Free Software Foundation; either version 2       //
;// of the License, or (at your option) any later version.               //
;//                                                                      //
;// This program is distributed in the hope that it will be useful,      //
;// but WITHOUT ANY WARRANTY; without even the implied warranty of       //
;// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        //
;// GNU General Public License for more details.                         //
;//                                                                      //
;// You should have received a copy of the GNU General Public License    //
;// along with this program; if not, write to the Free Software          //
;// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA        //
;// 02110-1301, USA.                                                     //
;//                                                                      //
;// THE AUTHORS OF THIS PROGRAM ACCEPT ABSOLUTELY NO LIABILITY FOR       //
;// ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE   //
;// TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of      //
;// harming persons must have provisions for completely removing power   //
;// from all motors, etc, before persons enter any danger area.  All     //
;// machinery must be designed to comply with local and national safety  //
;// codes, and the authors of this software can not, and do not, take    //
;// any responsibility for such compliance.                              //
;//                                                                      //
;// This code was written as part of the LinuxCNC project.  For more     //
;// information, go to www.linuxcnc.org.                                 //
;//----------------------------------------------------------------------//

    .include "pru_tasks.inc"
    
    .include "pru_global_state.inc"
    .data

GState .sassign r0, global_state

State .sassign r4, stepdir_state ; r4 is assigned to GState.State_Reg0

GTask .sassign r12, task_header

    .define 31, DirHoldBit      
    .define 30, DirChgBit       
    .define 29, PulseHoldBit
    .define 28, GuardBit        
    .define 27, StepBit         

    .define 0x1F, HoldMask        
    .define 0x3F, DirHoldMask     

    .text
    
    ; some stepper controlller, for example the TMC2160, allow a setting where
    ; every edge of the step signal steps the motor. This reduces the step
    ; signal frequency and there is no need for short pulses, just toggle the
    ; step signal.

    .def MODE_EDGESTEP_DIR

    .ref NEXT_TASK
    .ref SET_CLR_BIT

MODE_EDGESTEP_DIR:

    ; Read in task state data
    LBBO &State, GTask.addr, $sizeof(task_header), $sizeof(State)

    ; Accumulator MSBs are used for state/status encoding:
    ; t31 = Dir Hold (set if we're waiting for direction setup/hold)
    ; t30 = Dir Changed (set if rate changed direction and we need to update the direction output)
    ; t29 = Pulse Hold (set if we're waiting for minimum high/low pulse length)
    ; t28 = Guard bit (protects higher status bits from accumulator wrapping)
    ; t27 = Overflow bit (indicates we should generate a step)

    ; If the accumulator overflow bit is set here, we are holding for some reason
    ; (bits 29-31 should tell us why, but we'll deal with that later)
    QBBS    ESD_ACC_HOLD, State.Accum, StepBit
    ADD     State.Accum, State.Accum, State.Rate
ESD_ACC_HOLD:

    ; Check if direction changed
    XOR     r1.b0, (State.Rate).b3, State.RateQ
    MOV     State.RateQ, (State.Rate).b3
    QBBC    ESD_DIR_CHG_DONE, r1.b0, 7

    ; Flag direction change
    SET     State.Accum, State.Accum, DirChgBit

ESD_DIR_CHG_DONE:

    ; Update the pulse timings, if required
    QBBC    ESD_PULSE_DONE, State.Accum, PulseHoldBit

    ; Decrement timeout
    SUB     State.T_Pulse, State.T_Pulse, 1
    QBNE    ESD_PULSE_DONE, State.T_Pulse, 0

    ; Step pulse timer expired,
    ; so clear Pulse Hold bit in accumulator and we're done
    CLR     State.Accum, State.Accum, PulseHoldBit

ESD_PULSE_DONE:

    ; Decrement Direction timer if non-zero
    QBEQ    ESD_DIR_SKIP_SUB, State.T_Dir, 0
    SUB     State.T_Dir, State.T_Dir, 1

ESD_DIR_SKIP_SUB:

    ; Process direction updates if required (either DirHoldBit or DirChgBit is set)
    QBGE    ESD_DIR_DONE, (State.Accum).b3, DirHoldMask

    ; Wait for any pending timeout
    QBNE    ESD_DIR_DONE, State.T_Dir, 0

    ; Direction timer expired

    QBBC    ESD_DIR_SETUP_DLY, State.Accum, DirChgBit

    ; Dir Changed bit is set, we need to update Dir output and configure dir setup timer

    ; Update Direction output
    MOV     r3.b1, GTask.dataY
    LSR     r3.b0, State.Rate, 31
    JAL     (GState.Call_Reg).w2, SET_CLR_BIT

    ; Clear Dir Changed Bit
    CLR     State.Accum, State.Accum, DirChgBit
    SET     State.Accum, State.Accum, DirHoldBit
    MOV     State.T_Pulse, State.Dly_dir_setup
    JMP     ESD_DIR_DONE

ESD_DIR_SETUP_DLY:
    CLR     State.Accum, State.Accum, DirHoldBit

ESD_DIR_DONE:

    QBBC    EDGESTEP_DONE, State.Accum, StepBit
    QBLT    EDGESTEP_DONE, (State.Accum).b3, HoldMask

    ; Time for a step!

    ; Reset Accumulator status bits
    CLR     State.Accum, State.Accum, StepBit

    OR      (State.Accum).b3, (State.Accum).b3, 0x30    ; Set GuardBit and PulseHoldBit
;    SET     State.Accum, GuardBit
;    SET     State.Accum, PulseHoldBit

    ; Update position register
    ADD     State.Pos, State.Pos, 1
    QBBC    ESD_DIR_UP, State.Rate, 31
    SUB     State.Pos, State.Pos, 2
ESD_DIR_UP:

    ; Update state
    XOR     State.StepQ, State.StepQ, 1
    MOV     r3.b1, GTask.dataX
    MOV     r3.b0, State.StepQ
    JAL     (GState.Call_Reg).w2, SET_CLR_BIT
    MOV     State.T_Pulse, State.Delays

EDGESTEP_DONE:
    ; Save channel state data
    SBBO    &State.Accum, GTask.addr, $sizeof(task_header) + stepdir_state.Accum - stepdir_state.Rate, $sizeof(State) - $sizeof(State.StepInvert) - $sizeof(State.Reserved1) - stepdir_state.Accum + stepdir_state.Rate

    ; We're done here...carry on with the next task
    JMP     NEXT_TASK


