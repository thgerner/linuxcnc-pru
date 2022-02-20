;//----------------------------------------------------------------------//
;// Description: pru.pwm.p                                               //
;// PRU code implementing PWM task                                       //
;//                                                                      //
;// Author(s): Charles Steinkuehler                                      //
;// License: GNU GPL Version 2.0 or (at your option) any later version.  //
;//                                                                      //
;// Major Changes:                                                       //
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

State  .sassign r4, pwm_state      ; r4 is assigned to GState.State_Reg0 
Index  .sassign r6, pwm_index      ; r6 is assigned to GState.State_Reg2 
Output .sassign r7, pwm_output     ; r7 is assigned to GState.State_Reg3

GTask .sassign r12, task_header

    .text
    
    .ref NEXT_TASK
    .ref SET_CLR_BIT
    
    .def MODE_PWM
MODE_PWM:

    ; Skip everything if no outputs are configured
    QBEQ    PWM_DONE, GTask.len, 0

    ; Read in task state data
    LBBO    &State, GTask.addr, $sizeof(task_header), $sizeof(State)

    ; Increment Prescale Counter
    ADD     State.T_Prescale, State.T_Prescale, 1

    ; Prescale finished?
    QBLT    PWM_DONE, State.Prescale, State.T_Prescale
    LDI     State.T_Prescale, 0

    ; Increment Period Counter
    ADD     State.T_Period, State.T_Period, 1

    ; Are we finished with this period?
    QBLE    PrescaleNE, State.Period, State.T_Period
    LDI     State.T_Period, 0

    LDI     Index.Offset, $sizeof(task_header) + $sizeof(State)

    ; Set all outputs when period wraps
PWM_SET_LOOP:
    LBBO    &Output, GTask.addr, Index.Offset, $sizeof(Output)

    ; Only set if Value != 0, otherwise clear
    MOV     r3.b1, Output.Pin
    MIN     r3.b0, Output.Value, 1
    JAL     (GState.Call_Reg).w2, SET_CLR_BIT

    ADD     Index.Offset, Index.Offset, $sizeof(Output)
    SUB     GTask.len, GTask.len, 1
    QBNE    PWM_SET_LOOP, GTask.len, 0
    JMP     PWM_DONE

PrescaleNE:

    ; Cycle through outputs and see if we need to clear any
    LDI     Index.Offset, $sizeof(task_header) + $sizeof(State)

PWM_OUT_LOOP:
    LBBO    &Output.Value, GTask.addr, Index.Offset, $sizeof(Output)

    QBNE    ValueNE, State.T_Period, Output.Value
;  CLR     GState.PRU_Out, Output.Pin

    MOV     r3.b1, Output.Pin
    LDI     r3.b0, 0
    JAL     (GState.Call_Reg).w2, SET_CLR_BIT

ValueNE:

    ADD     Index.Offset, Index.Offset, $sizeof(Output)
    SUB     GTask.len, GTask.len, 1

    QBNE    PWM_OUT_LOOP, GTask.len, 0

PWM_DONE:
    ; Save channel state data
    SBBO    &State.T_Prescale, GTask.addr, $sizeof(task_header) + pwm_state.T_Prescale - pwm_state.Prescale, $sizeof(pwm_state) - pwm_state.T_Prescale + pwm_state.Prescale

    ; We're done here...carry on with the next task
    JMP     NEXT_TASK

