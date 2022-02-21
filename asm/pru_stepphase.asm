;// ----------------------------------------------------------------------;// 
;//  Description: pru.stepdir.p                                           ;// 
;//  PRU code implementing step/pahse generation task                     ;// 
;//                                                                       ;// 
;//  Author(s): Thomas Gerner                                             ;// 
;//  License: GNU GPL Version 2.0 or (at your option) any later version.  ;// 
;//                                                                       ;// 
;//  Major Changes:                                                       ;// 
;//  2014-Aug-18 Thomas Gerner                                            ;// 
;//              Initial version                                          ;// 
;// ----------------------------------------------------------------------;// 
;//  This file is part of LinuxCNC HAL                                    ;// 
;//                                                                       ;// 
;//  Copyright (C) 2013  Charles Steinkuehler                             ;// 
;//                      <charles AT steinkuehler DOT net>                ;// 
;//                                                                       ;// 
;//  This program is free software; you can redistribute it and/or        ;// 
;//  modify it under the terms of the GNU General Public License          ;// 
;//  as published by the Free Software Foundation; either version 2       ;// 
;//  of the License, or (at your option) any later version.               ;// 
;//                                                                       ;// 
;//  This program is distributed in the hope that it will be useful,      ;// 
;//  but WITHOUT ANY WARRANTY; without even the implied warranty of       ;// 
;//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        ;// 
;//  GNU General Public License for more details.                         ;// 
;//                                                                       ;// 
;//  You should have received a copy of the GNU General Public License    ;// 
;//  along with this program; if not, write to the Free Software          ;// 
;//  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA        ;// 
;//  02110-1301, USA.                                                     ;// 
;//                                                                       ;// 
;//  THE AUTHORS OF THIS PROGRAM ACCEPT ABSOLUTELY NO LIABILITY FOR       ;// 
;//  ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE   ;// 
;//  TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of      ;// 
;//  harming persons must have provisions for completely removing power   ;// 
;//  from all motors, etc, before persons enter any danger area.  All     ;// 
;//  machinery must be designed to comply with local and national safety  ;// 
;//  codes, and the authors of this software can not, and do not, take    ;// 
;//  any responsibility for such compliance.                              ;// 
;//                                                                       ;// 
;//  This code was written as part of the LinuxCNC project.  For more     ;// 
;//  information, go to www.linuxcnc.org.                                 ;// 
;// ----------------------------------------------------------------------;// 


    .include "pru_tasks.inc"
    
    .include "pru_global_state.inc"
    .data

GState .sassign r0, global_state

PhState .sassign r4, phasegen_state ; r4 is assigned to GState.State_Reg0

GTask .sassign r12, task_header

    .define 28, PhGuardBit
    .define 27, PhStepBit

    .text
    
    .ref NEXT_TASK
    .ref SET_CLR_BIT

    .def MODE_STEP_PHASE
MODE_STEP_PHASE:
    ;  Read in task state data
    LBBO &PhState, GTask.addr, $sizeof(task_header), $sizeof(phasegen_state)

    ;  Accumulator MSBs are used for state/status encoding:
    ;  t28 = Guard bit (protects higher status bits from accumulator wrapping)
    ;  t27 = Overflow bit (indicates we should generate a step)

    ;  If the accumulator overflow bit is set here, we are holding for some reason
    QBBS    SPH_ACC_HOLD, PhState.Accum, PhStepBit
    ADD     PhState.Accum, PhState.Accum, PhState.Rate
SPH_ACC_HOLD:

    ;  Decrement pulse timeout if non zero
    QBEQ    SPH_SKIP_PULSE_SUB, PhState.T_Pulse, 0
    SUB     PhState.T_Pulse, PhState.T_Pulse, 1

SPH_SKIP_PULSE_SUB:

    ;  Decrement Direction timer if non-zero
    QBEQ    SPH_DIR_SKIP_SUB, PhState.T_Dir, 0
    SUB     PhState.T_Dir, PhState.T_Dir, 1

SPH_DIR_SKIP_SUB:

	;  do nothing as long as T_Pulse is non zero
	QBNE    PHASE_DONE, PhState.T_Pulse, 0

	;  check if direction change
    XOR     r1.b0, (PhState.Rate).b3, PhState.RateQ
    QBBC	SPH_NO_DIR_CHANGE, r1.b0, 7
    QBNE	PHASE_DONE, PhState.T_Dir, 0
    MOV     PhState.RateQ, (PhState.Rate).b3

SPH_NO_DIR_CHANGE:

	QBBC    PHASE_DONE, PhState.Accum, PhStepBit

	;  time for a step
    CLR     PhState.Accum, PhState.Accum, PhStepBit	    ;  Clear Accumulator status bits
    SET     PhState.Accum, PhState.Accum, PhGuardBit    ;  Set GuardBit

        ;  Update position register
    ADD     PhState.Pos, PhState.Pos, 1
    QBBC    SPH_DIR_UP, PhState.Rate, 31
    SUB     PhState.Pos, PhState.Pos, 2

SPH_DIR_UP:

	;  calculate the index to the lut
	AND     r3.b0, (PhState.Pos).b0, 0x07
	LSL		r3.b0, r3.b0, 2
	LSR     r2, PhState.Lut, r3.b0

	;  Phase pin A
	MOV     r3.b1, GTask.dataX
	MOV     r3.b0, r2.b0
	JAL     (GState.Call_Reg).w2, SET_CLR_BIT

	;  Phase pin B
	MOV     r3.b1, GTask.dataY
	LSR     r3.b0, r2.b0, 1
	JAL     (GState.Call_Reg).w2, SET_CLR_BIT

	;  Phase pin C
	MOV     r3.b1, PhState.PinC
	LSR     r3.b0, r2.b0, 2
	JAL     (GState.Call_Reg).w2, SET_CLR_BIT

	;  Phase pin D
	MOV     r3.b1, PhState.PinD
	LSR     r3.b0, r2.b0, 3
	JAL     (GState.Call_Reg).w2, SET_CLR_BIT

	;  set timer
	MOV     PhState.T_Pulse, PhState.Delays

PHASE_DONE:
    ;  Save channel state data
    SBBO    &PhState.RateQ, GTask.addr, $sizeof(task_header) + phasegen_state.RateQ - phasegen_state.Rate, phasegen_state.Lut - phasegen_state.RateQ

    ;  We're done here...carry on with the next task
    JMP     NEXT_TASK

