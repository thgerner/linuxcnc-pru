;//----------------------------------------------------------------------//
;// Description: pru_global_state.inc                                    //
;// PRU code structure of the pru register usage                         //

;// Register Usage:
;// r0   Scratch / bn count value for LBBO/SBBO/LBCO/SBCO/XIN/XOUT/XCHG/SXIN/SXOUT/SXCHG / XFR Shift Value
;// r1   Scratch / Register pointers for MVIx instructions
;// r2   Scratch
;// r3   Scratch
;// r4   Task Data (task specific use)
;// r5   Task Data (task specific use)
;// r6   Task Data (task specific use)
;// r7   Task Data (task specific use)
;// r8   Task Data (task specific use)
;// r9   Task Data (task specific use)
;// r10  Task Data (task specific use)
;// r11  Task Data (task specific use)
;// r12  TASK_STATUS
;// r13  TASK_ADDR
;// r14  GPIO0_Clr
;// r15  GPIO0_Set
;// r16  GPIO1_Clr
;// r17  GPIO1_Set
;// r18  GPIO2_Clr
;// r19  GPIO2_Set
;// r20  GPIO3_Clr
;// r21  GPIO3_Set
;// r22  PRU_Out
;// r23  w0 TaskTable / w2 PinTable
;// r24  Call Register
;// r25  Scratch / Reserved (Multiplier mode/status)
;// r26  Scratch / Reserved (Multiplier Lower product)
;// r27  Scratch / Reserved (Multiplier Upper product)
;// r28  Scratch / Reserved (Multiplier Operand)
;// r29  Scratch / Reserved (Multiplier Operand)
;// r30  Direct Outputs
;// r31  Direct Inputs / Event Generation

addr_table .struct
TaskTable      .short
PinTable       .short
           .endstruct

global_state .struct
Scratch0 .int           ; r0
Scratch1 .int           ; r1
Scratch2 .int           ; r2
Scratch3 .int           ; r3
State_Reg0 .int         ; r4
State_Reg1 .int         ; r5
State_Reg2 .int         ; r6
State_Reg3 .int         ; r7
State_Reg4 .int         ; r8
State_Reg5 .int         ; r9
State_Reg6 .int         ; r10
State_Reg7 .int         ; r11
Task_Status .int        ; r12
Task_Addr .int          ; r13
GPIO0_Clr .int          ; r14
GPIO0_Set .int          ; r15
GPIO1_Clr .int          ; r16
GPIO1_Set .int          ; r17
GPIO2_Clr .int          ; r18
GPIO2_Set .int          ; r19
GPIO3_Clr .int          ; r20
GPIO3_Set .int          ; r21
PRU_Out .int            ; r22
      .tag addr_table  ; r23
Call_Reg .int           ; r24
Mul_Status .int         ; r25
Mul_Prod_L .int         ; r26
Mul_Prod_H .int         ; r27
Mul_Op1 .int            ; r28
Mul_Op2 .int            ; r29
global_state_size .endstruct

