//----------------------------------------------------------------------//
// Description: pru_tasks.h                                             //
// Header with definitions that MUST match between the PRU assembly     //
// and the LinuxCNC HAL driver code.  In hopes of keeping these         //
// definitions consistant, both versions are contained in this file,    //
// separated by #ifdefs                                                 //
//                                                                      //
// Author(s): Charles Steinkuehler                                      //
// License: GNU GPL Version 2.0 or (at your option) any later version.  //
//                                                                      //
// Major Changes:                                                       //
// 2013-May    Charles Steinkuehler                                     //
//             Initial version                                          //
//----------------------------------------------------------------------//
// This file is part of LinuxCNC HAL                                    //
//                                                                      //
// Copyright (C) 2012  Charles Steinkuehler                             //
//                     <charles AT steinkuehler DOT net>                //
//                                                                      //
// This program is free software; you can redistribute it and/or        //
// modify it under the terms of the GNU General Public License          //
// as published by the Free Software Foundation; either version 2       //
// of the License, or (at your option) any later version.               //
//                                                                      //
// This program is distributed in the hope that it will be useful,      //
// but WITHOUT ANY WARRANTY; without even the implied warranty of       //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        //
// GNU General Public License for more details.                         //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with this program; if not, write to the Free Software          //
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA        //
// 02110-1301, USA.                                                     //
//                                                                      //
// THE AUTHORS OF THIS PROGRAM ACCEPT ABSOLUTELY NO LIABILITY FOR       //
// ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE   //
// TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of      //
// harming persons must have provisions for completely removing power   //
// from all motors, etc, before persons enter any danger area.  All     //
// machinery must be designed to comply with local and national safety  //
// codes, and the authors of this software can not, and do not, take    //
// any responsibility for such compliance.                              //
//                                                                      //
// This code was written as part of the LinuxCNC project.  For more     //
// information, go to www.linuxcnc.org.                                 //
//----------------------------------------------------------------------//

// This file defines the task structures and static variables needed to
// communicate between the HAL code running on the ARM and real time
// code running on the PRU
//
// Pre-processor #ifdef stanzas are used to allow both PRU assembly and
// C style typedefs to exist in the same file, since it is critical that
// the data structures for both sides match exactly.
//
// The _hal_pru_generic_H_ define (from hal_pru_generic.h) is used to 
// deteremine if we are assembling pru code or compiling C code.

//
// Basic types used elsewhere
//

#ifndef _hal_pru_generic_H_
// pru_addr_t

PRU_DATA_START: .set 0
    
// pru_task_mode_t

#else

#define PRU_DATA_START 0

    typedef rtapi_u32 pru_addr_t;

    // Insure these values match the JUMPTABLE in the pru assembly code!
    typedef enum { 
        eMODE_INVALID    = -1,
        eMODE_NONE       = 0,
        eMODE_WAIT       = 1,
        eMODE_WRITE      = 2,    // Not implemented yet!
        eMODE_READ       = 3,    // Not implemented yet!
        eMODE_STEP_DIR   = 4,
        eMODE_UP_DOWN    = 5,    // Not implemented yet!
        eMODE_DELTA_SIG  = 6,
        eMODE_PWM        = 7,
        eMODE_ENCODER    = 8,
        eMODE_STEP_PHASE = 9
    } pru_task_mode_t;
#endif

//
// Task header
//

#ifndef _hal_pru_generic_H_
    task_hdr .struct
        mode    .byte
        len     .byte
        dataX   .byte
        dataY   .byte
    .endstruct
    
    task_header .struct 
                .tag task_hdr
        addr    .int
   .endstruct
#else
    typedef struct {
        rtapi_u8      mode;
        rtapi_u8      len;
        rtapi_u8      dataX;
        rtapi_u8      dataY;
        rtapi_u32     addr;
    } PRU_task_hdr_t;

    typedef union {
    	rtapi_u32     dword[2];
        rtapi_u16     word[4];
        rtapi_u8      byte[8];
    } PRU_task_raw_t;

    typedef union {
        PRU_task_raw_t raw;
        PRU_task_hdr_t hdr;
    } PRU_task_header_t;
#endif

//
// Static variables
//

#ifndef _hal_pru_generic_H_
    pru_statics .struct 
                .tag task_hdr
        addr    .int
        period  .int
    .endstruct
#else
    typedef struct {
        PRU_task_header_t task;
        rtapi_u32     period;
    } PRU_statics_t;
#endif

//
// Task structures, one for each 'flavor'
// The PRU versions do not include the task header, as the current task header
// is a global variable in the PRU assembly code
//

//
// stepgen task
//

#ifndef _hal_pru_generic_H_
    stepdir_dly   .struct
        Dly_step_space  .short
        Dly_dir_setup   .short
    .endstruct

    stepgen_times .struct
        T_Pulse         .short
        T_Dir           .short
    .endstruct
    
    stepdir_misc  .struct
        StepQ           .byte
        RateQ           .byte
        Reserved1       .byte
        StepInvert      .byte
    .endstruct
        
    stepdir_state .struct
        Rate            .int
        Delays          .int ; really a struct of two short, Dly_step_len and Dly_dir_hold, but not accessed as different parts
                             ; otherwise we would need a union here in order to access the register as a whole and the parts
                             ; this is necessary because the clpru assembler doesnot transfer MVID to AND as pasm does
                        .tag stepdir_dly
        Accum           .int
        Pos             .int
                        .tag stepgen_times
                        .tag stepdir_misc
    .endstruct
    
        
    phasegen_misc .struct
        PinC            .byte
        PinD            .byte
        Reserved1       .byte
        RateQ           .byte
    .endstruct
    
    phasegen_times .struct
        T_Pulse         .short
        T_Dir           .short
    .endstruct

    phasegen_state .struct
        Rate            .int
        Delays          .int ; really a struct of two short, steplen and dirhold, but not accessed as different parts
                        .tag phasegen_misc
        Accum           .int
        Pos             .int
                        .tag stepgen_times
        Lut             .int
    .endstruct
#else
    typedef struct  {
        PRU_task_header_t task;

        rtapi_s32     rate;
        rtapi_u16     steplen;
        rtapi_u16     dirhold;
        union {
            rtapi_u16     stepspace;
            struct  {
                rtapi_u8      c;
                rtapi_u8      d;
            } pin;
        };
        union {
            rtapi_u16     dirsetup;
            rtapi_u16     reserved0;
        };
        rtapi_u32     accum;
        rtapi_u32     pos;
        rtapi_u32     reserved1;
        union {
          rtapi_u32     lut;
          struct {
            rtapi_u16     resvd2;
            rtapi_u8      resvd3;
            rtapi_u8      inv;
          } step;
        };
    } PRU_task_stepgen_t;
#endif

//
// delta-sigma modulator task
//

#ifndef _hal_pru_generic_H_
    delta_index .struct
        Offset      .short
        Reserved    .short
    .endstruct

    delta_output .struct
        Value       .short           // WARNING: Range is 14-bits: 0x0000 to 0x4000 inclusive!
        Pin         .byte
        Reserved    .byte
        Integrate   .short
        Quantize    .short
    .endstruct

    delta_state .struct 
        Reserved    .int
    .endstruct
#else
    typedef struct {
        rtapi_u16     value;          // WARNING: Range is 14-bits: 0x0000 to 0x4000 inclusive!
        rtapi_u8      pin;
        rtapi_u8      reserved;
        rtapi_u32     state;
    } PRU_delta_output_t;

    typedef struct {
        PRU_task_header_t task;

        rtapi_u32     reserved;
    //  PRU_delta_output_t out[task.len];
    } PRU_task_delta_t;
#endif

//
// pwmgen task
//

#ifndef _hal_pru_generic_H_
    pwm_index .struct
        Offset      .short
        Reserved    .short
    .endstruct

    pwm_output .struct 
        Value       .short
        Pin         .byte
        Reserved    .byte
    .endstruct

    pwm_state .struct 
        Prescale    .short
        Period      .short
        T_Prescale  .short
        T_Period    .short
    .endstruct
#else
    typedef struct {
        rtapi_u16     value;
        rtapi_u8      pin;
        rtapi_u8      reserved;
    } PRU_pwm_output_t;

    typedef struct {
        PRU_task_header_t task;

        rtapi_u16     prescale;
        rtapi_u16     period;
        rtapi_u32     reserved;
    //  PRU_pwm_output_t out[task.len];
    } PRU_task_pwm_t;
#endif

//
// encoder task
//

#ifndef _hal_pru_generic_H_
    encoder_index .struct 
        wraddr      .int          // Task address + sizeof(read-only objects in encoder_chan)
        Offset      .short
        Reserved    .short
    .endstruct

    encoder_chan .struct 
        A_pin       .byte
        B_pin       .byte
        Z_pin       .byte           // Index
        mode        .byte

        AB_State    .byte
        AB_scratch  .byte
        count       .short

        Z_capture   .short
        Z_count     .byte         // Used by driver to compute "index seen"
        Z_State     .byte

    .endstruct

    encoder_state .struct 
        pins    .int            // XOR mask to invert all input pins in one instruction
        LUT     .int            // Base address of LUT for counter modes
    .endstruct
#else
    typedef struct {
        rtapi_u8      A_pin;
        rtapi_u8      B_pin;
        rtapi_u8      Z_pin;          // Index
        rtapi_u8      mode;

        rtapi_u8      AB_State;
        rtapi_u8      AB_scratch;
        rtapi_u16     count;

        rtapi_u16     Z_capture;
        rtapi_u8      Z_count;        // Used by driver to compute "index seen"
        rtapi_u8      Z_State;
    } PRU_encoder_hdr_t;

    typedef union {
        rtapi_u32     dword[3];
        rtapi_u16     word[6];
        rtapi_u8      byte[12];
    } PRU_encoder_raw_t;

    typedef union {
        PRU_encoder_raw_t raw;
        PRU_encoder_hdr_t hdr;
    } PRU_encoder_chan_t;

    typedef struct {
        rtapi_u8      byte[64];
    } PRU_encoder_LUT_t;

    typedef struct {
        PRU_task_header_t task;

        rtapi_u32     pin_invert;     // XOR mask to invert all input pins in one instruction
        rtapi_u32     LUT;            // Base address of LUT for counter modes
    //  PRU_encoder_chan_t enc[task.len];
    } PRU_task_encoder_t;
#endif

//
// wait task
//

#ifndef _hal_pru_generic_H_

#else
    typedef struct {
        PRU_task_header_t task;
    } PRU_task_wait_t;
#endif
