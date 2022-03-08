//----------------------------------------------------------------------//
// Description: stepgen.c                                               //
// Code to interface to a PRU driven step generator                     //
//                                                                      //
// Author(s): Charles Steinkuehler                                      //
// License: GNU GPL Version 2.0 or (at your option) any later version.  //
//                                                                      //
// Major Changes:                                                       //
// 2013-May    Charles Steinkuehler                                     //
//             Split into several files                                 //
//             Added support for PRU task list                          //
//             Refactored code to more closely match mesa-hostmot2      //
// 2012-Dec-30 Charles Steinkuehler                                     //
//             Initial version, based in part on:                       //
//               hal_pru.c      Micheal Halberler                       //
//               supply.c       Matt Shaver                             //
//               stepgen.c      John Kasunich                           //
//               hostmot2 code  Sebastian Kuzminsky                     //
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

// Use config_module.h instead of config.h so we can use RTAPI_INC_LIST_H
#include <config.h>

#include <rtapi.h>
#include <rtapi_string.h>
#include <rtapi_math.h>

#include <hal.h>

#include "hal_pru_generic.h"

#define MAX_CYCLE 8
#define f_period_s ((double)(l_period_ns * 1e-9))

/*
 * The step modes 5 till 10 are according to stepgen component from John Kasunich
 * The modes 11 - 14 are not supported
 */

static unsigned char master_lut[][MAX_CYCLE] = {
        {1, 2, 4, 8, 1, 2, 4, 8},    /* 5: Unipolar Full Step 1 */
        {3, 6, 12, 9, 3, 6, 12, 9},  /* 6: Unipoler Full Step 2 */
        {1, 7, 14, 8, 1, 7, 14, 8},  /* 7: Bipolar Full Step 1 */
        {5, 6, 10, 9, 5, 6, 10, 9},  /* 8: Bipoler Full Step 2 */
        {1, 3, 2, 6, 4, 12, 8, 9},   /* 9: Unipolar Half Step */
        {1, 5, 7, 6, 14, 10, 8, 9},  /* 10: Bipolar Half Step 1 */
        {5, 1, 9, 8, 10, 2, 6, 4}    /* 11: Bipolar Half Step 2 */
};

// local function prototypes
static int export_stepdir(hal_pru_generic_t *hpg, int i);
static int export_stepphase(hal_pru_generic_t *hpg, int i);

static void hpg_stepdir_update(hal_pru_generic_t *hpg, int i, PRU_task_stepgen_t *pru);
static void hpg_stepphase_update(hal_pru_generic_t *hpg, int i, PRU_task_stepgen_t *pru);

static rtapi_u32 create_lut(hpg_stepgen_instance_t *instance);


// Start out with default pulse length/width and setup/hold delays of 1 mS (1000000 nS) 
#define DEFAULT_DELAY 1000000


/***********************************************************************
 *                       REALTIME FUNCTIONS                             *
 ************************************************************************/
// 
// read accumulator to figure out where the stepper has gotten to
// 

void hpg_stepgen_read(hal_pru_generic_t *hpg, long l_period_ns) {
    // Read data from the PRU here...
    int i;

    for (i = 0; i < hpg->stepgen.num_instances; i ++) {
        rtapi_s64 x, y;
        rtapi_u32 acc;
        rtapi_s64 acc_delta;

        // "atomic" read of accumulator and position register from PRU
        y = * (rtapi_s64 *) ((rtapi_u32) hpg->pru_data + hpg->stepgen.instance[i].task.addr + (rtapi_u32) offsetof(PRU_task_stepgen_t, accum));
        do {
            x = y;
            y = * (rtapi_s64 *) ((rtapi_u32) hpg->pru_data + hpg->stepgen.instance[i].task.addr + (rtapi_u32) offsetof(PRU_task_stepgen_t, accum));
        } while ( x != y );

        // Update internal state
        hpg->stepgen.instance[i].pru.accum = x & 0xFFFFFFFF;
        hpg->stepgen.instance[i].pru.pos   = x >> 32;

        *(hpg->stepgen.instance[i].hal.pin.test1) = hpg->stepgen.instance[i].pru.accum;
        *(hpg->stepgen.instance[i].hal.pin.test2) = hpg->stepgen.instance[i].pru.pos;

        // Mangle 32-bit step count and 27 bit accumulator (with 5 bits of status)
        // into a 16.16 value to match the hostmot2 stepgen logic and generally make
        // things less confusing
        acc  = (hpg->stepgen.instance[i].pru.accum >> 11) & 0x0000FFFF;
        acc |= (hpg->stepgen.instance[i].pru.pos << 16);

        *(hpg->stepgen.instance[i].hal.pin.test3) = acc;

        // those tricky users are always trying to get us to divide by zero
        if (fabs(hpg->stepgen.instance[i].hal.param.position_scale) < 1e-6) {
            if (hpg->stepgen.instance[i].hal.param.position_scale >= 0.0) {
                hpg->stepgen.instance[i].hal.param.position_scale = 1.0;
                HPG_ERR("stepgen %d position_scale is too close to 0, resetting to 1.0\n", i);
            } else {
                hpg->stepgen.instance[i].hal.param.position_scale = -1.0;
                HPG_ERR("stepgen %d position_scale is too close to 0, resetting to -1.0\n", i);
            }
        }

        // The HM2 Accumulator Register is a 16.16 bit fixed-point
        // representation of the current stepper position.
        // The fractional part gives accurate velocity at low speeds, and
        // sub-step position feedback (like sw stepgen).
        acc_delta = (rtapi_s64)acc - (rtapi_s64)hpg->stepgen.instance[i].prev_accumulator;
        if (acc_delta > INT32_MAX) {
            acc_delta -= UINT32_MAX;
        } else if (acc_delta < INT32_MIN) {
            acc_delta += UINT32_MAX;
        }

        hpg->stepgen.instance[i].subcounts += acc_delta;

        *(hpg->stepgen.instance[i].hal.pin.counts) = hpg->stepgen.instance[i].subcounts >> 16;

        // note that it's important to use "subcounts/65536.0" instead of just
        // "counts" when computing position_fb, because position_fb needs sub-count
        // precision
        *(hpg->stepgen.instance[i].hal.pin.position_fb) = ((double)hpg->stepgen.instance[i].subcounts / 65536.0) / hpg->stepgen.instance[i].hal.param.position_scale;

        hpg->stepgen.instance[i].prev_accumulator = acc;

    }
}

//
// Here's the stepgen position controller.  It uses first-order
// feedforward and proportional error feedback.  This code is based
// on John Kasunich's software stepgen code.
//

static void hpg_stepgen_instance_position_control(hal_pru_generic_t *hpg, long l_period_ns, int i, double *new_vel) {
    double ff_vel;
    double velocity_error;
    double match_accel;
    double seconds_to_vel_match;
    double position_at_match;
    double position_cmd_at_match;
    double error_at_match;
    double velocity_cmd;


    hpg_stepgen_instance_t *s = &hpg->stepgen.instance[i];

    *(s->hal.pin.dbg_pos_minus_prev_cmd) = *(s->hal.pin.position_fb) - s->old_position_cmd;

    // calculate feed-forward velocity in machine units per second
    ff_vel = (*(s->hal.pin.position_cmd) - s->old_position_cmd) / f_period_s;
    *(s->hal.pin.dbg_ff_vel) = ff_vel;

    s->old_position_cmd = *(s->hal.pin.position_cmd);

    velocity_error = *(s->hal.pin.velocity_fb) - ff_vel;
    *(s->hal.pin.dbg_vel_error) = velocity_error;

    // Do we need to change speed to match the speed of position-cmd?
    // If maxaccel is 0, there's no accel limit: fix this velocity error
    // by the next servo period!  This leaves acceleration control up to
    // the trajectory planner.
    // If maxaccel is not zero, the user has specified a maxaccel and we
    // adhere to that.
    if (velocity_error > 0.0) {
        if (s->hal.param.maxaccel == 0) {
            match_accel = -velocity_error / f_period_s;
        } else {
            match_accel = -s->hal.param.maxaccel;
        }
    } else if (velocity_error < 0.0) {
        if (s->hal.param.maxaccel == 0) {
            match_accel = velocity_error / f_period_s;
        } else {
            match_accel = s->hal.param.maxaccel;
        }
    } else {
        match_accel = 0;
    }

    if (match_accel == 0) {
        // vel is just right, dont need to accelerate
        seconds_to_vel_match = 0.0;
    } else {
        seconds_to_vel_match = -velocity_error / match_accel;
    }
    *(s->hal.pin.dbg_s_to_match) = seconds_to_vel_match;

    // compute expected position at the time of velocity match
    // Note: this is "feedback position at the beginning of the servo period after we attain velocity match"
    {
        double avg_v;
        avg_v = (ff_vel + *s->hal.pin.velocity_fb) * 0.5;
        position_at_match = *s->hal.pin.position_fb + (avg_v * (seconds_to_vel_match + f_period_s));
    }

    // Note: this assumes that position-cmd keeps the current velocity
    position_cmd_at_match = *s->hal.pin.position_cmd + (ff_vel * seconds_to_vel_match);
    error_at_match = position_at_match - position_cmd_at_match;

    *s->hal.pin.dbg_err_at_match = error_at_match;

    if (seconds_to_vel_match < f_period_s) {
        // we can match velocity in one period
        // try to correct whatever position error we have
        velocity_cmd = ff_vel - (0.5 * error_at_match / f_period_s);

        // apply accel limits?
        if (s->hal.param.maxaccel > 0) {
            if (velocity_cmd > (*(s->hal.pin.velocity_fb) + (s->hal.param.maxaccel * f_period_s))) {
                velocity_cmd =  *(s->hal.pin.velocity_fb) + (s->hal.param.maxaccel * f_period_s);
            } else if (velocity_cmd < (*(s->hal.pin.velocity_fb) - (s->hal.param.maxaccel * f_period_s))) {
                velocity_cmd = *s->hal.pin.velocity_fb - (s->hal.param.maxaccel * f_period_s);
            }
        }

    } else {
        // we're going to have to work for more than one period to match velocity

        double dv;  // delta V, change in velocity
        double dp;  // delta P, change in position

        /* calculate change in final position if we ramp in the opposite direction for one period */
        dv = -2.0 * match_accel * f_period_s;   // Change in velocity if we apply match_accel the opposite direction
        dp = dv * seconds_to_vel_match;         // Resulting position change if we invert match_accel

        /* decide which way to ramp */
        if (fabs(error_at_match + (dp * 2.0)) < fabs(error_at_match)) {
            match_accel = -match_accel;
        }

        /* and do it */
        velocity_cmd = *s->hal.pin.velocity_fb + (match_accel * f_period_s);
    }

    *new_vel = velocity_cmd;
}


// This function was invented by Jeff Epler.
// It forces a floating-point variable to be degraded from native register
// size (80 bits on x86) to C double size (64 bits).
static double force_precision(double d) __attribute__((__noinline__));
static double force_precision(double d) {
    return d;
}

static void update_stepgen(hal_pru_generic_t *hpg, long l_period_ns, int i) {
    double new_vel;

    double physical_maxvel;  // max vel supported by current step timings & position-scale
    double maxvel;           // actual max vel to use this time

    double steps_per_sec_cmd;

    hpg_stepgen_instance_t *s = &(hpg->stepgen.instance[i]);
    pru_task_mode_t mode = hpg->stepgen.instance[i].pru.task.hdr.mode;


    //
    // first sanity-check our maxaccel and maxvel params
    //

    // maxvel must be >= 0.0, and may not be faster than 1 step per (steplen+stepspace) seconds
    {
        double min_ns_per_step, max_steps_per_s;

        if (mode == eMODE_STEP_DIR) {
            min_ns_per_step = (s->pru.steplen + s->pru.stepspace) * hpg->config.pru_period;
        } else if (mode == eMODE_STEP_PHASE) {
            min_ns_per_step = s->pru.steplen * hpg->config.pru_period;
        }
        max_steps_per_s = 1.0e9 / min_ns_per_step;

        physical_maxvel = max_steps_per_s / fabs(s->hal.param.position_scale);
        physical_maxvel = force_precision(physical_maxvel);

        if (s->hal.param.maxvel < 0.0) {
            HPG_ERR("stepgen.%02d.maxvel < 0, setting to its absolute value\n", i);
            s->hal.param.maxvel = fabs(s->hal.param.maxvel);
        }

        if (s->hal.param.maxvel > physical_maxvel) {
            HPG_ERR("stepgen.%02d.maxvel is too big for current step timings & position-scale, clipping to max possible\n", i);
            s->hal.param.maxvel = physical_maxvel;
        }

        if (s->hal.param.maxvel == 0.0) {
            maxvel = physical_maxvel;
        } else {
            maxvel = s->hal.param.maxvel;
        }
    }

    // maxaccel may not be negative
    if (s->hal.param.maxaccel < 0.0) {
        HPG_ERR("stepgen.%02d.maxaccel < 0, setting to its absolute value\n", i);
        s->hal.param.maxaccel = fabs(s->hal.param.maxaccel);
    }


    // select the new velocity we want
    if (*(s->hal.pin.control_type) == 0) {
        hpg_stepgen_instance_position_control(hpg, l_period_ns, i, &new_vel);
    } else {
        // velocity-mode control is easy
        new_vel = *s->hal.pin.velocity_cmd;
        if (s->hal.param.maxaccel > 0.0) {
            if (((new_vel - *s->hal.pin.velocity_fb) / f_period_s) > s->hal.param.maxaccel) {
                new_vel = *(s->hal.pin.velocity_fb) + (s->hal.param.maxaccel * f_period_s);
            } else if (((new_vel - *s->hal.pin.velocity_fb) / f_period_s) < -s->hal.param.maxaccel) {
                new_vel = *(s->hal.pin.velocity_fb) - (s->hal.param.maxaccel * f_period_s);
            }
        }
    }

    // clip velocity to maxvel
    if (new_vel > maxvel) {
        new_vel = maxvel;
    } else if (new_vel < -maxvel) {
        new_vel = -maxvel;
    }

    *s->hal.pin.velocity_fb = (hal_float_t)new_vel;

    steps_per_sec_cmd = new_vel * s->hal.param.position_scale;
    s->pru.rate = steps_per_sec_cmd * (double)0x08000000 * (double) hpg->config.pru_period * 1e-9;

    // clip rate just to be safe...should be limited by code above
    if ((s->pru.rate < 0x80000000) && (s->pru.rate > 0x03FFFFFF)) {
        s->pru.rate = 0x03FFFFFF;
    } else if ((s->pru.rate >= 0x80000000) && (s->pru.rate < 0xFC000001)) {
        s->pru.rate = 0xFC000001;
    }

    *s->hal.pin.dbg_step_rate = s->pru.rate;
}

int export_stepgen(hal_pru_generic_t *hpg, int i)
{
    char name[HAL_NAME_LEN + 1];
    int r;

    // Pins
    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.position-cmd", hpg->config.name, i);
    r = hal_pin_float_new(name, HAL_IN, &(hpg->stepgen.instance[i].hal.pin.position_cmd), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.velocity-cmd", hpg->config.name, i);
    r = hal_pin_float_new(name, HAL_IN, &(hpg->stepgen.instance[i].hal.pin.velocity_cmd), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.velocity-fb", hpg->config.name, i);
    r = hal_pin_float_new(name, HAL_OUT, &(hpg->stepgen.instance[i].hal.pin.velocity_fb), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.position-fb", hpg->config.name, i);
    r = hal_pin_float_new(name, HAL_OUT, &(hpg->stepgen.instance[i].hal.pin.position_fb), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.counts", hpg->config.name, i);
    r = hal_pin_s32_new(name, HAL_OUT, &(hpg->stepgen.instance[i].hal.pin.counts), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.enable", hpg->config.name, i);
    r = hal_pin_bit_new(name, HAL_IN, &(hpg->stepgen.instance[i].hal.pin.enable), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.control-type", hpg->config.name, i);
    r = hal_pin_bit_new(name, HAL_IN, &(hpg->stepgen.instance[i].hal.pin.control_type), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    // debug pins

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.dbg_pos_minus_prev_cmd", hpg->config.name, i);
    r = hal_pin_float_new(name, HAL_OUT, &(hpg->stepgen.instance[i].hal.pin.dbg_pos_minus_prev_cmd), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.dbg_ff_vel", hpg->config.name, i);
    r = hal_pin_float_new(name, HAL_OUT, &(hpg->stepgen.instance[i].hal.pin.dbg_ff_vel), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.dbg_s_to_match", hpg->config.name, i);
    r = hal_pin_float_new(name, HAL_OUT, &(hpg->stepgen.instance[i].hal.pin.dbg_s_to_match), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.dbg_vel_error", hpg->config.name, i);
    r = hal_pin_float_new(name, HAL_OUT, &(hpg->stepgen.instance[i].hal.pin.dbg_vel_error), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.dbg_err_at_match", hpg->config.name, i);
    r = hal_pin_float_new(name, HAL_OUT, &(hpg->stepgen.instance[i].hal.pin.dbg_err_at_match), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.dbg_step_rate", hpg->config.name, i);
    r = hal_pin_s32_new(name, HAL_OUT, &(hpg->stepgen.instance[i].hal.pin.dbg_step_rate), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.test1", hpg->config.name, i);
    r = hal_pin_s32_new(name, HAL_OUT, &(hpg->stepgen.instance[i].hal.pin.test1), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.test2", hpg->config.name, i);
    r = hal_pin_s32_new(name, HAL_OUT, &(hpg->stepgen.instance[i].hal.pin.test2), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.test3", hpg->config.name, i);
    r = hal_pin_s32_new(name, HAL_OUT, &(hpg->stepgen.instance[i].hal.pin.test3), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding pin '%s', aborting\n", name);
        return r;
    }

    // Parameters
    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.position-scale", hpg->config.name, i);
    r = hal_param_float_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.position_scale), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.maxvel", hpg->config.name, i);
    r = hal_param_float_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.maxvel), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.maxaccel", hpg->config.name, i);
    r = hal_param_float_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.maxaccel), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.steplen", hpg->config.name, i);
    r = hal_param_u32_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.steplen), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.dirhold", hpg->config.name, i);
    r = hal_param_u32_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.dirhold), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    // init
    *(hpg->stepgen.instance[i].hal.pin.position_cmd) = 0.0;
    *(hpg->stepgen.instance[i].hal.pin.counts) = 0;
    *(hpg->stepgen.instance[i].hal.pin.position_fb) = 0.0;
    *(hpg->stepgen.instance[i].hal.pin.velocity_fb) = 0.0;
    *(hpg->stepgen.instance[i].hal.pin.enable) = 0;
    *(hpg->stepgen.instance[i].hal.pin.control_type) = 0;

    hpg->stepgen.instance[i].hal.param.position_scale = 1.0;
    hpg->stepgen.instance[i].hal.param.maxvel = 0.0;
    hpg->stepgen.instance[i].hal.param.maxaccel = 1.0;

    hpg->stepgen.instance[i].subcounts = 0;

    hpg->stepgen.instance[i].hal.param.steplen   = ceil((double)DEFAULT_DELAY / (double)hpg->config.pru_period);
    hpg->stepgen.instance[i].hal.param.dirhold   = ceil((double)DEFAULT_DELAY / (double)hpg->config.pru_period);

    hpg->stepgen.instance[i].written_steplen = 0;
    hpg->stepgen.instance[i].written_stepspace = 0;
    hpg->stepgen.instance[i].written_dirsetup = 0;
    hpg->stepgen.instance[i].written_dirhold = 0;
    hpg->stepgen.instance[i].written_task = 0;

    // Start with 1/2 step offset in accumulator
    //hpg->stepgen.instance[i].PRU.accum = 1 << 26;
    hpg->stepgen.instance[i].pru.accum = 0;
    hpg->stepgen.instance[i].prev_accumulator = 0;
    hpg->stepgen.instance[i].old_position_cmd = *(hpg->stepgen.instance[i].hal.pin.position_cmd);

    // call class specfic export function
    if (hpg->stepgen.instance[i].export_stepclass != 0) {
        hpg->stepgen.instance[i].export_stepclass(hpg, i);
    } else {
        HPG_ERR("Initializing error, export function pointer undefined, aborting\n");
        return -1;
    }

    return 0;
}

static int export_stepdir(hal_pru_generic_t *hpg, int i) {
    char name[HAL_NAME_LEN + 1];
    int r;

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.stepspace", hpg->config.name, i);
    r = hal_param_u32_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.dir.stepspace), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.dirsetup", hpg->config.name, i);
    r = hal_param_u32_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.dir.dirsetup), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.steppin", hpg->config.name, i);
    r = hal_param_u32_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.dir.steppin), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.dirpin", hpg->config.name, i);
    r = hal_param_u32_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.dir.dirpin), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.stepinvert", hpg->config.name, i);
    r = hal_param_bit_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.dir.stepinv), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    hpg->stepgen.instance[i].hal.param.dir.stepspace = ceil((double)DEFAULT_DELAY / (double)hpg->config.pru_period);
    hpg->stepgen.instance[i].hal.param.dir.dirsetup  = ceil((double)DEFAULT_DELAY / (double)hpg->config.pru_period);

    hpg->stepgen.instance[i].hal.param.dir.steppin = PRU_DEFAULT_PIN;
    hpg->stepgen.instance[i].hal.param.dir.dirpin  = PRU_DEFAULT_PIN;
    hpg->stepgen.instance[i].hal.param.dir.stepinv = 0;

    return 0;
}

static int export_stepphase(hal_pru_generic_t *hpg, int i) {
    char name[HAL_NAME_LEN + 1];
    int r;

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.pin-a", hpg->config.name, i);
    r = hal_param_u32_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.phase.pin_a), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.pin-b", hpg->config.name, i);
    r = hal_param_u32_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.phase.pin_b), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.pin-c", hpg->config.name, i);
    r = hal_param_u32_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.phase.pin_c), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.pin-d", hpg->config.name, i);
    r = hal_param_u32_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.phase.pin_d), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    rtapi_snprintf(name, sizeof(name), "%s.stepgen.%02d.step-type", hpg->config.name, i);
    r = hal_param_u32_new(name, HAL_RW, &(hpg->stepgen.instance[i].hal.param.phase.type), hpg->config.comp_id);
    if (r < 0) {
        HPG_ERR("Error adding param '%s', aborting\n", name);
        return r;
    }

    hpg->stepgen.instance[i].hal.param.phase.pin_a = PRU_DEFAULT_PIN;
    hpg->stepgen.instance[i].hal.param.phase.pin_b = PRU_DEFAULT_PIN;
    hpg->stepgen.instance[i].hal.param.phase.pin_c = PRU_DEFAULT_PIN;
    hpg->stepgen.instance[i].hal.param.phase.pin_d = PRU_DEFAULT_PIN;
    // set default to phase_type bipolar full step, see stepgen from John Kasunich
    hpg->stepgen.instance[i].hal.param.phase.type = 6;

    return 0;
}

int hpg_stepgen_init(hal_pru_generic_t *hpg){
    int r,i;

    if (hpg->config.num_stepgens <= 0)
        return 0;

    rtapi_print("hpg_stepgen_init\n");

    hpg->stepgen.num_instances = hpg->config.num_stepgens;

    // Allocate HAL shared memory for state data
    hpg->stepgen.instance = (hpg_stepgen_instance_t *) hal_malloc(sizeof(hpg_stepgen_instance_t) * hpg->stepgen.num_instances);
    if (hpg->stepgen.instance == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "%s: ERROR: hal_malloc() failed\n", hpg->config.name);
        hal_exit(hpg->config.comp_id);
        return -1;
    }

    // Clear memory
    memset(hpg->stepgen.instance, 0, (sizeof(hpg_stepgen_instance_t) * hpg->stepgen.num_instances) );

    for (i=0; i < hpg->stepgen.num_instances; i++) {
        hpg->stepgen.instance[i].task.addr = pru_malloc(hpg, sizeof(hpg->stepgen.instance[i].pru));
        switch (hpg->config.step_class[i]) {
        case eCLASS_STEP_DIR :
            hpg->stepgen.instance[i].pru.task.hdr.mode = eMODE_STEP_DIR;
            hpg->stepgen.instance[i].export_stepclass = export_stepdir;
            hpg->stepgen.instance[i].stepgen_updateclass = hpg_stepdir_update;
            break;
        case eCLASS_STEP_PHASE :
            hpg->stepgen.instance[i].pru.task.hdr.mode = eMODE_STEP_PHASE;
            hpg->stepgen.instance[i].export_stepclass = export_stepphase;
            hpg->stepgen.instance[i].stepgen_updateclass = hpg_stepphase_update;
            break;
        default :
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "%s: ERROR: unknown step generator class %i\n", hpg->config.name,hpg->config.step_class[i]);
            return -1;
        }
        pru_task_add(hpg, &(hpg->stepgen.instance[i].task));

        if ((r = export_stepgen(hpg,i)) != 0){
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "%s: ERROR: failed to export stepgen %i: %i\n", hpg->config.name,i,r);
            return -1;
        }
    }

    return 0;
}

void hpg_stepgen_update(hal_pru_generic_t *hpg, long l_period_ns) {
    int i;

    for (i = 0; i < hpg->stepgen.num_instances; i ++) {

        hpg_stepgen_instance_t *instance = &(hpg->stepgen.instance[i]);

        if (*(instance->hal.pin.enable) == 0) {
            instance->pru.rate = 0;
            instance->old_position_cmd = *(instance->hal.pin.position_cmd);
            *(instance->hal.pin.velocity_fb) = 0;
        } else {
            // call update function
            update_stepgen(hpg, l_period_ns, i);
        }

        PRU_task_stepgen_t *pru = (PRU_task_stepgen_t *) ((rtapi_u32) hpg->pru_data + (rtapi_u32) instance->task.addr);

        // Update timing parameters if changed
        if (instance->hal.param.dirhold   != instance->written_dirhold) {
            instance->pru.dirhold    = ns2periods(hpg, instance->hal.param.dirhold);
            pru->dirhold    = instance->pru.dirhold;
            instance->written_dirhold   = instance->hal.param.dirhold;
        }

        if (instance->hal.param.steplen   != instance->written_steplen) {
            instance->pru.steplen    = ns2periods(hpg, instance->hal.param.steplen);
            pru->steplen    = instance->pru.steplen;
            instance->written_steplen   = instance->hal.param.steplen;
        }

        // update class specific values
        instance->stepgen_updateclass(hpg, i, pru);

        // Update control word if changed
        if (instance->pru.task.raw.dword[0] != instance->written_task) {
            pru->task.raw.dword[0]                = instance->pru.task.raw.dword[0];
            instance->written_task = instance->pru.task.raw.dword[0];
        }

        // Send rate update to the PRU
        pru->rate = instance->pru.rate;
    }
}

static void hpg_stepdir_update(hal_pru_generic_t *hpg, int i, PRU_task_stepgen_t *pru) {
    hpg_stepgen_instance_t *instance = &(hpg->stepgen.instance[i]);

    // Update shadow of PRU control registers
    if (instance->pru.task.hdr.dataX != instance->hal.param.dir.steppin) {
        instance->pru.task.hdr.dataX   = instance->hal.param.dir.steppin;
    }

    if (instance->pru.task.hdr.dataY != instance->hal.param.dir.dirpin) {
        instance->pru.task.hdr.dataY   = instance->hal.param.dir.dirpin;
    }

    // update class specific parameters if changed
    if (instance->hal.param.dir.dirsetup  != instance->written_dirsetup) {
        instance->pru.dirsetup   = ns2periods(hpg, instance->hal.param.dir.dirsetup);
        pru->dirsetup   = instance->pru.dirsetup;
        instance->written_dirsetup  = instance->hal.param.dir.dirsetup;
    }

    if (instance->hal.param.dir.stepspace != instance->written_stepspace) {
        instance->pru.stepspace  = ns2periods(hpg, instance->hal.param.dir.stepspace);
        pru->stepspace  = instance->pru.stepspace;
        instance->written_stepspace = instance->hal.param.dir.stepspace;
    }

    if (instance->pru.step.inv != instance->hal.param.dir.stepinv) {
        instance->pru.step.inv = instance->hal.param.dir.stepinv;
        pru->step.inv    = instance->pru.step.inv;
    }
}

static void hpg_stepphase_update(hal_pru_generic_t *hpg, int i, PRU_task_stepgen_t *pru) {
    hpg_stepgen_instance_t *instance = &(hpg->stepgen.instance[i]);

    // Update shadow of PRU control registers
    if (instance->pru.task.hdr.dataX != instance->hal.param.phase.pin_a) {
        instance->pru.task.hdr.dataX   = instance->hal.param.phase.pin_a;
    }

    if (instance->pru.task.hdr.dataY != instance->hal.param.phase.pin_b) {
        instance->pru.task.hdr.dataY   = instance->hal.param.phase.pin_b;
    }

    // update class specific parameters if changed
    if (instance->hal.param.phase.pin_c != instance->pru.pin.c) {
        instance->pru.pin.c = instance->hal.param.phase.pin_c;
        pru->pin.c      = instance->pru.pin.c;
    }

    if (instance->hal.param.phase.pin_d != instance->pru.pin.d) {
        instance->pru.pin.d = instance->hal.param.phase.pin_d;
        pru->pin.d      = instance->pru.pin.d;
    }

    if (instance->hal.param.phase.type != instance->written_phase) {
        instance->pru.lut = create_lut(instance);
        pru->lut = instance->pru.lut;
        instance->written_phase = instance->hal.param.phase.type;
    }
}

void hpg_stepgen_force_write(hal_pru_generic_t *hpg) {
    int i;

    if (hpg->stepgen.num_instances <= 0) return;

    for (i = 0; i < hpg->stepgen.num_instances; i ++) {

        hpg_stepgen_instance_t *instance = &(hpg->stepgen.instance[i]);
        pru_task_mode_t mode = instance->pru.task.hdr.mode;

        instance->pru.task.hdr.len   = 0;
        instance->pru.task.hdr.addr    = instance->task.next;
        instance->pru.rate             = 0;
        instance->pru.steplen          = ns2periods(hpg, instance->hal.param.steplen);
        instance->pru.dirhold          = ns2periods(hpg, instance->hal.param.dirhold);
        if (mode == eMODE_STEP_DIR) {
            instance->pru.task.hdr.dataX = instance->hal.param.dir.steppin;
            instance->pru.task.hdr.dataY = instance->hal.param.dir.dirpin;
            instance->pru.stepspace      = ns2periods(hpg, instance->hal.param.dir.stepspace);
            instance->pru.dirsetup       = ns2periods(hpg, instance->hal.param.dir.dirsetup);
            instance->pru.step.resvd2    = 0;
            instance->pru.step.resvd3    = 0;
            instance->pru.step.inv       = 0;
        } else if (mode == eMODE_STEP_PHASE) {
            instance->pru.task.hdr.dataX = instance->hal.param.phase.pin_a;
            instance->pru.task.hdr.dataY = instance->hal.param.phase.pin_b;
            instance->pru.pin.c          = instance->hal.param.phase.pin_c;
            instance->pru.pin.d          = instance->hal.param.phase.pin_d;
            instance->pru.reserved0      = 0;
            instance->pru.lut            = create_lut(instance);

        }
        instance->pru.accum          = 0;
        instance->pru.pos            = 0;
        instance->pru.reserved1      = 0;

        PRU_task_stepgen_t *pru = (PRU_task_stepgen_t *) ((rtapi_u32) hpg->pru_data + (rtapi_u32) instance->task.addr);
        *pru = instance->pru;
    }
}

static rtapi_u32 create_lut(hpg_stepgen_instance_t *instance)
{
    rtapi_u32 ret = 0;
    int j;

    // phase type. check allowed range, see stepgen from John Kasunich
    hal_u32_t type = instance->hal.param.phase.type - 5;
    if (type < 0 || type > sizeof(master_lut)/sizeof(master_lut[0])) {
        HPG_ERR("stepgen: step_type %d out of range: allowed 5 to 11\n", type);
        type = 1;
        instance->hal.param.phase.type = type + 5;
    }

    for (j = 7; j >= 0; j--) {
        ret <<= 4;
        ret |= master_lut[type][j];
    }
    return ret;
}
