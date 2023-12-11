//----------------------------------------------------------------------//
// Description: hal_pru_generic.c                                       //
// HAL module to communicate with PRU code implementing step/dir        //
// generation and other functions of hopeful use to off-load timing     //
// critical code from LinuxCNC HAL                                      //
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
//               hal_pru.c      Micheal Haberler                       //
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


#include <rtapi.h>          /* RTAPI realtime OS API */
#include <rtapi_app.h>      /* RTAPI realtime module decls */
#include <rtapi_math.h>
#include <hal.h>            /* HAL public API decls */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <fcntl.h>

#include "hal_pru_generic.h"

MODULE_AUTHOR("Charles Steinkuehler");
MODULE_DESCRIPTION("AM335x PRU demo component");
MODULE_LICENSE("GPL");

/***********************************************************************
*                    MODULE PARAMETERS AND DEFINES                     *
************************************************************************/

// Maximum number of PRU "channels"
#define MAX_CHAN 8

// Default PRU code to load (prefixed by EMC_RTLIB_DIR)
// Fixme: This should probably be compiled in, via a header file generated
//        by pasm -PRUv2 -c myprucode.p
#define  DEFAULT_CODE  "stepgen.bin"

// Default pin to use for PRU modules...use a pin that does not leave the PRU
#define PRU_DEFAULT_PIN 17

// Start out with default pulse length/width and setup/hold delays of 1 mS (1000000 nS) 
#define DEFAULT_DELAY 1000000

#define f_period_s ((double)(l_period_ns * 1e-9))

static int num_stepgens = 0;
RTAPI_MP_INT(num_stepgens, "Number of step generators (default: 0)");

/*
 * Use step classes for defining classes of step generators.
 * Users are annoyed by configuration changes. Therefore compatibility rule:
 * - If step_class[i] with i = 0 ... num_stepgens is undefined default to step/dir otherwise
 *   create the step generator of step_class[i]
 */
static char *step_class[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(step_class,MAX_CHAN,"Class of step generator, s ... step/dir, 4 ... 4 pin phase, e ... edge step/dir");

static int num_pwmgens = 0;
RTAPI_MP_INT(num_pwmgens, "Number of PWM outputs (default: 0)");
//int num_pwmgens[MAX_CHAN] = { -1, -1, -1, -1, -1, -1, -1, -1 };
//RTAPI_MP_ARRAY_INT(num_pwmgens, "Number of PWM outputs for up to 8 banks (default: 0)");

static int num_encoders = 0;
RTAPI_MP_INT(num_encoders, "Number of encoder channels (default: 0)");

static char *prucode = "";
RTAPI_MP_STRING(prucode, "filename of PRU code (.bin, default: stepgen.bin)");

static int pru = 1;
RTAPI_MP_INT(pru, "PRU number to execute this code (0 or 1, default: 1)");

static int pru_period = 10000;
RTAPI_MP_INT(pru_period, "PRU task period (in nS, default: 10,000 nS or 100 KHz)");

static int disabled = 0;
RTAPI_MP_INT(disabled, "start the PRU in disabled state for debugging (0=enabled, 1=disabled, default: enabled");

/***********************************************************************
*                   STRUCTURES AND GLOBAL VARIABLES                    *
************************************************************************/

/* other globals */
static int comp_id;     /* component ID */

static const char *modname = "hal_pru_generic";

// pru data
typedef struct {
    unsigned int        pruss_inst;
    unsigned int        pruss_data;
    unsigned int        pruss_ctrl;
    char                *pruss_dir;
} pru_data_t;

struct {
    unsigned int        pruss_address;
    unsigned int        pruss_len;
    const pru_data_t    data[2];
} pruss = {
    .pruss_address         = 0x4A300000,                // Page 184 am335x TRM
    .pruss_len             = 0x80000,
    .data = {
        {
            .pruss_inst    = 0x34000,        // Byte addresses, page 20 of PRU Guide
            .pruss_data    = 0x00000,
            .pruss_ctrl    = 0x22000,
            .pruss_dir     = "/sys/class/remoteproc/remoteproc1"
        },
        {
            .pruss_inst    = 0x38000,
            .pruss_data    = 0x02000,
            .pruss_ctrl    = 0x24000,
            .pruss_dir     = "/sys/class/remoteproc/remoteproc2"
        }
    }
};
     

// shared with PRU
static unsigned long *pruss_mmapped_ram;     // points to PRU data RAM

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
static void hpg_read(void *void_hpg, long period);
static void hpg_write(void *arg, long l);
int export_pru(hal_pru_generic_t *hpg);
int pru_init(int pru, char *filename, int disabled, hal_pru_generic_t *hpg);
int setup_pru(int pru, char *filename, int disabled, hal_pru_generic_t *hpg);
void pru_shutdown(int pru);

int hpg_wait_init(hal_pru_generic_t *hpg);
void hpg_wait_force_write(hal_pru_generic_t *hpg);
void hpg_wait_update(hal_pru_generic_t *hpg);

static hpg_step_class_t parse_step_class(const char *sclass);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    hal_pru_generic_t *hpg;
    int retval;

    comp_id = hal_init("hal_pru_generic");
    if (comp_id < 0) {
        HPG_ERR("ERROR: hal_init() failed\n");
        return -1;
    }

    // Allocate HAL shared memory for state data
    hpg = hal_malloc(sizeof(hal_pru_generic_t));
    if (hpg == 0) {
        HPG_ERR("ERROR: hal_malloc() failed\n");
        hal_exit(comp_id);
        return -1;
    }

    // Clear memory
    memset(hpg, 0, sizeof(hal_pru_generic_t));

    // Initialize PRU and map PRU data memory
    if ((retval = pru_init(pru, prucode, disabled, hpg))) {
        HPG_ERR("ERROR: failed to initialize PRU\n");
        hal_exit(comp_id);
        return -1;
    }

    // Setup global state
    hpg->config.num_pwmgens   = num_pwmgens;
    hpg->config.num_stepgens  = num_stepgens;
    hpg->config.num_encoders  = num_encoders;
    hpg->config.comp_id       = comp_id;
    hpg->config.pru_period    = pru_period;
    hpg->config.name          = modname;

    // create step class configuration
    if (num_stepgens > 0) {
        int i;

        hpg->config.step_class = hal_malloc(num_stepgens * sizeof(hpg_step_class_t));
        for (i = 0; i < num_stepgens; i++) {
            if (i < MAX_CHAN) {
                hpg_step_class_t sc = parse_step_class(step_class[i]);
                if (sc == eCLASS_NONE) {
                    HPG_ERR("ERROR: unsupported step class %c for channel %d\n", step_class[i], i);
                    hal_exit(comp_id);
                }
                hpg->config.step_class[i] = sc;
            } else {
                hpg->config.step_class[i] = eCLASS_STEP_DIR;
            }
        }
    }

    rtapi_print("num_pwmgens  : %d\n",hpg->config.num_pwmgens);
    rtapi_print("num_stepgens : %d\n",hpg->config.num_stepgens);
    rtapi_print("num_encoders : %d\n",hpg->config.num_encoders);

    rtapi_print("Init pwm\n");
    // Initialize various functions and generate PRU data ram contents
    if ((retval = hpg_pwmgen_init(hpg))) {
        HPG_ERR("ERROR: pwmgen init failed: %d\n", retval);
        hal_exit(comp_id);
        return -1;
    }

    rtapi_print("Init stepgen\n");
    if ((retval = hpg_stepgen_init(hpg))) {
        HPG_ERR("ERROR: stepgen init failed: %d\n", retval);
        hal_exit(comp_id);
        return -1;
    }

    rtapi_print("Init encoder\n");
    if ((retval = hpg_encoder_init(hpg))) {
        HPG_ERR("ERROR: encoder init failed: %d\n", retval);
        hal_exit(comp_id);
        return -1;
    }

    if ((retval = hpg_wait_init(hpg))) {
        HPG_ERR("ERROR: global task init failed: %d\n", retval);
        hal_exit(comp_id);
        return -1;
    }

    if ((retval = export_pru(hpg))) {
        HPG_ERR("ERROR: var export failed: %d\n", retval);
        hal_exit(comp_id);
        return -1;
    }

    hpg_stepgen_force_write(hpg);
    hpg_pwmgen_force_write(hpg);
    hpg_encoder_force_write(hpg);
    hpg_wait_force_write(hpg);

    if ((retval = setup_pru(pru, prucode, disabled, hpg))) {
        HPG_ERR("ERROR: failed to initialize PRU\n");
        hal_exit(comp_id);
        return -1;
    }
    HPG_INFO("installed\n");

    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void) {
    int i;

    pru_shutdown(pru);
    hal_exit(comp_id);
}

/***********************************************************************
*                       REALTIME FUNCTIONS                             *
************************************************************************/
static void hpg_read(void *void_hpg, long period) {
    hal_pru_generic_t *hpg = void_hpg;

    hpg_stepgen_read(hpg, period);
    hpg_encoder_read(hpg);

}

rtapi_u16 ns2periods(hal_pru_generic_t *hpg, hal_u32_t ns) {
    rtapi_u16 p = ceil((double)ns / (double)hpg->config.pru_period);
    return p;
}

static void hpg_write(void *void_hpg, long period) {
    hal_pru_generic_t *hpg      = void_hpg;

    hpg_stepgen_update(hpg, period);
    hpg_pwmgen_update(hpg);
    hpg_encoder_update(hpg);
    hpg_wait_update(hpg);

}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

// Allocate 32-bit words from PRU data memory
// Start at the beginning of memory, and contiguously allocate RAM
// until we run out of requests.  No free, no garbage colletion, etc.
// Feel free to enhance this when you start adding and removing PRU
// tasks at run-time!  :)
pru_addr_t pru_malloc(hal_pru_generic_t *hpg, int len) {
    // Return address is first free memory location
    pru_addr_t a = hpg->pru_data_free;

    // Insure length is a natural 32-bit length
    int len32 = (len & ~0x03);
    if ((len % 4) != 0) len32 += 4;

    // Point to the next free location
    hpg->pru_data_free += len32;

    HPG_DBG("pru_malloc requested %d bytes, allocated %d bytes starting at %04x\n", len, len32, a);

    // ...and we're done
    return a;
}

int export_pru(hal_pru_generic_t *hpg)
{
    int r;
    char name[HAL_NAME_LEN + 1];

    // Export functions
    rtapi_snprintf(name, sizeof(name), "%s.update", modname);
    r = hal_export_funct(name, hpg_write, hpg, 1, 0, comp_id);
    if (r != 0) {
        HPG_ERR("ERROR: function export failed: %s\n", name);
        hal_exit(comp_id);
        return -1;
    }

    rtapi_snprintf(name, sizeof(name), "%s.capture-position", modname);
    r = hal_export_funct(name, hpg_read, hpg, 1, 0, comp_id);
    if (r != 0) {
        HPG_ERR("ERROR: function export failed: %s\n", name);
        hal_exit(comp_id);
        return -1;
    }

    return 0;
}

static int pru_stop(int pru)
{
    // read status of remotproc device and if it is not "offline" stop PRU
    char status_file_name[PATH_MAX];
    rtapi_snprintf(status_file_name, sizeof(status_file_name), "%s/state", pruss.data[pru].pruss_dir);
    int fd = open(status_file_name, O_RDWR | O_SYNC);
    if (fd == -1) {
        HPG_ERR("ERROR: could not open PRU state %s\n", status_file_name);
        return -1;
    }
    char status[32];
    size_t len = read(fd, status, sizeof(status));
    if (len == -1) {
        HPG_ERR("ERROR: could read PRU state %s\n", status_file_name);
        close(fd);
        return -1;
    }
    int retval = 0;
    if (strcmp("offline\n", status) != 0) {
        // pru is not stopped. stop it
        retval = write(fd, "stop\n", 5);
        if (retval != 5) {
            HPG_ERR("ERROR: could not stop PRU %s\n", status_file_name);
        } else {
            retval = 0;
        }
    }
    close(fd);
    
    return retval;
}

static int pru_start(int pru)
{
    char status_file_name[PATH_MAX];
    rtapi_snprintf(status_file_name, sizeof(status_file_name), "%s/state", pruss.data[pru].pruss_dir);
    int fd = open(status_file_name, O_RDWR | O_SYNC);
    if (fd == -1) {
        HPG_ERR("ERROR: could not open PRU state %s\n", status_file_name);
        return -1;
    }
    int retval = write(fd, "start\n", 6);
    close(fd);
    if (retval != 6) {
        HPG_ERR("ERROR: could not start PRU %s\n", status_file_name);
    } else {
        retval = 0;
    }
    return retval;
}

int pru_init(int pru, char *filename, int disabled, hal_pru_generic_t *hpg) 
{
    if (pru != 1) {
        HPG_ERR("WARNING: PRU is %d and not 1\n",pru);
    }
    if (pru < 0 || pru > 1) {
        HPG_ERR("ERROR: only PRU 0 and PRU 1 possible!\n");
        return -1;
    }

    uid_t euid = geteuid();
    if (euid) {
        HPG_ERR("ERROR: not running as root - need to 'sudo make setuid'?\n");
        return -1;
    }
    uid_t ruid = getuid();
    if (setresuid(euid, euid, ruid) == -1) {
        HPG_ERR("ERROR: setresuid failed\n");
        return -1;
    }
    
    // make sure the PRU is stopped
    if (pru_stop(pru) == -1) {
        return -1;
    }
    
    // map pru data memory via /dev/mem
    rtapi_print("Mapping PRUSS memory\n");
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1) {
        HPG_ERR("ERROR: could not open /dev/mem.\n");
        return -1;
    }
    pruss_mmapped_ram = mmap(0, pruss.pruss_len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, pruss.pruss_address);
    if (pruss_mmapped_ram == MAP_FAILED) {
        HPG_ERR("ERROR: could not map memory.\n");
        return -1;
    }
    close(fd);
    // restore permissions
    if (setresuid(ruid, euid, ruid) == -1) {
        HPG_ERR("ERROR: restore uid failed\n");
        return -1;
    }
    
    unsigned long *data_addr = pruss_mmapped_ram + pruss.data[pru].pruss_data / 4;
    rtapi_print("PRU data ram mapped\n");
    rtapi_print_msg(RTAPI_MSG_DBG, "%s: PRU data ram mapped at %p\n", modname, data_addr);
    hpg->pru_data = (rtapi_u32 *) data_addr;

    // Zero PRU data memory
    for (int i = 0; i < 8192/4; i++) {
        hpg->pru_data[i] = 0;
    }

    // Reserve PRU memory for static configuration variables
    hpg->pru_stat_addr = PRU_DATA_START;
    hpg->pru_data_free = hpg->pru_stat_addr + sizeof(PRU_statics_t);

    // Setup PRU globals
    hpg->pru_stat.task.hdr.dataX = 0xAB;
    hpg->pru_stat.task.hdr.dataY = 0xFE;
    hpg->pru_stat.period = pru_period;
    hpg->config.pru_period = pru_period;

    PRU_statics_t *stat = (PRU_statics_t *) ((rtapi_u32) hpg->pru_data + (rtapi_u32) hpg->pru_stat_addr);
    *stat = hpg->pru_stat;

    return 0;
}

int setup_pru(int pru, char *filename, int disabled, hal_pru_generic_t *hpg)
{
    // Load and execute binary on PRU
    if (!strlen(filename)) {
        filename = DEFAULT_CODE;
    }
    // the firmware file is looked up in /lib/firmware by the remoteproc driver
    // open the remotproc driver
    char firmware_file_name[PATH_MAX];
    rtapi_snprintf(firmware_file_name, sizeof(firmware_file_name), "%s/firmware", pruss.data[pru].pruss_dir);
    int fd = open(firmware_file_name, O_RDWR | O_SYNC);
    if (fd == -1) {
        HPG_ERR("ERROR: could not open PRU firmware %s\n", firmware_file_name);
        return -1;
    }
    int retval = write(fd, filename, strlen(filename));
    if (retval != strlen(filename)) {
    	HPG_ERR("ERROR: could not set PRU firmware %s\n", firmware_file_name);
    } else {
    	retval = 0;
        if (!disabled) {
            retval = pru_start(pru);
        }
    }
    close(fd);
    return retval;
}

void pru_task_add(hal_pru_generic_t *hpg, pru_task_t *task)
{
    if (hpg->last_task == 0) {
        // This is the first task
        HPG_DBG("Adding first task: addr=%04x\n", task->addr);
        hpg->pru_stat.task.hdr.addr = task->addr;
        task->next = task->addr;
        hpg->last_task  = task;
    } else {
        // Add this task to the end of the task list
        HPG_DBG("Adding task: addr=%04x prev=%04x\n", task->addr, hpg->last_task->addr);
        task->next = hpg->pru_stat.task.hdr.addr;
        hpg->last_task->next = task->addr;
        hpg->last_task = task;
    }
}

void pru_shutdown(int pru)
{
    pru_stop(pru);
    munmap(pruss_mmapped_ram, pruss.pruss_len);
}

int hpg_wait_init(hal_pru_generic_t *hpg)
{
    char name[HAL_NAME_LEN + 1];
    int r;

    hpg->wait.task.addr = pru_malloc(hpg, sizeof(hpg->wait.pru));

    pru_task_add(hpg, &(hpg->wait.task));

    rtapi_snprintf(name, sizeof(name), "%s.pru_busy_pin", hpg->config.name);
    r = hal_param_u32_new(name, HAL_RW, &(hpg->hal.param.pru_busy_pin), hpg->config.comp_id);
    if (r != 0) { return r; }

    hpg->hal.param.pru_busy_pin = 0x80;

    return 0;
}

void hpg_wait_force_write(hal_pru_generic_t *hpg)
{
    hpg->wait.pru.task.hdr.mode = eMODE_WAIT;
    hpg->wait.pru.task.hdr.dataX = hpg->hal.param.pru_busy_pin;
    hpg->wait.pru.task.hdr.dataY = 0x00;
    hpg->wait.pru.task.hdr.addr = hpg->wait.task.next;

    PRU_task_wait_t *pru = (PRU_task_wait_t *) ((rtapi_u32) hpg->pru_data + (rtapi_u32) hpg->wait.task.addr);
    *pru = hpg->wait.pru;

    PRU_statics_t *stat = (PRU_statics_t *) ((rtapi_u32) hpg->pru_data + (rtapi_u32) hpg->pru_stat_addr);
    *stat = hpg->pru_stat;
}

void hpg_wait_update(hal_pru_generic_t *hpg) {
    if (hpg->wait.pru.task.hdr.dataX != hpg->hal.param.pru_busy_pin)
        hpg->wait.pru.task.hdr.dataX = hpg->hal.param.pru_busy_pin;

    PRU_task_wait_t *pru = (PRU_task_wait_t *) ((rtapi_u32) hpg->pru_data + (rtapi_u32) hpg->wait.task.addr);
    *pru = hpg->wait.pru;
}

static hpg_step_class_t parse_step_class(const char *sclass)
{
	  hpg_step_class_t ret_class;
	  switch (*sclass) {
	  case 's' :
	  case 'S' :
	  case '\0' :	// default to step/dir
	  	ret_class = eCLASS_STEP_DIR;
	  	break;
	  case '4' :
	  	ret_class = eCLASS_STEP_PHASE;
	  	break;
	  case 'e' :
	  case 'E' :
	  	ret_class = eCLASS_EDGESTEP_DIR;
	  	break;
	  default :
	  	ret_class = eCLASS_NONE;
	  }
    if (*sclass == 'e' || *sclass == 'E') return eCLASS_EDGESTEP_DIR;
    return ret_class;
}

