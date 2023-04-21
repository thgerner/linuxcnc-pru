ifndef PRU_CGT
define ERROR_BODY

*******************************************************************************
PRU_CGT environment variable is not set. For example:
(Desktop Linux) export PRU_CGT=/path/to/pru/code/gen/tools/ti-cgt-pru_2.3.3
(Windows) set PRU_CGT=C:/path/to/pru/code/gen/tools/ti-cgt-pru_2.3.3
(ARM Linux*) export PRU_CGT=/usr/share/ti/cgt-pru

*ARM Linux also needs to create a symbolic link to the /usr/bin/ directory in
order to use the same Makefile
(ARM Linux) ln -s /usr/bin/ /usr/share/ti/cgt-pru/bin

The latest PRU CGT can be found at https://www.ti.com/tool/PRU-CGT or in Code
Composer Studio (CCS) App Center.
*******************************************************************************

endef
$(error $(ERROR_BODY))
endif

MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
CURRENT_DIR := $(notdir $(patsubst %/,%,$(dir $(MKFILE_PATH))))
PROJ_NAME=$(CURRENT_DIR)
LINKER_COMMAND_FILE=./AM335x_PRU.cmd

CGT_CFLAGS=-v3 -O2 --display_error_number --endian=little --hardware_mac=on -ppd -ppa
CGT_LFLAGS=--reread_libs --warn_sections 

TARGET=pru_generic-pru1.fw
MAP=pru_generic-pru1.map
SOURCES=$(wildcard *.asm)
OBJECTS=pru_generic.obj pru_stepphase.obj pru_wait.obj pru_stepdir.obj pru_deltasigma.obj pru_pwm.obj pru_encoder.obj

ECHO = @echo
INSTALL = install
DESTDIR = /lib/firmware
CC = gcc
CCOPTS = -E
SED = sed

PRU_SRCS := $(wildcard *.p)

all: pru_tasks.inc $(OBJECTS) $(TARGET)

install: $(TARGET)
	$(INSTALL) -m 0644 -o root -g root -t $(DESTDIR) $<

%.obj: %.asm
	@echo 'Invoking: PRU Compiler'
	$(PRU_CGT)/bin/clpru --include_path=$(PRU_CGT)/include $(INCLUDE) $(CGT_CFLAGS) -al -fe $@ $<

$(TARGET): $(OBJECTS) $(LINKER_COMMAND_FILE)
	@echo ''
	@echo 'Building target: $@'
	$(PRU_CGT)/bin/clpru $(CGT_CFLAGS) -z -i$(PRU_CGT)/lib -i$(PRU_CGT)/include $(CGT_LFLAGS) -o $(TARGET) $(OBJECTS) -m$(MAP) $(LINKER_COMMAND_FILE)
	@echo 'Finished building target: $@'

pru_tasks.inc: pru_tasks.h
	$(ECHO) Create tasks include file $@
	$(CC) $(CCOPTS) pru_tasks.h | $(SED) -e "s/^#/;/g" -e "s/^\s\+\([a-zA-Z]\)/\1/g" > $@

clean:
	rm -f *.obj *.lst *.abs *.out
