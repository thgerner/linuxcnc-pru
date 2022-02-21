ECHO = @echo
PASM = pasm
PASM_BINFLAGS := -b -L -d -V2 -l
INSTALL = install
DESTDIR = /usr/local/lib/linuxcnc/modules

PRU_SRCS := $(wildcard *.p)

all: pru_generic.bin

install: pru_generic.bin
	$(INSTALL) -m 0644 -o root -g root -t $(DESTDIR) $<

pru_generic.bin:
	$(ECHO) Assembling PRU code $@
	$(PASM) $(PASM_BINFLAGS) pru_generic.p $(basename $@)

clean:
	rm -f pru_generic.bin pru_generic.dbg pru_generic.txt
