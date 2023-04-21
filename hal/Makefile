ECHO = @echo
CC = gcc
CFLAGS = -g -O2 -DRTAPI -D_GNU_SOURCE -I/usr/include/linuxcnc -I. -I../asm
LDFLAGS = -L/usr/lib -Wl,-rpath,/usr/lib
INSTALL = install
DESTDIR = /usr/lib/linuxcnc/modules

all: hal_modules

install: hal_pru_generic.so
	$(INSTALL) -m 0644 -o root -g root -t $(DESTDIR) $<

hal_modules: hal_pru_generic.so

hal_pru_generic.so: hal_pru_generic.o stepgen.o encoder.o pwmgen.o

%.so:
	$(ECHO) Linking $@
	ld -d -r -o $*.tmp $^
	objcopy -j .rtapi_export -O binary $*.tmp $*.sym
	(echo '{ global : '; tr -s '\0' < $*.sym | xargs -r0 printf '%s;\n' | grep .; echo 'local : * ; };') > $*.ver
	$(CC) -shared -Bsymbolic $(LDFLAGS) -Wl,--version-script,$*.ver -o $@ $^ -lm 

.c.o:
	$(ECHO) Compiling realtime $<
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm -f *.o *.tmp *.sym *.ver *.so
