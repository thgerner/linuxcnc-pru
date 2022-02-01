## PRU hal Komponenten

Ordner hal: enthält pru_hal_generic mit Step-Generator, Encoder und PWM

Ordner pasm: enthält Assembler code für die PRU

Beispiel Aufruf im hal file: loadrt hal_pru_generic prucode=/usr/lib/linuxcnc/xenomai/pru_generic.bin pru=1 num_stepgens=3 step_class=4,4,4
