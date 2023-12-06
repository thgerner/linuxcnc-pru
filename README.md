## PRU hal Komponenten

This is a PRU component for LinuxCNC running on a BeagleBone Black. It contains a stepgen, an encoder and a pwmgen using a PRU.

### Build the component

The asm build needs the latest PRU CGT from TI: https://www.ti.com/tool/PRU-CGT

The hal build needs an installed LinuxCNC development package.

To build on a BeagelBone black type:

```
PRU_CGT=/usr/share/ti/cgt-pru make
```

Install the component:

```
sudo PRU_CGT=/usr/share/ti/cgt-pru make install
```

The pru code installs to /lib/firmware, the hal component to /usr/lib/linuxcnc/modules

### Using the component

```
loadrt hal_pru_generic prucode=pru_generic-pru1.fw pru=1 num_stepgens=3 step_class=s,s,s
```

Currently there is no documentaion. Please look at the code.

