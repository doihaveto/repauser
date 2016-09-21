# Repauser

USB-HID device based on attiny85.

The intended use of this device is to pause movies (press the space key) when the user rises from his chair, using a force-sensing resistor connected to the chair and the device.

## Schematic ##
![Project schematic](https://github.com/doihaveto/repauser/raw/master/board/schematic.png)

Included in this project:

 - Source code
 - Kicad project files
 - Fabrication-ready board image files

## Flashing instructions ###
I used USBASP to flash the code the attiny85, and AVRDUDE. The makefile included in this project uses AVRDUDE.

Default fuses (required for reflashing):
```avrdude -p attiny85 -c usbasp -U lfuse:w:0x62:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m```
Reading fuses:
```avrdude -p attiny85 -c usbasp -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h```

Compiling and flashing:
```
make
make resetfuses
make flash
make setfuses
```
