#!/bin/bash
#
# dl.sh
#
# Some example code

ttyusb=/dev/ttyUSB0
hexfile=uno.cpp.hex

# program
if [ "${1}" = "pgm" ]
then
  avrdude \
    -P ${ttyusb} -b 57600 \
    -C /usr/share/arduino/hardware/tools/avrdude.conf \
    -c avrisp \
    -p atmega328p \
    -v -v -v -v \
    -e -D \
    -U flash:w:$(hexfile)
# -U lock:w:0x0F:m
fi

# fuses (only once)
if [ "${1}" = "fuses" ]
then
  /usr/share/arduino/hardware/tools/avrdude -C/usr/share/arduino/hardware/tools/avrdude.conf -v -v -v -v -patmega328p -cstk500v2 -Pusb -e -Ulock:w:0x3F:m -Uefuse:w:0x05:m -Uhfuse:w:0xDA:m -Ulfuse:w:0xFF:m -F
fi

if [ "${1}" = "boot" ]
then
fi
