#!/bin/sh
# RN example script to setup GPIO pins for the camera button.
# reference: http://wiki.gumstix.org/index.php?title=GPIO

echo "Executing GPIO setup script..."

#
# Setup GPIO146 
#
gpiodir="/sys/class/gpio/gpio146"

/usr/bin/devmem2 0x48002196 h 0x10c

if [ ! -d "${gpiodir}" ]
then
  echo 146 >/sys/class/gpio/export
fi

echo out >${gpiodir}/direction
echo 0 >${gpiodir}/active_low
echo 0 >${gpiodir}/value

chmod 777 ${gpiodir}
chmod 666 ${gpiodir}/value


#
# Setup GPIO147 
#
gpiodir="/sys/class/gpio/gpio147"

/usr/bin/devmem2 0x48002178 h 0x10c

if [ ! -d "${gpiodir}" ]
then
  echo 147 >/sys/class/gpio/export
fi

echo in >${gpiodir}/direction
echo 1 >${gpiodir}/active_low
echo 0 >${gpiodir}/value

chmod 777 ${gpiodir}
chmod 666 ${gpiodir}/value
