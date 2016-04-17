# cross-compile module makefile

CROSS_COMPILE=/opt/pkg/overo-oe/tmp-RNMP32/sysroots/x86_64-linux/usr/armv7a/bin/arm-angstrom-linux-gnueabi-gcc
CFLAGS=-Wall -I/opt/pkg/overo-oe/tmp-RNMP32/sysroots/armv7a-angstrom-linux-gnueabi/usr/include/ -I/opt/pkg/overo-oe/tmp-RNMP32/sysroots/armv7a-angstrom-linux-gnueabi/usr/include/sysfs/ -lsysfs
SOURCES=battery-logger.c

default:
	$(CROSS_COMPILE) $(CFLAGS) $(SOURCES) -o battery-logger
install:
#	sudo cp udev_test /exports/overo/home/root

clean:
	@rm battery-logger

