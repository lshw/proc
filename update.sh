#!/bin/bash
cd `dirname $0`
avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b57600 -D -Uflash:w:prc.ino.hex:i
if [ $? != 0 ] ; then
avrdude -v -patmega328pb -carduino -P/dev/ttyUSB0 -b57600 -D -Uflash:w:prc.ino.hex:i
fi
