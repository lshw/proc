#!/bin/bash
echo 3
sleep 1
echo 2
sleep 1
echo please power on or reset.
sleep 1
cd `dirname $0`
avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b57600 -D -Uflash:w:prc.ino.hex:i
if [ $? != 0 ] ; then
avrdude -v -patmega328pb -carduino -P/dev/ttyUSB0 -b57600 -D -Uflash:w:prc.ino.hex:i
fi
