#!/bin/bash
cd `dirname $0`
avrdude -v -patmega328pb -carduino -P/dev/ttyUSB0 -b57600 -D -Uflash:w:prc.ino.hex:i
