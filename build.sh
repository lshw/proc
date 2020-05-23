#!/bin/bash
cd `dirname $0`
branch=`git branch |grep "^\*" |awk '{print $2}'`
a=`git rev-parse --short HEAD`
date=`git log --date=short -1 |grep ^Date: |awk '{print $2}' |tr -d '-'`
ver=$date-${a:0:7}
echo $ver
export COMMIT=$ver

echo "#define GIT_COMMIT_ID \"$ver\"" > prc/commit.h

mkdir /tmp/build -p
/opt/arduino-1.8.12/arduino-builder -dump-prefs -logger=machine -hardware /opt/arduino-1.8.12/hardware -hardware /home/liushiwei/.arduino15/packages -tools /opt/arduino-1.8.12/tools-builder -tools /opt/arduino-1.8.12/hardware/tools/avr -tools /home/liushiwei/.arduino15/packages -built-in-libraries /opt/arduino-1.8.12/libraries -libraries /home/liushiwei/sketchbook/libraries -fqbn=m328pb:avr:atmega328pbic:speed=8mhz -vid-pid=0403_6001 -ide-version=10812 -build-path /tmp/build -warnings=none -build-cache /tmp/arduino_cache_191088 -prefs=build.warn_data_percentage=75 -prefs=runtime.tools.avr-gcc.path=/home/liushiwei/.arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino5 -prefs=runtime.tools.avr-gcc-7.3.0-atmel3.6.1-arduino5.path=/home/liushiwei/.arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino5 -prefs=runtime.tools.arduinoOTA.path=/home/liushiwei/.arduino15/packages/arduino/tools/arduinoOTA/1.3.0 -prefs=runtime.tools.arduinoOTA-1.3.0.path=/home/liushiwei/.arduino15/packages/arduino/tools/arduinoOTA/1.3.0 -prefs=runtime.tools.avrdude.path=/home/liushiwei/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17 -prefs=runtime.tools.avrdude-6.3.0-arduino17.path=/home/liushiwei/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17 -verbose /home/liushiwei/sketchbook/prc/prc/prc.ino
/opt/arduino-1.8.12/arduino-builder -compile -logger=machine -hardware /opt/arduino-1.8.12/hardware -hardware /home/liushiwei/.arduino15/packages -tools /opt/arduino-1.8.12/tools-builder -tools /opt/arduino-1.8.12/hardware/tools/avr -tools /home/liushiwei/.arduino15/packages -built-in-libraries /opt/arduino-1.8.12/libraries -libraries /home/liushiwei/sketchbook/libraries -fqbn=m328pb:avr:atmega328pbic:speed=8mhz -vid-pid=0403_6001 -ide-version=10812 -build-path /tmp/build -warnings=none -build-cache /tmp/arduino_cache_191088 -prefs=build.warn_data_percentage=75 -prefs=runtime.tools.avr-gcc.path=/home/liushiwei/.arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino5 -prefs=runtime.tools.avr-gcc-7.3.0-atmel3.6.1-arduino5.path=/home/liushiwei/.arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino5 -prefs=runtime.tools.arduinoOTA.path=/home/liushiwei/.arduino15/packages/arduino/tools/arduinoOTA/1.3.0 -prefs=runtime.tools.arduinoOTA-1.3.0.path=/home/liushiwei/.arduino15/packages/arduino/tools/arduinoOTA/1.3.0 -prefs=runtime.tools.avrdude.path=/home/liushiwei/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17 -prefs=runtime.tools.avrdude-6.3.0-arduino17.path=/home/liushiwei/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17 -verbose /home/liushiwei/sketchbook/prc/prc/prc.ino |tee /tmp/info_wifi.log
cp /tmp/build/prc.ino.hex .
if [ $? == 0 ] ; then
/home/liushiwei/.arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino5/bin/avr-size /tmp/build/prc.ino.elf |head -n 5
 grep "Global vari" /tmp/info_wifi.log |awk -F[ '{printf $2}'|tr -d ']'|awk -F' ' '{print "内存：使用"$1"字节,"$3"%,剩余:"$4"字节"}'
 grep "Sketch uses" /tmp/info_wifi.log |awk -F[ '{printf $2}'|tr -d ']'|awk -F' ' '{print "ROM：使用"$1"字节,"$3"%"}'
fi
