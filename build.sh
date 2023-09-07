#!/bin/bash
cd `dirname $0`

me=`whoami`
if [ "$me" == "root" ] ; then
  home=/home/liushiwei
else
  home=~
fi

if [ -x $home/sketchbook/libraries ] ; then
 sketchbook=$home/sketchbook
else
 sketchbook=$home/Arduino
fi

arduino=/opt/arduino-1.8.19
if ! [ -x $arduino/arduino ] ; then
 echo install arduino to $arduino ?
 read yes
 if [ "a$yes" == 'ay' ] ; then
  wget "https://downloads.arduino.cc/arduino-1.8.19-linux64.tar.xz" -c  -O /opt/arduino-1.8.19-linux64.tar.xz
  tar Jxvf /opt/arduino-1.8.19-linux64.tar.xz -C /opt
 fi
fi
astyle  --options=$arduino/lib/formatter.conf ./prc/*.ino
rm -f ./prc/*.orig
a=`git rev-parse --short HEAD`
date=`git log --date=short -1 |grep ^Date: |awk '{print $2}' |tr -d '-'`
ver=$date-${a:0:7}
echo $ver
export COMMIT=$ver

arduinoset=$home/.arduino15
mkdir -p /tmp/${me}_build /tmp/${me}_cache

#开发板:Arduino AVR Boards -> Arduino Pro or Pro Mini
#处理器:Atmega328P(3.3V,8Mhz) 
fqbn="arduino:avr:pro:cpu=8MHzatmega328"

#fqbn="m328pb:avr:atmega328pbic:speed=8mhz"

#传递宏定义 GIT_VER 到源码中，源码git版本
CXXFLAGS="-DGIT_VER=\"$ver\" -DBUILD_SET=\"$fqbn\""
$arduino/arduino-builder \
-dump-prefs \
-logger=machine \
-hardware $arduino/hardware \
-hardware $arduinoset/packages \
-tools $arduino/tools-builder \
-tools $arduino/hardware/tools/avr \
-tools $arduinoset/packages \
-built-in-libraries $arduino/libraries \
-libraries $sketchbook/libraries \
-fqbn=$fqbn \
-ide-version=10819 \
-build-path /tmp/${me}_build \
-warnings=none \
-prefs build.extra_flags="$CXXFLAGS" \
-build-cache /tmp/${me}_cache \
-prefs=build.warn_data_percentage=75 \
-verbose \
./prc/prc.ino
if [ $? != 0 ] ; then
  exit
fi
rm -f /tmp/${me}_build/prc.ino.bin
$arduino/arduino-builder \
-compile \
-logger=machine \
-hardware $arduino/hardware \
-hardware $arduinoset/packages \
-tools $arduino/tools-builder \
-tools $arduino/hardware/tools/avr \
-tools $arduinoset/packages \
-built-in-libraries $arduino/libraries \
-libraries $sketchbook/libraries \
-fqbn=$fqbn \
-ide-version=10819 \
-build-path /tmp/${me}_build \
-warnings=none \
-prefs build.extra_flags="$CXXFLAGS" \
-build-cache /tmp/${me}_cache \
-prefs=build.warn_data_percentage=75 \
-verbose \
./prc/prc.ino |tee /tmp/${me}_info.log

if [ -e /tmp/${me}_build/prc.ino.hex ] ; then

  grep "Global vari" /tmp/${me}_info.log |sed -n "s/^.* \[\([0-9]*\) \([0-9]*\) \([0-9]*\) \([0-9]*\)\].*$/RAM:使用\1字节(\3%),剩余\4字节/p"
  grep "Sketch uses" /tmp/${me}_info.log |sed -n "s/^.* \[\([0-9]*\) \([0-9]*\) \([0-9]*\)\].*$/ROM:使用\1字节(\3%)/p"
  echo ver:$ver

  cp -a /tmp/${me}_build/prc.ino.hex .
fi
