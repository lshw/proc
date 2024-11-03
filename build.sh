#!/bin/bash
cd $( dirname $0 )

home=$( realpath ~ )

if ! [ -x prc ] ; then
 mkdir -p $home/Arduino
 cd $home/Arduino
 if ! [ -x proc ] ; then
  git clone https://github.com/lshw/proc
 fi
 cd proc
fi

me=`whoami`

for p  in wget git
do
 which $p
 if [ $? != 0 ]; then
  apt install -y $p
 fi
done
which arduino-cli
if [ $? != 0 ] ; then
 echo  $home/bin/arduino-cli
 if [ -x $home/bin/arduino-cli ] ; then
  arduino_cli=$home/bin/arduino-cli
 else
  echo 没有找到 arduino-cli
  echo 请到https://github.com/arduino/arduino-cli/releases 下载， 并放到 /usr/local/bin目录下
  mkdir ~/bin -p
  wget https://github.com/arduino/arduino-cli/releases/download/v1.0.4/arduino-cli_1.0.4_Linux_64bit.tar.gz -c
  tar zxf arduino-cli_1.0.4_Linux_64bit.tar.gz -C $home/bin arduino-cli
  if [ $? != 0 ] ; then
   exit
  fi
  if [ "$me" == "root" ] ;then
   mv $home/bin/arduino-cli /usr/local/bin
   arduino_cli=arduino-cli
  else
   arduino_cli=$home/bin/arduino-cli
  fi
 fi
else
 arduino_cli=arduino-cli
fi

echo $arduino_cli
rm -f ./prc/*.orig
a=`git rev-parse --short HEAD`
date=`git log --date=short -1 |grep ^Date: |awk '{print $2}' |tr -d '-'`
ver=$date-${a:0:7}
echo $ver
export COMMIT=$ver

#传递宏定义 GIT_VER 到源码中，源码git版本 编译参数
CXXFLAGS="-DGIT_VER=\"$ver\" -DBUILD_SET=\"$fqbn\""

mkdir -p /tmp/${me}_build /tmp/${me}_cache

fqbn="arduino:avr:pro:cpu=8MHzatmega328"
#fqbn="m328pb:avr:atmega328pbic:speed=8mhz"
#开发板:Arduino AVR Boards -> Arduino Pro or Pro Mini
#处理器:Atmega328P(3.3V,8Mhz)

#安装编译环境
echo $arduino_cli core install $( echo $fqbn |awk -F: '{print $1":"$2}' )
$arduino_cli core install $( echo $fqbn |awk -F: '{print $1":"$2}' )

#安装硬件驱动库
for lib in OneWire Ethernet3
do
echo $lib
 if ! [ -x $home/Arduino/libraries/$lib ] ; then
  $arduino_cli lib install $lib
 fi
done

$arduino_cli compile \
--fqbn $fqbn \
--verbose \
--build-property build.extra_flags="$CXXFLAGS" \
--build-path /tmp/${me}_build \
--build-cache-path /tmp/${me}_cache \
prc |tee /tmp/${me}_info.log
sync
if [ -e /tmp/${me}_build/prc.ino.hex ] ; then
  grep "Global vari" /tmp/${me}_info.log |sed -n "s/^Global variables use \([0-9]*\) bytes (\([0-9]*\)%) of dynamic memory, leaving \([0-9]*\) bytes for local variables. Maximum is 2048 bytes.$/RAM:使用\2%(\1字节),剩余\3字节/p"
  grep "^Sketch" /tmp/${me}_info.log |sed -n "s/Sketch uses \([0-9]*\) bytes (\([0-9]*\)%.*$/ROM:使用\2%(\1字节)/p"
  echo ver:$ver

  cp -a /tmp/${me}_build/prc.ino.hex ./prc.hex
ls -l $( realpath .)/prc.hex
fi
exit
Sketch uses 29078 bytes (94%) of program storage space. Maximum is 30720 bytes.
Global variables use 694 bytes (33%) of dynamic memory, leaving 1354 bytes for local variables. Maximum is 2048 bytes.

