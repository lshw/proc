#include "commit.h"  //版本号
#ifndef GIT_COMMIT_ID
#define GIT_COMMIT_ID "test"
#endif
#define _24V_OUT 3
#define PC_RESET A4
#define PC_POWER A5
#define PWM 5
#define DS A3
#define LED 13
#define NET_RESET 4

#include <EEPROM.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <OneWire.h>
#define EEPROM_OFFSET 12
int16_t celsius[11];
boolean alreadyConnected = false;
EthernetClient client;
uint8_t add_count = 0;
uint8_t pwm;
//定时器最长65536秒 18小时
uint16_t timer1 = 0; //秒 定时测温
uint16_t timer2 = 0; //秒
uint16_t volatile dogcount = 0; //超时重启，主程序循环清零，不清零的话100秒重启系统

#define S_TCP  1
#define S_SERIAL 0

uint8_t eeprom_read(uint16_t addr) {
  return EEPROM.read(addr + EEPROM_OFFSET);
}
void eeprom_write(uint16_t addr, uint8_t data) {
  if (eeprom_read(addr) != data)
    EEPROM.write(addr + EEPROM_OFFSET, data);
}

uint32_t eeprom_read_u32(uint16_t addr) {
  return ((uint32_t) eeprom_read(addr) << 24) |
         ((uint32_t) eeprom_read(addr + 1) << 16) |
         ((uint32_t) eeprom_read(addr + 2) << 8) |
         eeprom_read(addr + 3);
}
void eeprom_write_u32(uint16_t addr, uint32_t data) {
  eeprom_write(addr, (data >> 24) & 0xff);
  eeprom_write(addr + 1, (data >> 16) & 0xff);
  eeprom_write(addr + 2, (data >> 8) & 0xff);
  eeprom_write(addr + 3, data & 0xff);
}

uint8_t ds_addr[11][8];
OneWire ds(DS);

enum
{
  CAL38400,
  CAL57600,
  CAL115200,
  CAL230400,
  VOUT_SET,
  MAC0,
  MAC1,
  MAC2,
  MAC3,
  MAC4,
  MAC5,
  IS_DHCP,
  IP0,
  IP1,
  IP2,
  IP3,
  NETMARK0,
  NETMARK1,
  NETMARK2,
  NETMARK3,
  GW0,
  GW1,
  GW2,
  GW3,
  SPEED0,
  SPEED1,
  SPEED2,
  SPEED3,
  DATA_LEN,
  DATA_PARI,
  STOP_LEN,
  PASSWD0,
  PASSWD1,
  PASSWD2,
  PASSWD3,
  SN0,
  SN1,
  SN2,
  SN3,
  SN4,
  SN5,
  SN6,
  SN7,
  SN8,
  NAME0,
  NAME1,
  NAME2,
  NAME3,
  NAME4,
  NAME5,
  NAME6,
  NAME7,
  NAME8,
  NAME9,
  NAME10,
  WATCHDOG0,//r reset300ms
  WATCHDOG1,//w wait300ms
  WATCHDOG2,//P power 5 sec
  WATCHDOG3,//W water 5 sec
  WATCHDOG4,//o Vout off
  WATCHDOG5,//w
  WATCHDOG6,//O Vout on
  WATCHDOG7,
  WATCHDOG8,
  WATCHDOG9,
  WATCHDOG10,
  WATCHDOG_EN, //开启watchdog功能
  PWM_NOW, //当前PWM位置
  ROMCRC,
  //后面的不做校验
  REMOTE_CYCLE,
  REMOTE_PORT_H,
  REMOTE_PORT_L,
  REMOTE_HOST,
  ROMLEN
};
#define SCRIPT_SIZE 50 //每个脚本的长度  ;
#define SCRIPT_ADDR  ROMLEN+2 //起始地址

EthernetServer server(23);
uint8_t osc;
uint8_t get_cal(uint32_t speedx) {
  switch (speedx) {
    case 38400: return CAL38400;
    case 57600: return CAL57600;
    case 115200: return CAL115200;
    case 230400: return CAL230400;
    default:
      if (speedx > 230400) return CAL230400;
      return CAL38400;
  }
}
uint8_t remote_cycle;
void setup() {
  uint8_t  mac[6];
  pwm = eeprom_read(PWM_NOW);
  analogWrite(PWM, pwm);
  pinMode(_24V_OUT, OUTPUT);
  digitalWrite(_24V_OUT, HIGH); //默认24V开启输出
  pinMode(PC_RESET, OUTPUT);
  digitalWrite(PC_RESET, LOW);
  pinMode(PC_POWER, OUTPUT);
  digitalWrite(PC_POWER, LOW);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(NET_RESET, OUTPUT);
  digitalWrite(NET_RESET, LOW);
  osc = OSCCAL;
  ds1820_search();
  delay(100);
  ds1820_all();
  digitalWrite(NET_RESET, HIGH);
  while (eeprom_read(CAL38400) == 0xff || eeprom_read(CAL38400) == 0) {
    check_rom();
    Serial.begin(115200);
    rc_calibration(&Serial);
  }
  check_rom();
  uint32_t com_speed = eeprom_read_u32(SPEED0);
  OSCCAL = eeprom_read(get_cal(com_speed));
  if (com_speed == 0) com_speed = 115200;
  uint8_t com_set = get_comset();
  Serial.begin(com_speed, com_set);
  while (!Serial) ;
  hello(&Serial);
  digitalWrite(_24V_OUT, eeprom_read(VOUT_SET));
  mac[0] = 0xde; //mac的第一位必须是偶数，否则就是广播地址
  mac[1] = 0xad;
  mac[2] = 0xbe;
  mac[3] = eeprom_read(MAC3);
  mac[4] = eeprom_read(MAC4);
  mac[5] = eeprom_read(MAC5);
  IPAddress ip(eeprom_read(IP0), eeprom_read(IP1), eeprom_read(IP2), eeprom_read(IP3));
  IPAddress gateway(eeprom_read(GW0), eeprom_read(GW1), eeprom_read(GW2), eeprom_read(GW3));
  IPAddress subnet(eeprom_read(NETMARK0), eeprom_read(NETMARK1), eeprom_read(NETMARK2), eeprom_read(NETMARK3));

  bool dhcp_ok = false;
  while (Serial.available()) Serial.read();
  if (eeprom_read(IS_DHCP) != 'N' ) {
    Serial.print(F("\r\n#Use DHCP to get ip, please wait..."));
    for (uint8_t i = 0; i < 6; i++) {
      if (Serial.available()) break;
      delay(300);
      Serial.write('.');
    }
    if (!Serial.available()) {
      dhcp_ok = Ethernet.begin(mac);
      if (dhcp_ok) Serial.println(F("OK!"));
      else Serial.println(F("Failure."));
    } else Serial.println(F("Quit."));
  }
  if (dhcp_ok == false)
    Ethernet.begin(mac, ip, gateway, subnet); //dhcp==N 或者dhcp获取失败
  Serial.print(F("\r\n#ip: "));
  Serial.println(Ethernet.localIP());
  Serial.println(F("#+++++[enter] into main menu"));
  server.begin();
  setup_watchdog(WDTO_30MS);
  remote_cycle = eeprom_read(REMOTE_CYCLE);
}
EthernetClient clientn;
void loop() {
  dogcount = 0;
  clientn = server.available();

  if (clientn) {//有新的连接进来
    if ( alreadyConnected)
      client.stop(); //有bye状态的老的连接，就先踢掉
    else
      alreadyConnected = true;

    client = clientn;
  }

  if (timer1 == 0) {
    timer1 = 60; //60秒测温一次
    ds1820_all();
  }

  clientn = server.available();
  if (clientn) {//有数据进来
  if(clientn!=client) { 
    //有新的连接进来
    if ( alreadyConnected)
      client.stop(); //有bye状态的老的连接，就先踢掉
    alreadyConnected = true;
    client = clientn;
    }
  }

  if (!alreadyConnected) { //无连接
    if (remote_cycle > 0 && timer2 == 0) { //如果服务器没有客户端接入，就试试远程
      timer2 = 60;
      timer2 = remote_cycle; //每x分钟，连一次远程
      remote_link();
      remote_cycle = eeprom_read(REMOTE_CYCLE);
    }
  }

  if (alreadyConnected) {
    if (!client.connected()) {
      client.stop();
      alreadyConnected = false;
    }else {
      if (client.available() > 0) { //client tcp有数据进来
	menu(S_TCP);
	save_set();
      }
    }
  }

  if (Serial.available() > 0) {
    while (Serial.available()) {
      switch (Serial.read()) {
        case '+':
          add_count++;
          break;
        case 0xd:
        case 0xa:
          if (add_count > 4) {
            add_count = 0;
            menu(S_SERIAL);
            save_set();
          }
          break;
        default:
          if (add_count > 0 ) add_count = 0;
          else if (add_count > -100) add_count--;
      }
      if (add_count == -99) break; //持续的串口超过100字节就退出去，不要让串口锁定造成服务器忙。
    }
  }
}

void com_shell() {
  char ch, chs[512];
  uint16_t chlen = 0;
  add_count = 0;
  if (!client.connected()) return;
  s_clean(&Serial);
  s_clean(&client);
  while (1) {
    dogcount = 0;
    if (!client.connected()) {
      client.stop();
      alreadyConnected = false;
      return;
    }
    while (client.available() > 0) { //tcp有数据进来
      ch = client.read();
      if (ch == 0xd || ch == 0xa) {
        if (add_count > 4) {
          return;
        }
      }
      if (ch == '+' ) add_count++;
      else add_count = 0;
      Serial.write(ch);
    }
    uint32_t ms0 = millis() + 2000;
    while (Serial.available()) {
      chlen = 0;
      while (Serial.available()) {
        chs[chlen] = Serial.read();
        chlen++;
        if (chlen >= sizeof(chs)) break;
      }
      client.write(chs, chlen);
      if (ms0 < millis())
        break; //最多2秒
    }
  }
}

//看门狗中断做定时任务 30ms 1次
uint16_t volatile ms = 0;
int16_t volatile pc_reset_on = 0; //按下pc_reset键的ms时长
int16_t volatile pc_power_on = 0; //按下pc_power键的ms时长
ISR(WDT_vect) {
  dogcount++;  //30ms
  if (dogcount > 100000 / 30) {
    OSCCAL = osc;
    asm volatile ("  jmp 0"); //100秒看门狗超时重启
  }
  ms += 30;
  if (ms > 1000) {
    if (timer1 > 0) timer1--;//定时器1 测温
    if (timer2 > 0) timer2--;//定时器2 链接远程服务器
    ms -= 1000;
  }
  //处理reset键，其它程序只要修改 pc_reset_on=300，就可以按下300ms
  if (pc_reset_on > 0)
  {
    pc_reset_on -= 30; //30ms
    if (pc_reset_on > 0) { //reset开关按下
      if (digitalRead(PC_RESET) != HIGH)
        digitalWrite(PC_RESET, HIGH);
    } else { //reset开关松开
      if (digitalRead(PC_RESET) != LOW)
        digitalWrite(PC_RESET, LOW);
    }
  }
  //处理reset键，其它程序只要修改 pc_power_on=300，就可以按下300ms
  if (pc_power_on > 0) {
    pc_power_on -= 30; //30ms
    if (pc_power_on > 0) { //power开关按下
      if (digitalRead(PC_POWER) != HIGH)
        digitalWrite(PC_POWER, HIGH);
    } else { //power开关松开
      if (digitalRead(PC_POWER) != LOW)
        digitalWrite(PC_POWER, LOW);
    }
  }
}

//设置看门狗定时中断时间ii=WDTO_15MS .... WDTO_8S
void setup_watchdog(int ii) {
  byte bb;
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);
  MCUSR &= ~(1 << WDRF);
  // start timed sequence
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}
uint8_t vout;
void menu( uint8_t  stype) {
  uint32_t passwd, password;
  uint8_t ch;
  Stream *s;
  if (stype == S_SERIAL)
    s = &Serial;
  else
    s = &client;
  hello(s);
  s->setTimeout(10000);
  if ( stype != S_SERIAL) {
    password = eeprom_read_u32(PASSWD0);
    if (password != 0) {
      s->print(F("passwd: "));
      passwd = s->parseInt();
      if (client)
        s_clean(&client);
      s_clean(&Serial);
      if (passwd != password) {
        s->println();
        return;
      }
    }
  }
  s->print(F("ip="));
  s->print(Ethernet.localIP());
  while (1) {
    s_clean(s);
    s->println(F("\r\n======"));
    if (stype != S_SERIAL)
      s->println(F("0:com shell"));
    s->print(F("r:reset (300ms)\r\n"
               "R:reset (5 sec)\r\n"
               "p:powerdown(300ms)\r\n"
               "P:powerdown(5 sec)\r\n"
               "V:Vout= "));
    if (digitalRead(_24V_OUT) == HIGH) s->println(F("ON"));
    else s->println(F("OFF"));
    s->print(F("<,.> :PWM="));
    s->println(pwm);
    s->println(F("===script 1-9===="));
    disp_script( s, false); //从0号开始显示
    s->print(F("===set===\r\n"
               "a:reboot\r\n"
               "b:restore default set\r\n"
               /*      "Z:com speed calibration\r\n" */
               "c:network info &  modi\r\n"
               "d:com set\r\n"
               "e:setpasswd\r\n"
               "f:modi script 1-9\r\n"
              ));
    s->print(F("n:change name:"));
    disp_name(s);
    s->println(F("\r\nq:quit"));
    if (s->readBytes(&ch, 1) != 1) {
      s->println(F("Bye!"));
      return;
    }
    s->println();
    s->write(ch);
    switch (ch) {
      case '0':
        if (stype != S_SERIAL) {
          com_shell();
          return;
        }
        break;
      case '1'...'9':
        run_script(s, ch & 0xf);
        break;
      case 'f':
        modi_script(s);
        break;
      case 'n':
      case 'N':
        s->print(F("please input name: "));
        eeprom_set_str(s, NAME0, NAME10);
        set_rom_check();
      case 'r':
        pc_reset_on = 300;
        break;
      case 'R':
        pc_reset_on = 5000;
        break;
      case 'p':
        pc_power_on = 300;
        break;
      case 'P':
        pc_power_on = 5000;
        break;
      case 'v':
        digitalWrite(_24V_OUT, !digitalRead(_24V_OUT));
        eeprom_write(VOUT_SET, digitalRead(_24V_OUT));
        set_rom_check();
        break;
      case 'e':
      case 'E':
        set_passwd(s);
        break;
      case 'c':
      case 'C':
        info(stype);
        break;
      case 'd':
      case 'D':
        set_com_speed(s);
        break;
      case 'z':
      case 'Z':
        if (millis() < 200000)
          rc_calibration(stype);
        break;
      case 'b':
      case 'B':
        eeprom_write(ROMCRC, eeprom_read(ROMCRC) + 1);
      case 'a':
      case 'A':
        OSCCAL = osc;
        asm volatile ("  jmp 0"); //重启
        break;
      case '.':
        if (pwm < 255) pwm++;
      case ',':
        if (ch == ',' && pwm > 0) pwm--;
      case '>':
        if (ch == '>' && pwm < 255 - 10) pwm += 10;
      case '<':
        if (ch == '<' && pwm > 10) pwm -= 10;
        analogWrite(PWM, pwm);
        break;
      case 'q':
      case 'Q':
        s->println(F("Bye!"));
        s_clean(s);
        return;
    }
  }
}
uint8_t ds1820_count = 0;
void ds1820_search() {
  uint8_t i;
  uint8_t * addr;
  ds.reset_search();
  delay(250);
  celsius[0] = -400 * 16; //跳过0号
  memset(ds_addr, 0, sizeof(ds_addr));
  for (i = 0; i < 8; i++) ds_addr[0][i] = eeprom_read(SN0 + i);
  i = 1;

  while (i < 11) {
    if (!ds.search(ds_addr[i]))  {
      return;
    }
    if (OneWire::crc8(ds_addr[i], 7) != ds_addr[i][7]) {
      continue;
    }
    ds1820_count++;
    if (ds_addr[i][0] == ds_addr[0][0]
        && ds_addr[i][1] == ds_addr[0][1]
        && ds_addr[i][2] == ds_addr[0][2]
        && ds_addr[i][3] == ds_addr[0][3]
        && ds_addr[i][4] == ds_addr[0][4]
        && ds_addr[i][5] == ds_addr[0][5]
        && ds_addr[i][6] == ds_addr[0][6]
        && ds_addr[i][7] == ds_addr[0][7]
       ) {
      celsius[0] = 0; //存在0号温度探头 ,取消跳过
      continue; //跳过0号探头
    }    else {
      i++;
    }
  }
}

void ds1820_start() {
  ds.reset();
  ds.skip();
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
}
void ds1820_all() {
  ds1820_start();
  delay(800);
  for (uint8_t i = 0; i < 11; i++) {
    if (ds_addr[i][0] != 0 ) ds1820(i);
  }
}

void ds1820(uint8_t n) {
  byte i;
  byte present = 0;
  byte type_s;
  byte *addr;
  byte data[12];

  if (celsius[n] < -300 * 16) return;
  addr = ds_addr[n];
  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad
  if (OneWire::crc8(addr, 8) != addr[8]) {
    celsius[n] = -400 * 16;
    return;
  }
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  int16_t raw = (data[1] << 8) | data[0];
  if (data[0] == 0x10) { //
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {//ds18s20
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else { //0x28:ds18b20,0x22:ds1822
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius[n] = raw;
  //celsius = (float)raw / 16.0;
}
void ds1820_disp(Stream *s) {
  uint8_t n = 0;
  if (ds1820_count == 0) return;
  for (uint8_t i = 0; i < 11; i++) {
    if (celsius[i] < -300 * 16) continue;
    if (ds_addr[i][0] == 0) break;
    if (n > 0) s->write(';');
    s->write('C');
    if (ds1820_count > 1)
      for (uint8_t i0 = 5; i0 < 7; i0++) {
        if (ds_addr[i][i0] < 0x10) s->write('0');
        s->print(ds_addr[i][i0], HEX);
      }
    s->write('=');
    s->print((float) celsius[i] / 16.0);
    s->print(F("℃"));
    n++;
  }
  s->println();
}
void check_rom() {
  char * addr;
  uint8_t ch = 0, i;
  uint8_t sets[ROMLEN];
  for (i = 0; i < ROMLEN; i++) {
    sets[i] = eeprom_read(i);
    if (i <= ROMCRC) ch += sets[i];
  }

  addr = &sets[SN0];
  if (ds1820_count == 1 && ds_addr[1][0] != 0)
    for (i = 0; i < 8; i++) {
      addr[i] = ds_addr[1][i];
    }
  if (OneWire::crc8(addr, 7) !=  (uint8_t)addr[7])  {//SN不对
    if (OneWire::crc8(ds_addr[0], 7) == ds_addr[0][7])//当前SN有效
      for (i = 0; i < 8; i++) {
        addr[i] = ds_addr[0][i]; //复制当前SN
      }
    ch = 0xff;//重置ROM
  }
  if (ch != 0) {
    //set default
    for (i = 0; i < 10; i++)
      eeprom_write(SCRIPT_ADDR + SCRIPT_SIZE * i, 0); //清开机脚本
    sets[MAC0] = 0xDE;
    sets[MAC1] = 0xAD;
    sets[MAC2] = 0xBE;
    uint8_t * addr;
    if (ds1820_count == 1 && ds_addr[1][0] != 0)
      addr = ds_addr[1];
    else
      addr = ds_addr[0];
    sets[NAME0] = 'P';
    sets[NAME0 + 1] = 'R';
    sets[NAME0 + 2] = 'O';
    sets[NAME0 + 3] = 'C';
    if (ds_addr[0] != 0) {
      sets[MAC3] = addr[5];
      sets[MAC4] = addr[6];
      sets[MAC5] = addr[7];
      sprintf(&sets[NAME0 + 4], "%02X%02X%02X\x0", addr[5], addr[6], addr[7]);
    } else {
      sets[MAC3] = 0;
      sets[MAC4] = 1;
      sets[MAC5] = 0x25;
    }
    if (sets[CAL115200] == 0 || sets[CAL115200] == 0xff) {
      sets[CAL38400] = osc - 1;
      sets[CAL57600] = osc - +1;
      sets[CAL115200] = osc + 5;
      sets[CAL230400] = osc - 10;
    }
    sets[IS_DHCP] = 'N';
    sets[IP0] = 192;
    sets[IP1] = 168;
    sets[IP2] = 1;
    sets[IP3] = 2;
    sets[NETMARK0] = 255;
    sets[NETMARK1] = 255;
    sets[NETMARK2] = 255;
    sets[NETMARK3] = 0;
    sets[GW0] = 192;
    sets[GW1] = 168;
    sets[GW2] = 1;
    sets[GW3] = 1;
    sets[SPEED0] = 115200 >> 24;
    sets[SPEED1] = 115200 >> 16;
    sets[SPEED2] = 115200 >> 8;
    sets[SPEED3] = 115200;
    sets[DATA_LEN] = '8';
    sets[DATA_PARI] = 'N';
    sets[STOP_LEN] = '1';
    sets[PASSWD0] = 0;
    sets[PASSWD1] = 0;
    sets[PASSWD2] = 0;
    sets[PASSWD3] = 0;
    sets[WATCHDOG_EN] = 'N';
    sets[WATCHDOG0] = 'r';
    sets[WATCHDOG1] = 'v';
    sets[WATCHDOG2] = 'p';
    sets[WATCHDOG3] = 'W';
    sets[WATCHDOG4] = 'P';
    sets[WATCHDOG5] = 'V';
    sets[WATCHDOG6] = 0xff;
    sets[REMOTE_HOST] = 0;
    sets[REMOTE_CYCLE] = 0;
    sets[REMOTE_PORT_H] = 1234 / 0x100;
    sets[REMOTE_PORT_L] = 1234 % 0x100;
    sets[PWM_NOW] = 128;
    for (i = 0; i < sizeof(sets); i++) eeprom_write(i, sets[i]);
    set_rom_check();
  }
}
void set_rom_check() {
  uint8_t ch = 0;
  for (uint8_t i = 0; i < ROMCRC; i++)
    ch += eeprom_read(i);
  eeprom_write(ROMCRC, 0 - ch);
}
void set_passwd(Stream *s) {
  uint32_t passwd, password;
  s_clean(s);
  password = eeprom_read_u32(PASSWD0);
  if (password > 0) {
    s->print(F("current password: "));
    passwd = s->parseInt();
    s->println();
    if (passwd != password) {
      s->println(F("Bye!"));
      return;
    }
  }
  s_clean(s);
  s->print(F("New password: "));
  passwd = s->parseInt();
  s->println();
  s_clean(s);
  s->print(F("Retype new passwd: "));
  if (passwd != s->parseInt()) {
    s->println(F("\r\nerror"));
    return;
  }
  s->println();

  eeprom_write_u32(PASSWD0, passwd);
  set_rom_check();
}

//校准rc振荡器
void rc_calibration(uint8_t stype) {
  uint8_t osc1, osc0 = OSCCAL;
  uint8_t oscs[256];
  int8_t i, i0;
  uint8_t ch;
  uint32_t com_speed;
  Stream *s;
  memset(oscs, 0, sizeof(oscs));
  if (stype == S_SERIAL)
    s = &Serial;
  else
    s = &client;
  com_speed = eeprom_read_u32(SPEED0);
  s->print(F("osc="));
  s->print(osc);
  s->print(F(", CAL38400="));
  s->print(eeprom_read(CAL38400));
  s->print(F(", CAL57600="));
  s->print(eeprom_read(CAL57600));
  s->print(F(", CAL115200="));
  s->print(eeprom_read(CAL115200));
  s->print(F(", CAL230400="));
  s->println(eeprom_read(CAL230400));

  s->print(F("\r\nOn the Serial console("));
  disp_comset(s);
  s->println(F("),  Press the 'U' key and hold...."));
  s->flush();
  uint8_t com_set = get_comset();
  Serial.begin(com_speed, com_set);
  s_clean(&Serial);
  while (Serial.available() < 10) ;
  s_clean(&Serial);
  uint8_t b = 0;
  while (b == 0) {
    for (i = -20; i < 20; i++) {
      OSCCAL = osc + i;
      s_clean(&Serial);
      Serial.readBytes(&ch, 1);
      if (ch == 'U' || ch == 'u') {
        delay(100);
        s_clean(&Serial);
        Serial.readBytes(&ch, 1);
        if (ch == 'U' || ch == 'u') {
          i0 = i;
          b = 1;
          break;
        }
      }
    }
  }
  while (b == 1) {
    for (i = i0; i < 20; i++) {
      OSCCAL = osc + i;
      s_clean(&Serial);
      Serial.readBytes(&ch, 1);
      if (ch != 'u' &&  ch != 'U') {
        delay(100);
        s_clean(&Serial);
        Serial.readBytes(&ch, 1);
        if (ch != 'U' && ch != 'u') {
          i = ( i - i0) / 2 + i0;
          OSCCAL = osc + i ;
          eeprom_write(get_cal(com_speed), osc + i);
          set_rom_check();
          Serial.print(F("ok! calibration="));
          Serial.println(OSCCAL);
          if (stype != S_SERIAL) {
            client.print(F("ok! calibration="));
            client.println(OSCCAL);
            client.print(osc);
            if (OSCCAL > osc) {
              client.write('+');
              client.println(OSCCAL - osc);
            } else {
              client.write('-');
              client.println(osc - OSCCAL);
            }
          }
          delay(1000);
          return;
        }
      }
      if (client) {
        client.println(OSCCAL);
        oscs[OSCCAL] = 1;
      }
    }
  }
  OSCCAL = osc0;
  return ;
}

void info(uint8_t stype) {
  uint8_t ch;
  uint16_t i;
  uint32_t ms0;
  Stream *s;
  if (stype == S_SERIAL)
    s = &Serial;
  else
    s = &client;
  while (1) {
    s->println(F("\r\n============"));
    s->print(F("0.MAC: "));
    for (i = 0; i < 6; i++) {
      ch = eeprom_read(MAC0 + i);
      if (ch < 0x10)  s->write('0');
      s->print(ch, HEX);
    }
    s->print(F("\r\n1.DHCP: "));
    ch = eeprom_read(IS_DHCP);
    s->write(ch);
    if (ch == 'N' || ch == 'n') {
      s->print(F("\r\n2.IP: "));
      for (i = 0; i < 4; i++) {
        ch = eeprom_read(IP0 + i);
        s->print(ch);
        if (i < 3)
          s->write('.');
      }
      s->print(F("\r\n3.NETMARK: "));
      for (i = 0; i < 4; i++) {
        ch = eeprom_read(NETMARK0 + i);
        s->print(ch);
        if (i < 3)
          s->write('.');
      }
      s->print(F("\r\n4.GETWAY: "));
      for (i = 0; i < 4; i++) {
        ch = eeprom_read(GW0 + i);
        s->print(ch);
        if (i < 3)      s->write('.');
      }
    } else {
      s->print(F("\r\n5.REMOTE ENABLE: "));
      remote_cycle = eeprom_read(REMOTE_CYCLE);
      if (remote_cycle == 0) s->print(F("Disable"));
      else {
        s->print(remote_cycle);
        s->print(F(" minute"));
        if (remote_cycle > 1) s->write('s');
        s->print(F("\r\n6.hostname: "));
        for (i = 0; i < 30; i++) {
          ch = eeprom_read(REMOTE_HOST + i);
          if (ch == 0 || ch == 0xff) break;
          s->write(ch);
        }
        s->print(F("\r\n7.port: "));
        i = eeprom_read(REMOTE_PORT_H);
        i = (i << 8) | eeprom_read(REMOTE_PORT_L);
        s->print(i);
      }
    }
    s->println(F("\r\nplease select(1-7, q):"));
    dogcount = 0;
    s_clean(s);
    if (s->readBytes(&ch, 1) != 1) {
      s->println(F("Bye!"));
      return;
    }
    delay(100);
    while (s->available()) s->read();
    switch (ch) {
      case '0':
        s->println(F("todo..."));
        s_clean(s);
        s->readBytes(&ch, 1);
        break;
      case '1':
        if (eeprom_read(IS_DHCP) == 'Y')
          eeprom_write(IS_DHCP, 'N');
        else
          eeprom_write(IS_DHCP, 'Y');
        set_rom_check();
        s_clean(s);
        break;
      case '2':
        s->print(F("please input ip: "));
        save_set(IP0, s);
        break;
      case '3':
        s->print(F("please input netmark: "));
        save_set(NETMARK0, s);
        break;
      case '4':
        s->print(F("please input gateway: "));
        save_set(GW0, s);
        break;
      case '5':
        s->print(F("please input cycle(minutes, 0=disable): "));
        ch = s->parseInt();
        s->println(ch, HEX);
        eeprom_write(REMOTE_CYCLE, ch);
        remote_cycle = ch;
        timer2 = 0; //马上发送一次
        break;

      case '6':
        s->print(F("please input remote hostname: "));
        eeprom_set_str(s, REMOTE_HOST, REMOTE_HOST + 30);
        break;
      case '7':
        s->print(F("please input remote port: "));
        i = s->parseInt();
        eeprom_write(REMOTE_PORT_H, i / 0x100);
        eeprom_write(REMOTE_PORT_L, i & 0xFF);
        break;
      case 'q':
      case 'Q':
      case 0x1b:
        return;
    }
  }
}
void save_set(uint16_t addr,  Stream *s) {
  uint8_t ch;
  ch = s->parseInt();
  s->print(ch);
  s->write('.');
  eeprom_write(addr, ch);
  ch = s->parseInt();
  s->print(ch);
  s->write('.');
  eeprom_write(addr + 1, ch);
  ch = s->parseInt();
  s->print(ch);
  s->write('.');
  eeprom_write(addr + 2, ch);
  ch = s->parseInt();
  s->println(ch);
  eeprom_write(addr + 3, ch);
  set_rom_check();
}
void set_com_speed(Stream *s) {
  uint8_t ch;
  uint32_t speed1;
  s->print(F("current speed: "));
  s->println(eeprom_read_u32(SPEED0));
  s->print(F("please input new speed: "));
  s_clean(s);
  speed1 = s->parseInt(SKIP_NONE, 0xa);
  if (speed1 >= 10) {
    if (speed1 > 600000)   speed1 = 921600;
    else if (speed1 > 300000) speed1 = 460800;
    else if (speed1 > 180000) speed1 = 230400;
    else if (speed1 > 70000) speed1 = 115200;
    else if (speed1 > 48000) speed1 = 57600;
    else if (speed1 > 28800) speed1 = 38400;
    else if (speed1 > 14400) speed1 = 19200;
    else if (speed1 > 7200) speed1 = 9600;
    else if (speed1 > 3600) speed1 = 4800;
    else if (speed1 > 1800) speed1 = 2400;
    else if (speed1 > 900) speed1 = 1200;
    else if (speed1 > 450) speed1 = 600;
    else speed1 = 300;
    s->println(speed1);
    eeprom_write_u32(SPEED0, speed1);
    set_rom_check();
  }
  s_clean(s);
  s->print(F("data len(5-8): "));
  if (s->readBytes(&ch, 1) == 1)
    if (ch >= '5' &&  ch <= '8') {
      eeprom_write(DATA_LEN, ch);
      set_rom_check();
    }
  delay(100);
  s_clean(s);
  s->print(F("\r\nParity(N, O, E): "));
  s->readBytes(&ch, 1);
  if (ch == 'o') ch = 'O';
  if (ch == 'e') ch = 'E';
  if (ch == 'n') ch = 'N';
  if (ch == 'O' || ch == 'E' || ch == 'N') {
    eeprom_write(DATA_PARI, ch);
    set_rom_check();
  }
  s_clean(s);
  s->print(F("\r\nstop len(1, 2): "));
  s->readBytes(&ch, 1);
  if (ch == '2' || ch == '1') {
    eeprom_write(STOP_LEN, ch);
    set_rom_check();
  }
  s->print(F("set speed ok, "));
  disp_comset(s);
  s->println();
  s_clean(s);
  check_rom();
}
void disp_comset(Stream *s) {
  s->print(eeprom_read_u32(SPEED0));
  s->write(',');
  s->write(eeprom_read(DATA_LEN));
  s->write(eeprom_read(DATA_PARI));
  s->write(eeprom_read(STOP_LEN));
}
uint8_t get_comset() {
  switch (eeprom_read(DATA_PARI)) {
    case 'N':
      if (eeprom_read(STOP_LEN) == '1') {
        switch (eeprom_read(DATA_LEN)) {
          case '5': return SERIAL_5N1;//0x00
          case '6': return SERIAL_6N1;//0x02
          case '7': return SERIAL_7N1;//0x04
          case '8': return SERIAL_8N1;//0x06
        }//switch DATA_LEN
      } else {//if stop_len
        switch (eeprom_read(DATA_LEN)) {
          case '5': return SERIAL_5N2;//0x08
          case '6': return SERIAL_6N2;//0x0A
          case '7': return SERIAL_7N2;//0x0C
          case '8': return SERIAL_8N2;//0x0E
        } //switch DATA_LEN
      }//if stop_len
      break;
    case 'E':
      if (eeprom_read(STOP_LEN) == '1') {
        switch (eeprom_read(DATA_LEN)) {
          case '5': return SERIAL_5E1;//0x20
          case '6': return SERIAL_6E1;//0x22
          case '7': return SERIAL_7E1;//0x24
          case '8': return SERIAL_8E1;//0x26
        }//switch data_len
      } else { //if stop len
        switch (eeprom_read(DATA_LEN)) {
          case '5': return SERIAL_5E2;//0x28
          case '6': return SERIAL_6E2;//0x2A
          case '7': return SERIAL_7E2;//0x2C
          case '8': return SERIAL_8E2;//0x2E
        }//switch data len
      }//if stop len
      break;
    case 'O':
      if (eeprom_read(STOP_LEN) == '1') {
        switch (eeprom_read(DATA_LEN)) {
          case '5': return SERIAL_5O1;//0x30
          case '6': return SERIAL_6O1;//0x32
          case '7': return SERIAL_7O1;//0x34
          case '8': return SERIAL_8O1;//0x36
        }
      } else {//if stop len
        switch (eeprom_read(DATA_LEN)) {
          case '5': return SERIAL_5O2;//0x38
          case '6': return SERIAL_6O2;//0x3A
          case '7': return SERIAL_7O2;//0x3C
          case '8': return SERIAL_8O2;//0x3E
        } //switch data len
      } //if stop len
      break;
  }//switch pari
  return SERIAL_8N1;
}
void s_clean(Stream * s) {
  delay(10);
  while (s->available()) s->read();
}
void hello(Stream *s) {
  uint8_t ch;
  s->print(F("\r\n#DOC HTTPS://bjlx.org.cn/node/914\r\n#Ver:PROC-V1-" GIT_COMMIT_ID  "\r\n#name:"));
  disp_name(s);
  s->print(F("\r\n#SN:"));
  for (uint8_t i = 0; i < sizeof(ds_addr[0]); i++) {
    ch = eeprom_read(SN0 + i);
    if (ch < 0x10) s->write('0');
    s->print((uint8_t)ch, HEX);
  }
  s->print(F("\r\n#com:"));
  disp_comset(s);
  s->print(F("\r\n#"));
  ds1820_disp(s);
}
void remote_link() {
  char hostname[30];
  uint8_t i;
  for (i = 0; i < sizeof(hostname); i++) {
    hostname[i] = eeprom_read(REMOTE_HOST + i);
    if (hostname[i] == 0) break;
  }
  client.connect(hostname, (uint16_t)(eeprom_read(REMOTE_PORT_H) << 8) | eeprom_read(REMOTE_PORT_L));
}
void disp_name(Stream * s) {
  char ch;
  for (uint16_t i = NAME0; i <= NAME10; i++) {
    ch = eeprom_read(i);
    if (ch == 0)
      break;
    s->write(ch);
  }
}

//通过console 输入字符串，保存到eeprom的 addr0- addr1
void eeprom_set_str(Stream * s, uint16_t addr0, uint16_t addr1) {
  uint32_t ms0;
  uint8_t ch;
  for (uint16_t i = addr0; i < addr1; i++) {
    ms0 = millis() + 10000;
    while (s->available() == 0) if (ms0 < millis()) break;
    if (ms0 < millis()) break;
    ch = s->read();
    s->write(ch);
    switch (ch) {
      case 0xd:
      case 0xa:
        ch = 0;
      case '0'...'9':
      case 'a'...'z':
      case 'A'...'Z':
      case '.':
      case '_':
      case '-':
        eeprom_write(i, ch);
        if (i < addr1) eeprom_write(i + 1, 0);
        break;
      case 0x8:
        if (i > addr0 + 1) {
          i = i - 2;
          eeprom_write(i + 1, 0);
        }
        break;
    }
    if (ch == 0) break;
  }
}
void run_script( Stream * s, uint8_t script_n) {
  uint8_t i, ch;
  uint16_t eeprom_addr, n;
  eeprom_addr = SCRIPT_ADDR + SCRIPT_SIZE * script_n;
  s->print(F("run script")); s->println(script_n);
  ch = eeprom_read(eeprom_addr);
  if (ch == 0 || ch >= 0x80) return;
  while (ch != 0 && ch < 0x80) {
    s->write(ch);
    switch (ch) {
      case 'p':
      case 'P':
        if (next_is_number(eeprom_addr)) {
          eeprom_addr++;
          pc_power_on = get_uint16(eeprom_addr); //addr要被更新
          s->print(pc_power_on);
        } else {
          if (ch == 'P')
            digitalWrite(PC_POWER, HIGH);
          else
            digitalWrite(PC_POWER, LOW);
        }
        break;
      case 'r':
      case 'R':
        if (next_is_number(eeprom_addr)) {
          eeprom_addr++;
          pc_reset_on = get_uint16(eeprom_addr); //i要被更新
          s->print(pc_reset_on);
        } else {
          if (ch == 'R')
            digitalWrite(PC_RESET, HIGH);
          else
            digitalWrite(PC_RESET, LOW);
        }
        break;
      case 'v':
        digitalWrite(_24V_OUT, LOW);
      case 'V':
        if (next_is_number(eeprom_addr)) {
          eeprom_addr++;
          n = get_uint16(eeprom_addr);
          analogWrite(_24V_OUT, n);
          s->print(n);
        } else {
          if (ch == 'V')
            digitalWrite(_24V_OUT, HIGH);
          else
            digitalWrite(_24V_OUT, LOW);
        }
        break;
      case 't':
      case 'T':
        if (next_is_number(eeprom_addr)) {
          eeprom_addr++;
          n = get_uint16(eeprom_addr);
          s->print(n);
          delay(n);
        }
        break;
      case 'm':
      case 'M':
        if (next_is_number(eeprom_addr)) {
          eeprom_addr++;
          pwm = get_uint16(eeprom_addr);
          s->print(pwm);
          analogWrite(PWM, pwm);
        }
        break;
    }
    eeprom_addr++;
    ch = eeprom_read(eeprom_addr);
  }
}
uint16_t get_uint16(uint16_t & addr) {
  uint16_t ret = 0;
  uint8_t ch;

  ch = eeprom_read(addr);
  while (ch >= '0' && ch <= '9') {
    ret *= 10;
    ret += ch & 0xf;
    addr++;
    ch = eeprom_read(addr);
  }
  addr--;
  return ret;
}
bool next_is_number(uint16_t addr) {
  uint8_t ch;
  ch = eeprom_read(addr + 1);
  return ch >= '0' && ch <= '9';
}
void modi_script( Stream * s) {

  uint8_t ch;
  delay(100);
  s_clean(s);
  s->println();
  disp_script( s, true); //从0号开始显示
  while (!s->available())  dogcount = 0;
  ch = s->read();
  Serial.write(ch);
  switch (ch) {
    case '1'...'9':
      Serial.print(F("-please input script:"));
      ch = ch & 0xf;
      eeprom_set_str(s, SCRIPT_ADDR +  SCRIPT_SIZE * ch, SCRIPT_ADDR + (ch + 1)*SCRIPT_SIZE);
      break;
    case 'q':
    case 'Q':
      return;
  }
}
void disp_script( Stream * s, bool disp_zero) {
  uint8_t i, ch;
  for (i = 1; i < 10; i++) {
    ch = eeprom_read(SCRIPT_ADDR + i * SCRIPT_SIZE);
    if (!disp_zero && (ch == 0 || ch == 0xff)) continue;
    s->print(i);
    s->write(':');
    for (uint8_t m = 0; m < SCRIPT_SIZE; m++) {
      ch = eeprom_read(SCRIPT_ADDR + i * SCRIPT_SIZE + m);
      if (ch == 0 || ch == 0xff) break;
      if (ch < 0x80) s->write(ch);
    }
    s->println();
  }
}

void save_set() { //退出菜单时，保存pwm位置等设置
  eeprom_write(PWM_NOW, pwm);
  set_rom_check();
}
