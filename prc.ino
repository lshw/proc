#define debug  false
#define _24V_OUT 3
#define PC_RESET A4
#define PC_POWER A5
#define DS A3
#define LED 13
#define NET_RESET 4

#include <EEPROM.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <OneWire.h>
#define EEPROM_OFFSET 12
//eeprom 内存分配
float celsius;//温度
boolean alreadyConnected = false;
EthernetClient client;
uint8_t add_count = 0;
uint32_t timer1 = 0; //秒
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

uint8_t goto_bootloader __attribute__ ((section (".noinit"))); //需要进入bootloader
uint8_t goto_bootloader_crc __attribute__ ((section (".noinit")));

byte ds_addr[8];
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
  ROM_NC,
  ROMCRC,
  ROMLEN
};

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
void setup() {
  byte mac[6];
  pinMode(_24V_OUT, OUTPUT);
  digitalWrite(_24V_OUT, HIGH); //默认24V开启输出
  pinMode(PC_RESET, OUTPUT);
  digitalWrite(PC_RESET, LOW);
  pinMode(PC_POWER, OUTPUT);
  digitalWrite(PC_POWER, LOW);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(NET_RESET, OUTPUT);
  digitalWrite(NET_RESET, HIGH);
  osc = OSCCAL;
  ds1820();
  check_rom();
  uint32_t com_speed = eeprom_read_u32(SPEED0);
  OSCCAL = eeprom_read(get_cal(com_speed));
  if (com_speed == 0) com_speed = 115200;
  uint8_t com_set = get_comset();
  Serial.begin(com_speed, com_set);
  while (!Serial) ;
  Serial.write('#');
  disp_comset(&Serial);
  digitalWrite(_24V_OUT, eeprom_read(VOUT_SET));
  for (uint8_t i = 0; i < 6; i++)
    mac[i] = eeprom_read(MAC0 + i);
  IPAddress ip(eeprom_read(IP0), eeprom_read(IP1), eeprom_read(IP2), eeprom_read(IP3));
  IPAddress gateway(eeprom_read(GW0), eeprom_read(GW1), eeprom_read(GW2), eeprom_read(GW3));
  IPAddress subnet(eeprom_read(NETMARK0), eeprom_read(NETMARK1), eeprom_read(NETMARK2), eeprom_read(NETMARK3));

  if (eeprom_read(IS_DHCP) == 'N' ||  Ethernet.begin(mac) == 0)
    Ethernet.begin(mac, ip, gateway, subnet); //dhcp==N 或者dhcp获取失败
  Serial.print(F("\r\n#ip:"));
  Serial.println(Ethernet.localIP());
  server.begin();
  setup_watchdog(WDTO_30MS);
}
void loop() {
  char ch;
  dogcount = 0;
  if (!alreadyConnected) {
    client = server.available();
    if (client) {
      alreadyConnected = true;
      delay(100);
      s_clean(&client);
      client.print(F("C="));
      client.println(celsius);
    }
  }

  if (alreadyConnected) {
    if (!client.connected()) {
      client.stop();
      alreadyConnected = false;
    }
    if (client.available() > 0) { //tcp有数据进来
      menu(S_TCP);
    }
  }

  if (timer1 == 0) {
    timer1 = 60; //60秒测温一次
    ds1820();
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
          }
          break;
        default:
          add_count = 0;
      }
    }
  }
}

void com_shell() {
  char ch, chlen = 0, chs[250];

  if (!client.connected()) return;
  s_clean(&Serial);
  s_clean(&client);
  while (1) {
    dogcount = 0;
    if (!client.connected()) return;
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
    while (Serial.available()) {
      chlen = 0;
      while (Serial.available()) {
        chs[chlen] = Serial.read();
        chlen++;
        if (chlen >= 250) break;
      }
      client.write(chs, chlen);
    }
  }
}

//看门狗中断做定时任务 30ms 1次
uint16_t volatile sec = 0, ms = 0;
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
    if (timer1 > 0) timer1--;
    ms -= 1000;
    sec++;
    if (debug)
      if (sec % 10 == 0)
        Serial.println(sec);
  }
  //处理reset键，其它程序只要修改 pc_reset_on=300，就可以按下300ms
  if (pc_reset_on > 0) pc_reset_on -= 30; //30ms
  if (pc_reset_on > 0) { //reset开关按下
    if (digitalRead(PC_RESET) != HIGH)
      digitalWrite(PC_RESET, HIGH);
  } else { //reset开关松开
    if (digitalRead(PC_RESET) != LOW)
      digitalWrite(PC_RESET, LOW);
  }

  //处理reset键，其它程序只要修改 pc_power_on=300，就可以按下300ms
  if (pc_power_on > 0) pc_power_on -= 30; //30ms
  if (pc_power_on > 0) { //power开关按下
    if (digitalRead(PC_POWER) != HIGH)
      digitalWrite(PC_POWER, HIGH);
  } else { //power开关松开
    if (digitalRead(PC_POWER) != LOW)
      digitalWrite(PC_POWER, LOW);
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

void menu( uint8_t  stype) {
  uint32_t passwd, password;
  char ch;
  Stream *s;
  if (stype == S_SERIAL)
    s = &Serial;
  else
    s = &client;
  s->print(F("C="));
  s->println(celsius);
  s->print(F("SN="));
  for (uint8_t i = 0; i < sizeof(ds_addr); i++) {
    if (ds_addr[i] < 0x10) s->write('0');
    s->print(ds_addr[i], HEX);
  }
  s->setTimeout(10000);
  password = eeprom_read_u32(PASSWD0);
  if (password != 0) {
    s->print(F("\r\npasswd:"));
    passwd = s->parseInt();
    if (client)
      s_clean(&client);
    s_clean(&Serial);
    if (passwd != password) {
      s->println();
      return;
    }
  }
  s->print(F("\r\nip="));
  s->print(Ethernet.localIP());
  while (1) {
    s_clean(s);
    s->print(F("\r\n============Vout:"));
    if (digitalRead(_24V_OUT) == HIGH) s->println(F("On"));
    else s->println(F("Off"));
    if (stype != S_SERIAL)
      s->println(F("0-com shell"));
    s->print(F("1-reset (300ms)\r\n"
               "2-powerdown(300ms)\r\n"
               "3-powerdown(5 sec)\r\n"
               "4-set Vout to "));
    if (digitalRead(_24V_OUT) == HIGH) s->print(F("Off"));
    else s->print(F("On"));
    s->print(F("(3 sec)\r\n"
               "5-set Vout to "));
    if (digitalRead(_24V_OUT) == HIGH) s->println(F("Off"));
    else s->println(F("On"));
    s->println(F("6-setpasswd\r\n"
                 "7-network info &  modi\r\n"
                 "8-com set\r\n"
                 "9-com speed calibration\r\n"
                 "a-reboot\r\n"
                 "b-restore default set\r\n"
                 "q-quit"));
    if (s->readBytes(&ch, 1) != 1) return;
    switch (ch) {
      case '0':
        if (stype != S_SERIAL)
          com_shell();
        break;
      case '1':
        s->write(ch);
        pc_reset_on = 300;
        break;
      case '2':
        s->write(ch);
        pc_power_on = 300;
        break;
      case '3':
        s->write(ch);
        pc_power_on = 5000;
        break;
      case '4':
        s->write(ch);
        digitalWrite(_24V_OUT, !digitalRead(_24V_OUT));
        s->print(F("\r\nVout="));
        s->print(digitalRead(_24V_OUT));
        delay(3000);
        digitalWrite(_24V_OUT, !digitalRead(_24V_OUT));
        s->print(F("\r\nVout="));
        s->println(digitalRead(_24V_OUT));
        delay(1000);
        s_clean(s);
        break;
      case '5':
        s->write(ch);
        digitalWrite(_24V_OUT, !digitalRead(_24V_OUT));
        eeprom_write(VOUT_SET, digitalRead(_24V_OUT));
        set_rom_check();
        break;
      case '6':
        set_passwd(s);
        break;
      case '7':
        info(stype);
        break;
      case '8':
        set_com_speed(s);
        break;
      case '9':
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
      case 'q':
      case 'Q':
        s_clean(s);
        return;
    }
  }
}
void ds1820() {
  byte i;
  byte present = 0;
  byte type_s;
  byte *addr;
  byte data[12];
  addr = ds_addr;
  if ( !ds.search(addr)) {
    ds.reset_search();
    delay(250);
    return;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    return;
  }

  switch (addr[0]) {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
}

void check_rom() {
  uint8_t ch = 0, i;
  uint8_t sets[ROMLEN];
  for (i = 0; i < ROMLEN; i++) {
    sets[i] = eeprom_read(i);
    ch += sets[i];
  }
  if (ch != 0) {
    //set default
    if (ds_addr[0] != 0) {
      sets[MAC0] = ds_addr[2];
      sets[MAC1] = ds_addr[3];
      sets[MAC2] = ds_addr[4];
      sets[MAC3] = ds_addr[5];
      sets[MAC4] = ds_addr[6];
      sets[MAC5] = ds_addr[7];
    } else {
      sets[MAC0] = 0xDE;
      sets[MAC1] = 0xAD;
      sets[MAC2] = 0xBE;
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
    sets[SPEED0] = 38400 >> 24;
    sets[SPEED1] = 38400 >> 16;
    sets[SPEED2] = 38400 >> 8;
    sets[SPEED3] = 38400;
    sets[DATA_LEN] = '8';
    sets[DATA_PARI] = 'N';
    sets[STOP_LEN] = '1';
    sets[PASSWD0] = 0;
    sets[PASSWD1] = 0;
    sets[PASSWD2] = 0;
    sets[PASSWD3] = 0;
    sets[ROM_NC] = 0;
    ch = 0;
    for (i = 0; i < ROMLEN; i++) {
      ch += sets[i];
      if (i == ROMCRC)
        sets[i] = -ch;
      eeprom_write(i, sets[i]);
    }
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
    s->print(F("current password:"));
    passwd = s->parseInt();
    s->println();
    if (passwd != password) {
      s->println(F("bye!"));
      return;
    }
  }
  s_clean(s);
  s->print(F("New password:"));
  passwd = s->parseInt();
  s->println();
  s_clean(s);
  s->print(F("Retype new passwd:"));
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
  s->print(F(",CAL38400="));
  s->print(eeprom_read(CAL38400));
  s->print(F(",CAL57600="));
  s->print(eeprom_read(CAL57600));
  s->print(F(",CAL115200="));
  s->print(eeprom_read(CAL115200));
  s->print(F(",CAL230400="));
  s->println(eeprom_read(CAL230400));

  s->print(F("\r\nOn the Serial console("));
  disp_comset(s);
  s->println(F("), Press the 'U' key and hold...."));
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
  uint8_t i, ch;

  Stream *s;
  if (stype == S_SERIAL)
    s = &Serial;
  else
    s = &client;
  while (1) {
    s->println(F("\r\n============"));
    s->print(F("0.MAC:"));
    for (i = 0; i < 6; i++) {
      ch = eeprom_read(MAC0 + i);
      if (ch < 0x10)  s->write('0');
      s->print(ch, HEX);
    }
    s->print(F("\r\n1.DHCP:"));
    s->write(eeprom_read(IS_DHCP));
    s->print(F("\r\n2.IP:"));
    for (i = 0; i < 4; i++) {
      ch = eeprom_read(IP0 + i);
      s->print(ch);
      if (i < 3)
        s->write('.');
    }
    s->print(F("\r\n3.NETMARK:"));
    for (i = 0; i < 4; i++) {
      ch = eeprom_read(NETMARK0 + i);
      s->print(ch);
      if (i < 3)
        s->write('.');
    }
    s->print(F("\r\n4.GETWAY:"));
    for (i = 0; i < 4; i++) {
      ch = eeprom_read(GW0 + i);
      s->print(ch);
      if (i < 3)      s->write('.');
    }
    s->println(F("\r\nplease select(1-4,q):"));
    dogcount = 0;
    s_clean(s);
    if (s->readBytes(&ch, 1) != 1) {
      s->println(F("bye!"));
      return;
    }
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
        s->println(F("please input ip:"));
        save_set(IP0, s);
        break;
      case '3':
        s->println(F("please input netmark:"));
        save_set(NETMARK0, s);
        break;
      case '4':
        s->println(F("please input gateway:"));
        save_set(GW0, s);
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
  s->print(F("current speed:"));
  s->println(eeprom_read_u32(SPEED0));
  s->print(F("please input new speed:"));
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
  s->print(F("data len(5-8):"));
  if (s->readBytes(&ch, 1) == 1)
    if (ch >= '5' &&  ch <= '8') {
      eeprom_write(DATA_LEN, ch);
      set_rom_check();
    }
  delay(100);
  s_clean(s);
  s->print(F("\r\nParity(N,O,E):"));
  s->readBytes(&ch, 1);
  if (ch == 'o') ch = 'O';
  if (ch == 'e') ch = 'E';
  if (ch == 'n') ch = 'N';
  if (ch == 'O' || ch == 'E' || ch == 'N') {
    eeprom_write(DATA_PARI, ch);
    set_rom_check();
  }
  s_clean(s);
  s->print(F("\r\nstop len(1,2):"));
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
