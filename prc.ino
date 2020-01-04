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
#define EEPROM_OFFSET 13
//eeprom 内存分配
float celsius;//温度

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
uint8_t goto_bootloader __attribute__ ((section (".noinit"))); //需要进入bootloader
uint8_t goto_bootloader_crc __attribute__ ((section (".noinit")));

byte ds_addr[8];
uint32_t password;
OneWire ds(DS);

enum
{
  RC_CAL,
  VOUT_SET,
  MAC0,
  MAC1,
  MAC2,
  MAC3,
  MAC4,
  MAC5,
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
  SPEED,
  COMSET_H,
  COMSET_L,
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
  digitalWrite(_24V_OUT, eeprom_read(VOUT_SET));
  password = 0;
  password += eeprom_read(PASSWD3);
  password = (password << 8) | eeprom_read(PASSWD2);
  password = (password << 8) | eeprom_read(PASSWD1);
  password = (password << 8) | eeprom_read(PASSWD0);
  OSCCAL = eeprom_read(RC_CAL);
  for (uint8_t i = 0; i < 6; i++)
    mac[i] = eeprom_read(MAC0 + i);
  IPAddress ip(eeprom_read(IP0), eeprom_read(IP1), eeprom_read(IP2), eeprom_read(IP3));
  IPAddress gateway(eeprom_read(GW0), eeprom_read(GW1), eeprom_read(GW2), eeprom_read(GW3));
  IPAddress subnet(eeprom_read(NETMARK0), eeprom_read(NETMARK1), eeprom_read(NETMARK2), eeprom_read(NETMARK3));
  Ethernet.begin(mac, ip, gateway, subnet);
  server.begin();
  Serial.begin(115200);
  while (!Serial) ;
  setup_watchdog(WDTO_30MS);
}
boolean alreadyConnected = false;
EthernetClient client;
uint8_t add_count = 0;
uint32_t timer1 = 0; //秒
void loop() {
  dogcount = 0;
  if (!alreadyConnected) {
    client = server.available();
    if (client) {
      alreadyConnected = true;
      delay(100);
      while (client.available()) client.read();
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
    menu(S_SERIAL);
  }
}

void com_shell() {
  char ch, chlen = 0, chs[250];

  if (!client.connected()) return;
  while (Serial.available()) Serial.read();
  while (client.available()) client.read();
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
  uint32_t passwd;
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
  s->print(F("\r\npasswd:"));
  s->setTimeout(10000);
  passwd = s->parseInt();
  s->println();
  if (client)
    while (client.available()) client.read();
  while (Serial.available()) Serial.read();

  if (passwd != password) return;
  while (1) {
    while (s->available()) s->read();
    s->print(F("\r\n============Vout:"));
    if (digitalRead(_24V_OUT) == HIGH) s->println(F("On"));
    else s->println(F("Off"));
    if (stype != S_SERIAL)
      s->println(F("0-com shell"));
    s->println(F("1-reset (300ms)\r\n"
                 "2-powerdown(5s)\r\n"
                 "3-Vout off\r\n"
                 "4-Vout on\r\n"
                 "5-setpasswd\r\n"
                 "6-com speed calibration\r\n"
                 "7-network info &  modi\r\n"
                 "8-restore default sets\r\n"
                 "9-reboot\r\n"
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
        pc_power_on = 5000;
        break;
      case '3':
        s->write(ch);
        digitalWrite(_24V_OUT, LOW);
        eeprom_write(VOUT_SET, LOW);
        set_rom_check();
        break;
      case '4':
        s->write(ch);
        digitalWrite(_24V_OUT, HIGH);
        eeprom_write(VOUT_SET, HIGH);
        set_rom_check();
        break;
      case '5':
        set_passwd(s);
        break;
      case '6':
        rc_calibration(stype);
        break;
      case '7':
        info(stype);
        break;
      case '8':
        eeprom_write(ROMCRC, eeprom_read(ROMCRC) + 1);
      case '9':
        dogcount = 99999;
        while (1) ;
        break;
      case 'q':
      case 'Q':
        while (s->available()) s->read();
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

  // the first ROM byte indicates which chip
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
  // we might do a ds.depower() here, but the reset will take care of it.

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
  for (i = 0; i < ROMLEN; i++)
    ch += eeprom_read(i);
  if (ch != 0) {
    Serial.println(F("sets error!"));
    //set default
    sets[RC_CAL] = osc + 5;
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
    sets[SPEED] = 0;
    sets[COMSET_H] = 0x08;
    sets[COMSET_L] = 1;
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
  uint32_t passwd;
  while (s->available()) s->read();
  s->print(F("current password:"));
  passwd = s->parseInt();
  s->println();
  if (passwd != password) {
    s->println(F("bye!"));
    return;
  }
  while (s->available()) s->read();
  s->print(F("New password:"));
  passwd = s->parseInt();
  s->println();
  while (s->available()) s->read();
  s->print(F("Retype new passwd:"));
  if (passwd != s->parseInt()) {
    s->println(F("\r\nerror"));
    return;
  }
  s->println();
  eeprom_write(PASSWD0, (uint8_t)(passwd & 0xff));
  eeprom_write(PASSWD1, (uint8_t)((passwd >> 8) & 0xff));
  eeprom_write(PASSWD2, (uint8_t)((passwd >> 16) & 0xff));
  eeprom_write(PASSWD3, (uint8_t)((passwd >> 24) & 0xff));

  set_rom_check();
}

//校准rc振荡器
bool rc_calibration(uint8_t stype) {
  uint8_t osc0 = OSCCAL;
  int8_t i, i0;
  uint8_t ch;
  if (stype == S_SERIAL) {
    Serial.println(F("\r\nOn the serial console, Press the 'U' key and hold...."));
    Serial.flush();
  } else {
    client.println(F("\r\nOn the serial console, Press the 'U' key and hold...."));
    client.flush();
  }
  while (Serial.available()) Serial.read();
  while (Serial.available() < 10) ;
  while (Serial.available()) Serial.read();
  uint8_t b = 0;
  while (b == 0) {
    for (i = -20; i < 20; i++) {
      OSCCAL = osc + i;
      while (Serial.available()) Serial.read();
      Serial.readBytes(&ch, 1);
      if (ch == 'U' || ch == 'u') {
        OSCCAL = osc0;
        i0 = i;
        b = 1;
        break;
      }
    }
  }
  while (b == 1) {
    for (i = i0; i < 20; i++) {
      OSCCAL = osc + i;

      while (Serial.available()) Serial.read();
      Serial.readBytes(&ch, 1);
      if (ch != 'u' &&  ch != 'U') {
        b = 2;
        ch = (i - i0) / 2;
        OSCCAL = osc + ch;
        eeprom_write(RC_CAL, osc + ch);
        set_rom_check();
        Serial.print(F("ok! calibration="));
        Serial.println(OSCCAL);
        if (stype != S_SERIAL) {
          client.print(F("ok! calibration="));
          client.println(OSCCAL);
        }
        while (1) {
          delay(1000);
          if (!Serial.available()) break;
          while (Serial.available()) Serial.read();
        }
        return true;
      }

    }
  }
  OSCCAL = osc0;
  return false;
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
    s->print(F("1.MAC:"));
    for (i = 0; i < 6; i++) {
      ch = eeprom_read(MAC0 + i);
      if (ch < 0x10)  s->write('0');
      s->print(ch, HEX);
    }
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
    if (s->readBytes(&ch, 1) != 1) {
      s->println(F("bye!"));
      return;
    }
    switch (ch) {
      case '1':
        s->println(F("todo..."));
        while (s->available()) s->read();
        s->read();
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
