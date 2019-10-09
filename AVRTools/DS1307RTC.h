/*
 * DS1307RTC.h - library for DS1307 RTC
 * This library is intended to be uses with Arduino Time library functions
 */

/*
 * DS1307 register map
 *
 *  00h-06h: seconds, minutes, hours, day-of-week, date, month, year (all in
 * BCD) bit 7 of seconds enables/disables clock bit 6 of hours toggles 12/24h
 * mode (1 for 12h, 0 for 24h) when 12h mode is selected bit 5 is high for PM,
 * low for AM 07h: control bit7: OUT bit6: 0 bit5: 0 bit4: SQWE bit3: 0 bit2: 0
 *      bit1: RS0
 *      bit0: RS1
 *  08h-3fh: 56 bytes of SRAM
 *
 * DS3231 register map
 *
 *  00h-06h: seconds, minutes, hours, day-of-week, date, month, year (all in
 * BCD) bit 7 should be set to zero: The DS3231 clock is always running 07h:
 * A1M1  Alarm 1 seconds 08h: A1M2  Alarm 1 minutes 09h: A1M3  Alarm 1 hour
 * (bit6 is am/pm flag in 12h mode) 0ah: A1M4  Alarm 1 day/date (bit6: 1 for
 * day, 0 for date) 0bh: A2M2  Alarm 2 minutes 0ch: A2M3  Alarm 2 hour (bit6 is
 * am/pm flag in 12h mode) 0dh: A2M4  Alarm 2 day/data (bit6: 1 for day, 0 for
 * date) <see data sheet page12 for Alarm register mask bit tables: for alarm
 * when hours, minutes and seconds match set 1000 for alarm 1> 0eh: control
 *      bit7: !EOSC
 *      bit6: BBSQW
 *      bit5: CONV
 *      bit4: RS2
 *      bit3: RS1
 *      bit2: INTCN
 *      bit1: A2IE
 *      bit0: A1IE
 *  0fh: control/status
 *      bit7: OSF
 *      bit6: 0
 *      bit5: 0
 *      bit4: 0
 *      bit3: EN32kHz
 *      bit2: BSY
 *      bit1: A2F alarm 2 flag
 *      bit0: A1F alarm 1 flag
 * 10h: aging offset (signed)
 * 11h: MSB of temp (signed)
 * 12h: LSB of temp in bits 7 and 6 (0.25 degrees for each 00, 01, 10, 11)
 *
 */


#ifndef DS1307RTC_h
#define DS1307RTC_h
#include <stdint.h>
#include <time.h>

#define TIME_STX 0x02  // Header tag for start serial time sync message
#define TIME_ETX 0x03  // Header tag for end serial time sync message
#define TIME_REQUEST 0x07 // ASCII bell character requests a time sync message
#define DS1307_SYNC_TIMEOUT 5000 //sync timeout

#ifndef DS1307_TIMEZONE
#define DS1307_TIMEZONE 3 * ONE_HOUR
#endif

//I2C Slave Address
const uint8_t DS1307_ADDRESS = 0x68;

//DS1307 Register Addresses
const uint8_t DS1307_REG_TIMEDATE = 0x00;
const uint8_t DS1307_REG_STATUS   = 0x00;
const uint8_t DS1307_REG_CONTROL  = 0x07;
const uint8_t DS1307_REG_RAMSTART = 0x08;
const uint8_t DS1307_REG_RAMEND   = 0x3f;
const uint8_t DS1307_REG_RAMSIZE  = DS1307_REG_RAMEND - DS1307_REG_RAMSTART;

//DS1307 Register Data Size if not just 1
const uint8_t DS1307_REG_TIMEDATE_SIZE = 7;

// DS1307 Control Register Bits
const uint8_t DS1307_RS0  = 0;
const uint8_t DS1307_RS1  = 1;
const uint8_t DS1307_SQWE = 4;
const uint8_t DS1307_OUT  = 7;

// DS1307 Status Register Bits
const uint8_t DS1307_CH = 7;

enum DS1307SquareWaveOut {
  DS1307SquareWaveOut_1Hz   = (1 << DS1307_SQWE),//0b00010000,
  DS1307SquareWaveOut_4kHz  = (1 << DS1307_SQWE) | (1 << DS1307_RS0),//0b00010001,
  DS1307SquareWaveOut_8kHz  = (1 << DS1307_SQWE) | (1 << DS1307_RS1),//0b00010010,
  DS1307SquareWaveOut_32kHz = (1 << DS1307_SQWE) | (1 << DS1307_RS0) | (1 << DS1307_RS1),//0b00010011,
  DS1307SquareWaveOut_High  = (1 << DS1307_OUT),//0b10000000,
  DS1307SquareWaveOut_Low   = (0 << DS1307_OUT),//0b00000000,
};

const bool s_is_ds3231 = false;

class DS1307RTC {
public:
  //синглтон Майерса
//  static DS1307RTC *instance() {
//    static DS1307RTC inst;
//    return &inst;
//  }
  bool read();
  bool write();
  bool fail() { return halted; };
  void sync();
  const char *chartime();
  const time_t timer(){return rtcTime;};

  DS1307RTC()= default;                                  // Private constructor
private:
//  ~DS1307RTC() = default;
//  DS1307RTC(const DS1307RTC &);                 // Prevent copy-construction
//  DS1307RTC &operator=(const DS1307RTC &);      // Prevent assignment
  uint8_t dec2bcd(uint8_t d);
  uint8_t bcd2dec(uint8_t b);
  bool   halted;
  time_t rtcTime;
//  char buffer[80];
};

DS1307RTC &rtc(); //singleton via @static local variable

#endif
 

