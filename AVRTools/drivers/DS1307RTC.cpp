#include "I2cMaster.h"
#include "DS1307RTC.h"
#include "USART0Minimal.h"
#include "SystemClock.h"

// Aquire data from the RTC chip in BCD format and return ERROR
bool DS1307RTC::read() {
  tm      timeinfo;
  uint8_t rtc[7];
  uint8_t century = 0;
  uint8_t age     = 100;
  uint8_t error;

  I2cMaster::start(I2cMaster::kI2cBusFast); //The DS1307 operates in the standard mode (100kHz) only
  error = I2cMaster::readSync(DS1307_ADDRESS, DS1307_REG_TIMEDATE, 7, rtc);
  I2cMaster::stop();

  if (error) {
    return true;
  };
  // Clear clock halt bit from read data
  // This starts the clock for a DS1307, and has no effect for a DS3231
  timeinfo.tm_sec  = bcd2dec(rtc[0] & 0x7f);
  timeinfo.tm_min  = bcd2dec(rtc[1]);
  timeinfo.tm_hour = bcd2dec(rtc[2]);
  timeinfo.tm_wday = bcd2dec(rtc[3]); // returns 1-7
  timeinfo.tm_mday = bcd2dec(rtc[4]);
  timeinfo.tm_mon  = bcd2dec(rtc[5] & 0x1F); // returns 1-12
  if (s_is_ds3231) {
    century = (rtc[5] & 0x80) >> 7; // century only on ds3231
    age     = century == 1 ? 100 : 0;
  }
  timeinfo.tm_year = age + bcd2dec(rtc[6]); // year 0-99 //years since 1900
  halted  = ((rtc[0] & 0x80) != 0x00);// clock is halted = 8bit is CLOCK HALTED
  rtcTime = mk_gmtime(&timeinfo);
  return false;
}

bool DS1307RTC::write() {
  // To eliminate any potential race conditions,
  // stop the clock before writing the values,
  // then restart it after.
  tm *timeinfo = gmtime(&rtcTime);
  uint8_t rtc[8];
  uint8_t century = 0; // only on ds3231
  uint8_t age     = 100;
  uint8_t year;
  if (s_is_ds3231) {
    if (timeinfo->tm_year > 100) {
      century = 0x80;
    } else {
      age = 0;
    }
  }
  year            = timeinfo->tm_year - age;
  rtc[0] = 0x80; // Stop the clock. The seconds will be written last
  rtc[1] = dec2bcd(timeinfo->tm_min);
  rtc[2] = dec2bcd(timeinfo->tm_hour);      // sets 24 hour format
  rtc[3] = dec2bcd(timeinfo->tm_wday);
  rtc[4] = dec2bcd(timeinfo->tm_mday);
  rtc[5] = dec2bcd(timeinfo->tm_mon) | century;
  rtc[6] = dec2bcd(year);
  I2cMaster::start(I2cMaster::kI2cBusFast);
  I2cMaster::writeSync(DS1307_ADDRESS, DS1307_REG_TIMEDATE, rtc, 7);
//  I2cMaster::stop();
//  I2cMaster::start(I2cMaster::kI2cBusSlow);
  I2cMaster::writeSync(DS1307_ADDRESS, DS1307_REG_TIMEDATE, timeinfo->tm_sec); // start seconds;
  I2cMaster::stop();

  return true;
}

// PRIVATE FUNCTIONS

// Convert Decimal to Binary Coded Decimal (BCD)
uint8_t DS1307RTC::dec2bcd(uint8_t d) {
//  return ((num/10 * 16) + (num % 10));
  return d + 6 * (d / 10);

}

// Convert Binary Coded Decimal (BCD) to Decimal
uint8_t DS1307RTC::bcd2dec(uint8_t b) {
  //return ((num/16 * 10) + (num % 16));
  return b - 6 * (b >> 4);
}
void DS1307RTC::sync() {
  millis_t timeout = millis() + DS1307_SYNC_TIMEOUT;
  initUSART0(57600);
  transmitUSART0(TIME_REQUEST);
  while ((timeout > millis())&&(receiveUSART0() != TIME_STX));
  if(timeout < millis())
    return;//fail
  uint8_t      rx[sizeof(time_t)];
  for (uint8_t i = 0; i < sizeof(time_t); i++) {
    uint8_t c;
    c = receiveUSART0();
    if (c == TIME_ETX) {
      break;
    }
    rx[i] = c;
  }
  rtcTime = rx[0];
  rtcTime = (rtcTime <<8) | rx[1];
  rtcTime = (rtcTime <<8) | rx[2];
  rtcTime = (rtcTime <<8) | rx[3];

  rtcTime -= UNIX_OFFSET;
  write();
//  set_zone(DS1307_TIMEZONE);
//  set_system_time(rtcTime);
}
const char *DS1307RTC::chartime() {
//   strftime(buffer, 16, "%F%t%X",localtime(&rtcTime));
  return ctime(&rtcTime);
}

DS1307RTC &rtc() {
  static DS1307RTC ds1307;
  return ds1307;
}