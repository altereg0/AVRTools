/*
* TinyDHT.cpp
*
* Created: 18.04.2018 7:03:47
* Author: alter
*/
/* TinyDHT library
Integer version of the Adafruit DHT library for the
Trinket and Gemma mini microcontrollers
MIT license
written by Adafruit Industries
*/
#include <stddef.h>
#include <inttypes.h>
#include "main.h"
#include "TinyDHT.h"

DHT::DHT(uint8_t type, uint8_t count) {
	_type = type;
	_count = count;
	firstreading = true;
}

void DHT::begin(void) {
	// set up the pins!
	ampin_set_dir_input();
	ampin_set_level_high();
	_lastreadtime = 0;
}

//boolean S == Scale.  True == Farenheit; False == Celcius
uint16_t DHT::readTemperature(bool S) {
	uint16_t f;

	if (read()) {
		switch (_type) {
			case DHT11:
			f = (uint16_t) data[2];
			if(S)
			f = convertCtoF(f);
			return f;
			case DHT22:
			case DHT21:
			f = (uint16_t)(data[2] & 0x7F);
			f *= 256;
			f += (uint16_t) data[3];
			f /= 10;
			if (data[2] & 0x80)
			f *= -1;
			if(S)
			f = convertCtoF(f);

			return f;
		}
	}
	/* Serial.print("Read fail"); */
	return BAD_TEMP; // Bad read, return value (from TinyDHT.h)
}

uint16_t DHT::convertCtoF(uint16_t c) {
	return (c * 9) / 5 + 32;
}

uint8_t DHT::readHumidity(void) {  //  0-100 %
	uint8_t f;
	uint16_t f2;  // bigger to allow for math operations
	if (read()) {
		switch (_type) {
			case DHT11:
			f = data[0];
			return f;
			case DHT22:
			case DHT21:
			f2 = (uint16_t) data[0];
			f2 *= 256;
			f2 += data[1];
			f2 /= 10;
			f = (uint8_t) f2;
			return f;
		}
	}
	/* Serial.print("Read fail"); */
	return BAD_HUM; // return bad value (defined in TinyDHT.h)
}


uint8_t * DHT::d(void)
{
	return data;
}

bool DHT::read(void) {
	uint8_t laststate = true;
	uint8_t counter = 0;
	uint8_t j = 0, i;
	uint32_t currenttime;

	currenttime = mls();
	//currenttime = 0;
	
	if (currenttime < _lastreadtime) {
		// ie there was a rollover
		_lastreadtime = 0;
	}
	if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
		return true; // return last correct measurement
		//delay(2000 - (currenttime - _lastreadtime));
	}
	firstreading = false;

	_lastreadtime = mls();
	//_lastreadtime = 0;

	data[0] = data[1] = data[2] = data[3] = data[4] = 0;
	
	// pull the pin high and wait 250 milliseconds
	ampin_set_level_high();
	user_delay_ms_250();

	// now pull it low for ~20 milliseconds
	ampin_set_dir_output();
	ampin_set_level_low();
	user_delay_ms_20();
	
	disable_gi();
	ampin_set_level_high();
	user_delay_us_40();
	ampin_set_dir_input();

	// read in timings
	for ( i=0; i< MAXTIMINGS; i++) {
		counter = 0;
		while (ampin_get_current_level() == laststate) {
			counter++;
			user_delay_us_1();
			if (counter == 255) {
				break;
			}
		}
		laststate = ampin_get_current_level();

		if (counter == 255) break;

		// ignore first 3 transitions
		if ((i >= 4) && (i%2 == 0)) {
			// shove each bit into the storage bytes
			data[j/8] <<= 1;
			if (counter > _count)
			{
				data[j/8] |= 1;
			}
			j++;
		}

	}

	enable_gi();

	// check we read 40 bits and that the checksum matches
	if ((j >= 40) &&
	(data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) ) {
		return true;
	}
	
	return false;

}
