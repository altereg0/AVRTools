/* 
* TinyDHT.h
*
* Created: 18.04.2018 7:03:47
* Author: alter
*/


#ifndef __TINYDHT_H__
#define __TINYDHT_H__


// how many timing transitions we need to keep track of. 2 * number bits + extra
#define MAXTIMINGS 85

#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21

// NAN code in DHT library takes space, define bad values here
#define BAD_HUM    -1  // Bad humitidy reading
#define BAD_TEMP -999  // Bad temperature reading

class DHT {
	private:
	uint8_t data[6];
	uint8_t _type, _count;
	bool read(void);
	unsigned long _lastreadtime;
	bool firstreading;

	public:
	DHT(uint8_t type, uint8_t count=6);
	void begin(void);
	uint16_t readTemperature(bool S=false);
	uint16_t convertCtoF(uint16_t);
	uint8_t readHumidity(void);
	
	uint8_t * d(void);
	

};

#endif //__TINYDHT_H__
