/****************************************************************************/
//  Function:       Header file for TroykaDHT
//  Hardware:       DHT11, DHT21, DHT22
//  Arduino IDE:    Arduino-1.8.2
//  Author:         Igor Dementiev
//  Date:           Feb 22,2018
//  Version:        v1.0
//  by www.amperka.ru
/****************************************************************************/

#ifndef __TROYKA_DHT_H__
#define __TROYKA_DHT_H__

#include <GpioPinMacros.h>

#define DHT11 11
#define DHT21 21
#define DHT22 22

// DHT11: temperature value is [0, 50]�C, accurracy is �2�C
// DHT21: temperature value is [-40, 80]�C, accurracy is �0.5�C
// DHT22: temperature value is [-40, 125]�C, accurracy is �0.2�C
// DHT11: humidity value is [20, 80]%, accurracy �5%
// DHT21: humidity value is [0, 100]%, accurracy �3%
// DHT22: humidity value is [0, 100]%, accurracy �2%

#define DHT_OK                   0
#define DHT_ERROR_CHECKSUM      -1
#define DHT_ERROR_TIMEOUT       -2
#define DHT_ERROR_NO_REPLY      -3

#define CELSIUS_TO_KELVIN   273.15


class DHT {
public:
    explicit DHT(uint8_t type);

    void begin(GpioPinVariable pinVariable);

    int8_t read();

    int8_t getState() const { return _state; }

    float getTemperatureC() const { return _temperatureC; }

    float getTemperatureF() const { return _temperatureF; }

    float getTemperatureK() const { return _temperatureK; }

    float getHumidity() const { return _humidity; }

private:
    unsigned long pulseInLength(bool state, unsigned long timeout = 0xFF);

    int8_t _type;
    int8_t _state;
    float _temperatureC;
    float _temperatureF;
    float _temperatureK;
    float _humidity;
    GpioPinVariable pin;
};

#endif  // __TROYKA_DHT_H__