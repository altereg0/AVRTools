#ifndef ONEWIRE_H
#define ONEWIRE_H

#include "ArduinoPins.h"
#include "GpioPinMacros.h"

#define MAXDEVICES 4

// USART-driven 1-wire emulation
//#define UART_AS_OneWire

// 2-pins emulation
//#define OW_TWO_PINS

#ifdef UART_AS_OneWire
//#define USART_BAUDRATE_57600 (((F_CPU / (57600 * 16UL))) - 1)
#define USART_BAUDRATE_115200 115200
#define USART_BAUDRATE_9600 9600
#else
#include <util/delay.h>

//#define OW_ALTER_PIN pPin12
#ifndef OW_TWO_PINS
#define OW_ALTER_PIN pPin12
#else
#define OW_ALTER_PIN_IN pPin12
#define OW_ALTER_PIN_OUT pPin13
#endif
#endif

#define OW_CMD_SEARCHROM 0xF0
#define OW_CMD_READROM 0x33
#define OW_CMD_MATCHROM 0x55
#define OW_CMD_SKIPROM 0xCC

#define OW_SEARCH_FIRST 0xFF // start new search
#define OW_PRESENCE_ERR 0xFF
#define OW_DATA_ERR 0xFE
#define OW_LAST_DEVICE 0x00 // last device found
//			0x01 ... 0x40: continue searching

#define OW_DS1990_FAMILY_CODE 1
#define OW_DS2405_FAMILY_CODE 5
#define OW_DS2413_FAMILY_CODE 0x3A
#define OW_DS1822_FAMILY_CODE 0x22
#define OW_DS2430_FAMILY_CODE 0x14
#define OW_DS1990_FAMILY_CODE 1
#define OW_DS2431_FAMILY_CODE 0x2d
#define OW_DS18S20_FAMILY_CODE 0x10
#define OW_DS18B20_FAMILY_CODE 0x28
#define OW_DS2433_FAMILY_CODE 0x23

// rom-code size including CRC
#define OW_ROMCODE_SIZE 8

uint8_t ow_identify(void);

uint8_t ow_reset(void);
void ow_write_bit(uint8_t bit);
uint8_t ow_read_bit(void);

#ifndef UART_AS_OneWire
uint8_t ow_read_byte(void);
void ow_write_byte(uint8_t byte);
#else
uint8_t ow_write_byte(uint8_t byte);
#define ow_read_byte() ow_write_byte(0xFF)
#endif

uint8_t ow_search_rom(uint8_t diff, uint8_t *id);
void ow_find_rom(uint8_t *diff, uint8_t id[]);
uint8_t ow_read_rom(uint8_t *buffer);
uint8_t ow_match_rom(uint8_t *rom);

#endif
