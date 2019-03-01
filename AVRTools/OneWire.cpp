#include "OneWire.h"

#ifdef UART_AS_OneWire
//#include <avr/interrupt.h>
#include "USART1Minimal.h"
#endif

#ifndef UART_AS_OneWire
void ow_set(uint8_t mode) {
#ifndef OW_TWO_PINS
  if (mode) {
    setGpioPinLow(OW_ALTER_PIN); // not really necessary
    setGpioPinModeOutput(OW_ALTER_PIN);
  } else {
    setGpioPinModeInput(OW_ALTER_PIN);
  }
#else
  if (mode)
    setGpioPinLow(OW_ALTER_PIN_OUT);
  else
    setGpioPinHigh(OW_ALTER_PIN_OUT);
#endif
}

uint8_t ow_checkin(void) {
#ifndef OW_TWO_PINS
  // return CheckBit(OW_PIN, OW_BIT);
  return readGpioPinDigital(OW_ALTER_PIN);
#else
  // return CheckBit(OW_PIN, OW_BIT_IN);
  return readGpioPinDigital(OW_ALTER_PIN_IN);
#endif
}

#endif

uint8_t ow_identify(void) {
#ifdef UART_AS_OneWire
  return 1;
#else
  return 0;
#endif // _DEBUG
}

uint8_t ow_reset(void) {
#ifdef UART_AS_OneWire
  // UCSRB = (1 << RXEN) | (1 << TXEN);
  //// 9600
  // UBRRL = USART_BAUDRATE_9600;
  // UBRRH = (USART_BAUDRATE_9600 >> 8);
  // UCSRA &= ~(1 << U2X);
  //
  // while (CheckBit(UCSRA, RXC))
  // UDR; //clear buffer
  // cli();
  // UDR = 0xF0;
  // UCSRA = (1 << TXC);
  // sei();
  //// while(!CheckBit(UCSRA, TXC)) OthersTasks();
  // while (!CheckBit(UCSRA, RXC))
  // OthersTasks();
  // if (UDR != 0xF0)
  // return 1;
  // return 0;
  uint8_t c;
  initUSART1(USART_BAUDRATE_9600);
  // UCSR1A &= ~(1 << U2X1);
  transmitUSART1(0xF0);
  c = receiveUSART1();
  // initUSART1(USART_BAUDRATE_115200);
  if (c != 0xF0)
    return 1;
  return 0;

#else
  uint8_t status;
  ow_set(1);
  _delay_us(480);
  ow_set(0);
  _delay_us(60);
  // Store line value and wait until the completion of 480uS period
  status = ow_checkin();
  _delay_us(420);
  // Return the value read from the presence pulse (0=OK, 1=WRONG)
  return !status;
#endif
  //	return 1 if found
  //	return 0 if not found
}

void ow_write_bit(uint8_t bit) {
#ifdef UART_AS_OneWire
  //// 115200
  // UBRRL = USART_BAUDRATE_115200;
  // UBRRH = (USART_BAUDRATE_115200 >> 8);
  // UCSRA |= (1 << U2X);
  //
  // uint8_t d = 0x00;
  // while (CheckBit(UCSRA, RXC))
  // UDR; //clear buffer
  // if (bit)
  // d = 0xFF;
  // cli();
  // UDR = d;
  // UCSRA = (1 << TXC);
  // sei();
  // while (!CheckBit(UCSRA, TXC))
  //;
  // while (CheckBit(UCSRA, RXC))
  // UDR; //clear buffer
  uint8_t d = 0x00;
  if (bit)
    d = 0xFF;
  initUSART1(115200);
  // UCSR1A |= (1 << U2X1); // double speed ?
  transmitUSART1(d);
  // releaseUSART1();
#else
  // Pull line low for 1uS
  ow_set(1);
  _delay_us(1);
  // If we want to write 1, release the line (if not will keep low)
  if (bit)
    ow_set(0);
  // Wait for 60uS and release the line
  _delay_us(60);
  ow_set(0);
#endif
}

uint8_t ow_read_bit(void) {
#ifdef UART_AS_OneWire
  //// 115200
  // UBRRL = USART_BAUDRATE_115200;
  // UBRRH = (USART_BAUDRATE_115200 >> 8);
  // UCSRA |= (1 << U2X);
  //
  // uint8_t c;
  // while (CheckBit(UCSRA, RXC))
  // UDR; //clear buffer
  // cli();
  // UDR = 0xFF;
  // UCSRA = (1 << TXC);
  // sei();
  // while (!CheckBit(UCSRA, TXC))
  //;
  // while (!CheckBit(UCSRA, RXC))
  //;
  // c = UDR;
  // if (c > 0xFE)
  // return 1;
  // return 0;
  uint8_t d;
  initUSART1(USART_BAUDRATE_115200);
  // UCSR1A |= (1 << U2X1); // double speed?
  d = receiveUSART1();
  if (d > 0xFE)
    return 1;
  return 0;
#else
  uint8_t bit = 0;
  // Pull line low for 1uS
  ow_set(1);
  _delay_us(1);
  // Release line and wait for 14uS
  ow_set(0);
  _delay_us(14);
  // Read line value
  if (ow_checkin())
    bit = 1;
  // Wait for 45uS to end and return read value
  _delay_us(45);
  return bit;
#endif
}

#ifdef UART_AS_OneWire
uint8_t ow_write_byte(uint8_t b) {
  //// 115200
  // UBRRL = USART_BAUDRATE_115200;
  // UBRRH = (USART_BAUDRATE_115200 >> 8);
  // UCSRA |= (1 << U2X);
  //
  // do {
  // uint8_t d = 0x00;
  // if (byte & 1)
  // d = 0xFF;
  // cli();
  // UDR = d;
  // UCSRA = (1 << TXC);
  // sei();
  // OthersTasks();
  // while (!CheckBit(UCSRA, RXC))
  // OthersTasks();
  // byte >>= 1;
  // if (UDR > 0xFE)
  // byte |= 128;
  //} while (--i);
  uint8_t i = 8;
  initUSART1(USART_BAUDRATE_115200);
  // UCSR1A |= (1 << U2X1); // double speed?
  do {
    uint8_t d = 0x00;
    if (b & 1)
      d = 0xFF;
    transmitUSART1(d);
    b >>= 1;
    if (receiveUSART1() > 0xFE)
      b |= 0x80;
  } while (--i);
  return b & 0xFF;
}
#else
void ow_write_byte(uint8_t b) {
  for (uint8_t p = 8; p; p--) {
    ow_write_bit(b & 1);
    b >>= 1;
  }
}

uint8_t ow_read_byte(void) {
  uint8_t r = 0;
  for (uint8_t p = 8; p; p--) {
    r >>= 1;
    if (ow_read_bit())
      r |= 0x80;
  }
  return r;
}
#endif

uint8_t ow_search_rom(uint8_t diff, uint8_t *id) {
  uint8_t i, j, next_diff;
  uint8_t b;

  if (!ow_reset())
    return OW_PRESENCE_ERR; // error, no device found

  ow_write_byte(OW_CMD_SEARCHROM); // ROM search command
  next_diff = OW_LAST_DEVICE;      // unchanged on last device

  i = OW_ROMCODE_SIZE * 8; // 8 bytes
  do {
    j = 8; // 8 bits
    do {
      b = ow_read_bit();      // read bit
      if (ow_read_bit()) {    // read complement bit
        if (b)                // 11
          return OW_DATA_ERR; // data error
      } else {
        if (!b) { // 00 = 2 devices
          if (diff > i || ((*id & 1) && diff != i)) {
            b = 1;         // now 1
            next_diff = i; // next pass 0
          }
        }
      }
      ow_write_bit(b); // write bit
      *id >>= 1;
      if (b)
        *id |= 0x80; // store bit
      i--;
    } while (--j);
    id++; // next byte
  } while (i);
  return next_diff; // to continue search
}

void ow_find_rom(uint8_t *diff, uint8_t id[]) {
  while (1) {
    *diff = ow_search_rom(*diff, &id[0]);
    if (*diff == OW_PRESENCE_ERR || *diff == OW_DATA_ERR ||
        *diff == OW_LAST_DEVICE)
      return;
    // if ( id[0] == DS18B20_ID || id[0] == DS18S20_ID )
    return;
  }
}

uint8_t ow_read_rom(uint8_t *buffer) {
  if (!ow_reset())
    return 0;
  ow_write_byte(OW_CMD_READROM);
  for (uint8_t i = 0; i < 8; i++) {
    buffer[i] = ow_read_byte();
  }
  return 1;
}

uint8_t ow_match_rom(uint8_t *rom) {
  if (!ow_reset())
    return 0;
  ow_write_byte(OW_CMD_MATCHROM);
  for (uint8_t i = 0; i < 8; i++) {
    ow_write_byte(rom[i]);
  }
  return 1;
}
