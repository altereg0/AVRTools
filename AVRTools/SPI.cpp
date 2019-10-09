/*
    SPI.cpp - an interface to the SPI subsystem of the
    AVR ATMega328p (Arduino Uno) and ATMega2560 (Arduino Mega).
    This is part of the AVRTools library.
    Copyright (c) 2015 Igor Mikolic-Torreira.  All right reserved.
    Various portions of this code adapted from Arduino SPI code that
    is Copyright (c) 2010 by Cristian Maglie, Copyright (c) 2014 by Paul Stoffregen,
    Copyright (c) 2014 by Matthijs Kooijman, and Copyright (c) 2014 by Andrew J. Kroll
    and licensed under the terms of either the GNU General Public License version 2
    or the GNU Lesser General Public License version 2.1.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "SPI.h"

#include <stdint.h>
#include <stddef.h>

#include <avr/io.h>
#include <util/atomic.h>

#include "ArduinoPins.h"

void SPI::enable() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    //enable power
#if defined(__AVR_ATmega328P__)
    PRR &= ~(1 << PRSPI);
#elif defined(__AVR_ATmega2560__)
    PRR0 &= ~(1 << PRSPI);
#endif
    // Set SS to high so a connected chip will be "deselected" by default
    // If the SS pin is not already configured as an output
    // then set it high (to enable the internal pull-up resistor)
    if (isGpioPinModeInput(pSS)) {
      setGpioPinHigh(pSS);
    }

    // When the SS pin is set as OUTPUT, it can be used as
    // a general purpose output port (it doesn't influence
    // SPI operations).
    setGpioPinModeOutput(pSS);

    // Warning: if the SS pin ever becomes a LOW INPUT then SPI
    // automatically switches to Slave, so the data direction of
    // the SS pin MUST be kept as OUTPUT.
    SPCR |= _BV(MSTR);
    SPCR |= _BV(SPE);

    // Set direction register for SCK and MOSI pin.
    // MISO pin automatically overrides to INPUT.
    // By doing this AFTER enabling SPI, we avoid accidentally
    // clocking in a single bit since the lines go directly
    // from "input" to SPI control.
    // http://code.google.com/p/arduino/issues/detail?id=888
    setGpioPinModeOutput(pSCK);
    setGpioPinModeOutput(pMOSI);

    // Set a known configuration
    SPI::configure(SPISettings());
  }
}

void SPI::disable() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    SPCR &= ~_BV(SPE);
    //disable power
#if defined(__AVR_ATmega328P__)
    PRR |= (1 << PRSPI);
#elif defined(__AVR_ATmega2560__)
    PRR0 |= (1 << PRSPI);
#endif
  }
}

uint32_t SPI::transmit32(uint32_t data) {
  union {
    uint32_t val;
    struct _anon_type {
      uint8_t b0;
      uint8_t b1;
      uint8_t b2;
      uint8_t b3;
    }        _anon;
  } in, out;

  in.val = data;

  if (SPCR & _BV(DORD)) {
    SPDR = in._anon.b0;
    asm volatile("nop"); // See transmit( uint8_t ) function
    while (!(SPSR & _BV(SPIF)));
    out._anon.b0 = SPDR;

    SPDR = in._anon.b1;
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF)));
    out._anon.b1 = SPDR;

    SPDR = in._anon.b2;
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF)));
    out._anon.b2 = SPDR;

    SPDR = in._anon.b3;
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF)));
    out._anon.b3 = SPDR;
  } else {
    SPDR = in._anon.b3;
    asm volatile("nop"); // See transmit( uint8_t ) function
    while (!(SPSR & _BV(SPIF)));
    out._anon.b3 = SPDR;

    SPDR = in._anon.b2;
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF)));
    out._anon.b2 = SPDR;

    SPDR = in._anon.b1;
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF)));
    out._anon.b1 = SPDR;

    SPDR = in._anon.b0;
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF)));
    out._anon.b0 = SPDR;
  }

  return out.val;
}
