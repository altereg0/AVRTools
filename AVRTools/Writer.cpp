/*
    Writer.cpp - a base class for writing data.
    For AVR ATMega328p (Arduino Uno) and ATMega2560 (Arduino Mega).
    This is part of the AVRTools library.
    Copyright (c) 2014 Igor Mikolic-Torreira.  All right reserved.
    Functions printNumber() and printFloat() adapted from Arduino code that
    is Copyright (c) 2008 David A. Mellis and licensed under LGPL.

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

#include "Writer.h"

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include <avr/pgmspace.h>

size_t Writer::write(char c) {
  // Default does nothing
  return 0;
}

size_t Writer::write(const char *str) {
  // Default does nothing
  return 0;
}

size_t Writer::write(const char *buffer, size_t size) {
  // Default does nothing
  return 0;
}

size_t Writer::write(const uint8_t *buffer, size_t size) {
  // Default does nothing
  return 0;
}

void Writer::flush() {
  // Default does nothing
}

size_t Writer::print(const __FlashStringHelper *ifsh)
{
  PGM_P p = reinterpret_cast<PGM_P>(ifsh);
  size_t n = 0;
  while (1) {
    unsigned char c = pgm_read_byte(p++);
    if (c == 0) break;
    if (write(c)) n++;
    else break;
  }
  return n;
}

size_t Writer::print(const char *str, bool addLn) {
  size_t n = write(str);
  if (addLn) {
    n += println();
  }
  return n;
}

size_t Writer::print(const uint8_t *buf, size_t size, bool addLn) {
  size_t n = write(buf, size);
  if (addLn) {
    n += println();
  }
  return n;
}

size_t Writer::print(char c, bool addLn) {
  uint8_t n = write(c);
  if (addLn) {
    n += println();
  }
  return n;
}

size_t Writer::print(int8_t n, int base, bool addLn) {
  uint8_t tmp = 0;
  if (base == kDec && n < 0) {
    tmp += print('-');
    n = -n;
  }

  tmp += printNumber(static_cast<uint8_t>(n), base); // cast essential to correctly display 2's complement negatives
  if (addLn) {
    tmp += println();
  }
  return tmp;
}

size_t Writer::print(int n, int base, bool addLn) {
  uint8_t tmp = 0;
  if (base == kDec && n < 0) {
    tmp += print('-');
    n = -n;
  }

  tmp +=
      printNumber(static_cast<unsigned int>(n), base); // cast essential to correctly display 2's complement negatives
  if (addLn) {
    tmp += println();
  }
  return tmp;
}

size_t Writer::print(long n, int base, bool addLn) {
  uint8_t tmp = 0;
  if (base == kDec && n < 0) {
    tmp += print('-');
    n = -n;
  }

  tmp += printNumber(static_cast<unsigned long>(n), base);
  if (addLn) {
    tmp += println();
  }
  return tmp;
}

size_t Writer::print(unsigned long n, int base, bool addLn) {
  uint8_t tmp = 0;

  tmp = printNumber(n, base);
  if (addLn) {
    tmp += println();
  }
  return tmp;
}

size_t Writer::print(double d, int digits, bool addLn) {
  uint8_t n = printFloat(d, digits);
  if (addLn) {
    n += println();
  }
  return n;
}

size_t Writer::println(const __FlashStringHelper *ifsh)
{
  size_t n = print(ifsh);
  n += println();
  return n;
}

size_t Writer::println() {
  uint8_t n = write(SERIAL_OUTPUT_EOL);

  return n;
}

size_t Writer::printNumber(unsigned long n, uint8_t base) {
  char buf[8 * sizeof(long) + 1 + 2]; // Assumes 8-bit chars plus zero byte plus optional 2-char base designator
  char *str = &buf[sizeof(buf) - 1];

  *str = 0;

  // prevent crash if called with base == 1
  if (base < 2) {
    base = 10;
  }

  do {
    unsigned long m = n;
    n /= base;
    char c = m - base * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while (n);

  switch (base) {
  case kBin:*--str = 'b';
    *--str         = '0';
    break;

  case kOct:*--str = 'o'; // Slightly non-standard, but more visible, way to represent octal numbers
    *--str         = '0';
    break;

  case kDec:break;

  case kHex:*--str = 'x';
    *--str         = '0';
    break;

  default:*--str = '?'; // Used to indicate a non-decimal, non-standard base
    *--str       = '0';
  }

  return write(str);
}

namespace {
const PROGMEM char sNan[] = "Nan";
const PROGMEM char sInf[] = "Inf";
const PROGMEM char sOvf[] = "Ovf";
} // namespace

size_t Writer::printFloat(double number, uint8_t digits) {
  size_t n = 0;

  if (isnan(number)) {
    char tmp[4];
    strncpy_P(tmp, sNan, 3);
    // Ensure null-terminated
    tmp[3] = 0;

    return print(tmp);
  }
  if (isinf(number)) {
    char tmp[4];
    strncpy_P(tmp, sInf, 3);
    // Ensure null-terminated
    tmp[3] = 0;

    return print(tmp);
  }
  if (number > 4294967040.0 || number < -4294967040.0) // constants determined empirically
  {
    char tmp[4];
    strncpy_P(tmp, sOvf, 3);
    // Ensure null-terminated
    tmp[3] = 0;

    return print(tmp);
  }

  // Handle negative numbers
  if (number < 0.0) {
    n += print('-');
    number = -number;
  }

  // Round correctly so that print( 1.999, 2 ) prints as "2.00"
  double       rounding = 0.5;
  for (uint8_t i        = 0; i < digits; ++i) {
    rounding /= 10.0;
  }

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part  = static_cast<unsigned long>(number);
  double        remainder = number - static_cast<double>(int_part);
  n += print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) {
    n += print('.');
  }

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = static_cast<int>(remainder);
    n += print(toPrint);
    remainder -= toPrint;
  }

  return n;
}
