/*
    InitSystem.cpp - Functions to initialize AVR processors.
    For AVR ATMega328p (Arduino Uno) and ATMega2560 (Arduino Mega).
    This is part of the AVRTools library.
    Copyright (c) 2014 Igor Mikolic-Torreira.  All right reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "InitSystem.h"

#include <inttypes.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

void initSystem() {
  // Clear out any left over settings from the bootloader

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // The bootloader connects pins 0 and 1 to the USART; disconnect them
    // so they are restored as normal digital i/o.
    // Reconnect them explicitly if serial i/o is desired
    UCSR0B = 0;

    // Clear Timer0
    TCCR0A = 0; // Clear all settings
    TCCR0B = 0; // Clear all settings
    TIMSK0 = 0; // Disable all interrupts

    // Clear Timer1
    TCCR1A = 0; // Clear all settings
    TCCR1B = 0; // Clear all settings
    TIMSK1 = 0; // Disable all interrupts

    // Clear Timer2
    TCCR2A = 0; // Clear all settings
    TCCR2B = 0; // Clear all settings
    TIMSK2 = 0; // Disable all interrupts

#if defined(__AVR_ATmega2560__)

    // Clear Timer3
    TCCR3A = 0; // Clear all settings
    TCCR3B = 0; // Clear all settings
    TIMSK3 = 0; // Disable all interrupts

    // Clear Timer4
    TCCR4A = 0; // Clear all settings
    TCCR4B = 0; // Clear all settings
    TIMSK4 = 0; // Disable all interrupts

    // Clear Timer5
    TCCR5A = 0; // Clear all settings
    TCCR5B = 0; // Clear all settings
    TIMSK5 = 0; // Disable all interrupts

#endif

#if defined(__AVR_ATmega2560__)
    /* On AVR devices all peripherals are enabled from power on reset, this
     * disables all peripherals to save power. Driver shall enable peripheral if used */
    //PRR0 =  PRTWI   PRTIM2  PRTIM0  –       PRTIM1  PRSPI     PRUSART0  PRADC
    //PRR1 =  –       –       PRTIM5  PRTIM4  PRTIM3  PRUSART3  PRUSART2  PRUSART1
    PRR0  = (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM0) | (1 << PRTIM1) | (1 << PRSPI) | (1 << PRUSART0) | (1 << PRADC);
    PRR1  = (1 << PRTIM5) | (1 << PRTIM4) | (1 << PRTIM3) | (1 << PRUSART3) | (1 << PRUSART2) | (1 << PRUSART1);
    /* Set all pins to input pullup mode */
    DDRA  = 0x00;
    PORTA = 0xFF;

    DDRB  = 0x00;
    PORTB = 0xFF;

    DDRC  = 0x00;
    PORTC = 0xFF;

    DDRD  = 0x00;
    PORTD = 0xFF;

    DDRE  = 0x00;
    PORTE = 0xFF;

    DDRF  = 0x00;
    PORTF = 0xFF;

    DDRG  = 0x00;
    PORTG = 0x3F; // 6-bit port

    DDRH  = 0x00;
    PORTH = 0xFF;

    DDRJ  = 0x00;
    PORTJ = 0xFF;

    DDRK  = 0x00;
    PORTK = 0xFF;

    DDRL  = 0x00;
    PORTL = 0xFF;
#endif
  }

  // Enable interrupts
  sei();
}
