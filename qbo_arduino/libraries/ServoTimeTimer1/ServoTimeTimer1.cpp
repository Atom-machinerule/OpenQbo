#include <ServoTimeTimer1.h>
#include <avr/interrupt.h>
#include <wiring.h>

/*
  ServoTimeTimer1 - Hardware serial library based on ServoTimer1 by Jim Studt, jim@federated.com
  Author: Thomas Ouellet Fredericks, tof@danslchamp.org
  Copyright (c) 2007 David A. Mellis.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


uint8_t ServoTimeTimer1::attached9 = 0;
uint8_t ServoTimeTimer1::attached10 = 0;

void ServoTimeTimer1::seizeTimer1()
{
    uint8_t oldSREG = SREG;

    cli();
    TCCR1A = _BV(WGM11); /* Fast PWM, ICR1 is top */
    TCCR1B = _BV(WGM13) | _BV(WGM12) /* Fast PWM, ICR1 is top */
	| _BV(CS11) /* div 8 clock prescaler */
	;
    OCR1A = 3000;
    OCR1B = 3000;
    ICR1 = clockCyclesPerMicrosecond()*(20000L/8);  // 20000 uS is a bit fast for the refresh, 20ms, but 
                                                    // it keeps us from overflowing ICR1 at 20MHz clocks
                                                    // That "/8" at the end is the prescaler.
#if defined(__AVR_ATmega168__) 
    // In Jim Studt's (jim@federated.com) code the next line was
    // TIMSK0 &= ~(_BV(TICIE1) | _BV(OCIE1A) | _BV(OCIE1B) | _BV(TOIE1) );
    // But it did not seem to work and was changed following a suggestion
    // by Terry Thrift (terry@thrift.com)
    TIMSK1 &= ~( _BV(ICIE1) | _BV(OCIE1A) | _BV(OCIE1B) | _BV(TOIE1) );
#else
    TIMSK1 &= ~( _BV(ICIE1) | _BV(OCIE1A) | _BV(OCIE1B) | _BV(TOIE1) );
#endif
    SREG = oldSREG;  // undo cli()    
}

void ServoTimeTimer1::releaseTimer1()
{;}


#define NO_ANGLE 1500

ServoTimeTimer1::ServoTimeTimer1() : pin(0),angle(NO_ANGLE)
{}


uint8_t ServoTimeTimer1::attach(int pinArg)
{
    if ( pinArg != 9 && pinArg != 10) return 0;

    pin = pinArg;
    angle = NO_ANGLE;
    digitalWrite(pin,0);
    pinMode(pin,OUTPUT);

    if ( !attached9 && !attached10) seizeTimer1();

    // muck with timer flags
    if ( pin == 9) {
	attached9 = 1;
	TCCR1A = TCCR1A & ~_BV(COM1A0) | _BV(COM1A1);
    } else {
	attached10 = 1;
	TCCR1A = TCCR1A & ~_BV(COM1B0) | _BV(COM1B1);
    }
    return 1;
}

void ServoTimeTimer1::detach()
{
    // muck with timer flags
    if ( pin == 9) {
	attached9 = 0;
	TCCR1A = TCCR1A & ~_BV(COM1A0) & ~_BV(COM1A1);
    } else {
	attached10 = 0;
	TCCR1A = TCCR1A & ~_BV(COM1B0) & ~_BV(COM1B1);
    }
    pinMode(pin,INPUT);

    if ( !attached9 && !attached10) releaseTimer1();
}

void ServoTimeTimer1::write(int angleArg)
{
    uint16_t p;

    if ( angleArg < 500) angleArg = 500;
    if ( angleArg > 2500) angleArg = 2500;
    angle = angleArg;

    
    p = ((angle* clockCyclesPerMicrosecond())/8); 
    if ( pin == 9) OCR1A = p;
    else OCR1B = p;
    
}

int ServoTimeTimer1::read()
{
    return angle;
}

uint8_t ServoTimeTimer1::attached()
{
    if ( pin == 9 && attached9) return 1;
    if ( pin == 10 && attached10) return 1;
    return 0;
}


