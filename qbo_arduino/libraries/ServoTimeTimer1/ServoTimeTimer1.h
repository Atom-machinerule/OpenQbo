#ifndef SERVOTIMETIMER1_IS_IN
#define SERVOTIMETIMER1_IS_IN

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

#include <inttypes.h>

class ServoTimeTimer1
{
  private:
    uint8_t pin;
    uint16_t angle;       // in microseconds
    static void seizeTimer1();
    static void releaseTimer1();
    static uint8_t attached9;
    static uint8_t attached10;
  public:
    ServoTimeTimer1();
    uint8_t attach(int);     // attach to a pin, sets pinMode, returns 0 on failure, won't
                             // position the servo until a subsequent write() happens
                             // Only works for 9 and 10.
    void detach();
    void write(int);         // specify the angle in microseconds ( 500 to 2500)
    int read(); 			 // returns the angle in microseconds ( 500 to 2500)
    uint8_t attached();
};

#endif
