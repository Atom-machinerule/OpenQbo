/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, TheCorpora.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TheCorpora nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.

 *
 * \author Miguel Angel Julian
 *********************************************************************/
 
#include <IRbeanSerial.h>  //Para leer los datos de los infrarojos

#include <I2C.h>
#include <LCDi2cR.h>
#include "LIS35DE.h"
#include "L3G4200D.h"

//---------------includes de las librerias usadas--------------//
#include "ArduBot.h"
#include "serialProtocol.h"  //Cambiar a baseSerialProtocol
//-------------------------------------------------------------//
arduBot::ArduBot QBO(0.102,0.2736,1440);
arduBotSerial::SerialProtocol serialProtocol(&QBO);
//----------Inicializacion del sistema----------------------//
void setup()
{
  QBO.begin(0.005, 9.0, 1.0, 0.2); //Ts,kp,kd,ki
  //QBO.begin(0.005, 9.0, 0.0, 0.0); //Ts,kp,kd,ki
  //Serial.begin(9600);
  //bitSet(PORTE,PE2);
  //bitClear(PORTG,PG3);
  
  //bitSet(PORTG,PG4);
  //bitClear(PORTH,PH2);
  
  //bitClear(PORTE,PE2);
  //bitSet(PORTG,PG3);
  
  /*
  bitClear(PORTG,PG4);
  bitSet(PORTH,PH2);
  */
}
//----------------------------------------------------------//
//----------------------Loop principal----------------------//
void loop()
{
  
  serialProtocol.processSerial();
  QBO.spinOnce();
  //QBO.setSpeeds(0.0, 2.0);
  //QBO.setSpeeds(0.05, 0.0);
  //analogWrite(7,0);
  //analogWrite(8,0);
}
//----------------------------------------------------------//
