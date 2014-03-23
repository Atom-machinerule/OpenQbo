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

#ifndef ArduBot_h
#define ArduBot_h

#include <inttypes.h>
#include <WProgram.h> 
//---------------includes de las librerias usadas--------------//
#include <I2C.h>
#include <LCDi2cR.h>
#include "LIS35DE.h"
#include "L3G4200D.h"
#include <IRbeanSerial.h>
//-------------------------------------------------------------//
#include "comands.h"
#include "motores.h"
//-------------------------------------------------------------//
#define MOTOR_1_CONTROL_1_PIN PE2
#define MOTOR_1_CONTROL_2_PIN PG3
#define MOTOR_2_CONTROL_2_PIN PH2
#define MOTOR_2_CONTROL_1_PIN PG4

#define MOTOR_1_COTROL_1_PORT PORTE
#define MOTOR_1_COTROL_2_PORT PORTG
#define MOTOR_2_COTROL_2_PORT PORTH
#define MOTOR_2_COTROL_1_PORT PORTG

#define MOTOR_1_COTROL_1_REGISTER DDRE
#define MOTOR_1_COTROL_2_REGISTER DDRG
#define MOTOR_2_COTROL_2_REGISTER DDRH
#define MOTOR_2_COTROL_1_REGISTER DDRG

#define MOTOR_1_PWM_ARDUINO_PIN 8 //PH5
#define MOTOR_2_PWM_ARDUINO_PIN 7 //PH4

#define MOTOR_1_HALL_A_PIN PJ2
#define MOTOR_1_HALL_B_PIN PJ3
#define MOTOR_2_HALL_A_PIN PJ4
#define MOTOR_2_HALL_B_PIN PJ5

#define MIN_FLOOR_DISTANCE_CM 11
#define MAX_FLOOR_DISTANCE_CM 35

#define FLOOR_DISTANCE_SENSOR_ARDUINO_PIN 8
#define BATTERY_LEVEL_INPUT_ARDUINO_PIN 9
#define BATTERY_LEVEL_DIGITAL_INPUT_ARDUINO_PIN 63

#define CHARGER_ARDUINO_PIN 22
//-------------------------------------------------------------//

namespace arduBot
{
  //--------------------version de la libreria-------------------//
  const uint8_t boardId=0;
  const uint8_t libraryVersion=1;
  //-------------------------------------------------------------//
  //----------defines para la comunicacion con los SRF10---------//
  const uint8_t srfCmdByte=0x00;                           // Command byte
  const uint8_t srfLightByte=0x01;                         // Byte to read light sensor
  const uint8_t srfRangeByte=0x02;                         // Byte for start of ranging data
  const uint8_t srfGainByte=0x01;                         // Byte to read light sensor
  //-------------------------------------------------------------//
  class ArduBot
  {
    private:
      byte alert_stop;
      //-------------------------------------------------------------//
      boolean isSrfUpdateContinuous;
      //-------------------------------------------------------------//
      static long spinLoopPeriodMs;
      //-------------------------------------------------------------//
      LIS35DE accelerometer;
      L3G4200D gyro;
      long gyro0;
      //-------------------------------------------------------------//
      double xCoordinate;
      double yCoordinate;
      double thetaCoordinate;
      //double xCoordinateNoGyro;
      //double yCoordinateNoGyro;
      //double thetaCoordinateNoGyro;
      void estimatePosition();
      
      //static double max_accel;
      static bool wheelStopAlertFlag;
      static bool robotFallFlag;
      static bool robotCrashFlag;
    public:
      static bool robotStallFlag;
      static CMotors leftMotor;
      static CMotors rightMotor;
      static CBaseMovement par_motores;
      LCDi2cR lcd;
      boolean lcdState;
      boolean energyState;
      boolean gyroState;
      boolean accelerometerState;
      //----variables para controlar los sensores de ultrasonido-----//
      byte NUM_SRFs;
      int SRFs_ADRESS[16];
      unsigned int SRFs_VALUES[16];
      byte NUM_SRFs_FRONT;
      int SRFs_FRONT[4];
      bool SRFs_FRONT_CRASH_FLAGS[16];
      byte NUM_SRFs_BACK;
      int SRFs_BACK[4];
      //-------------------------------------------------------------//
      //----variables para comprobar el sentido los motores----------//
      static bool leftMotorHallA;
      static bool leftMotorHallB;
      static bool rightMotorHallA;
      static bool rightMotorHallB;
      //-------------------------------------------------------------//
      //--------variables para de los sensores de infrarrojos--------//
      static byte ir1data;
      static byte ir2data;
      static byte ir3data;
      //-------------------------------------------------------------//
      //------------------variables para el IMU----------------------//
      int gyroX;
      int gyroY;
      int gyroZ;
      int8_t accelerometerX;
      int8_t accelerometerY;
      int8_t accelerometerZ;
      //-------------------------------------------------------------//
      //------------------------Constructor--------------------------//
      ArduBot(double wheelRadious=0.102, double wheelDistance=0.2736, int encoderResolution=1560);
      //-------------------------------------------------------------//
      void begin(double spinLoopPeriodS=0.005, double kp=0.0, double ki=0.0, double kd=0.0);
      //-------------------------------------------------------------//
      //--------------Funcion para posicion y velocidad--------------//
      inline void getSpacePosition(double& x, double& y, double& angle)
      {
        x=xCoordinate;
        y=yCoordinate;
        angle=thetaCoordinate;
      };
      inline void resetPosition()
      {
        xCoordinate=0;
        yCoordinate=0;
        thetaCoordinate=0;
      };
      void setSpeeds(double linealSpeed, double angularSpeed);
      void updatePosition();
      //-------------------------------------------------------------//
      //----------Funcion para leer el sensor GP2D12-----------------//
      inline float getFloorDistance()
      {
        int analogValue=analogRead(FLOOR_DISTANCE_SENSOR_ARDUINO_PIN);
       
       /* 
        if(analogValue<=3)
        {
          ArduBot::wheelStopAlertFlag=true;
          return -1;
        }
        float distance = (6787.0 /((float)analogValue - 3.0)) - 4.0;
        //float distance = (2914 / ((float)analogValue + 5)) - 1;
        */
        float distance = 12343.85 * pow((float)analogValue,-1.15);
        
        if(distance<MIN_FLOOR_DISTANCE_CM || distance>MAX_FLOOR_DISTANCE_CM)
          ArduBot::wheelStopAlertFlag=true;
        else
          ArduBot::wheelStopAlertFlag=false;
          
        return distance;
      };
      inline unsigned int adcRead(byte pin)
      {
        return analogRead(pin);
      }
      //-------------------------------------------------------------//
      //--------Funcion para leer el nivel de la bateria-------------//
      inline void getBatteryLevel(byte *value, byte *stat)
      {
        I2c.read(0x14,2);
        *stat=I2c.receive();
        *value=I2c.receive();
        energyState=!I2c.returnStatusWire;
      };
      //-------------------------------------------------------------//
      //---------------Funcion para leer los SRF10-------------------//
      int updateSrfRange(int address);
      unsigned int getSrfRange(int address);
      int getSrfLight(int address);
      int changeSrfAddress(int oldAddress, int newAddress);
      int changeSrfGain(int address, int gain);
      int changeSrfDistance(int address, int range);
      void testSrfs();
      void setSrfsRegisters();
      inline void setSrfContinuousUpdate(boolean value)
      {
        isSrfUpdateContinuous=value;
      }
      //-------------------------------------------------------------//
      //----------------Funcion para leer el IMU---------------------//
      void updateIMU();
      //-------------------------------------------------------------//
      inline void setK(byte k, float value)
      {
        switch (k)
        {
          case 0:
            //kp
            ArduBot::par_motores.kp=value;
            break;
          case 1:
            //ki
            ArduBot::par_motores.ki=value;
            break;
          case 2:
            //kd
            ArduBot::par_motores.kd=value;
            break;
        }
      }
      //-------------------------------------------------------------//
      void spinOnce();
  };
}

#endif
