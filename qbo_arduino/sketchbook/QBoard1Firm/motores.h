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
 * \author Daniel Julian
 *********************************************************************/

#ifndef motores_h
#define motores_h

#include <inttypes.h>
#include <avr/io.h>
#include <WProgram.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

namespace arduBot
{
  class CMotors {
    public:
      int pulsesDifference;
      int pwmValue;
      
      volatile uint8_t *control1Port;
      volatile uint8_t *control2Port;
      byte control1Pin;
      byte control2Pin;
      byte pwmPin;  //arduino form
      void setDirection()
      {
        if(pwmValue<0)
        {
          bitClear(*control1Port,control1Pin);
          bitSet(*control2Port,control2Pin);
        }
        else
        {
          bitClear(*control2Port,control2Pin);
          bitSet(*control1Port,control1Pin);
        }
      };
      void pushPwm(int newPwmValue)
      {
        pwmValue=newPwmValue;
        setDirection();
        analogWrite(pwmPin,abs(pwmValue));
      };
      int getPulsesDifference()
      {
        int pulsesDifference=0;
        ATOMIC_BLOCK(ATOMIC_FORCEON)
        {
          pulsesDifference=this->pulsesDifference;
          this->pulsesDifference=0;
        }
        return pulsesDifference;
      };
      CMotors(volatile uint8_t *control1Register, volatile uint8_t *control2Register, volatile uint8_t *control1Port, volatile uint8_t *control2Port, byte control1Pin, byte control2Pin, byte pwmPin) :
            control1Port(control1Port), control2Port(control2Port), control1Pin(control1Pin), control2Pin(control2Pin), pwmPin(pwmPin)
      {
        pinMode(pwmPin,OUTPUT);
        bitSet(*control1Register,control1Pin);
        bitSet(*control2Register,control2Pin);
      };
  };
  
  class CBaseMovement {
      public:
        CMotors *leftMotor;
        CMotors *rightMotor;
        double desiredLinearSpeed;
        double desiredAngularSpeed;
        double actualLinearMovement;
        double actualAngularMovement;
        
        double desiredLinearPulses;
        double desiredAngularPulses;
        double referenceLinearPulses;
        double referenceAngularPulses;
        double referenceLeftWheelPulses;
        double referenceRightWheelPulses;
        double actualLeftWheelPulses;
        double actualRightWheelPulses;
        
        double linearError;
        double angularError;
        double leftError;
        double rightError;
        double integralLeftError;
        double integralRightError;
        int leftMotorPwm;
        int rightMotorPwm;
        
        double sampleTime;
        
        double kp;
        double ki;
        double kd;
        
        double maxLinearAcceleration;
        double maxAngularAcceleration;
        int encoderResolution;
        double wheelRadius;
        double distanceBetweenWheels;
        double linearPulse2LinearMovementConstant;
        double angularPulse2AngularMovementConstant;
        
        uint16_t leftStallCount;
        uint16_t rightStallCount;
        bool leftStallDetected;
        bool rightStallDetected;
        double minStallDetectionTime;
        double minStallStopSecurityTime;
        
        byte variablesInitialized;
        
        CBaseMovement(CMotors *leftMotor,CMotors *rightMotor, double kp=0, double kd=0, double ki=0, double sampleTime=0.005, int encoderResolution=1440, double distanceBetweenWheels=0.2736, double wheelRadius=0.102) : leftMotor(leftMotor), rightMotor(rightMotor),
                      kp(kp), kd(kd), ki(ki), sampleTime(sampleTime), encoderResolution(encoderResolution), distanceBetweenWheels(distanceBetweenWheels), wheelRadius(wheelRadius)
        {
          desiredLinearSpeed=0;
          desiredAngularSpeed=0;
          actualLinearMovement=0;
          actualAngularMovement=0;

          leftStallCount=0;
          rightStallCount=0;
          leftStallDetected=false;
          rightStallDetected=false;
          minStallDetectionTime=2.0;
          minStallStopSecurityTime=5;
          
          integralLeftError=0;
          leftError=0;
          integralRightError=0;
          rightError=0;
          
          maxLinearAcceleration=1; //1m/ss
          maxAngularAcceleration=3;
          
          if(encoderResolution==0)
          {
            variablesInitialized=0;
            return;
          }
          variablesInitialized=1;
          linearPulse2LinearMovementConstant=PI*wheelRadius/encoderResolution;
          angularPulse2AngularMovementConstant=2*PI*wheelRadius/(distanceBetweenWheels*encoderResolution);
        };
        void initializePhisicalVariables(int _encoderResolution, double _distanceBetweenWheels, double _wheelRadius)
        {
          encoderResolution=_encoderResolution;
          distanceBetweenWheels=_distanceBetweenWheels;
          wheelRadius=_wheelRadius;
          if(encoderResolution==0)
          {
            variablesInitialized=0;
            return;
          }
          variablesInitialized=1;
        };
        void initializePIDVariables(double _kp, double _kd, double _ki, double _sampleTime)
        {
          kp=_kp;
          kd=_kd;
          ki=_ki;
          sampleTime=_sampleTime;
        };
        
        void transformSpeeds2Pulses()
        {
          if(variablesInitialized==0)
            return;
          desiredLinearPulses=desiredLinearSpeed*sampleTime/linearPulse2LinearMovementConstant;
          desiredAngularPulses=desiredAngularSpeed*sampleTime/angularPulse2AngularMovementConstant;
        };
        
        void limitAcceleration()
        {
          double accelerationGap=desiredLinearPulses-referenceLinearPulses;
          if(abs(accelerationGap)<maxLinearAcceleration)
            referenceLinearPulses=desiredLinearPulses;
          else
          {
            if(accelerationGap>0)
              referenceLinearPulses+=maxLinearAcceleration;
            else
              referenceLinearPulses-=maxLinearAcceleration;
          }
          
          accelerationGap=desiredAngularPulses-referenceAngularPulses;
          if(abs(accelerationGap)<maxAngularAcceleration)
            referenceAngularPulses=desiredAngularPulses;
          else
          {
            if(accelerationGap>0)
              referenceAngularPulses+=maxAngularAcceleration;
            else
              referenceAngularPulses-=maxAngularAcceleration;
          }
        };
        
        void linearPlusAngular2LeftPlusRight()
        {
          referenceRightWheelPulses=(referenceLinearPulses+referenceAngularPulses)/2;
          referenceLeftWheelPulses=(referenceLinearPulses-referenceAngularPulses)/2;
          /*
          Serial.print("reference left pulses: ");
          Serial.println(referenceLeftWheelPulses,DEC);
          Serial.print("reference right pulses: ");
          Serial.println(referenceRightWheelPulses,DEC);
          */
        };
        
        void doPIDControl()
        {
          integralLeftError+=leftError;
          integralRightError+=rightError;
          double oldLeftError=leftError;
          double oldRightError=rightError;
          leftError=actualLeftWheelPulses-referenceLeftWheelPulses;
          rightError=actualRightWheelPulses-referenceRightWheelPulses;
          double derivativeLeftError=leftError-oldLeftError;
          double derivativeRightError=rightError-oldRightError;
          /*
          Serial.print("error: ");
          Serial.println(leftError,DEC);
          Serial.print("diferencial: ");
          Serial.println(derivativeLeftError,DEC);
          Serial.print("integral: ");
          Serial.println(integralLeftError,DEC);
          */
          leftMotorPwm=kp*leftError+kd*derivativeLeftError+ki*integralLeftError;
          rightMotorPwm=kp*rightError+kd*derivativeRightError+ki*integralRightError;
          /*
          Serial.print("left motor pwm: ");
          Serial.println(leftMotorPwm,DEC);
          Serial.print("right motor pwm: ");
          Serial.println(rightMotorPwm,DEC);
          */
        };
        
        int checkMotorsLimits()
        {
          int limitStatus=0;
          if((abs(leftMotorPwm)>255)||(abs(rightMotorPwm)>255))
          {
            int maxMotorPwm=max(abs(leftMotorPwm),abs(rightMotorPwm));
            leftMotorPwm=((double)leftMotorPwm*255)/maxMotorPwm;
            rightMotorPwm=((double)rightMotorPwm*255)/maxMotorPwm;
            limitStatus=1;
          }
          /*
          Serial.print("left motor pwm after check: ");
          Serial.println(leftMotorPwm,DEC);
          Serial.print("right motor pwm after check: ");
          Serial.println(rightMotorPwm,DEC);
          */
          /*
          if(stall<0)
          {
            leftMotorPwm=0;
            rightMotorPwm=0;
            return -1;
          }
          */
          return limitStatus;
        };
        
        void updateMotorsPwm()
        {
          leftMotor->pushPwm(leftMotorPwm);
          rightMotor->pushPwm(rightMotorPwm);
        };
        
        void updateMotorsMovement()
        {
          actualLeftWheelPulses=leftMotor->getPulsesDifference();
          actualRightWheelPulses=rightMotor->getPulsesDifference();
          actualLinearMovement=(actualLeftWheelPulses+actualRightWheelPulses)*linearPulse2LinearMovementConstant;
          actualAngularMovement=(actualRightWheelPulses-actualLeftWheelPulses)*angularPulse2AngularMovementConstant;
          /*
          Serial.print("actual left pulses: ");
          Serial.println(actualLeftWheelPulses,DEC);
          Serial.print("actual right pulses: ");
          Serial.println(actualRightWheelPulses,DEC);
          */
        };
        
        void stallDetection()
        {
          //if((abs(referenceRightWheelPulses/actualRightWheelPulses)>3)||(abs(referenceLeftWheelPulses/actualLeftWheelPulses)>3))
          //if(((referenceRightWheelPulses!=0)&&(actualRightWheelPulses==0))||((referenceLeftWheelPulses!=0)&&(actualLeftWheelPulses==0)))
          //  stall++;
          //else
          //  stall=0;
          //if(stall>minStallDetectionTime/sampleTime)
          //  stall=-1;
          //if(stall<0)
          //  stall--;
          //if(stall<(-(minStallStopSecurityTime/sampleTime)))
          //  stall=0;
//          if((referenceLeftWheelPulses!=0)&&(actualLeftWheelPulses==0))
          //if(abs(referenceLeftWheelPulses-actualLeftWheelPulses)>3)
          if(abs(referenceLeftWheelPulses)>0.5 && abs(actualLeftWheelPulses/referenceLeftWheelPulses)<0.5)
          {
            if(leftStallCount!=0xFFFF)
              leftStallCount++;
          }
          else if (abs(referenceLeftWheelPulses)<=0.5 || abs(actualLeftWheelPulses/referenceLeftWheelPulses)>=0.5)
          {
            leftStallCount=0;
            //leftStallDetected=false;
            //if(leftStallCount>2)
            //  leftStallCount-=2;
            //else
            //  leftStallCount=0;
          }
          //if((abs(referenceRightWheelPulses)>30)&&(actualRightWheelPulses==0))
          if(abs(referenceRightWheelPulses)>0.5 && abs(actualRightWheelPulses/referenceRightWheelPulses)<0.5)
          {
            if(rightStallCount!=0xFFFF)
              rightStallCount++;
          }
          else if (abs(referenceRightWheelPulses)<=0.5 || abs(actualRightWheelPulses/referenceRightWheelPulses)>=0.5)
          {
            rightStallCount=0;
            //rightStallDetected=false;
            //if(rightStallCount>2)
            //  rightStallCount-=2;
            //else
            //  rightStallCount=0;
          }
          if(leftStallCount>minStallDetectionTime/sampleTime)
            leftStallDetected=true;
          else if(leftStallCount<=minStallDetectionTime/sampleTime)
            leftStallDetected=false;
          //else if(leftStallDetected&&leftStallCount==0)
          //  leftStallDetected=false;
          if(rightStallCount>minStallDetectionTime/sampleTime)
            rightStallDetected=true;
          else if(rightStallCount>minStallDetectionTime/sampleTime)
            rightStallDetected=false;
          //else if(rightStallDetected&&rightStallCount==0)
          //  rightStallDetected=false;
          /*
          Serial.print(referenceLeftWheelPulses,DEC);
          Serial.print("    ");
          Serial.print(abs(actualLeftWheelPulses/referenceLeftWheelPulses),DEC);
          Serial.print("    ");
          Serial.println(leftStallCount,DEC);
          */
        };
        
        bool doControlLoop()
        {
          if(variablesInitialized==0)
            return 0;
          
          updateMotorsMovement();
          if((desiredLinearPulses==0)&&(desiredAngularPulses==0))
          {
            freeMotors();
            //updateMotorsMovement();
            return 0;
          }
          
          //transformSpeeds2Pulses();
          limitAcceleration();
          linearPlusAngular2LeftPlusRight();
          doPIDControl();
          checkMotorsLimits();
          updateMotorsPwm();
          //stallDetection();
          return (leftStallDetected || rightStallDetected);
          /*
          if(stall<0)
            return 0;
          else
            return 1;
          */
        };
        void freeMotors()
        {
          desiredLinearSpeed=0;
          desiredAngularSpeed=0;
          linearPlusAngular2LeftPlusRight();
          //stallDetection();
          
          integralLeftError=0;
          leftError=0;
          integralRightError=0;
          rightError=0;
          leftMotor->pushPwm(0);
          rightMotor->pushPwm(0);
        };
  };
}

#endif
