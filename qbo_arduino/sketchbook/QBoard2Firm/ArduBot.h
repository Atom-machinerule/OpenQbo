#ifndef ArduBot_h
#define ArduBot_h

#include <inttypes.h>
#include <WProgram.h> 
//---------------includes de las librerias usadas--------------//
#include <ServoTimeTimer1.h>
#include <ServoTimer2.h>
//-------------------------------------------------------------//

#define HEAD_SERVO_1_PWM_ARDUINO_PIN 5 //9
#define HEAD_SERVO_2_PWM_ARDUINO_PIN 6 //10
#define EYE_SERVO_1_PWM_ARDUINO_PIN 9 //5
#define EYE_SERVO_2_PWM_ARDUINO_PIN 10 //6

#define SHIFT_CLK_ARDUINO_PIN 13  //PB5
#define SHIFT_LCK_ARDUINO_PIN 8  //PB0
#define SHIFT_DATA_ARDUINO_PIN 11  //PB3

#define SWITCH0_ARDUINO_PIN 2
#define SWITCH1_ARDUINO_PIN 4

#define MAX_EYE_AMPLITUDE 200
#define EYE_SERVO_1_CENTER 1500
#define EYE_SERVO_2_CENTER 1500

namespace arduBot
{
  const uint8_t boardId=1;
  const uint8_t libraryVersion=1;
  //-------------------------------------------------------------//
  class ArduBot
  {
    private:
      //----variables para controlar los servos de la cabeza---------//
      
      ServoTimeTimer1 headServo1;
      ServoTimeTimer1 headServo2;
      ServoTimer2 eyeServo1;
      ServoTimer2 eyeServo2;
      
      byte servosType[4];
      float headServo1Info[3];     //Posicion actual, posicion deseada, velocidad
      float headServo2Info[3];        //Posicion actual, posicion deseada, velocidad
      float eyeServo1Info[3];    //Posicion actual, posicion deseada, velocidad
      float eyeServo2Info[3];   //Posicion actual, posicion deseada, velocidad
      float eyeServo1SettedPosition;
      float eyeServo2SettedPosition;
      float * servosInfo[4];
      void* servos[4];
      void servoWriteMicroseconds(void* servo, byte type, int uptime);
      void updateServos();
      void updateMics();
      void updateMouth();
      boolean ledsMouth[4][5];
      boolean isMouthMoving;
      
      unsigned int oldMicLevel;
      
      boolean talkMouths[7][4][5];
      byte movementMouth[12];
      boolean **selectedMouth;

      long mouthData;
      byte noseColor;
      //-------------------------------------------------------------//
      void shiftRegisterWrite(word data);
    public:
      int mics[4][20];
      //------------------------Constructor--------------------------//
      ArduBot();
      //-------------------------------------------------------------//
      void begin();
      //-------------------------------------------------------------//
      //-------------------Funcion para los servos-------------------//
      //void setUpDownAngle(unsigned int angle, unsigned int vel);
      //void setLeftRightAngle(unsigned int angle, unsigned int vel);
      void setHeadServoAngle(byte servoIndex, unsigned int angleMilliseconds, unsigned int servoSpeed);
      //void setLeftEyeAngle(unsigned int angle, unsigned int vel, boolean setRelaxPosition=true);
      //void setRightEyeAngle(unsigned int angle, unsigned int vel, boolean setRelaxPosition=true);
      void setEyeServoAngle(byte servoIndex, unsigned int angleMilliseconds, unsigned int servoSpeed, boolean setRelaxPosition=true);
      inline int getServoPosition(byte servo)
      {
        return (int(servosInfo[servo][0]));
      }
      //-------------------------------------------------------------//
      void testMouth();
      void setImage(byte b1, byte b2, byte b3);
      inline void setState(byte stateColor)
      {
        noseColor=B00000111&stateColor;
      }
      //-------------------------------------------------------------//
      inline void setMic0()
      {
        digitalWrite(SWITCH0_ARDUINO_PIN,LOW);
        digitalWrite(SWITCH1_ARDUINO_PIN,LOW);
      }
      inline void setMic1()
      {
        digitalWrite(SWITCH0_ARDUINO_PIN,HIGH);
        digitalWrite(SWITCH1_ARDUINO_PIN,LOW);
      }
      inline void setMic2()
      {
        digitalWrite(SWITCH1_ARDUINO_PIN,HIGH);
        digitalWrite(SWITCH0_ARDUINO_PIN,LOW);
      }
      inline void setMicMute()
      {
        digitalWrite(SWITCH1_ARDUINO_PIN,HIGH);
        digitalWrite(SWITCH0_ARDUINO_PIN,HIGH);
      }
      //-------------------------------------------------------------//
      void spinOnce();
  };
}

#endif
