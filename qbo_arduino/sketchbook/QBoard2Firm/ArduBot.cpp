#include "ArduBot.h"
#include <avr/interrupt.h>

using namespace arduBot;
//---------Definicion de las variables globales--------------------//
long servosUpdateTime=0;
long micsUpdateTime=0;
long mouthUpdateTime=0;
byte lastMouthIndex=0;
byte nNoSoundIterations=0;
byte lastMicArrayIndex=0;
//-----------------------------------------------------------------//
//----------Inicializacion del sistema----------------------//
ArduBot::ArduBot()
{
}
void ArduBot::begin()
{
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
  pinMode(SHIFT_CLK_ARDUINO_PIN,OUTPUT);
  pinMode(SHIFT_LCK_ARDUINO_PIN,OUTPUT);
  pinMode(SHIFT_DATA_ARDUINO_PIN,OUTPUT);
  pinMode(SWITCH0_ARDUINO_PIN,OUTPUT);
  pinMode(SWITCH1_ARDUINO_PIN,OUTPUT);
  digitalWrite(3,HIGH);
  digitalWrite(SHIFT_LCK_ARDUINO_PIN,LOW);
  digitalWrite(SHIFT_DATA_ARDUINO_PIN,LOW);
  digitalWrite(SHIFT_CLK_ARDUINO_PIN,LOW);
//-----Inicializacion del puerto serie---------//
  Serial.begin(115200);
  Serial.flush();
//---------------------------------------------//
  //setMic0();
  
  mouthData=0;
  noseColor=0;
  for(int i=0;i<4;i++)
  {
    for(int j=0;j<5;j++)
    {
      ledsMouth[i][j]=false;
    }
  }
  
  //Mouths de animacion de habla
  for(int i=0;i<7; i++)
  {
    for(int j=0;j<4;j++)
    {
      for(int k=0;k<5;k++)
      {
        talkMouths[i][j][k]=false;
      }
    }
  }
 
//Mouth "realista" de tres movimientos  
  //Speek1
  talkMouths[0][1][0]=true;
  talkMouths[0][1][1]=true;
  talkMouths[0][1][2]=true;
  talkMouths[0][1][3]=true;
  talkMouths[0][1][4]=true;
  //Speek2
  talkMouths[1][0][0]=true;
  talkMouths[1][0][1]=true;
  talkMouths[1][0][2]=true;
  talkMouths[1][0][3]=true;
  talkMouths[1][0][4]=true;
  talkMouths[1][1][0]=true;
  talkMouths[1][1][4]=true;
  talkMouths[1][2][1]=true;
  talkMouths[1][2][2]=true;
  talkMouths[1][2][3]=true;
  
  //Speek3
  talkMouths[2][0][0]=true;
  talkMouths[2][0][1]=true;
  talkMouths[2][0][2]=true;
  talkMouths[2][0][3]=true;
  talkMouths[2][0][4]=true;
  talkMouths[2][1][0]=true;
  talkMouths[2][1][4]=true;
  talkMouths[2][2][0]=true;
  talkMouths[2][2][4]=true;
  talkMouths[2][3][1]=true;
  talkMouths[2][3][2]=true;
  talkMouths[2][3][3]=true;
 
   //Speek4
  talkMouths[3][0][0]=true;
  talkMouths[3][0][1]=true;
  talkMouths[3][0][2]=true;
  talkMouths[3][0][3]=true;
  talkMouths[3][0][4]=true;
  talkMouths[3][1][0]=true;
  talkMouths[3][1][1]=true;
  talkMouths[3][1][2]=true;
  talkMouths[3][1][3]=true;
  talkMouths[3][1][4]=true;
  talkMouths[3][2][0]=true;
  talkMouths[3][2][1]=true;
  talkMouths[3][2][2]=true;
  talkMouths[3][2][3]=true;
  talkMouths[3][2][4]=true;
  talkMouths[3][3][1]=true;
  talkMouths[3][3][2]=true;
  talkMouths[3][3][3]=true;

  //Speek5
  talkMouths[4][0][0]=true;
  talkMouths[4][0][1]=true;
  talkMouths[4][0][2]=true;
  talkMouths[4][0][3]=true;
  talkMouths[4][0][4]=true;
  talkMouths[4][1][0]=true;
  talkMouths[4][1][1]=true;
  talkMouths[4][1][2]=true;
  talkMouths[4][1][3]=true;
  talkMouths[4][1][4]=true;
  talkMouths[4][2][1]=true;
  talkMouths[4][2][2]=true;
  talkMouths[4][2][3]=true;


//  talkMouths[1][2][1]=true;
//  talkMouths[1][2][2]=true;
//  talkMouths[1][2][3]=true;
  
 /* 
  //Speek1
  talkMouths[0][0][2]=true;
  talkMouths[0][1][1]=true;
  talkMouths[0][1][3]=true;
  talkMouths[0][2][2]=true;
  
  //Speek2
  talkMouths[1][0][2]=true;
  talkMouths[1][1][1]=true;
  talkMouths[1][1][3]=true;
  talkMouths[1][2][1]=true;
  talkMouths[1][2][3]=true;
  talkMouths[1][3][2]=true;
  
  //Speek1
  talkMouths[2][1][0]=true;
  talkMouths[2][1][1]=true;
  talkMouths[2][1][2]=true;
  talkMouths[2][1][3]=true;
  talkMouths[2][1][4]=true;

  //Speek2
  talkMouths[3][0][0]=true;
  talkMouths[3][0][1]=true;
  talkMouths[3][0][2]=true;
  talkMouths[3][0][3]=true;
  talkMouths[3][0][4]=true;
  talkMouths[3][1][0]=true;
  talkMouths[3][1][4]=true;
  talkMouths[3][2][1]=true;
  talkMouths[3][2][2]=true;
  talkMouths[3][2][3]=true;
  
  //Speek3
  talkMouths[4][0][0]=true;
  talkMouths[4][0][1]=true;
  talkMouths[4][0][2]=true;
  talkMouths[4][0][3]=true;
  talkMouths[4][0][4]=true;
  talkMouths[4][1][0]=true;
  talkMouths[4][1][4]=true;
  talkMouths[4][2][0]=true;
  talkMouths[4][2][4]=true;
  talkMouths[4][3][1]=true;
  talkMouths[4][3][2]=true;
  talkMouths[4][3][3]=true;
  
  //Speek5
  talkMouths[5][0][1]=true;
  talkMouths[5][0][2]=true;
  talkMouths[5][0][3]=true;
  talkMouths[5][1][0]=true;
  talkMouths[5][1][4]=true;
  talkMouths[5][2][0]=true;
  talkMouths[5][2][4]=true;
  talkMouths[5][3][1]=true;
  talkMouths[5][3][2]=true;
  talkMouths[5][3][3]=true;
  
  //Speek6
  talkMouths[6][0][1]=true;
  talkMouths[6][0][2]=true;
  talkMouths[6][0][3]=true;
  talkMouths[6][1][0]=true;
  talkMouths[6][1][4]=true;
  talkMouths[6][2][1]=true;
  talkMouths[6][2][2]=true;
  talkMouths[6][2][3]=true;
  
  //Speek7: Todo en off que ya esta
  
/*  
  //Speek1
  talkMouths[0][0][2]=true;
  talkMouths[0][1][2]=true;
  //Speek2
  talkMouths[1][3][2]=true;
  talkMouths[1][2][2]=true;
  talkMouths[1][1][2]=true;
  talkMouths[1][0][2]=true;
  //Speek3
  talkMouths[2][3][2]=true;
  talkMouths[2][2][2]=true;
  talkMouths[2][1][2]=true;
  talkMouths[2][0][2]=true;
  talkMouths[2][0][1]=true;
  talkMouths[2][0][3]=true;
  talkMouths[2][1][1]=true;
  talkMouths[2][1][3]=true;
  //Speek4
  talkMouths[3][3][2]=true;
  talkMouths[3][2][2]=true;
  talkMouths[3][1][2]=true;
  talkMouths[3][0][2]=true;
  talkMouths[3][3][1]=true;
  talkMouths[3][3][3]=true;
  talkMouths[3][2][1]=true;
  talkMouths[3][2][3]=true;
  talkMouths[3][1][1]=true;
  talkMouths[3][1][3]=true;
  talkMouths[3][0][1]=true;
  talkMouths[3][0][3]=true;
  //Speek5
  talkMouths[4][3][2]=true;
  talkMouths[4][2][2]=true;
  talkMouths[4][1][2]=true;
  talkMouths[4][0][2]=true;
  talkMouths[4][3][1]=true;
  talkMouths[4][3][3]=true;
  talkMouths[4][2][1]=true;
  talkMouths[4][2][3]=true;
  talkMouths[4][1][1]=true;
  talkMouths[4][1][3]=true;
  talkMouths[4][0][1]=true;
  talkMouths[4][0][3]=true;
  talkMouths[4][0][0]=true;
  talkMouths[4][0][4]=true;
  talkMouths[4][1][0]=true;
  talkMouths[4][1][4]=true;
  //Speek6
  for(int j=0;j<4;j++)
  {
    for(int k=0;k<5;k++)
    {
      talkMouths[5][j][k]=true;
    }
  }
  //Speek7: Todo en off que ya esta
  
  */
  //Inicializacin del array de movimiento de la boca que tiene 12 elementos
  for(byte i=0;i<6;i++)
    movementMouth[i]=i;
    //movementMouth[i]=5;
  for(byte i=0;i<5;i++)
    movementMouth[6+i]=4-i;
    //movementMouth[6+i]=5;
  movementMouth[11]=6;
  isMouthMoving=true;
  selectedMouth=(boolean **)ledsMouth;
//Serial.println("hola");

  //ledsMouth[0][0]=true;
  /*
  ledsMouth[0][2]=true;
  ledsMouth[0][3]=true;
  ledsMouth[1][2]=true;
  ledsMouth[2][2]=true;
  ledsMouth[3][0]=true;
  ledsMouth[3][2]=true;
  ledsMouth[3][4]=true;
  */
//-----Inicializacion de servos----------------//
  servosType[0]=0;
  servosType[1]=0;
  servosType[2]=1;
  servosType[3]=1;
  servos[0]=(void*)&headServo1;
  servos[1]=(void*)&headServo2;
  servos[2]=(void*)&eyeServo1;
  servos[3]=(void*)&eyeServo2;
  //headServo1.attach(HEAD_SERVO_1_PWM_ARDUINO_PIN);
  //headServo2.attach(HEAD_SERVO_2_PWM_ARDUINO_PIN);
  eyeServo1.attach(EYE_SERVO_1_PWM_ARDUINO_PIN);
  eyeServo2.attach(EYE_SERVO_2_PWM_ARDUINO_PIN);
  
  headServo1Info[0]=1500;
  headServo1Info[1]=1500;
  headServo1Info[2]=1800*0.005;
  headServo2Info[0]=1500;
  headServo2Info[1]=1500;
  headServo2Info[2]=1800*0.005;
  eyeServo1Info[0]=1500;
  eyeServo1Info[1]=1500;
  eyeServo1Info[2]=1800*0.005;
  eyeServo2Info[0]=1500;
  eyeServo2Info[1]=1500;
  eyeServo2Info[2]=1800*0.005;
  servosInfo[0]=headServo1Info;
  servosInfo[1]=headServo2Info;
  servosInfo[2]=eyeServo1Info;
  servosInfo[3]=eyeServo2Info;
  
  eyeServo1SettedPosition=1500;//EYE_SERVO_1_CENTER;
  eyeServo2SettedPosition=1500;//EYE_SERVO_2_CENTER;
  
  for(int i=0;i<20;i++)
  {
    mics[0][i]=0;
    mics[1][i]=0;
    mics[2][i]=0;
  }
//---------------------------------------------//
  servosUpdateTime=micsUpdateTime=mouthUpdateTime=millis();
}
//----------------------------------------------------------//
void ArduBot::servoWriteMicroseconds(void* servo, byte type, int uptime)
{
  switch(type)
  {
    case 0:
      ((ServoTimeTimer1*)servo)->write(uptime);
      break;
    case 1:
      ((ServoTimer2*)servo)->write(uptime);
      break;
  }
}

void ArduBot::updateServos()
{
  for(int i=0;i<4;i++)
  {
    if(servosInfo[i][0]!=servosInfo[i][1])
    {
      if(servosInfo[i][0]<servosInfo[i][1])
      {
        servosInfo[i][0]+=servosInfo[i][2];
        if(servosInfo[i][0]>servosInfo[i][1])
          servosInfo[i][0]=servosInfo[i][1];
      }
      else
      {
        servosInfo[i][0]-=servosInfo[i][2];
        if(servosInfo[i][0]<servosInfo[i][1])
          servosInfo[i][0]=servosInfo[i][1];
      }
      
    }
    servoWriteMicroseconds(servos[i],servosType[i],int(servosInfo[i][0]));
  }
}

void ArduBot::updateMics()
{
  lastMicArrayIndex=(lastMicArrayIndex+1)%20;
  mics[0][lastMicArrayIndex]=analogRead(0);
  mics[1][lastMicArrayIndex]=analogRead(1);
  mics[2][lastMicArrayIndex]=analogRead(2);
  mics[3][lastMicArrayIndex]=analogRead(3);
  //Para el movimiento de la boca
  int maxLevel=0;
  unsigned int minLevel=0xFFFF;
  for(int k=0;k<20;k++)
  {
    maxLevel=max(maxLevel,mics[3][k]);
    minLevel=min(minLevel,(unsigned int)mics[3][k]);
  }
  maxLevel-=minLevel;
  oldMicLevel=0;
  if(maxLevel>50)
  {
    oldMicLevel=(min(300,((maxLevel-50)))*5);
    if(!isMouthMoving)
    {
      lastMouthIndex=0;
      isMouthMoving=true;
      mouthUpdateTime=millis();
    }
    nNoSoundIterations=0;
  }
  else if(isMouthMoving)
  {
    nNoSoundIterations++;
    if(nNoSoundIterations>20)
    {
      nNoSoundIterations=0;
      isMouthMoving=false;
      lastMouthIndex=0;
    }
  }
}
//----------------------------------------------------------//
//-------------------Funcion para los servos----------------//
void ArduBot::setHeadServoAngle(byte servoIndex, unsigned int angleMilliseconds, unsigned int servoSpeed)  //ArrAb
{
  float newAngleMilliseconds=float(angleMilliseconds);
  float newSpeed=float(servoSpeed)*0.05;
  if(servoIndex==1)
  {
    headServo1Info[1]=newAngleMilliseconds;
    headServo1Info[2]=newSpeed;
  }
  else if(servoIndex==2)
  {
    headServo2Info[1]=newAngleMilliseconds;
    headServo2Info[2]=newSpeed;
  }
}
void ArduBot::setEyeServoAngle(byte servoIndex, unsigned int angleMilliseconds, unsigned int servoSpeed, boolean setRelaxPosition)
{
  //return;
  float newAngleMilliseconds=float(angleMilliseconds);
  if (newAngleMilliseconds<1195) newAngleMilliseconds=1195;
  if (newAngleMilliseconds>1806) newAngleMilliseconds=1806;
  float newSpeed=float(servoSpeed)*0.05;
  if(servoIndex==1)
  {
    eyeServo1Info[1]=newAngleMilliseconds;
    eyeServo1Info[2]=newSpeed;
    if (setRelaxPosition)
      eyeServo1SettedPosition=newAngleMilliseconds;
  }
  else if(servoIndex==2)
  {
    eyeServo2Info[1]=newAngleMilliseconds;
    eyeServo2Info[2]=newSpeed;
    if (setRelaxPosition)
      eyeServo2SettedPosition=newAngleMilliseconds;
  }
}
//----------------------------------------------------------//
//-------------------Funcion para la boca-------------------//
void ArduBot::testMouth()
{
  //Si el bit de la columna esta a 1, se enciende, si está a 0, se apaga
  shiftRegisterWrite(0x0000);  //All off
  delay(2000);
  //shiftRegisterWrite(0x00F8);  //All on except solitario
  shiftRegisterWrite(0x00FF);  //All on
  delay(2000);
}

void ArduBot::shiftRegisterWrite(word data)
{
  //Direct access to the ports for fast execution of rutine
  word mask=0x8000;
  for(int i=0;i<16;i++)
  {
    if((data&mask)!=0)
      PORTB |= B00001000;
    else
      PORTB &= B11110111;
    PORTB |= B00100000;
    PORTB &= B11011111;
    mask>>=1;
  }
  PORTB |= B00000001;
  PORTB &= B11111110;
}

void ArduBot::updateMouth()
{
  for(int fila=0;fila<4;fila++)
  {
    word data=0x000F & (~(1<<fila));
    data<<=8;
    for(int columna=0;columna<5;columna++)
    {
      word led=((boolean *)selectedMouth)[fila*5+columna];
      data |= (led<<(7-columna));
    }
    data|=noseColor;
    shiftRegisterWrite(data);
  }
  shiftRegisterWrite(noseColor);
}
void ArduBot::setImage(byte b1, byte b2, byte b3)
{
  long data=b3&B00011111;
  data<<=8;
  data|=b2;
  data<<=7;
  data|=(b1>>1);
  boolean* leds=(boolean*)ledsMouth;
  for(int i=0;i<20;i++)
  {
    if((data&0x0001)!=0)
      leds[i]=true;
    else
      leds[i]=false;
    data>>=1;
  }
}
//----------------------------------------------------------//
void ArduBot::spinOnce()
{
  long now=millis();
  //Movimiento de los servos
  if(now-servosUpdateTime>50)
  {
    servosUpdateTime+=50;
    updateServos();
  }
  updateMics();
  //Update de boca
  updateMouth();
  
  if(now-mouthUpdateTime>50)
  {
    mouthUpdateTime+=50;
    if(isMouthMoving)
    {
      //unsigned int bocaSelecionada=min(oldMicLevel/60,4);
      selectedMouth=(boolean **)talkMouths[min(oldMicLevel/60,4)];
      //Serial.print(bocaSelecionada,DEC);
      //Serial.print(" ");
      //Serial.println(oldMicLevel,DEC);
      //level va a ir hasta los 300 aprox
      setEyeServoAngle(1,EYE_SERVO_1_CENTER+oldMicLevel,1200, false);
      setEyeServoAngle(2,EYE_SERVO_2_CENTER-oldMicLevel,1200, false);
    }
    else
    {
      selectedMouth=(boolean **)ledsMouth;
      //level va a ir hasta los 300 aprox
      setEyeServoAngle(1,eyeServo1SettedPosition,1200);
      setEyeServoAngle(2,eyeServo2SettedPosition,1200);
    }
  }
}

