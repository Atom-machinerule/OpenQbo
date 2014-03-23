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

#include "serialProtocol.h"
#include <inttypes.h>
#include <WProgram.h> 
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

using namespace arduBotSerial;

uint8_t pearsondata[] = {

  0x00, 0x77, 0xee, 0x99, 0x07, 0x70, 0xe9, 0x9e, 0x0e, 0x79, 0xe0, 0x97,

  0x09, 0x7e, 0xe7, 0x90, 0x1d, 0x6a, 0xf3, 0x84, 0x1a, 0x6d, 0xf4, 0x83,

  0x13, 0x64, 0xfd, 0x8a, 0x14, 0x63, 0xfa, 0x8d, 0x3b, 0x4c, 0xd5, 0xa2,

  0x3c, 0x4b, 0xd2, 0xa5, 0x35, 0x42, 0xdb, 0xac, 0x32, 0x45, 0xdc, 0xab,

  0x26, 0x51, 0xc8, 0xbf, 0x21, 0x56, 0xcf, 0xb8, 0x28, 0x5f, 0xc6, 0xb1,

  0x2f, 0x58, 0xc1, 0xb6, 0x76, 0x01, 0x98, 0xef, 0x71, 0x06, 0x9f, 0xe8,

  0x78, 0x0f, 0x96, 0xe1, 0x7f, 0x08, 0x91, 0xe6, 0x6b, 0x1c, 0x85, 0xf2,

  0x6c, 0x1b, 0x82, 0xf5, 0x65, 0x12, 0x8b, 0xfc, 0x62, 0x15, 0x8c, 0xfb,

  0x4d, 0x3a, 0xa3, 0xd4, 0x4a, 0x3d, 0xa4, 0xd3, 0x43, 0x34, 0xad, 0xda,

  0x44, 0x33, 0xaa, 0xdd, 0x50, 0x27, 0xbe, 0xc9, 0x57, 0x20, 0xb9, 0xce,

  0x5e, 0x29, 0xb0, 0xc7, 0x59, 0x2e, 0xb7, 0xc0, 0xed, 0x9a, 0x03, 0x74,

  0xea, 0x9d, 0x04, 0x73, 0xe3, 0x94, 0x0d, 0x7a, 0xe4, 0x93, 0x0a, 0x7d,

  0xf0, 0x87, 0x1e, 0x69, 0xf7, 0x80, 0x19, 0x6e, 0xfe, 0x89, 0x10, 0x67,

  0xf9, 0x8e, 0x17, 0x60, 0xd6, 0xa1, 0x38, 0x4f, 0xd1, 0xa6, 0x3f, 0x48,

  0xd8, 0xaf, 0x36, 0x41, 0xdf, 0xa8, 0x31, 0x46, 0xcb, 0xbc, 0x25, 0x52,

  0xcc, 0xbb, 0x22, 0x55, 0xc5, 0xb2, 0x2b, 0x5c, 0xc2, 0xb5, 0x2c, 0x5b,

  0x9b, 0xec, 0x75, 0x02, 0x9c, 0xeb, 0x72, 0x05, 0x95, 0xe2, 0x7b, 0x0c,

  0x92, 0xe5, 0x7c, 0x0b, 0x86, 0xf1, 0x68, 0x1f, 0x81, 0xf6, 0x6f, 0x18,

  0x88, 0xff, 0x66, 0x11, 0x8f, 0xf8, 0x61, 0x16, 0xa0, 0xd7, 0x4e, 0x39,

  0xa7, 0xd0, 0x49, 0x3e, 0xae, 0xd9, 0x40, 0x37, 0xa9, 0xde, 0x47, 0x30,

  0xbd, 0xca, 0x53, 0x24, 0xba, 0xcd, 0x54, 0x23, 0xb3, 0xc4, 0x5d, 0x2a,

  0xb4, 0xc3, 0x5a, 0x2d};

uint8_t pearson(uint8_t *key, uint8_t len)
{
  uint8_t hash=0;
  for (uint8_t i=0; i<len; i++)
  {
    hash = pearsondata[hash^key[i]];
  }
  return (hash);
}

SerialProtocol::SerialProtocol(arduBot::ArduBot *robot) : robot(robot),
                                       INPUT_FLAG(0xFF), OUTPUT_FLAG(0xFE), INPUT_ESCAPE(0xFD),
                                       isInputEscaped_(false), isInputCorrect_(true)
{
  Serial.begin(115200);
  Serial.flush();
}

boolean SerialProtocol::procesaEntrada(byte* buf, byte length)
{
  if (length<5) return false;
  if(buf[0]!=INPUT_FLAG) return false;
  if(buf[length-1]!=OUTPUT_FLAG) return false;
  
  uint8_t check=pearson(buf+1,length-3);
  uint8_t inCheck=buf[length-2];
  
  if(check!=inCheck) return false;
  if(buf[2]!=length-5) return false;
  if(buf[2]>50) return false;
  command_.commandNumber=buf[1];
  command_.nInputData=length-5;
  for(int i=3;i<length-2;i++)
    command_.inputData[i-3]=buf[i];
  return true;
}

byte length=0;
byte buf[128];

void SerialProtocol::processSerial()
{
  if (Serial.available() > 0)
  {
    int incoming = Serial.read();
    byte incomingByte=0;
    if(incoming!=-1)
      incomingByte=byte(incoming);
    if(incomingByte==INPUT_FLAG)  //Llega el flage de entrada
    {
      length=0;
      buf[length]=incomingByte;
      length++;
      return;
    }
    else if(incomingByte==OUTPUT_FLAG)  //Llega el flag de salida
    {
      buf[length]=incomingByte;
      length++;
      if (procesaEntrada(buf,length))
      {
        processCommands();
        sendResponse();
      }
      else
      {
        sendNack();
      }
      length=0;
      return;
    }
    if(isInputEscaped_)
    {
      incomingByte+=2;  //Desescapamos la entrada (ya hemos pasado la comprobación de los flags de entrada y salida)
      isInputEscaped_=false;  //Si no estaba activo, seguimos, pues ya se ha hecho la corrección al dato de entrada
    }
    else if(incomingByte==INPUT_ESCAPE)  //Llega el escape de entrada
    {
      isInputEscaped_=true;  //Activamos el escape de la entrada
      return;              //y vamos a por el siguiente dato
    }
    if(length>128)
    {
      length=0;
      return;
    }
    buf[length]=incomingByte;
    length++;
  }
}

void SerialProtocol::processCommands()
{
  isInputCorrect_=true;
    switch (command_.commandNumber)
    {
      case GET_VERSION:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=2;
          command_.outputData[0]=arduBot::boardId;
          command_.outputData[1]=arduBot::libraryVersion;
        }
        break;
      case SET_MOUTH_VALUE:
        if(command_.nInputData!=3)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=0;
          robot->setImage(command_.inputData[0],command_.inputData[1],command_.inputData[2]);
        }
        break;
      case SET_STATE:
        if(command_.nInputData!=1)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=0;
          robot->setState(command_.inputData[0]);
        }
        break;
      case SET_SERVO:
        if(command_.nInputData!=5)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=0;
          unsigned int angleMilliseconds=*((unsigned int *)(command_.inputData+1));
          unsigned int servoSpeed=*((unsigned int *)(command_.inputData+3));
          byte servoIndex=command_.inputData[0];
          switch (servoIndex)
          {
            case 1: case 2:
              robot->setHeadServoAngle(servoIndex,angleMilliseconds,servoSpeed);
              break;
            case 3: case 4:
              robot->setEyeServoAngle(servoIndex-2,angleMilliseconds,servoSpeed);
              break;
          }
        }
        //sin valor de retorno
        //3 valores de entrada
        break;
      case GET_MIC_REPORT:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=6;
          for (int j=0;j<3;j++)
          {
            int maxLevel=0;
            int minLevel=1024;
            for(int k=0;k<10;k++)
            {
              maxLevel=max(maxLevel,robot->mics[j][k]);
              minLevel=min(minLevel,robot->mics[j][k]);
            }
            maxLevel=maxLevel-minLevel;
            byte* intToBytes=(byte*)&maxLevel;
            command_.outputData[2*j]=intToBytes[0];
            command_.outputData[2*j+1]=intToBytes[1];
          }
        }
        break;
      case SET_MIC_INPUT:
        if(command_.nInputData!=1)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=0;
          switch (command_.inputData[0])
          {
            case 0:
              robot->setMic0();
              break;
            case 1:
              robot->setMic1();
              break;
            case 2:
              robot->setMic2();
              break;
          }
        }
        break;
      case GET_HEAD_SERVOS:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=4;
          int servoPosition=robot->getServoPosition(0);
          byte* intToBytes=(byte*)&servoPosition;
          command_.outputData[0]=intToBytes[0];
          command_.outputData[1]=intToBytes[1];
          servoPosition=robot->getServoPosition(1);
          intToBytes=(byte*)&servoPosition;
          command_.outputData[2]=intToBytes[0];
          command_.outputData[3]=intToBytes[1];
        }
        //sin datos de entrada
        break;
      case GET_EYE_SERVOS:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=4;
          int servoPosition=robot->getServoPosition(2);
          byte* intToBytes=(byte*)&servoPosition;
          command_.outputData[0]=intToBytes[0];
          command_.outputData[1]=intToBytes[1];
          servoPosition=robot->getServoPosition(3);
          intToBytes=(byte*)&servoPosition;
          command_.outputData[2]=intToBytes[0];
          command_.outputData[3]=intToBytes[1];
        }
        //sin datos de entrada
        break;
      case GET_SERVO:
        if(command_.nInputData!=1)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=2;
          unsigned int servoPosition=0;
          switch (command_.inputData[0])
          {
            case 1:
              servoPosition=(unsigned int)(robot->getServoPosition(0));
              break;
            case 2:
              servoPosition=(unsigned int)(robot->getServoPosition(1));
              break;
            case 3:
              servoPosition=(unsigned int)(robot->getServoPosition(2));
              break;
            case 4:
              servoPosition=(unsigned int)(robot->getServoPosition(3));
              break;
          }
          byte* intToBytes=(byte*)&servoPosition;
          command_.outputData[0]=intToBytes[0];
          command_.outputData[1]=intToBytes[1];
        }
        //sin datos de entrada
        break;
      default:
        command_.nOutputData=0;
        isInputCorrect_=false;
        break;
    }
}

void SerialProtocol::sendResponse()
{
  if(isInputCorrect_)
  {
    byte temporalDataArray[128];
    byte outputData[128];
    byte nOutputData=0;
    outputData[nOutputData]=INPUT_FLAG;
    nOutputData++;
    
    temporalDataArray[0]=command_.commandNumber;
    temporalDataArray[1]=command_.nOutputData;
    
    for(int i=0;i<command_.nOutputData;i++)
      temporalDataArray[i+2]=command_.outputData[i];
      
    uint8_t check=pearson(temporalDataArray,command_.nOutputData+2);
    temporalDataArray[command_.nOutputData+2]=check;
  
    for(int i=0;i<command_.nOutputData+3;i++)
    {
      if (temporalDataArray[i]==INPUT_FLAG||temporalDataArray[i]==INPUT_ESCAPE||temporalDataArray[i]==OUTPUT_FLAG)
      {
        outputData[nOutputData]=INPUT_ESCAPE;
        nOutputData++;
        outputData[nOutputData]=temporalDataArray[i]-2;
        nOutputData++;
      }
      else
      {
        outputData[nOutputData]=temporalDataArray[i];
        nOutputData++;
      }
    }
    
    outputData[nOutputData]=OUTPUT_FLAG;
    nOutputData++;
    //ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
      Serial.flush();
      Serial.write(outputData,nOutputData);
    }
  }
  else
  {
    sendNack();
    isInputCorrect_=true;
  }
}
void SerialProtocol::sendNack()
{
  byte outputData[2]={INPUT_FLAG,OUTPUT_FLAG};
  //ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    Serial.flush();
    Serial.write(outputData,2);
  }
}
