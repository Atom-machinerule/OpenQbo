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
      case ALL_SRF_UPDATE:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          //sin valor de retorno
          command_.nOutputData=0;
          for (int j=0;j<robot->NUM_SRFs;j++)
          {
            robot->updateSrfRange(robot->SRFs_ADRESS[j]);
            delay(69);  //To avoid sound collision
          }
        }
        //sin datos de entrada
        break;
      case CHANGE_I2C_DIR:
        if(command_.nInputData!=2)
        {
          isInputCorrect_=false;
        }
        else
        {
          //sin valor de retorno
          //2 valores de entrada
          robot->changeSrfAddress(int(command_.inputData[0]>>1),int(command_.inputData[1]));
          command_.nOutputData=0;
        }
        break;
      
      case GET_BASE_INFRARED:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          //reservamos para 3 valor de retorno
          command_.nOutputData=3;
          command_.outputData[0]=robot->ir1data;
          command_.outputData[0]=robot->ir2data;
          command_.outputData[0]=robot->ir3data;
          robot->ir1data=0;
          robot->ir2data=0;
          robot->ir3data=0;
          //sin datos de entrada
        }
        break;
      
      case GET_I2C:
        if(command_.nInputData!=2)
        {
          isInputCorrect_=false;
        }
        else
        {
          /*
          I2c.beginTransmission(int(command_.inputData[0]>>1));
          I2c.send(int(command_.inputData[1]));                           // Call register to get light reading
          I2c.endTransmission();
          
          I2c.requestFrom(int(command_.inputData[0]>>1), 1);                // Request 1 byte
          */
          I2c.read(int(command_.inputData[0]>>1), int(command_.inputData[1]), 1);
          //while(I2c.available() < 0);
          byte registro = byte(I2c.receive());
          command_.nOutputData=1;
          command_.outputData[0]=registro;
        }
        //2 valores de entrada
        break;
      case GET_I2C_STATE:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=1;
          command_.outputData[0]=robot->energyState;
          command_.outputData[0]=(command_.outputData[0]<<1) + robot->lcdState;
          command_.outputData[0]=(command_.outputData[0]<<1) + robot->accelerometerState;
          command_.outputData[0]=(command_.outputData[0]<<1) + robot->gyroState;
        }
        //2 valores de entrada
        break;
      case RESET_STALL:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          //sin valor de retorno
          command_.nOutputData=0;
          robot->setSpeeds(0.0, 0.0);
          robot->par_motores.leftStallCount=0;
          robot->par_motores.rightStallCount=0;
          robot->par_motores.leftStallDetected=false;
          robot->par_motores.rightStallDetected=false;
        }
        //2 valores de entrada
        break;
      case GET_MOTORS_STATE:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=1;
          command_.outputData[0]=!robot->par_motores.leftStallDetected;
          command_.outputData[0]=(command_.outputData[0]<<1) + !robot->par_motores.rightStallDetected;
        }
        //2 valores de entrada
        break;
      case GET_GP2D12:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=4;
          float floorDistance=robot->getFloorDistance();
          byte* floatToBytes=(byte*)&floorDistance;
          command_.outputData[0]=floatToBytes[0];
          command_.outputData[1]=floatToBytes[1];
          command_.outputData[2]=floatToBytes[2];
          command_.outputData[3]=floatToBytes[3];
        }
        //sin datos de entrada
        break;
      case SRF_UPDATE:
        if(command_.nInputData<1||command_.nInputData>4)
        {
          isInputCorrect_=false;
        }
        else
        {
          if(command_.nInputData>2)  //El rango es un int que son 2 bytes y va desde 43 hasta 11008 que es la distancia en milímetros
          {
            int range=*((int *)command_.inputData+1);
            if(range<43) range=43;
            if(range>11008) range=11008;
            range=(range-43)/43;
            robot->changeSrfDistance(int(command_.inputData[0]>>1), range);
          }
          if(command_.nInputData==4)
          {
            int gain=command_.inputData[3];
            if(gain<0) gain=0;
            if(gain>16) gain=16;
            robot->changeSrfGain(int(command_.inputData[0]>>1), gain);
          }
          robot->updateSrfRange(int(command_.inputData[0]>>1));
          command_.nOutputData=0;
        }
        break;
      case SET_LCD:
        if(command_.nInputData==0)
        {
          isInputCorrect_=false;
        }
        else
        {
          if(command_.nInputData>20/*49*/)
            command_.nInputData=20;//49;
          uint8_t line=command_.inputData[0];
          if(line=='1')
            line=1;
          else if(line=='2')
            line=2;
          else if(line=='3')
            line=3;
          else
            line=0;
          robot->lcd.setCursor(line,0);
          robot->lcd.print("                    ");
          robot->lcd.setCursor(line,0);
          for(int k=line;k<command_.nInputData+line;k++)
          {
            uint8_t letter=command_.inputData[k];
            robot->lcd.command(letter);
            
          }
          //sin valor de retorno
          //cadena de texto como entrada
          command_.nOutputData=0;
        }
        break;
      case SET_MOTOR:
        if(command_.nInputData!=8)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=0;
          float linealSpeed=*((float *)command_.inputData);  //Lineal
          float angularSpeed=*((float *)(command_.inputData+4));  //Angular
          robot->setSpeeds(linealSpeed,angularSpeed);  //motor_1 es la rueda derecha y motor_2 la izquierda
        }
        //Colocar velocidad deseada
        //sin valor de retorno
        //8 valores de entrada
        break;
        /*
      case SET_MOTOR_EASY:
        if(command_.nInputData!=4)
        {
          isInputCorrect_=false;
          break;
        }
        command_.nOutputData=0;
        {
          robot->setSpeedsEasy(command_.inputData[0],command_.inputData[1],command_.inputData[2],command_.inputData[3]);
        }
        //Colocar velocidad deseada
        //sin valor de retorno
        //8 valores de entrada
        break;
        */
      case GET_ALL_SRF:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          //reservamos para 4 valores de retorno
          command_.nOutputData=3*robot->NUM_SRFs;
          for (int j=0;j<robot->NUM_SRFs;j++)
          {
            unsigned int range=robot->getSrfRange(robot->SRFs_ADRESS[j]);
            byte* intToBytes=(byte*)&range;
            command_.outputData[3*j]=(byte(robot->SRFs_ADRESS[j]))<<1;
            command_.outputData[3*j+1]=intToBytes[0];
            command_.outputData[3*j+2]=intToBytes[1];
          }
        }
        //sin datos de entrada
        break;
      case GET_SRF:
        if(command_.nInputData!=1)
        {
          isInputCorrect_=false;
        }
        else
        {
          //reservamos para 2 valores de retorno
          command_.nOutputData=2;
          {
            //unsigned int range=robot->getSrfRange(command_.inputData[0]>>1);
            unsigned int range=robot->SRFs_VALUES[(command_.inputData[0]>>1)-112];
            byte* intToBytes=(byte*)&range;
            command_.outputData[0]=intToBytes[0];
            command_.outputData[1]=intToBytes[1];
          }
        }
        //Un valor de entrada
        break;
      case SET_AUTOUPDATE_SRFS:
        if(command_.nInputData>8 || (command_.nInputData%2)!=0)  //el command_ es [direcc0,posicion0][direcc1,posicion1][direcc2,posicion2][direcc3,posicion3]
        {
          isInputCorrect_=false;
        }
        else
        {
          //reservamos para 2 valores de retorno
          command_.nOutputData=0;
          byte frontSrfCount=0;
          byte backSrfCount=0;
          for (int j=0;j<(command_.nInputData/2);j++)
          {
            int address=int(*((byte*)(command_.inputData+2*j)))>>1;
            byte location=(*((byte*)(command_.inputData+2*j+1)));
            isInputCorrect_=false;
            for (int k=0;k<robot->NUM_SRFs;k++)
            {
              if(address==robot->SRFs_ADRESS[k])
                isInputCorrect_=true;
            }
            if (!isInputCorrect_) break;  //Si la direccion no esta en las que se han autodetectado al inicio salimos con fallo
            if(location==0) //back
            {
              robot->SRFs_BACK[backSrfCount]=address;
              backSrfCount++;
            }
            if(location==1) //front
            {
              robot->SRFs_FRONT[frontSrfCount]=address;
              frontSrfCount++;
            }
          }
          robot->NUM_SRFs_FRONT=frontSrfCount;
          robot->NUM_SRFs_BACK=backSrfCount;
        }
        //Un valor de entrada
        break;
      case SET_K:
        if(command_.nInputData!=5)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=0;
          byte kValue=command_.inputData[0];
          float value=*((float *)(command_.inputData+1));
          robot->setK(kValue,value);
        }
        //sin valor de retorno
        //3 valores de entrada
        break;
      case TEST:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=(robot->NUM_SRFs<<1);  //Numero de srfs*2
          for(byte j=0;j<robot->NUM_SRFs;j++)
          {
            command_.outputData[j<<1]=byte(robot->getSrfRange(robot->SRFs_ADRESS[j])>>8);
            command_.outputData[(j<<1)+1]=byte(robot->getSrfRange(robot->SRFs_ADRESS[j])&0xFF);
          }
        }
        //especial
        //sin datos de entrada
        
        break;
      case SET_I2C:
        if(command_.nInputData!=3)
        {
          isInputCorrect_=false;
        }
        else
        {
          /*
          I2c.beginTransmission(int(command_.inputData[0]>>1));
          I2c.send(int(command_.inputData[1]));                           // Call register
          I2c.send(int(command_.inputData[2]));                           // Send data
          I2c.endTransmission();
          */
          I2c.write((uint8_t)(command_.inputData[0]>>1), (uint8_t)(command_.inputData[1]), (uint8_t)(command_.inputData[2]));
          command_.nOutputData=0;
        }
        break;
      case GET_BATTERY:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          //reservamos para 1 valor de retorno
          command_.nOutputData=2;
          byte value=0;
          byte stat=0;
          robot->getBatteryLevel(&value, &stat);
          command_.outputData[0]=value;
          command_.outputData[1]=stat;
        }
        //sin datos de entrada
        break;
      case I2C_EXTENDED:
        if(command_.nInputData<3)
        {
          isInputCorrect_=false;
        }
        else
        {
          uint8_t address=command_.inputData[0]>>1;
          uint8_t nBytesToWrite=command_.inputData[1];
          uint8_t nBytesToRead=command_.inputData[2];
          command_.nOutputData=nBytesToRead;
          if(nBytesToWrite>0)
          {
            /*
            I2c.beginTransmission(address);
            for(int j=0;j<nBytesToWrite;j++)
            {
              I2c.send(int(command_.inputData[3+j]));
            }
            I2c.endTransmission();
            */
            I2c.write(address,(uint8_t)command_.inputData[3],(uint8_t *)(command_.inputData+3),nBytesToWrite-1);
          }
          command_.nOutputData=nBytesToRead;
          /*
          I2c.requestFrom(address, nBytesToRead);                // Request bytesToRead bytes
          for(int j=0;j<nBytesToRead;j++)
          {
            command_.outputData[j]= byte(I2c.receive());
          }
          */
          I2c.read(address, nBytesToRead, (uint8_t *)command_.outputData);
        }
        //de 3 a N valores de entrada
        break;
      case GET_POSITION:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          //12 valores de retorno
          command_.nOutputData=12;
          double xCoordinate,yCoordinate,angle;
          robot->getSpacePosition(xCoordinate,yCoordinate,angle);
          byte* floatToBytes=(byte*)&xCoordinate;
          command_.outputData[0]=floatToBytes[0];
          command_.outputData[1]=floatToBytes[1];
          command_.outputData[2]=floatToBytes[2];
          command_.outputData[3]=floatToBytes[3];
          floatToBytes=(byte*)&yCoordinate;
          command_.outputData[4]=floatToBytes[0];
          command_.outputData[5]=floatToBytes[1];
          command_.outputData[6]=floatToBytes[2];
          command_.outputData[7]=floatToBytes[3];
          floatToBytes=(byte*)&angle;
          command_.outputData[8]=floatToBytes[0];
          command_.outputData[9]=floatToBytes[1];
          command_.outputData[10]=floatToBytes[2];
          command_.outputData[11]=floatToBytes[3];
        }
        //Un valor de entrada
        break;
      case RESET_ODOMETRY:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          robot->resetPosition();
          command_.nOutputData=0;
        }
        //Un valor de entrada
        break;
      case GET_ANALOG_PINS:
        if(command_.nInputData<1 || command_.nInputData>16)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=2*command_.nInputData;
          for (byte j=0;j<command_.nOutputData;j++)
          {
            unsigned int readedValue=0;
            if (command_.inputData[j]>=0 && command_.inputData[j]<16)
              readedValue=robot->adcRead(command_.inputData[j]);
            byte * intToBytes=(byte*)(&readedValue);
            command_.outputData[2*j]=intToBytes[0];
            command_.outputData[2*j+1]=intToBytes[1];
          }
        }
        //de 3 a N valores de entrada
        break;
      case GET_IMU:
        if(command_.nInputData!=0)
        {
          isInputCorrect_=false;
        }
        else
        {
          command_.nOutputData=9; //3 int16_t y 3 int8_t
        
          byte* shortToBytes=(byte*)&(robot->gyroX);
          command_.outputData[0]=shortToBytes[0];
          command_.outputData[1]=shortToBytes[1];
          shortToBytes=(byte*)&(robot->gyroY);
          command_.outputData[2]=shortToBytes[0];
          command_.outputData[3]=shortToBytes[1];
          shortToBytes=(byte*)&(robot->gyroZ);
          command_.outputData[4]=shortToBytes[0];
          command_.outputData[5]=shortToBytes[1];
          
          command_.outputData[6]=(byte)robot->accelerometerX;
          command_.outputData[7]=(byte)robot->accelerometerY;
          command_.outputData[8]=(byte)robot->accelerometerZ;
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
    ATOMIC_BLOCK(ATOMIC_FORCEON)
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
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    Serial.flush();
    Serial.write(outputData,2);
  }
}
