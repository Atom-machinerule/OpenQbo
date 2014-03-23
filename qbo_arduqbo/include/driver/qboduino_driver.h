/*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2012 Thecorpora, Inc.
 *
 * This program is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
 * MA 02110-1301, USA.
 *
 * Authors: Miguel Angel Julian <miguel.a.j@openqbo.org>;
 * 
 */

#ifndef QBODUINO_DRIVER_H
#define QBODUINO_DRIVER_H

#include <cereal_port/CerealPort.h>
#include "arduqbo_instructions.h"
#include "dataUnion.h"
#include <sstream>
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <boost/thread/mutex.hpp>

//! Checksum calculation variable.
/*!
  This variable is used in the message checksum calculation function.
*/
const uint8_t pearsondata[] = {
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

//!  Message checksum calculation function. 
/*!
  \param key a pointer to the byte array message.
  \param len a byte indicating the byte array message length.
  \return The byte array message checksum
*/
uint8_t pearson(uint8_t *key, uint8_t len);
    
const uint8_t INPUT_FLAG=0xFF;  /*!< This variable is the value of the byte that indicates the start of a message */
const uint8_t OUTPUT_FLAG=0xFE; /*!< This variable is the value of the byte that indicates the end of a message */
const uint8_t INPUT_SCAPE=0xFD; /*!< This variable is the value of the byte that indicates that the next byte is escaped */

//!  The driver class. 
/*!
  This is the driver class where all the functions are defined.
  Only one instance of the class can exist in the program.
  It offers a simple set of functions to control the robot and its sensors.
  It communicates with the Q.board1 and Q.board2 though two serial ports.
*/
class CQboduinoDriver
{
public:
    //! Class constructor.
    /*!
      The constructor opens the needed serial ports and detects the Q.boards attached to them.
      \param port1 an string indicating one of the serial ports were the Q.boards are connected.
      \param baud1 an integer indicating the baud rate of the first serial port.
      \param port2 an string indicating the second of the serial ports were the Q.boards are connected.
      \param baud2 an integer indicating the baud rate of the second serial port.
      \param timeout1 a float indicating the time that the driver waits for the Q.board connected to the first serial port to respond to a command .
      \param timeout2 a float indicating the time that the driver waits for the Q.board connected to the second serial port to respond to a command .
    */
    CQboduinoDriver(std::string port1="/dev/ttyUSB0", int baud1=115200, std::string port2="/dev/ttyUSB1", int baud2=115200, float timeout1=0.01, float timeout2=0.01);
    //Public functions
    int getVersion(std::string board, int& board_number, int& version);
    int setSpeed(float linear, float angular);
    int setServo(uint8_t idx,unsigned short tics, unsigned short tics_per_second=1800);
    int getOdometry(float& x, float& y, float& th);
    int getServoPosition(uint8_t idx, unsigned short& tics);
    int getHeadServosPositions(std::vector<unsigned short>& tics);
    int getEyesServosPositions(std::vector<unsigned short>& tics);
    int setMouth(uint8_t b0, uint8_t b1, uint8_t b2);
    int setNose(uint8_t color);
    int setLCD(std::string msg);
    int getBattery(float& level, uint8_t& stat);
    int getMics(uint16_t& m0,uint16_t& m1,uint16_t& m2);
    int setMic(uint8_t mic);
    int setAutoupdateSensors(std::map<uint8_t,uint8_t> sensors);
    int getDistanceSensors(std::map<uint8_t,unsigned short>& sensorsDistances);
    int getAdcReads(std::vector<uint8_t> addreses, std::vector<unsigned int>& readedValues);
    int getIMU(int16_t& gyroX,int16_t& gyroY,int16_t& gyroZ,int8_t& accelerometerX,int8_t& accelerometerY,int8_t& accelerometerZ);
    int resetStall();
    int getMotorsState(uint8_t& state);
    int getIRs(uint8_t& ir0,uint8_t& ir1,uint8_t& ir2);
    int getI2cDevicesState(uint8_t& state);

protected:

  cereal::CerealPort firstDevice;
  cereal::CerealPort secondDevice;

  long timeout1_;
  long timeout2_;
  
  boost::timed_mutex sending_data_mutex_;
  boost::timed_mutex sending_data_head_mutex_;
  
  ComandosSet comandosSet_;
  
  std::map<std::string,cereal::CerealPort *> boards_;
  std::map<std::string,long *> timeouts_;

  int read(cereal::CerealPort *ser, std::string& lectura, long timeout);
  int write(cereal::CerealPort *ser, std::string& escritura);
  int processResponse(uint8_t *buf, uint32_t length, std::string& lectura);
  void prepareData(std::string& escritura, std::string& preparedData);
  int lockAndSendComand(std::string board, CComando& comand, std::vector<dataUnion>& response, std::vector<dataUnion>& data);
  int sendComand(std::string board, CComando& comand, std::vector<dataUnion>& response, std::vector<dataUnion>& data);
};

#endif
