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

#ifndef ARDUQBO_INSTRUCTIONS_H
#define ARDUQBO_INSTRUCTIONS_H

#include "dataUnion.h"
#include <string>
#include <vector>
#include <stdint.h>

class CComando
{
    public:
        CComando(uint8_t number, int outDataLen, int inDataLen, std::string outType="b", std::string inType="b");
        int serialize(std::vector<dataUnion> outData, std::string& serializedData);
        int deserialize(std::string inData, std::vector<dataUnion>& recivedData);
    protected:
        int calcsize(std::string& type);
        uint8_t number_;
        int outDataLen_;
        int inDataLen_;
        std::string outType_;
        std::string inType_;
};

class ComandosSet
{
    public:
        CComando version;
        CComando setSpeed;
        CComando getSpeed;
        CComando setOdometry;
        CComando getOdometry;
        CComando wheelMovement;
        CComando floorDistance;
        CComando battery;
        CComando baseInfraRed;
        CComando setEncoders;
        CComando getEncoders;
        CComando distanceSensors;
        CComando distanceSensor;
        CComando setAutoupdateSensors;
        CComando adcReads;
        CComando lcd;
        CComando pid;
        CComando pidAdjustment;
        CComando srfContinuousUpdate;
        CComando powerOff;
        CComando mouth;
        CComando nose;
        CComando setServo;
        CComando getServo;
        CComando getHeadServos;
        CComando getEyeServos;
        CComando setMic;
        CComando getMics;
        CComando getIMU;
        CComando resetStall;
        CComando getMotorsState;
        CComando getI2cDevicesState;
        ComandosSet() : version(0x40,0,2), setSpeed(0x4d,2,0,"f"), getSpeed(0x56,0,2,"b","f"),
            setOdometry(0x42,3,0,"f"), getOdometry(0x59,0,3,"b","f"), wheelMovement(0x51,0,2,"b","f"),
            floorDistance(0x48,0,1,"b","f"), battery(0x57,0,2), baseInfraRed(0x46,0,3), setEncoders(0x70,2,0,"l"),
            getEncoders(0x4f,0,2,"b","l"), distanceSensors(0x4e,0,-1,"b","xbh"), distanceSensor(0x52,1,1,"b","h"),
            setAutoupdateSensors(0x72,-1,0), adcReads(0x73,-1,-1,"b","h"), lcd(0x4c,1,0,"s"), pid(0x50,1,0),
            pidAdjustment(0x71,4,0,"f"), srfContinuousUpdate(0x5d,1,0), powerOff(0x5a,1,0), mouth(0x44,3,0), nose(0x45,1,0),
            setServo(0x53,3,0,"bhh"), getServo(0x5d,1,1,"b","h"), getHeadServos(0x5c,0,2,"b","h"), getEyeServos(0x5f,0,2,"b","h"),
            setMic(0x4a,1,0), getMics(0x4b,0,3,"b","h"), getIMU(0x74,0,6,"b","hhhbbb"), resetStall(0x80,0,0), getMotorsState(0x81,0,1),
            getI2cDevicesState(0x82,0,1)
        {}
};

#endif
