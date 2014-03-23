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

#include <driver/arduqbo_instructions.h>

//TODO: Comentar el código

CComando::CComando(uint8_t number, int outDataLen, int inDataLen, std::string outType, std::string inType) :
    number_(number), outDataLen_(outDataLen), inDataLen_(inDataLen), outType_(outType), inType_(inType)
{
}

int CComando::calcsize(std::string& type)
{
    int size=0;
    for (unsigned int i=0;i<type.size();i++)
    {
        switch(type[i])
        {
            case 'b':
                size+=sizeof(uint8_t);
                break;
            case 'h':
                size+=sizeof(unsigned short);
                break;
            case 'f':
                size+=sizeof(float);
                break;
            case 'l':
                size+=sizeof(long);
                break;
            case 's':
                size+=sizeof(char);
                break;
        }
    }
    return size;
}

int CComando::serialize(std::vector<dataUnion> outData, std::string& serializedData)
{
    if (outDataLen_!=-1 && (int)outData.size()!=outDataLen_)
        return -1;
    serializedData.clear();
    serializedData.push_back(number_);
    if(outDataLen_==0)
        serializedData.push_back(0x00);
    else
    {
        int outDataLen=outDataLen_;
        if(outDataLen_==-1)
            outDataLen=(int)outData.size();
        std::string outType;
        if(outType_.size()==1)
        {
            for(int i=0;i<outDataLen;i++)
                outType+=outType_;
        }
        else
        {
            if(outType_[0]!='x' && (int)outType_.size()!=outDataLen)
                return -1;
            std::string type;
            if(outType_[0]!='x')
            {
                type=outType_;
            }
            else
            {
                std::string xType(outType_);
                xType.erase(0,1);
                for(int i=0;i<(outDataLen/calcsize(xType));i++)
                    type+=xType;
            }
            outType=type;
        }
        if(outDataLen!=outData.size()) return -1;
        std::string sd;
        uint8_t *data_p;
        for (int i=0;i<outDataLen;i++)
        {
            switch(outType[i])
            {
                case 'b':
                    sd.push_back(outData[i].b);
                    break;
                case 'h':
                    data_p=(uint8_t *)(&outData[i].h);
                    sd.push_back(data_p[0]);
                    sd.push_back(data_p[1]);
                    break;
                case 'f':
                    data_p=(uint8_t *)(&outData[i].f);
                    sd.push_back(data_p[0]);
                    sd.push_back(data_p[1]);
                    sd.push_back(data_p[2]);
                    sd.push_back(data_p[3]);
                    break;
                case 'l':
                    data_p=(uint8_t *)(&outData[i].l);
                    sd.push_back(data_p[0]);
                    sd.push_back(data_p[1]);
                    sd.push_back(data_p[2]);
                    sd.push_back(data_p[3]);
                    break;
                case 's':
                    sd+=outData[i].s;
                    break;
            }
        }
        serializedData.push_back(sd.size());
        serializedData+=sd;
    }
    return 1;
}

//TODO: Revisar éste método que a veces da error
int CComando::deserialize(std::string inData, std::vector<dataUnion>& recivedData)
{
    recivedData.clear();
    uint8_t cn=static_cast<uint8_t>(inData[0]);
    uint8_t idl=static_cast<uint8_t>(inData[1]);
    
    int inDataLen=inDataLen_;
    if(inDataLen==-1)
        inDataLen=idl/calcsize(inType_);
    
    std::string inType;
    if(inType_.size()==1)
    {
        for(int i=0;i<inDataLen;i++)
            inType+=inType_;
    }
    else
    {
        if(inType_[0]!='x' && (int)inType_.size()!=inDataLen)
            return -1;
        std::string type;
        if(inType_[0]!='x')
        {
            type=inType_;
        }
        else
        {
            std::string xType(inType_);
            xType.erase(0,1);
            for(int i=0;i<inDataLen;i++)
                type+=xType;
            inDataLen=type.size();
        }
        inType=type;
    }
    
    if(cn!=number_ || idl!=calcsize(inType))
        return -1;
    if (inDataLen==0)
        return 1;
    dataUnion data;
    uint8_t readed=2;
    uint8_t* data_p=(uint8_t *)(inData.c_str());
    for (int i=0;i<inDataLen;i++)
    {
        switch(inType[i])
        {
            case 'b':
                data.b=static_cast<uint8_t>(inData[readed]);
                readed+=sizeof(uint8_t);
                break;
            case 'h':
                data.h=*((short *)(data_p+readed));
                readed+=sizeof(short);
                break;
            case 'f':
                data.f=*((float *)(data_p+readed));
                readed+=sizeof(float);
                break;
            case 'l':
                data.l=*((long *)(data_p+readed));
                readed+=sizeof(long);
                break;
            case 's':
                data.s=std::string((char *)(data_p+readed));
                readed+=data.s.size();
                break;
        }
        recivedData.push_back(data);
    }
    return 1;
}
