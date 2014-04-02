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

#include <driver/qboduino_driver.h>

CQboduinoDriver::CQboduinoDriver(std::string port1, int baud1, std::string port2, int baud2, float timeout1, float timeout2) :
    firstDevice(), secondDevice(), timeout1_(timeout1*1000), timeout2_(timeout2*1000)
{
    try{
    firstDevice.open(port1.c_str(),baud1);
    }
    catch (...){}

    try{
    secondDevice.open(port2.c_str(),baud2);
    }
    catch(...){}
    usleep(5500000);
    if(!firstDevice.portOpen()) {
      std::cout << "Unable to open " << port1 << " port" << std::endl;
    }
    else
    {
        int board_id=-1, version=-1;
        boards_["first"]=&firstDevice;
        timeouts_["first"]=&timeout1_;
        int code = getVersion("first", board_id, version);
        if(code>=0 && board_id==0)
        {
            boards_["base"]=&firstDevice;
            timeouts_["base"]=&timeout1_;
            std::cout << "Base control board fount at " << port1 << " and inicialiced at " << baud1 << " baudrate" << std::endl;
        }
        else if(code>=0 && board_id==1)
        {
            boards_["head"]=&firstDevice;
            timeouts_["head"]=&timeout1_;
            std::cout << "Head control board fount at " << port1 << " and inicialiced at " << baud1 << " baudrate" << std::endl;
        }
        else
        {
          std::cout << "Not QBO Board detected at " << port1 << std::endl;
        }
    }
    if(!secondDevice.portOpen()) {
      std::cout << "Unable to open " << port2 << " port" << std::endl;
    }
    else
    {
        int board_id=-1, version=-1;
        boards_["second"]=&secondDevice;
        timeouts_["second"]=&timeout2_;
        int code = getVersion("second", board_id, version);
        if(code>=0 && board_id==0)
        {
            boards_["base"]=&secondDevice;
            timeouts_["base"]=&timeout2_;
            std::cout << "Base control board fount at " << port2 << " and inicialiced at " << baud2 << " baudrate" << std::endl;
        }
        else if(code>=0 && board_id==1)
        {
            boards_["head"]=&secondDevice;
            timeouts_["head"]=&timeout2_;
            std::cout << "Head control board fount at " << port2 << " and inicialiced at " << baud2 << " baudrate" << std::endl;
        }
        else
        {
          std::cout << "Not QBO Board detected at " << port2 << std::endl;
        }
    }
    if(boards_.count("base")==0 && boards_.count("head")==0)
      exit(-1);
}

int CQboduinoDriver::read(cereal::CerealPort *ser, std::string& lectura, long timeout)
{
    std::string buf;
    int leidos=0;
    int totalLeidos=0;
    
    try
    {
        if(!ser->readBetween(&buf,0xFF,0XFE,timeout))
        {
            return -1;
        }
    }
    //TODO: Capturar excepcion de forma correcta
    catch(...)
    {
        return -1;
    }
    
    int code=processResponse((uint8_t*)buf.c_str(),buf.size(),lectura);
    switch(code)
    {
        case 1:
            break;
        case -1:
            lectura.clear();
            break;
        case -2:
            lectura.clear();
            break;
        case -3:
            lectura.clear();
            break;
        case -4:
            lectura.clear();
            break;
        default:
            lectura.clear();
            break;
    }
    return code;
}
        
int CQboduinoDriver::write(cereal::CerealPort *ser, std::string& escritura)
{
    std::string serialData;
    prepareData(escritura, serialData);
    return ser->write(serialData.c_str(),serialData.size());
}

int CQboduinoDriver::processResponse(uint8_t *buf, uint32_t length, std::string& lectura)
{
  lectura.clear();
  if (length<5) return -1;
  if(buf[0]!=INPUT_FLAG) return -2;
  if(buf[length-1]!=OUTPUT_FLAG) return -3;
  uint8_t data[128];
  bool escapeEntrada=false;
  int datosComando=0;
  for(uint32_t i=1;i<length-1;i++)
  {
    if(escapeEntrada)
    {
      data[datosComando]=buf[i]+2;
      escapeEntrada=false;
      datosComando++;
    }
    else if(buf[i]==INPUT_SCAPE)
      escapeEntrada=true;
    else
    {
      data[datosComando]=buf[i];
      escapeEntrada=false;
      datosComando++;
    }
  }
  uint8_t check=pearson(data,datosComando-1);
  uint8_t inCheck=data[datosComando-1];
  
  if(check!=inCheck){
    return -4;
  }
  for(int i=0;i<datosComando-1;i++)
    lectura.push_back(data[i]);
  return 1;
}

void CQboduinoDriver::prepareData(std::string& escritura, std::string& preparedData)
{
    preparedData.clear();
    preparedData.push_back(INPUT_FLAG);
        
    uint8_t check=pearson((uint8_t *)escritura.c_str(),(uint8_t)escritura.size());
    escritura.push_back((char)check);
    
    for(unsigned int i=0;i<escritura.size();i++)
    {
      if((uint8_t)escritura[i]==INPUT_FLAG||(uint8_t)escritura[i]==INPUT_SCAPE||(uint8_t)escritura[i]==OUTPUT_FLAG)
      {
        preparedData.push_back(INPUT_SCAPE);
        preparedData.push_back(escritura[i]-2);
      }
      else
      {
        preparedData.push_back(escritura[i]);
      }
    }
    
    preparedData.push_back(OUTPUT_FLAG);
}

int CQboduinoDriver::lockAndSendComand(std::string board, CComando& comand, std::vector<dataUnion>& response, std::vector<dataUnion>& data)
{
    int code=-3;
    if(boards_.count(board)==0) return -2;
    if(board.compare("base")==0 && sending_data_mutex_.timed_lock(boost::posix_time::millisec(500)))
    {
        code=sendComand(board,comand,response,data);
        sending_data_mutex_.unlock();
        return code;
    }
    else
        code=-3;
    if(board.compare("head")==0 && sending_data_head_mutex_.timed_lock(boost::posix_time::millisec(500)))
    {
        code=sendComand(board,comand,response,data);
        sending_data_head_mutex_.unlock();
        return code;
    }
    else
        code=-3;
    std::cout << "Mutex timeout" << std::endl;
    return code;
}

int CQboduinoDriver::sendComand(std::string board, CComando& comand, std::vector<dataUnion>& response, std::vector<dataUnion>& data)
{
    response.clear();
    cereal::CerealPort *ser=boards_[board];
    long timeout=*timeouts_[board];
    ser->flush();
    std::string oud;
    if(comand.serialize(data, oud)<0)
        return -4;
    if(!write(ser,oud))
        return -5;
    std::string ind;
    if(!read(ser,ind,timeout))
        return -6;
    if(comand.deserialize(ind, response)<0)
    {
        return -7;
    }
    return 1;
}
    
int CQboduinoDriver::getVersion(std::string board, int& board_number, int& version)
{
    CComando comand=comandosSet_.version;
    std::vector<dataUnion> data, sent;
    int code=sendComand(board,comand,data,sent);
    if (code<0) return code;
    board_number=(int)data[0].b;
    version=(int)data[1].b;
    return code;
}
    
int CQboduinoDriver::setSpeed(float linear, float angular)
{
    dataUnion d;
    std::vector<dataUnion> data, resp;
    d.f=linear;
    data.push_back(d);
    d.f=angular;
    data.push_back(d);
    CComando comand=comandosSet_.setSpeed;
    return (lockAndSendComand("base",comand,resp,data));
}

int CQboduinoDriver::setServo(uint8_t idx,unsigned short tics, unsigned short tics_per_second)
{
    dataUnion d;
    std::vector<dataUnion> data, resp;
    d.b=idx;
    data.push_back(d);
    d.h=tics;
    data.push_back(d);
    d.h=tics_per_second;
    data.push_back(d);
    CComando comand=comandosSet_.setServo;
    return (lockAndSendComand("head",comand,resp,data));
}
    
int CQboduinoDriver::getOdometry(float& x, float& y, float& th)
{
    std::vector<dataUnion> data, sent;
    CComando comand=comandosSet_.getOdometry;
    int code=lockAndSendComand("base",comand,data,sent);
    if (code<0) return code;
    x=data[0].f;
    y=data[1].f;
    th=data[2].f;
    return code;
}

int CQboduinoDriver::getServoPosition(uint8_t idx, unsigned short& tics)
{
    dataUnion d;
    std::vector<dataUnion> data,sent;
    d.b= idx;
    sent.push_back(d);
    CComando comand=comandosSet_.getServo;
    int code=lockAndSendComand("head",comand,data,sent);
    if (code<0) return code;
    tics=data[0].h;
    return code;
}
int CQboduinoDriver::getHeadServosPositions(std::vector<unsigned short>& tics)
{
    dataUnion d;
    std::vector<dataUnion> data,sent;
    CComando comand=comandosSet_.getHeadServos;
    int code=lockAndSendComand("head",comand,data,sent);
    if (code<0) return code;
    tics.push_back(data[0].h);
    tics.push_back(data[1].h);
    return code;
}
int CQboduinoDriver::getEyesServosPositions(std::vector<unsigned short>& tics)
{
    dataUnion d;
    std::vector<dataUnion> data,sent;
    CComando comand=comandosSet_.getEyeServos;
    int code=lockAndSendComand("head",comand,data,sent);
    if (code<0) return code;
    tics.push_back(data[0].h);
    tics.push_back(data[1].h);
    return code;
}
int CQboduinoDriver::setMouth(uint8_t b0, uint8_t b1, uint8_t b2)
{
    dataUnion d;
    std::vector<dataUnion> data,resp;
    d.b= b0;
    data.push_back(d);
    d.b= b1;
    data.push_back(d);
    d.b= b2;
    data.push_back(d);
    
    CComando comand=comandosSet_.mouth;
    return (lockAndSendComand("head",comand,resp,data));
}

int CQboduinoDriver::setNose(uint8_t nose)
{
      dataUnion d;
      std::vector<dataUnion> data,resp;
      d.b= nose;
      data.push_back(d);

      CComando comand=comandosSet_.nose;
      return (lockAndSendComand("head",comand,resp,data));
}

int CQboduinoDriver::setLCD(std::string msg)
{
    dataUnion d;
    std::vector<dataUnion> data,resp;
    d.s= msg;
    data.push_back(d);
    
    CComando comand=comandosSet_.lcd;
    return (lockAndSendComand("base",comand,resp,data));
}

int CQboduinoDriver::getBattery(float& level, uint8_t& stat)
{
    std::vector<dataUnion> data,sent;
    
    CComando comand=comandosSet_.battery;
    int code=lockAndSendComand("base",comand,data,sent);
    if (code<0) return code;
    level=((float)data[0].b);
    stat=((uint8_t)data[1].b);
    return code;
}

int CQboduinoDriver::getMics(uint16_t& m0,uint16_t& m1,uint16_t& m2)
{
    std::vector<dataUnion> data,sent;
    
    CComando comand=comandosSet_.getMics;
    int code=lockAndSendComand("head",comand,data,sent);
    if (code<0) return code;
    m0=(uint16_t)data[0].h;
    m1=(uint16_t)data[1].h;
    m2=(uint16_t)data[2].h;
    return code;
}

int CQboduinoDriver::setMic(uint8_t mic)
{
    dataUnion d;
    std::vector<dataUnion> data,resp;
    d.b=mic;
    data.push_back(d);
    
    CComando comand=comandosSet_.setMic;
    return (lockAndSendComand("head",comand,resp,data));
}

int CQboduinoDriver::setAutoupdateSensors(std::map<uint8_t,uint8_t> sensors)
{
    dataUnion d;
    std::vector<dataUnion> data,resp;
    std::map< uint8_t,uint8_t >::iterator it;
    for(it=sensors.begin();it!=sensors.end();it++)
    {
      d.b=(*it).first;
      data.push_back(d);
      d.b=(*it).second;
      data.push_back(d);
    }
    
    CComando comand=comandosSet_.setAutoupdateSensors;
    return (lockAndSendComand("base",comand,resp,data));
}
int CQboduinoDriver::getDistanceSensors(std::map<uint8_t,unsigned short>& sensorsDistances)
{
    dataUnion d;
    std::vector<dataUnion> data,sent;
    sensorsDistances.clear();
    
    CComando comand=comandosSet_.distanceSensors;
    int code=lockAndSendComand("base",comand,data,sent);
    if (code<0) return code;
    for (unsigned int i=0;i<data.size()/2;i++)
    {
      sensorsDistances[data[2*i].b]=data[2*i+1].h;
    }
    return code;
}

int CQboduinoDriver::getAdcReads(std::vector<uint8_t> addreses, std::vector<unsigned int>& readedValues)
{
    dataUnion d;
    std::vector<dataUnion> recived,sent;
    readedValues.clear();
    if(addreses.size()==0)
      return 1;
    for(int i=0;i<addreses.size();i++)
    {
      d.b=addreses[i];
      sent.push_back(d);
    }
    
    CComando comand=comandosSet_.adcReads;
    int code=lockAndSendComand("base",comand,recived,sent);
    if (code<0) return code;
    for (unsigned int i=0;i<recived.size();i++)
    {
      readedValues.push_back(recived[i].h);
    }
    return code;
  
}

int CQboduinoDriver::getIMU(int16_t& gyroX,int16_t& gyroY,int16_t& gyroZ,int8_t& accelerometerX,int8_t& accelerometerY,int8_t& accelerometerZ)
{
    std::vector<dataUnion> data,sent;

    CComando comand=comandosSet_.getIMU;
    int code=lockAndSendComand("base",comand,data,sent);
    if (code<0) return code;
    gyroX=(int16_t)data[0].h;
    gyroY=(int16_t)data[1].h;
    gyroZ=(int16_t)data[2].h;
    accelerometerX=(int8_t)data[3].b;
    accelerometerY=(int8_t)data[4].b;
    accelerometerZ=(int8_t)data[5].b;
    return code;
}

int CQboduinoDriver::resetStall()
{
    std::vector<dataUnion> data,sent;

    CComando comand=comandosSet_.resetStall;
    int code=lockAndSendComand("base",comand,data,sent);
    return code;
}

int CQboduinoDriver::getMotorsState(uint8_t& state)
{
    std::vector<dataUnion> data,sent;

    CComando comand=comandosSet_.getMotorsState;
    int code=lockAndSendComand("base",comand,data,sent);
    if (code<0) return code;
    state=(uint8_t)data[0].b;
    return code;
}

int CQboduinoDriver::getIRs(uint8_t& ir0,uint8_t& ir1,uint8_t& ir2)
{
    std::vector<dataUnion> data,sent;

    CComando comand=comandosSet_.baseInfraRed;
    int code=lockAndSendComand("base",comand,data,sent);
    if (code<0) return code;
    ir0=(uint16_t)data[0].b;
    ir1=(uint16_t)data[1].b;
    ir2=(uint16_t)data[2].b;
    return code;
}

int CQboduinoDriver::getI2cDevicesState(uint8_t& state)
{
    std::vector<dataUnion> data,sent;

    CComando comand=comandosSet_.getI2cDevicesState;
    int code=lockAndSendComand("base",comand,data,sent);
    if (code<0) return code;
    state=(uint16_t)data[0].b;
    return code;
}

uint8_t pearson(uint8_t *key, uint8_t len)
{
  uint8_t hash=0;
  for (uint8_t i=0; i<len; i++)
  {
    hash = pearsondata[hash^key[i]];
  }
  return (hash);
}
