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

#include <controllers/srf10_controller.h>
#include "ros/ros.h"
#include <ros/console.h>
#include <map>
#include <vector>
#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/ros/conversions.h>
#include <myXmlRpc.h>
#include <pthread.h>

CDistanceSensor::CDistanceSensor(std::string name, uint8_t address, std::string topic, ros::NodeHandle& nh, std::string type, std::string frame_id, float min_alert_distance, float max_alert_distance) :
    name_(name), address_(address), nh_(nh), type_(type), min_alert_distance_(min_alert_distance), max_alert_distance_(max_alert_distance), alert_(false)
{
    cloud_.points.resize(1);
    cloud_.channels.resize(1);
    //cloud_.set_points_size(1);
    //cloud_.set_channels_size(0);
    cloud_.header.frame_id=frame_id;
    cloud_.points[0].x=0;
    cloud_.points[0].y=0;
    cloud_.points[0].z=0;
    //sensor_pub_ = nh.advertise<pcl::PointXYZ>(topic, 1);
    //sensor_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic, 1);
    sensor_pub_ = nh.advertise<sensor_msgs::PointCloud>(topic, 1);
    base_stop_service_client_ = nh.serviceClient<qbo_arduqbo::BaseStop>("stop_base");
}

void CDistanceSensor::publish(unsigned int readedValue, ros::Time time)
{
    float distance=0;
    if(type_.compare("srf10")==0)
    {
        distance=((float)readedValue)/100;
    }
    else if(type_.compare("gp2d120")==0)
    {
        distance=(2914 / ((float)readedValue + 5)) - 1;
    }
    else if(type_.compare("gp2d12")==0)
    {
        if (readedValue<3)
            distance=-1;
        else
            distance=(6787.0 /((float)readedValue - 3.0)) - 4.0;
    }
    else if(type_.compare("GP2Y0A21YK")==0)
    {
       distance = 12343.85 * pow((float)readedValue,-1.15);
    }
    bool send_stop;
    nh_.param("autostop", send_stop, false);
    //if(send_stop)
        //ROS_INFO("Alarm set");
    if((min_alert_distance_!=-1 && distance<=min_alert_distance_) && (max_alert_distance_!=-1 && distance>=max_alert_distance_))
    {
        this->setAlarm(true, distance);
    }
    if(min_alert_distance_!=-1 && distance<=min_alert_distance_)
    {
        this->setAlarm(true, distance);
    }
    else if(max_alert_distance_!=-1 && distance>=max_alert_distance_)
    {
        this->setAlarm(true, distance);
    }
    else
    {
        this->setAlarm(false);
    }
    cloud_.points[0].x=distance;
    cloud_.header.stamp=time;
    //pcl::toROSMsg(cloud_,msg_)
    sensor_pub_.publish(cloud_);
    //sensor_pub_.publish(msg_);
}

qbo_arduqbo::BaseStop CDistanceSensor::base_stop_service_msg_;
void * CDistanceSensor::serviceCallFunction( void *args )
{
   if(!ros::service::call("/qbo_arduqbo/stop_base", base_stop_service_msg_))
       ROS_ERROR("Unable to call service");
   return NULL;
}

void CDistanceSensor::setAlarm(bool state, float distance)
{
    bool send_stop;
    nh_.param("autostop", send_stop, false);
    if(send_stop)
    {
        if(state && !alert_)
        {
            ROS_INFO("Sensor: %s Alarm SET. Detected: %f Min limit: %f Max limit: %f", name_.c_str(),distance,min_alert_distance_,max_alert_distance_);
            alert_=true;
            if(base_stop_service_client_.exists())
            {
              pthread_t thread;
              base_stop_service_msg_.request.sender = name_;
              base_stop_service_msg_.request.state = true;
              pthread_create( &thread, NULL, &CDistanceSensor::serviceCallFunction, NULL);
            }
            else
            {
              ROS_INFO("No stop service available");
            }
        }
        else if(!state && alert_)
        {
            ROS_INFO("Sensor: %s Alarm UNSET", name_.c_str());
            alert_=false;
            qbo_arduqbo::BaseStop srv;
            if(base_stop_service_client_.exists())
            {
              pthread_t thread;
              base_stop_service_msg_.request.sender = name_;
              base_stop_service_msg_.request.state = false;
              pthread_create( &thread, NULL, &CDistanceSensor::serviceCallFunction, NULL);
            }
            else
            {
              ROS_INFO("No stop service available");
            }
        }
    }
    else if(!send_stop && alert_)
    {
      ROS_INFO("Sensor: %s Alarm UNSET", name_.c_str());
      alert_=false;
      qbo_arduqbo::BaseStop srv;
      if(base_stop_service_client_.exists())
      {
        pthread_t thread;
        base_stop_service_msg_.request.sender = name_;
        base_stop_service_msg_.request.state = false;
        pthread_create( &thread, NULL, &CDistanceSensor::serviceCallFunction, NULL);
      }
      else
      {
        ROS_INFO("No stop service available");
      }
    }
}

std::string CDistanceSensor::getName()
{
    return name_;
}

CSrf10Controller::CSrf10Controller(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) :
    CController(name,device_p,nh)
{
    std::string topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("srf10_state"));
    nh.param("controllers/"+name+"/rate", rate_, 15.0);
    if(nh.hasParam("controllers/"+name+"/sensors/front"))
    {
      std::map< std::string, XmlRpc::XmlRpcValue >::iterator it;
      std::map< std::string, XmlRpc::XmlRpcValue > value;
      XmlRpc::MyXmlRpcValue sensors;
      nh.getParam("controllers/"+name+"/sensors/front", sensors);
      ROS_ASSERT(sensors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      value=sensors;
      for(it=value.begin();it!=value.end();it++)
      {
        ROS_ASSERT((*it).second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        if(!nh.hasParam("controllers/"+name+"/sensors/front/"+(*it).first+"/address"))
        {
            ROS_WARN_STREAM("You need to set the address atribute for the sensor " << (*it).first);
            continue;
        }
        if(!nh.hasParam("controllers/"+name+"/sensors/front/"+(*it).first+"/type"))
        {
            ROS_WARN_STREAM("You need to set the type of the sensor " << (*it).first);
            continue;
        }
        int address;
        std::string type;
        std::string frame_id;
        std::string sensor_topic;
        double max_alert_distance;
        double min_alert_distance;
        nh.getParam("controllers/"+name+"/sensors/front/"+(*it).first+"/address", address);
        nh.getParam("controllers/"+name+"/sensors/front/"+(*it).first+"/type", type);
        nh.param("controllers/"+name+"/sensors/front/"+(*it).first+"/frame_id", frame_id, std::string(""));
        nh.param("controllers/"+name+"/sensors/front/"+(*it).first+"/topic", sensor_topic, topic+"/"+(*it).first);
        nh.param("controllers/"+name+"/sensors/front/"+(*it).first+"/min_alert_distance", min_alert_distance, -1.0);
        nh.param("controllers/"+name+"/sensors/front/"+(*it).first+"/max_alert_distance", max_alert_distance, -1.0);
        //printf("%f\n",alert_distance);
        if (type.compare("srf10")==0)
        {
            srf10Sensors_[(uint8_t)address]=new CDistanceSensor((*it).first, (uint8_t)address, sensor_topic, nh, type, frame_id, min_alert_distance, max_alert_distance);
            srf10SensorsUpdateGroup_[(uint8_t)address]=1;
        }
        else if (type.compare("gp2d12")==0 || type.compare("gp2d120")==0 || type.compare("GP2Y0A21YK")==0)
        {
            adcSensors_[(uint8_t)address]=new CDistanceSensor((*it).first, (uint8_t)address, sensor_topic, nh, type, frame_id, min_alert_distance, max_alert_distance);
            adcSensorsAddresses_.push_back((uint8_t)address);
        }
        ROS_INFO_STREAM("Sensor " << (*it).first << " of type " << type << " inicializado");
      }
    }
    if(nh.hasParam("controllers/"+name+"/sensors/back"))
    {
      std::map< std::string, XmlRpc::XmlRpcValue >::iterator it;
      std::map< std::string, XmlRpc::XmlRpcValue > value;
      XmlRpc::MyXmlRpcValue sensors;
      nh.getParam("controllers/"+name+"/sensors/back", sensors);
      ROS_ASSERT(sensors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      value=sensors;
      for(it=value.begin();it!=value.end();it++)
      {
        ROS_ASSERT((*it).second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        if(!nh.hasParam("controllers/"+name+"/sensors/back/"+(*it).first+"/address"))
        {
            ROS_WARN_STREAM("You need to set the address atribute for the sensor " << (*it).first);
            continue;
        }
        if(!nh.hasParam("controllers/"+name+"/sensors/back/"+(*it).first+"/type"))
        {
            ROS_WARN_STREAM("You need to set the type of the sensor " << (*it).first);
            continue;
        }
        int address;
        std::string type;
        std::string frame_id;
        std::string sensor_topic;
        double min_alert_distance;
        double max_alert_distance;
        nh.getParam("controllers/"+name+"/sensors/back/"+(*it).first+"/address", address);
        nh.getParam("controllers/"+name+"/sensors/back/"+(*it).first+"/type", type);
        nh.param("controllers/"+name+"/sensors/back/"+(*it).first+"/frame_id", frame_id, std::string(""));
        nh.param("controllers/"+name+"/sensors/back/"+(*it).first+"/topic", sensor_topic, topic+"/"+(*it).first);
        nh.param("controllers/"+name+"/sensors/back/"+(*it).first+"/min_alert_distance", min_alert_distance, -1.0);
        nh.param("controllers/"+name+"/sensors/back/"+(*it).first+"/max_alert_distance", max_alert_distance, -1.0);
        //printf("%f\n",alert_distance);
        if (type.compare("srf10")==0)
        {
            srf10Sensors_[(uint8_t)address]=new CDistanceSensor((*it).first, (uint8_t)address, sensor_topic, nh, type, frame_id, min_alert_distance, max_alert_distance);
            srf10SensorsUpdateGroup_[(uint8_t)address]=1;
        }
        else if (type.compare("gp2d12")==0 || type.compare("gp2d120")==0  || type.compare("GP2Y0A21YK")==0)
        {
            adcSensors_[(uint8_t)address]=new CDistanceSensor((*it).first, (uint8_t)address, sensor_topic, nh, type, frame_id, min_alert_distance, max_alert_distance);
            adcSensorsAddresses_.push_back((uint8_t)address);
        }
        ROS_INFO_STREAM("Sensor " << (*it).first << " of type " << type << " inicializado");
      }
    }
    if(nh.hasParam("controllers/"+name+"/sensors/floor"))
    {
      std::map< std::string, XmlRpc::XmlRpcValue >::iterator it;
      std::map< std::string, XmlRpc::XmlRpcValue > value;
      XmlRpc::MyXmlRpcValue sensors;
      nh.getParam("controllers/"+name+"/sensors/floor", sensors);
      ROS_ASSERT(sensors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      value=sensors;
      for(it=value.begin();it!=value.end();it++)
      {
        ROS_ASSERT((*it).second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        if(!nh.hasParam("controllers/"+name+"/sensors/floor/"+(*it).first+"/address"))
        {
            ROS_WARN_STREAM("You need to set the address atribute for the sensor " << (*it).first);
            continue;
        }
        if(!nh.hasParam("controllers/"+name+"/sensors/floor/"+(*it).first+"/type"))
        {
            ROS_WARN_STREAM("You need to set the type of the sensor " << (*it).first);
            continue;
        }
        int address;
        std::string type;
        std::string frame_id;
        std::string sensor_topic;
        double min_alert_distance;
        double max_alert_distance;
        nh.getParam("controllers/"+name+"/sensors/floor/"+(*it).first+"/address", address);
        nh.getParam("controllers/"+name+"/sensors/floor/"+(*it).first+"/type", type);
        nh.param("controllers/"+name+"/sensors/floor/"+(*it).first+"/frame_id", frame_id, std::string(""));
        nh.param("controllers/"+name+"/sensors/floor/"+(*it).first+"/topic", sensor_topic, topic+"/"+(*it).first);
        nh.param("controllers/"+name+"/sensors/floor/"+(*it).first+"/min_alert_distance", min_alert_distance, -1.0);
        nh.param("controllers/"+name+"/sensors/floor/"+(*it).first+"/max_alert_distance", max_alert_distance, -1.0);
        //printf("%f\n",alert_distance);
        if (type.compare("srf10")==0)
        {
            ROS_WARN("srf10 sensors can only be declared at positions front or back");
            continue;
        }
        else if (type.compare("gp2d12")==0 || type.compare("gp2d120")==0  || type.compare("GP2Y0A21YK")==0)
        {
            adcSensors_[(uint8_t)address]=new CDistanceSensor((*it).first, (uint8_t)address, sensor_topic, nh, type, frame_id, min_alert_distance, max_alert_distance);
            adcSensorsAddresses_.push_back((uint8_t)address);
        }
        ROS_INFO_STREAM("Sensor " << (*it).first << " of type " << type << " inicializado");
      }
    }
    //config the sensors in the QBoard1
    int code=device_p_->setAutoupdateSensors(srf10SensorsUpdateGroup_);
    if (code<0)
        ROS_WARN("Unable to activate all srf10 sensors at the base control board");
    else
        ROS_INFO("All srf10 sensors updated correctly at the base control board");
    timer_=nh.createTimer(ros::Duration(1/rate_),&CSrf10Controller::timerCallback,this);
}

CSrf10Controller::~CSrf10Controller()
{
    std::map< uint8_t,CDistanceSensor * >::iterator it;
    for (it=srf10Sensors_.begin();it!=srf10Sensors_.end();it++)
    {
      delete (*it).second;
    }
    srf10Sensors_.clear();
}

void CSrf10Controller::timerCallback(const ros::TimerEvent& e)
{
    ros::Time now=ros::Time::now();
    std::map<uint8_t,unsigned short> updatedDistances;
    int code=device_p_->getDistanceSensors(updatedDistances);
    if (code<0)
        ROS_ERROR("Unable to get srf10 sensor distances from the base control board");
    else
    {
        std::map< uint8_t,CDistanceSensor * >::iterator it;
        for (it=srf10Sensors_.begin();it!=srf10Sensors_.end();it++)
        {
            if (updatedDistances.count((*it).first)>0)
            {
                //float distance=((float)updatedDistances[(*it).first])/100;  //distancia en metros
                ROS_DEBUG_STREAM("Obtained distance " << updatedDistances[(*it).first] << " for srf10 sensor " << (*it).second->getName() << " from the base control board");
                if(updatedDistances[(*it).first]>0)
                    (*it).second->publish((float)updatedDistances[(*it).first],now);
            }
            else
                ROS_WARN_STREAM("Could not obtain distance of srf10 sensor " << (int)((*it).first) << " from base control board");
        }
    }
    std::vector<unsigned int> adcReads;
    code=device_p_->getAdcReads(adcSensorsAddresses_,adcReads);
    if (code<0)
        ROS_ERROR("Unable to get adc sensor reads from the base control board");
    else
    {
        if(adcReads.size()!=adcSensorsAddresses_.size())
            ROS_ERROR("The asked addreses and the returned reads for the adc sensors do not match");
        else
        {
            for (int i=0;i<adcSensorsAddresses_.size();i++)
            {
                ROS_DEBUG_STREAM("Obtained distance " << adcReads[i] << " for adc sensor " << adcSensors_[adcSensorsAddresses_[i]]->getName() << " from the base control board");
                adcSensors_[adcSensorsAddresses_[i]]->publish(adcReads[i],now);
            }
        }
    }
}

std::set<uint8_t> CSrf10Controller::getConfiguredSrfs(void)
{
    std::set<uint8_t> configuredSrfs;
    std::map< uint8_t,CDistanceSensor * >::iterator it;
    for (it=srf10Sensors_.begin();it!=srf10Sensors_.end();it++)
    {
      configuredSrfs.insert((*it).first);
    }
    return configuredSrfs;
}
