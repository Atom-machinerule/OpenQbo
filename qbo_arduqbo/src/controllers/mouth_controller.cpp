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

#include <controllers/mouth_controller.h>
#include "ros/ros.h"
#include "qbo_arduqbo/Mouth.h"
#include <ros/console.h>
#include <boost/lexical_cast.hpp>

CMouthController::CMouthController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : CController(name,device_p,nh)
{
    std::string topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("cmd_mouth"));
    mouth_sub_ = nh.subscribe<qbo_arduqbo::Mouth>(topic, 1, &CMouthController::setMouth,this);
}

void CMouthController::setMouth(const qbo_arduqbo::Mouth::ConstPtr& msg)
{
    std::string debugStream = "Mouth comand arrived: ";
    for(uint8_t i=0;i<20;i++)
    {
      debugStream += boost::lexical_cast<std::string>(msg->mouthImage[i]) + " ";
    }
    ROS_DEBUG_STREAM(debugStream);
    uint8_t b1,b2,b3;
    b1=0;
    b2=0;
    b3=0;
    for (uint8_t i=0;i<4;i++)
    {
        for (uint8_t j=0;j<5;j++)
        {
            uint8_t index=i*5+j;
            uint8_t ledIndex=i*5+4-j;
            if(index<7)
            {
              b1 |= ((msg->mouthImage[ledIndex]&0x01)<<(index+1));
            }
            else if(index<15)
            {
              b2 |= ((msg->mouthImage[ledIndex]&0x01)<<(index-7));
            }
            else
            {
              b3 |= ((msg->mouthImage[ledIndex]&0x01)<<(index-15));
            }
        }
    }
    int code=device_p_->setMouth(b1, b2, b3);
    if (code<0)
        ROS_ERROR("Unable to send mouth to the head control board");
    else
    {
        debugStream = "Sent mouth: ";
        for(uint8_t i=0;i<20;i++)
        {
          debugStream += boost::lexical_cast<std::string>(msg->mouthImage[i]) + " ";
        }
        debugStream += " to the head board ";
        ROS_DEBUG_STREAM(debugStream);
    }
}
