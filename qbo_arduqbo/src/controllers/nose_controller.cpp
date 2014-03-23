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

#include <controllers/nose_controller.h>
#include "ros/ros.h"
#include "qbo_arduqbo/Nose.h"
#include <ros/console.h>

CNoseController::CNoseController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : CController(name,device_p,nh)
{
    std::string topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("cmd_nose"));
    nose_sub_ = nh.subscribe<qbo_arduqbo::Nose>(topic, 1, &CNoseController::setNose,this);
}

void CNoseController::setNose(const qbo_arduqbo::Nose::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Nose comand arrived: " << msg->color );
    int code=device_p_->setNose((uint8_t)(msg->color));
    //int code=device_p_->setNose(0);
    if (code<0)
        ROS_ERROR("Unable to send nose to the head control board");
    else
        ROS_DEBUG_STREAM("Sent nose " << msg->color << " to the head board ");
}
