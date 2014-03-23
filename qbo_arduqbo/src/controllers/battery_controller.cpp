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

#include <controllers/battery_controller.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "qbo_arduqbo/BatteryLevel.h"

CBatteryController::CBatteryController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : CController(name,device_p,nh)
{
    level_=0;
    stat_=0;
    std::string topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("battery_state"));
    nh.param("controllers/"+name+"/rate", rate_, 15.0);
    battery_pub_ = nh.advertise<qbo_arduqbo::BatteryLevel>(topic, 1);
    timer_=nh.createTimer(ros::Duration(1/rate_),&CBatteryController::timerCallback,this);
}

void CBatteryController::timerCallback(const ros::TimerEvent& e)
{
    int code=device_p_->getBattery(level_,stat_);
    if (code<0)
        ROS_ERROR("Unable to get battery level from the base control board");
    else
    {
        ROS_DEBUG_STREAM("Obtained battery level " << level_ << " and stat " << stat_ << " from the base control board ");
        qbo_arduqbo::BatteryLevel msg;
        msg.level=level_/10.0;
        msg.stat=stat_;
        msg.header.stamp = ros::Time::now();
        //publish
        battery_pub_.publish(msg);
    }
}
