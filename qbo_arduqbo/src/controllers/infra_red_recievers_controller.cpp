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

#include <controllers/infra_red_recievers_controller.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "qbo_arduqbo/Irs.h"

CInfraRedsController::CInfraRedsController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : CController(name,device_p,nh)
{
    ir0_=0;
    ir1_=0;
    ir2_=0;
    std::string topic,irs_topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("cmd_mics"));
    nh.param("controllers/"+name+"/irs_topic", irs_topic, std::string("irs_state"));
    nh.param("controllers/"+name+"/rate", rate_, 1.0);
    irs_pub_ = nh.advertise<qbo_arduqbo::Irs>(irs_topic, 1);
    timer_=nh.createTimer(ros::Duration(1/rate_),&CInfraRedsController::timerCallback,this);
}

void CInfraRedsController::timerCallback(const ros::TimerEvent& e)
{
    int code=device_p_->getIRs(ir0_,ir1_,ir2_);
    if (code<0)
        ROS_ERROR("Unable to get irs values from the base control board");
    else
    {
        ROS_DEBUG_STREAM("Obtained irs values (" << (int)ir0_ << "," << (int)ir1_ << "," << (int)ir2_ << ") from the base control board ");
        qbo_arduqbo::Irs msg;
        msg.ir0=ir0_;
        msg.ir1=ir1_;
        msg.ir2=ir2_;
        msg.header.stamp = ros::Time::now();
        //publish
        irs_pub_.publish(msg);
    }
}
