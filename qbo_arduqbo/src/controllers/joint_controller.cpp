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

#include <controllers/joint_controller.h>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <map>
#include <ros/console.h>
#include <servos.h>

CJointController::CJointController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh, std::map<std::string,CServo *>& servos) : CController(name,device_p,nh)
{
    joints_dirty_=true;
    servos_p_=&servos;
    std::string topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("cmd_joints"));
    nh.param("controllers/"+name+"/rate", rate_, 15.0);
    joint_sub_ = nh.subscribe<sensor_msgs::JointState>(topic, 10, &CJointController::jointCallback, this);
    timer_=nh.createTimer(ros::Duration(1/rate_),&CJointController::timerCallback,this);
}

void CJointController::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Joint move");
    joint_msg_=*msg;
    joints_dirty_=true;
    // clean up and log values
}

void CJointController::timerCallback(const ros::TimerEvent& e)
{
    if(servos_p_)
    {
        if(joints_dirty_)
        {
            if(joint_msg_.position.size()!=joint_msg_.name.size())
            {
                ROS_ERROR("Malformed JointState message has arrived. Names size and positions size do not match");
                joints_dirty_=false;
                return;
            }
            bool velocityComandIncluded=false;
            if(joint_msg_.velocity.size()==joint_msg_.name.size()) velocityComandIncluded=true;
            for (unsigned int i=0;i<joint_msg_.name.size();i++)
            {
                if (servos_p_->count(joint_msg_.name[i])>0)
                {
                    if (velocityComandIncluded)
                      (*servos_p_)[joint_msg_.name[i]]->setAngle( joint_msg_.position[i], joint_msg_.velocity[i] );
                    else
                      (*servos_p_)[joint_msg_.name[i]]->setAngle( joint_msg_.position[i] );
                }
            }
            joints_dirty_=false;
        }
    }
}
