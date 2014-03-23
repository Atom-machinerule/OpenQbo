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

#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include <driver/qboduino_driver.h>
#include <controllers/controllers_class.h>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <map>
#include <ros/console.h>
#include <servos.h>

class CJointController : public CController
{
    public:
        CJointController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh, std::map<std::string,CServo *>& servos);
    
    protected:
        
        ros::Subscriber joint_sub_;
        sensor_msgs::JointState joint_msg_;
        std::map<std::string,CServo *> *servos_p_;
        
        
        // internal data
        bool joints_dirty_;
        
        void timerCallback(const ros::TimerEvent& e);
        void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
};
        
#endif
