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

#include <controllers/lcd_controller.h>
#include "ros/ros.h"
#include "qbo_arduqbo/LCD.h"
#include <ros/console.h>

CLCDController::CLCDController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : CController(name,device_p,nh)
{
    std::string topic;
    nh.param("controllers/"+name+"/topic", topic, std::string("cmd_lcd"));
    lcd_sub_ = nh.subscribe<qbo_arduqbo::LCD>(topic, 1, &CLCDController::setLCD,this);
    std::string hello;
    hello+="PC connected        ";
    device_p_->setLCD(hello);
}

void CLCDController::setLCD(const qbo_arduqbo::LCD::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("LCD comand arrived: " << msg->msg);
    int code=device_p_->setLCD(msg->msg);
    if (code<0)
        ROS_ERROR_STREAM("Unable to send string \"" << msg->msg << "\" to the base control board");
    else
        ROS_DEBUG_STREAM("Sent string " << msg->msg << " to the base control board ");
}
