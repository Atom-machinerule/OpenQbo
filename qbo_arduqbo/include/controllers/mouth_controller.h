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

#ifndef MOUTH_CONTROLLER_H
#define MOUTH_CONTROLLER_H

#include <driver/qboduino_driver.h>
#include <controllers/controllers_class.h>
#include "ros/ros.h"
#include "qbo_arduqbo/Mouth.h"
#include <ros/console.h>

class CMouthController : public CController
{
    public:
        CMouthController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh);
        
    protected:
        ros::Subscriber mouth_sub_;
        void setMouth(const qbo_arduqbo::Mouth::ConstPtr& msg);
};

#endif
