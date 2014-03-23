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

#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include <driver/qboduino_driver.h>
#include <controllers/controllers_class.h>
#include <cmath>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <qbo_arduqbo/BaseStop.h>
#include <set>
#include <string>

class CBaseController : public CController
{
    public:
        CBaseController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh);
        
    protected:
        ros::Subscriber twist_sub_;
        ros::Publisher odom_pub_;
        ros::ServiceServer stall_unlock_service_;
        ros::ServiceServer base_stop_service_;
        tf::TransformBroadcaster odom_broadcaster_;
        std::set<std::string> base_stoppers_;
        
        // internal data            
        float v_linear_;             // current setpoint velocity
        float v_angular_;
        bool v_dirty_;
        float x_;                  // position in xy plane
        float y_;
        float th_;
        nav_msgs::Odometry odom_;
        ros::Time then_;
        bool is_odom_broadcast_enabled_;
        
        void timerCallback(const ros::TimerEvent& e);
        void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
        bool unlockStall(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res );
        bool baseStopService(qbo_arduqbo::BaseStop::Request  &req,
                         qbo_arduqbo::BaseStop::Response &res);
};

#endif
