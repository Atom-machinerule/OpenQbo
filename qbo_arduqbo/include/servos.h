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

#ifndef SERVOS_H
#define SERVOS_H

#include <driver/qboduino_driver.h>
#include <cmath>
#include <algorithm>
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <qbo_arduqbo/motor_state.h>
#include <qbo_arduqbo/TorqueEnable.h>
#include <dynamixel.h>

// Control table address
#define P_LIMIT_TEMPERATURE     11
#define P_TORQUE_ENABLE         24
#define P_LED_ENABLE            25
#define P_CW_COMPILANCE_MARGIN  26
#define P_CCW_COMPILANCE_MARGIN 27
#define P_CW_COMPILANCE_SLOPE   28
#define P_CCW_COMPILANCE_SLOPE  29
#define P_GOAL_POSITION_L       30
#define P_GOAL_POSITION_H       31
#define P_GOAL_SPEED_L          32
#define P_GOAL_SPEED_H          33
#define P_TORQUE_LIMIT_L        34
#define P_TORQUE_LIMIT_H        35
#define P_PRESENT_POSITION_L    36
#define P_PRESENT_POSITION_H    37
#define P_PRESENT_SPEED_L       38
#define P_PRESENT_SPEED_H       39
#define P_PRESENT_LOAD_L        40
#define P_PRESENT_LOAD_H        41
#define P_PRESENT_VOLTAGE       42
#define P_PRESENT_TEMPERATURE   43
#define P_REGISTERED            44
#define P_MOVING                46
#define P_LOCK                  47
#define P_PUNCH_L               48
#define P_PUNCH_H               49

double radians(double angle)
{
    static double ratio =M_PI/180.0;
    return angle*ratio;
};

class CServo
{
    public:
        CServo(std::string name, CQboduinoDriver *device_p, bool single=false);
        virtual ~CServo() {}
        virtual void setParams(XmlRpc::XmlRpcValue params);
        virtual void setAngle(float ang, float velocity=1);
        virtual float getAngle();
        float getAngleStored();
        std::string getName();
    
    protected:
        std::string name_;
        CQboduinoDriver *device_p_;
        int id_;
        int neutral_;
        int ticks_;
        float max_angle_;
        float min_angle_;
        float rad_per_tick_;
        float max_speed_;
        int max_ticks_speed_;
        bool invert_;
        float angle_;
        float range_;
  
        void recalculateAngleParams();
};

class ControledServo : public CServo
{
    public:
        ControledServo(std::string name, CQboduinoDriver *device_p, bool single=false)
            : CServo(name,device_p,single)
        {
        }
        ~ControledServo() {}
        virtual float getAngle();
        
};

class DynamixelServo : public CServo
{
    public:
        DynamixelServo(ros::NodeHandle nh, std::string name, CQboduinoDriver *device_p, bool single=false) : CServo(name,device_p,single), nh_(nh)
        {
          servo_state_pub_ = nh_.advertise<qbo_arduqbo::motor_state>(name_+"/state", 1);
          servo_torque_enable_srv_=nh_.advertiseService(name_+"/torqueEnable", &DynamixelServo::servoTorqueEnable, this);
        }
        ~DynamixelServo()
        {
          dxl_write_byte( id_, P_TORQUE_ENABLE, 0 );
        }
        virtual void setAngle(float ang, float velocity=1);
        virtual float getAngle();
        void changeTorque(int torque);

    protected:
        ros::NodeHandle nh_;
        ros::Publisher servo_state_pub_;
        ros::ServiceServer servo_torque_enable_srv_;
        bool servoTorqueEnable(qbo_arduqbo::TorqueEnable::Request  &req, qbo_arduqbo::TorqueEnable::Response &res);
};

#endif
