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

#include <servos.h>
#include <cmath>
#include <algorithm>
#include <XmlRpcValue.h>
#include <ros/console.h>
//#include <std_msgs/Float64.h>
//#include <dynamixel_msgs/JointState.h>
//#include <dynamixel_controllers/SetSpeed.h>
//#include <dynamixel_controllers/TorqueEnable.h>

void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);

CServo::CServo(std::string name, CQboduinoDriver *device_p, bool single)
{
    name_ = name;
    device_p_ = device_p;

    id_ = -1;
    neutral_ = 1500;
    ticks_ = 1800;
    max_angle_ = M_PI/2;               // limit angle, radians
    min_angle_ = -M_PI/2;
    range_=M_PI;
    rad_per_tick_ = range_/ticks_;
    max_speed_ = M_PI;                 // radians per second
    max_ticks_speed_ = (int)(max_speed_/rad_per_tick_); // ticks per second
    invert_ = false;

    angle_ = 0.0;                            // current position
}

float CServo::getAngle()
{
    return angle_;
}

float CServo::getAngleStored()
{
    return angle_;
}

std::string CServo::getName()
{
    return name_;
}

void CServo::setAngle(float ang, float velocity)
{
    if (ang > max_angle_ || ang < min_angle_)
    {
        ROS_WARN_STREAM("Servo " << name_ << ": angle out of range (" << ang << ").Limits are: " << min_angle_ << "," << max_angle_);
        //return;
        ang=std::min(ang,max_angle_);
        ang=std::max(ang,min_angle_);
    }
    if (velocity > max_speed_)
    {
        ROS_WARN_STREAM("Servo " << name_ << ": velocity out of range (" << velocity << ").Limit is: " << max_speed_);
    }
    angle_ = ang;    // store it for joint state updates
    if (invert_)
        ang = ang * -1.0;
    int ticks = (int)(round( ang / rad_per_tick_ ));
    int speed = (int)(round(velocity / rad_per_tick_));
    ticks += neutral_;
    int code=device_p_->setServo(id_, ticks, speed);
    if (code<0)
        ROS_ERROR_STREAM("Unable to send angle for the servo " << name_ << " to the head control board " << code);
    else
        ROS_DEBUG_STREAM("Sent angle " << angle_ << " for the servo " << name_ << " to the head control board");
}

void CServo::setParams(XmlRpc::XmlRpcValue params)
{
    if(params.hasMember("id"))
    {
      id_=params["id"];
    }
    if(params.hasMember("max_angle_degrees"))
    {
      max_angle_=(double)radians(params["max_angle_degrees"]);
    }
    if(params.hasMember("min_angle_degrees"))
    {
      min_angle_=(double)radians(params["min_angle_degrees"]);
    }
    if(params.hasMember("max_angle_radians"))
    {
      max_angle_=(double)(params["max_angle_radians"]);
    }
    if(params.hasMember("min_angle_radians"))
    {
      min_angle_=(double)(params["min_angle_radians"]);
    }
    if(params.hasMember("max_speed"))
    {
      max_speed_=(double)params["max_speed"];
    }
    if(params.hasMember("range"))
    {
      range_=(double)radians(params["range"]);
    }
    if(params.hasMember("ticks"))
    {
      ticks_=params["ticks"];
    }
    if(params.hasMember("neutral"))
    {
      neutral_=params["neutral"];
    }
    if(params.hasMember("invert"))
    {
      invert_=params["invert"];
    }
    recalculateAngleParams();
}

void CServo::recalculateAngleParams()
{
    max_angle_=std::min(max_angle_,range_/2);
    min_angle_=std::max(min_angle_,-range_/2);
    rad_per_tick_ = range_/ticks_;
    max_ticks_speed_ = (int)(max_speed_/rad_per_tick_); // ticks per second
}

float ControledServo::getAngle()
{
    unsigned short pos;
    int code=device_p_->getServoPosition(id_,pos);
    if(code>=0)
    {
        float angle = ((int)pos - neutral_) * rad_per_tick_;
        if (invert_)
            angle = angle * -1.0;
        angle_ = angle;
        ROS_DEBUG_STREAM("Recibed angle " << angle_ << " for servo " << name_ << " from the head board");
        return angle_;
    }
    ROS_ERROR_STREAM("Unable to get angle for servo " << name_ << " from head board " << code);
    return -1000;
}

void DynamixelServo::setAngle(float ang, float velocity)
{

    if (ang > max_angle_ || ang < min_angle_)
    {
        ROS_WARN_STREAM("Servo " << name_ << ": angle out of range (" << ang << ").Limits are: " << min_angle_ << "," << max_angle_);
        ang=std::min(ang,max_angle_);
        ang=std::max(ang,min_angle_);
    }
    if (velocity > max_speed_)
    {
        ROS_WARN_STREAM("Servo " << name_ << ": velocity out of range (" << velocity << ").Limit is: " << max_speed_);
        velocity = max_speed_;
    }
    if (invert_)
        ang = ang * -1.0;

    int ticks = (int)(round( ang / rad_per_tick_ ));
    int speed = (int)(round(velocity / rad_per_tick_));
    if (speed==0) speed=1;
    ticks += neutral_;

    changeTorque(254);
    dxl_write_word(id_,P_TORQUE_LIMIT_L,1023);
    dxl_write_word(id_,P_GOAL_POSITION_L,ticks);
    dxl_write_word(id_,P_GOAL_SPEED_L,speed);
}

float DynamixelServo::getAngle()
{
    int pos = dxl_read_word( id_, P_PRESENT_POSITION_L );
    int CommStatus = dxl_get_result();
    if( CommStatus == COMM_RXSUCCESS )
    {
        PrintErrorCode();
    }
    else
    {
        PrintCommStatus(CommStatus);
    }
    qbo_arduqbo::motor_state motor_state;
    motor_state.header.stamp = ros::Time::now();
    motor_state.id=id_;
    motor_state.goal=dxl_read_word( id_, P_GOAL_POSITION_L );
    motor_state.position=pos;
    motor_state.error=motor_state.goal-pos;
    int speed=dxl_read_word( id_, P_PRESENT_SPEED_L );
    if(speed>=1024)
        speed=1024-speed;
    motor_state.speed=speed;
    int load=dxl_read_word( id_, P_PRESENT_LOAD_L );
    if(load>=1024)
        load=1024-load;
    motor_state.load=((float)load)/1024.0;
    motor_state.voltage=((float)dxl_read_byte(id_,P_PRESENT_VOLTAGE))/10.0;
    motor_state.temperature=dxl_read_byte( id_, P_PRESENT_TEMPERATURE );
    motor_state.moving=(dxl_read_byte( id_, P_MOVING)==1);
    servo_state_pub_.publish(motor_state);
    float angle = (pos - neutral_) * rad_per_tick_;
    if (invert_)
        angle = angle * -1.0;
    angle_ = angle;
    ROS_DEBUG_STREAM("Recibed angle " << angle_ << " for servo " << name_ << " from the head board");
    return angle_;
}

void DynamixelServo::changeTorque(int torque)
{
   dxl_write_byte( id_, P_CW_COMPILANCE_SLOPE, torque );
   dxl_write_byte( id_, P_CCW_COMPILANCE_SLOPE, torque ); 
   //dxl_write_byte( id_, P_LIMIT_TEMPERATURE, 99 ); 
}

bool DynamixelServo::servoTorqueEnable(qbo_arduqbo::TorqueEnable::Request  &req, qbo_arduqbo::TorqueEnable::Response &res)
{
   dxl_write_byte( id_, P_TORQUE_ENABLE, req.torque_enable );
}

void PrintCommStatus(int CommStatus)
{
    switch(CommStatus)
    {
    case COMM_TXFAIL:
        ROS_WARN("COMM_TXFAIL: Failed transmit instruction packet!");
        break;

    case COMM_TXERROR:
        ROS_WARN("COMM_TXERROR: Incorrect instruction packet!");
        break;

    case COMM_RXFAIL:
        ROS_WARN("COMM_RXFAIL: Failed get status packet from device!");
        break;

    case COMM_RXWAITING:
        ROS_WARN("COMM_RXWAITING: Now recieving status packet!");
        break;

    case COMM_RXTIMEOUT:
        ROS_WARN("COMM_RXTIMEOUT: There is no status packet!");
        break;

    case COMM_RXCORRUPT:
        ROS_WARN("COMM_RXCORRUPT: Incorrect status packet!");
        break;

    default:
        ROS_WARN("This is unknown error code!");
        break;
    }
}

// Print error bit of status packet
void PrintErrorCode()
{
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
        ROS_WARN("Input voltage error!");

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
        ROS_WARN("Angle limit error!");

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
        ROS_WARN("Overheat error!");

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
        ROS_WARN("Out of range error!");

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
        ROS_WARN("Checksum error!");

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
        ROS_WARN("Overload error!");

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
        ROS_WARN("Instruction code error!");
}
