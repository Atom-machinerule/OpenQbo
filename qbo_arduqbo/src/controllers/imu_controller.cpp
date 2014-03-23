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

#include <controllers/imu_controller.h>
#include "ros/ros.h"
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

double calcAutoCovariance(int16_t *data, uint8_t data_length)
{
 
    double mean=0;
    for(uint8_t i=0;i<data_length;i++)
    {
        mean+=((double)data[i])/data_length;
    }
 
    double covariance = 0;
 
    for(uint8_t i=0;i<data_length;i++)
    {
        covariance+=((double)data[i]-mean)*0.001132571*((double)data[i]-mean)*0.001132571/data_length;
    }

    return covariance;
}
CImuController::CImuController(std::string name, CQboduinoDriver *device_p, ros::NodeHandle& nh) : CController(name,device_p,nh)
{
    gyroX0_=0;
    gyroX1_=0;
    gyroX2_=0;
    gyroY0_=0;
    gyroY1_=0;
    gyroY2_=0;
    gyroZ0_=0;
    gyroZ1_=0;
    gyroZ2_=0;
    accelerometerX_=0;
    accelerometerY_=0;
    accelerometerZ_=0;
    /*
    for(int i=0;i<30;i++)
    {
      gyroXcalib_[i]=0;
      gyroYcalib_[i]=0;
      gyroZcalib_[i]=0;
    }
    */
    gyroXzero_=0;
    gyroYzero_=0;
    gyroZzero_=0;

    is_calibrated_=false;
    imu_calibrated_.data=false;

    imu_msg_.header.frame_id = "base_link";
    //imu_msg_.header.frame_id = "base_footprint";

    //Orientation - all these values must be float
    //imu_data.orientation.setRPY(0, 0, compass_reading);
    imu_msg_.orientation.x = 0;
    imu_msg_.orientation.y = 0;
    imu_msg_.orientation.z = 0;
    imu_msg_.orientation.w = 1;

    boost::array<double, 9> orientation_covariance = {{-1., 0, 0,
                                                       0, -1., 0,
                                                        0, 0, -1.}};
    imu_msg_.orientation_covariance = orientation_covariance;

    //Angular velocity
    imu_msg_.angular_velocity.x = 0;
    imu_msg_.angular_velocity.y = 0;
    imu_msg_.angular_velocity.z = 0;

    boost::array<double, 9> angular_velocity_covariance = {{0.008, 0, 0,
                                                            0, 0.008, 0,
                                                             0, 0, 0.008}};
    //boost::array<double, 9> angular_velocity_covariance = {{-1., 0, 0,
    //                                                   0, -1., 0,
    //                                                    0, 0, -1.}};

    imu_msg_.angular_velocity_covariance = angular_velocity_covariance;



    //Linear acceleration
    imu_msg_.linear_acceleration.x = 0;
    imu_msg_.linear_acceleration.y = 0;
    imu_msg_.linear_acceleration.z = 0;

    boost::array<double, 9> linear_acceleration_covariance = {{0.002, 0, 0,
                                                               0, 0.002, 0,
                                                                0, 0, 0.002}};
    //boost::array<double, 9> linear_acceleration_covariance = {{-1., 0, 0,
    //                                                   0, -1., 0,
    //                                                    0, 0, -1.}};

    imu_msg_.linear_acceleration_covariance = linear_acceleration_covariance;

    std::string topic,imu_topic;
    nh.param("controllers/"+name+"/topic", imu_topic, std::string("imu_state"));
    nh.param("controllers/"+name+"/rate", rate_, 1.0);
    imu_pub_ = nh.advertise<sensor_msgs::Imu>(imu_topic+"/data", 1);
    imu_calib_pub_ = nh.advertise<std_msgs::Bool>(imu_topic+"/is_calibrated", 1);
    calibrate_service_ = nh.advertiseService(imu_topic+"/calibrate", &CImuController::calibrateService,this);
    timer_=nh.createTimer(ros::Duration(1/rate_),&CImuController::timerCallback,this);
    calibrateMethod();
}

bool CImuController::calibrateMethod()
{
    ros::Rate r(rate_);
    int count=0;
    gyroXzero_=0;
    gyroYzero_=0;
    gyroZzero_=0;
    uint8_t calibrationTotalData=100;
    int16_t covarianceCalculationDataArray[calibrationTotalData];
    for(int i=0;i<calibrationTotalData;i++)
    {
        int16_t gyroX, gyroY, gyroZ;
        int8_t accelerometerX, accelerometerY, accelerometerZ;
        int code=device_p_->getIMU(gyroX,gyroY,gyroZ,accelerometerX,accelerometerY,accelerometerZ);
        if (code<0)
            ROS_ERROR("Unable to get IMU data from the base control board");
        else
        {
            ROS_DEBUG_STREAM("Obtained IMU data (" << (int)gyroX << "," << (int)gyroY << "," << (int)gyroZ << "," << (int)accelerometerX << "," << (int)accelerometerY << "," << (int)accelerometerZ << ") from the base control board ");
            /*
            gyroXcalib_[i]=gyroX;
            gyroYcalib_[i]=gyroY;
            gyroZcalib_[i]=gyroZ;
            */
            gyroXzero_+=gyroX;
            gyroYzero_+=gyroY;
            gyroZzero_+=gyroZ;
            //covarianceCalculationDataArray[count]=gyroZ;
            //covarianceCalculationDataArray[count]=accelerometerZ;
            count++;
        }
        r.sleep();
    }
    if(count<calibrationTotalData*0.9)
    {
      is_calibrated_=false;
      gyroXzero_=0;
      gyroYzero_=0;
      gyroZzero_=0;
    }
    else
    {
      is_calibrated_=true;
      gyroXzero_/=count;
      gyroYzero_/=count;
      gyroZzero_/=count;
      //ROS_INFO_STREAM("autocovarianza: " << calcAutoCovariance(covarianceCalculationDataArray,count));
      
    }
    imu_calibrated_.data=is_calibrated_;
    imu_calib_pub_.publish(imu_calibrated_);
    return is_calibrated_;

}

bool CImuController::calibrateService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res )
{
    return calibrateMethod();
}

//int16_t arrayZ[100];
//uint8_t arrayIndex=0;

void CImuController::timerCallback(const ros::TimerEvent& e)
{
    int16_t gyroX, gyroY, gyroZ;
    int8_t accelerometerX, accelerometerY, accelerometerZ;
    int code=device_p_->getIMU(gyroX,gyroY,gyroZ,accelerometerX,accelerometerY,accelerometerZ);
    if (code<0)
        ROS_ERROR("Unable to get IMU data from the base control board");
    else
    {
        //ROS_INFO_STREAM("v_z=" << gyroZ << " cero=" << gyroZzero_);
        gyroX-=gyroXzero_;
        gyroY-=gyroYzero_;
        gyroZ-=gyroZzero_;
        ROS_DEBUG_STREAM("Obtained IMU data (" << (int)gyroX << "," << (int)gyroY << "," << (int)gyroZ << "," << (int)accelerometerX << "," << (int)accelerometerY << "," << (int)accelerometerZ << ") from the base control board ");

///Rellenar IMU
        //imu_msg_.angular_velocity.x=(((float)gyroX0_)*0.25+((float)gyroX1_)*0.25+((float)gyroX2_)*0.25+((float)gyroX)*0.25)*0.07*0.017453293; //filter + sensebility + deg to rad
        //imu_msg_.angular_velocity.y=(((float)gyroY0_)*0.25+((float)gyroY1_)*0.25+((float)gyroY2_)*0.25+((float)gyroY)*0.25)*0.07*0.017453293;
        //imu_msg_.angular_velocity.z=(((float)gyroZ0_)*0.25+((float)gyroZ1_)*0.25+((float)gyroZ2_)*0.25+((float)gyroZ)*0.25)*0.0175*0.017453293;

/*
        arrayIndex=(++arrayIndex)%100;
        arrayZ[arrayIndex]=gyroZ;
        double suma=0;
        for(int i=0;i<100;i++)
        {
            suma+=arrayZ[i];
        }
        suma/=100;
        imu_msg_.angular_velocity.z=suma;
*/
        //imu_msg_.angular_velocity.z=((float)gyroZ)*0.0175*0.017453293;
        //imu_msg_.angular_velocity.z=(((float)gyroZ0_)*0.25+((float)gyroZ1_)*0.25+((float)gyroZ2_)*0.25+((float)gyroZ)*0.25)*0.001132571;
        imu_msg_.angular_velocity.x=((float)gyroX)*0.001132571;
        imu_msg_.angular_velocity.y=((float)gyroY)*0.001132571;
        imu_msg_.angular_velocity.z=((float)gyroZ)*0.001132571;
        gyroX2_=gyroX1_;
        gyroY2_=gyroY1_;
        gyroZ2_=gyroZ1_;
        gyroX1_=gyroX0_;
        gyroY1_=gyroY0_;
        gyroZ1_=gyroZ0_;
        gyroX0_=gyroX;
        gyroY0_=gyroY;
        gyroZ0_=gyroZ;

        //imu_msg_.linear_acceleration.x=(((float)accelerometerX_)*0.5+((float)accelerometerX)*0.5)*9.8*0.018; //filter + sensibility + g to m/ss
        //imu_msg_.linear_acceleration.y=(((float)accelerometerY_)*0.5+((float)accelerometerY)*0.5)*9.8*0.018;
        //imu_msg_.linear_acceleration.z=(((float)accelerometerZ_)*0.5+((float)accelerometerZ)*0.5)*9.8*0.018;
        imu_msg_.linear_acceleration.x=((float)accelerometerX)*0.5*9.8*0.018; //filter + sensibility + g to m/ss
        imu_msg_.linear_acceleration.y=((float)accelerometerY)*0.5*9.8*0.018; //filter + sensibility + g to m/ss
        imu_msg_.linear_acceleration.z=((float)accelerometerZ)*0.5*9.8*0.018; //filter + sensibility + g to m/ss
        accelerometerX_=accelerometerX;
        accelerometerY_=accelerometerY;
        accelerometerZ_=accelerometerZ;

        imu_msg_.header.stamp = ros::Time::now();
        //publish
        imu_pub_.publish(imu_msg_);
    }
}
