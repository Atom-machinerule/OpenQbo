/*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2012 Thecorpora, S.L.
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
 * Author: Arturo Bajuelos <arturo@openqbo.com>
 */

#include <iostream>
#include <cmath>

#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxmisc.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/SetCameraInfo.h>


#include <qbo_face_msgs/FacePosAndDist.h>

class FaceFollower
{
private:
	void onInit();

	ros::NodeHandle private_nh_;

	/*
	 * ROS Subscribers
	 */
	ros::Subscriber face_pos_sub_;
	ros::Subscriber joint_states_sub_;
	ros::Subscriber camera_info_sub_;
	/*
	 * ROS Publishers
	 */
	ros::Publisher base_control_pub_;
	ros::Publisher joint_pub_;

	int image_width_;
	int image_height_;


	/*
	 * Variables for ROS parameters
	 */
	bool move_base_bool_;
	bool move_head_bool_;
	double search_min_pan_;
	double search_max_pan_;
	double search_pan_vel_;
	double search_min_tilt_;
	double search_max_tilt_;
	double search_tilt_vel_;

	double desired_distance_;
	bool send_stop_;

	cv::Mat p_; //P matrix of the intrinsic parameters of the camera

	float yaw_from_joint_;
	float pitch_from_joint_;
	float min_pitch_;

	/*
	 * Variables for the PID control of the head and base
	 */

	//For head's pan movement
	float u_prev_;
	float u_act_;
	float diff_u_;
	float kp_u_;
	float ki_u_;
	float kd_u_;

	//For head's tilt movement
	float v_prev_;
	float v_act_;
	float diff_v_;
	float kp_v_;
	float ki_v_;
	float kd_v_;

	//For base's linear movement
	float distance_prev_;
	float distance_act_;
	float diff_distance_;
	float kp_distance_;
	float ki_distance_;
	float kd_distance_;

	//For base's angular movement
	float yaw_prev_;
	float yaw_act_;
	float diff_yaw_;
	float kp_yaw_;
	float ki_yaw_;
	float kd_yaw_;

	/*
	 * Callbacks
	 */
	void facePositionCallback(const qbo_face_msgs::FacePosAndDistConstPtr& head_pos_size);
	void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);
	void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info);


	/*
	 * Functions for Head and base movement
	 */

	//This function receives the face position in the image and the velocities
	//and will move the head trying to center the face in the image according to the camera parameters
	void setHeadPositionToFace(float pos_updown, float pos_leftright, float vel_updown, float vel_leftright);

	//This function directly moves the head to the designated position
	void setHeadPositionGlobal(float pos_updown, float pos_leftright, float vel_updown, float vel_leftright);

	//Moves the base of the robot
	void sendVelocityBase(float linear_vel, float angular_vel);

	/*
	 * Functions to set and delete ROS parameters
	 */
	void setROSParams();
	void deleteROSParams();

	/*
	 * PID control function
	 */
	float controlPID(float x, float ix, float dx, float Kp, float Ki, float Kd);

public:

	FaceFollower();
	~FaceFollower();
	void headToZeroPosition();
};
