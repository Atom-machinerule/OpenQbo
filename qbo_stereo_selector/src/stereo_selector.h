/*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2011 Thecorpora, S.L.
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
 * Authors: Arturo Bajuelos <arturo@openqbo.com>; 
 */

#ifndef STEREOSELECTOR_H_
#define STEREOSELECTOR_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <sstream>
#include <string>
#include <signal.h>

#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxmisc.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <qbo_arduqbo/Nose.h>

#include <sensor_msgs/JointState.h>

using namespace std;


class StereoSelector {
private:


	/*
	 * ROS elements
	 */
	ros::NodeHandle private_nh_;

	//Subscribers
	ros::Subscriber image_sub_;
	ros::Subscriber disparity_img_sub_;
	ros::Subscriber joint_states_;
	ros::Subscriber info_sub_; //Camera info sub to get the P Matrix for correct head movement

	//Publishers
	ros::Publisher nose_color_pub_;
	ros::Publisher joint_pub_;
	ros::Publisher object_mask_pub_;
	image_transport::ImageTransport it_;
	image_transport::Publisher viewer_image_pub_;
	image_transport::Publisher viewer_object_image_pub_;

	/*
	 * ROS Parameters
	 */
	bool move_head_bool_;

	/*
	 * Search values when looking for objects
	 */
	double search_min_pan_;
	double search_max_pan_;
	double search_pan_vel_;
	double search_min_tilt_;
	double search_max_tilt_;
	double search_tilt_vel_;

	/*
	 * Mean threshold used in find ROI to find the roi with the best mean.
	 * Value between 0 and 255
	 */
	int mean_threshold_;

	/*
	 * Mean threshold used in CAM Shift to discriminate bad values of bounding rect found
	 * Value between 0 and 255
	 */
	int match_threshold_cam_shift_;

	/*
	 * This consists in the number of iterations we will keep moving the head
	 * after have lost the object. It is useful to keep seeking for the object when
	 * it gets out of the image
	 */
	int undetect_threshold_;

	/*
	 * The distance_filter_threshold corresponds to the maximum distance in which incoming objects
	 * are considered. If a point distance surpasses this value, it will not be considered
	 * The value is in meters
	 */
	double distance_filter_threshold_;


	/*
	 * PID arguments for head movement
	 */
	float u_prev_;
	float u_act_;
	float diff_u_;
	float kp_u_;
	float ki_u_;
	float kd_u_;

	float v_prev_;
	float v_act_;
	float diff_v_;
	float kp_v_;
	float ki_v_;
	float kd_v_;

	/*
	 * Internal variables
	 */

	//Head position
	float yaw_from_joint_;
	float tilt_from_joint_;

	int undetected_count_; //Counter of undetected objects iterations

	bool object_detected_bool_;
	cv::Rect object_roi_;
	double object_roi_mean_;

	int init_roi_width_;
	int init_roi_height_;

	cv::Size image_size_;

	float object_distance_;
	cv::Point object_pos_;

	cv::Mat p_;

	cv::Mat object_mask_;

	/*
	 * Methods
	 */

	//Setting ROS Params
	void setROSParams();
	//Delete the ROS params so as to be newly loaded new time
	void deleteROSParams();

	void onInit();
	void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info);
	void imageCallback(const sensor_msgs::Image::ConstPtr& image_ptr);
	void disparityImageCallback(const stereo_msgs::DisparityImage::ConstPtr& disp_image_ptr);
	void detectCamShift(cv::Mat image);
	bool findRoi(cv::Mat image);

	//For Head movement
	void moveHead();
	void newJointState(const sensor_msgs::JointStateConstPtr& msg);
	void setHeadPosition(float pos_updown, float pos_leftright, float vel_updown, float vel_leftright);
	float controlPID(float x, float ix, float dx, float Kp, float Ki, float Kd);

public:
	StereoSelector();
	virtual ~StereoSelector();
	void headToZeroPosition();

};

#endif /* STEREOSELECTOR_H_ */
