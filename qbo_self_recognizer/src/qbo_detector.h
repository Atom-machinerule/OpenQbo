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
 * Author: Arturo Bajuelos <arturo@openqbo.com>
 */

#ifndef QBODETECTOR_H_
#define QBODETECTOR_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <sstream>

#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxmisc.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <qbo_arduqbo/Nose.h>
#include <qbo_self_recognizer/QboRecognize.h>


//#define WITH_GPU

using std::string;
using std::vector;

class QboDetector
{
private:
	void onInit();

	enum STATES {
		CALIBRATING_OFF,
		CALIBRATING_ON,
		PROCESSING_CODEWORD,
		CHANGE_WORD,
		FINISHED,

	};
	/*
	 * ROS Elements
	 */
	ros::NodeHandle private_nh_;
	ros::Subscriber image_sub_;
	ros::Publisher nose_color_pub_;
	ros::ServiceServer service_;


	image_transport::ImageTransport it_;
	image_transport::Publisher viewer_image_pub_;

	/*
	 * Internal variables
	 */
	double index_count_;

	cv::Mat nose_off_sample_;
	double nose_off_mean_;
	double nose_off_stddev_;

	cv::Mat nose_on_sample_;
	double nose_on_mean_;
	double nose_on_stddev_;

	bool recognized_; //Indicates of the nose is off or on
	unsigned int current_word_idx_;


	ros::Time word_time_;

	vector<bool> curr_codeword_values_;
	vector<bool> codeword_;

	/*
	 * ROS Paramters
	 */
	int codeword_size_;
	double codeword_time_diff_; //in seconds
	double codeword_checks_num_; //in seconds
	
	double good_ratio_threshold_;//A value between 0 and 1
	int num_of_samples_;

	/*
	 * State variables
	 */
	int state_; //One of possible STATES


	/*
	 * Methods
	 */

	//ROS Callbacks
	void imageCallback(const sensor_msgs::Image::ConstPtr& image_ptr);

	//Service
	bool recognizeQboService(qbo_self_recognizer::QboRecognize::Request  &req, qbo_self_recognizer::QboRecognize::Response &res);


public:
	QboDetector();

	virtual ~QboDetector();
};

#endif /* QBODETECTOR_H_ */
