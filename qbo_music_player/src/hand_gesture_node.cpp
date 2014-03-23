/*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2011 Thecorpora S.L.
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
 * Authors: Arturo Bajuelos <arturo@openqbo.com>
 */


#include "ros/ros.h"
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>
#include <ros/package.h>

#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxmisc.h>
#include <stdio.h>
#include <cxcore.h>
#include <climits>


#include "Orbit.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

using namespace std;

bool capture_image = true;


cv::Mat input_image;

cv::Ptr<Orbit> orbit;

ros::Publisher hand_pub;

ros::Subscriber image_sub_;

double certainty_threshold;

vector<int> stabilizer;

int max_stabilizer = 6;

int stabilizer_threshold = 5;

vector<string> hands;

ros::Time last_pub;
ros::Time last_hand_received;


cv::Mat histogram_;
int vmin = 0, vmax = 256, smin = 0;
int open_size_ = 10;
int median_blur_size_ = 3;

cv::Mat crop_hand(cv::Mat img)
{
	/*
	 * little_face is a ROI that contains exclusively the color skin
	 */
	cv::Mat hsv = cv::Mat(img.size(), CV_8UC3 );
	cv::Mat mask = cv::Mat(img.size(), CV_8UC1);
	cv::Mat grayscale;
	cv::Mat backproject = cv::Mat( img.size(), CV_8UC1 );
	cv::Mat histimg = cv::Mat::zeros( img.size(), CV_8UC3);
	cv::Mat hue[hsv.channels()];
	cv::MatND hist;

	int n = 16;
	float hranges_arr[] = {0,180};
	const float* hranges = hranges_arr;
	int channels[]={0};
	cv::Rect track_window, face_roi;
	cv::RotatedRect track_box;


	cv::cvtColor( img, hsv, CV_RGB2HSV );
	cv::cvtColor(img, grayscale, CV_RGB2GRAY);
	cv::inRange(hsv, cv::Scalar(0,smin,MIN(vmin,vmax),0),cv::Scalar(180,256,MAX(vmin,vmax),0), mask );
	split( hsv,hue);

	double max_val = 0.0;

	//ROI selection is a bit different now and uses operator ()

	/*
	 * little_face is a ROI that contains exclusively the color skin
	 */
	cv::Rect reference_roi;

	reference_roi.width = 5;
	reference_roi.height = 5;
	reference_roi.x = img.cols/2;
	reference_roi.y = img.rows/2;

	cv::Mat hueRoi=hue[0](reference_roi);
	cv::Mat maskRoi=mask(reference_roi);
	cv::calcHist( &hueRoi,1,channels, maskRoi , hist, 1 ,  &n  ,  &hranges, true, 0 );
	cv::minMaxLoc(hist, 0, &max_val, 0, 0);

	float scale =max_val ? 255. / max_val : 0.;
	hist.convertTo(hist,hist.type(),scale,0);
	track_window = face_roi;
	histimg.setTo(0);


	cv::calcBackProject(hue,1,channels,hist,backproject,&hranges,1,true);
	cv::bitwise_and( backproject, mask, backproject);

	/**************************/

	histogram_ = cv::Mat::ones(backproject.size(), CV_8UC1)*255;

	for(int i = 0; i<backproject.rows; i++)
		for(int j = 0; j<backproject.cols; j++)
			if(backproject.at<unsigned char>(i,j)<5)
				histogram_.at<unsigned char>(i,j) = 0;

//	cv::Mat median_blurDisk=cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(median_blur_size_,median_blur_size_));
//	cv::morphologyEx(histogram_,histogram_,cv::MORPH_CLOSE,median_blurDisk);
//
//	cv::Mat openDisk=cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(open_size_+1,open_size_+1));
//	cv::morphologyEx(histogram_,histogram_,cv::MORPH_OPEN,openDisk);
//	histogram_ =	backproject.mul(histogram_);

	cv::medianBlur(histogram_,  histogram_, median_blur_size_*2+1);

	cv::Mat histogram_rgb;
	cv::cvtColor(histogram_, histogram_rgb, CV_GRAY2RGB);
	cv::bitwise_and( img, histogram_rgb, img);

	return img;
}

void stereoSelectorCallback(const sensor_msgs::Image::ConstPtr& image_ptr)
{
    if(!capture_image)
    	return;

	

    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
    }

    cv::cvtColor(cv_ptr->image, input_image, CV_BGR2RGB);

    string object_name;

    //input_image = crop_hand(input_image);

    float certainty = orbit->recognizeObject(input_image, object_name, Orbit::BAG_OF_WORDS_SVM);


	/*
	*	Clean stabilizer if gesture has not been seen in a while
	*/

	ros::Time now = ros::Time::now();
	ros::Duration diff_last_hand_received = now - last_hand_received;
	
	last_hand_received = now;

	if(diff_last_hand_received.toSec()>1)
	{
		for(unsigned int i = 0; i<stabilizer.size(); i++)
		{
			stabilizer[i] = 0;
		}
	}

    /*
     * Update stabilizer when the gesture is not recognized
     */
	if(certainty<(float)certainty_threshold)
	{
		for(unsigned int i = 0; i<stabilizer.size()-1; i++)
		{

			if(stabilizer[i]>0)
				stabilizer[i]--;
		}

		if(stabilizer[stabilizer.size()-1] < max_stabilizer)
			stabilizer[stabilizer.size()-1]++;

		return;
	}
	else
	{
		if(stabilizer[stabilizer.size()-1] >= 2)
			stabilizer[stabilizer.size()-1]-=2;
		else if(stabilizer[stabilizer.size()-1] == 1)
			stabilizer[stabilizer.size()-1]--;
	}






	/*
	 * Update stabilizer when gesture is known
	 */
	for(unsigned int i = 0; i<stabilizer.size()-1; i++)
	{
		if(object_name == hands[i])
		{
			if(stabilizer[i] < max_stabilizer)
				stabilizer[i]++;
		}
		else
		{	if(stabilizer[i]>0)
				stabilizer[i]--;
		}
	}


	/*
	 * Print Stabilizer values
	 */
	for(unsigned int i = 0; i<stabilizer.size(); i++)
	{
		if(i<stabilizer.size()-1)
			printf("%s: %d, ",hands[i].c_str(), stabilizer[i]);
		else
			printf("Not known: %d\n", stabilizer[i]);
	}



	/*
	 * Find maximum element of the stabilizer
	 */
	int max_stab = stabilizer[0];
	unsigned int max_index = 0;

	for(unsigned int i = 0; i<stabilizer.size(); i++)
	{
		if(stabilizer[i]>max_stab)
		{
			max_stab = stabilizer[i];
			max_index = i;
		}
	}


    ros::Duration dur = now-last_pub;

	if(max_stab >= stabilizer_threshold && dur.toSec()>2 && max_index < stabilizer.size()-1)
	{
		std_msgs::String msg_to_sent;

		msg_to_sent.data = hands[max_index];

		hand_pub.publish(msg_to_sent);

		last_pub = now;

		for(unsigned int i = 0; i<stabilizer.size(); i++)
		{
			stabilizer[i] = 0;
		}
	}




}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "hand_gesture_node");

	ros::NodeHandle private_nh_;

	ROS_INFO(" Launching Hand Gesture Publisher Noide");

	string path = ros::package::getPath("qbo_music_player");

	path+="/hand_gestures/";


	/*
	 * Set ROS parameters
	 */
	private_nh_.param("/hand_gesture_node/certainty_threshold", certainty_threshold, 0.26);
	private_nh_.param("/hand_gesture_node/stabilizer_max", max_stabilizer, 6);
	private_nh_.param("/hand_gesture_node/stabilizer_threshold", stabilizer_threshold, 5);

	//Building orbit
	ROS_INFO("Launching Gesture recognizer...");
	ROS_INFO("Certainty threshold: %lg, Stabilizer Threshold: %d, Stabilizer Max: %d", certainty_threshold, stabilizer_threshold, max_stabilizer);
	orbit = new Orbit(path);
	//orbit->linear_kernel = true;
	ROS_INFO("Preparing Gesture Recognizer for BOW with SVN...");
	orbit->prepareOrbit(Orbit::BAG_OF_WORDS_SVM);
	ROS_INFO("Gesture Recognizer ready with threshold %lg!", certainty_threshold);


	if(orbit->objects_.size()<2)
	{
		ROS_ERROR("Insufficient number of hand gestures. Calibrate Hand Gesture with more than 1 hand gesture");
		return 1;
	}

	/*
	 * Initializing stabilizer values
	 */
	for(unsigned int i = 0; i< orbit->objects_.size();i++)
	{
		hands.push_back(orbit->objects_[i].name_);
		stabilizer.push_back(0);
	}
	
	last_hand_received = ros::Time::now();

	//Initializing not known value
	stabilizer.push_back(0);

	image_sub_=private_nh_.subscribe<sensor_msgs::Image>("/qbo_stereo_selector/object",1,&stereoSelectorCallback);
	hand_pub = private_nh_.advertise<std_msgs::String>("/hand_gesture_node/hand_gesture", 10);

	ROS_INFO("Hand Gesture Node Launched!");
	ros::spin();

	return 0;
}
