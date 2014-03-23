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


using namespace std;

bool capture_image = false;


cv::Mat input_image, histogram_;
int vmin = 0, vmax = 256, smin = 0;

ros::Subscriber image_sub_;


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

//    if(cv_ptr->image.cols<5 || cv_ptr->image.rows<5)
//    	return;
//    input_image = crop_hand(input_image);
}




int main (int argc, char **argv)
{
	ros::init(argc, argv, "hand_gesture_calib");

	if(argc<2)
	{
		ROS_ERROR("Insufficient number of input hand gesture. Should be at least one.");
		return 1;
	}

	ros::NodeHandle private_nh_;

	bool replace_images;

	int images_per_gesture;

	//Advertise param
	private_nh_.param("/hand_gesture_calib/images_per_gesture", images_per_gesture, 40);
	private_nh_.param("/hang_gesture_calib/replace_images", replace_images, true);

	if(replace_images)
		ROS_INFO("Hand gesture calibration initialized with image replacement");
	else
		ROS_INFO("Hand gesture calibration initialized without image replacement");

	ROS_INFO("The following hand gestures will be trained with %d images for each one: ", images_per_gesture);

	//Store the arguments in a string vector
	vector<string> hands;

	for(int i = 1; i<argc; i++)
	{
		hands.push_back(argv[i]);
	}

	for(unsigned int i = 0; i<hands.size(); i++)
	{
		ROS_INFO("\t-%s", hands[i].c_str());
	}

	string path = ros::package::getPath("qbo_music_player");

	path+="/hand_gestures/";

	cv::namedWindow("Hand Gesture Image",CV_GUI_EXPANDED);
	cvMoveWindow("Hand Gesture Image", 800,0);

//	cv::namedWindow("Histogram",CV_GUI_EXPANDED);
//	cvMoveWindow("Histogram", 1100,0);
//	cvCreateTrackbar( "Open Size","Histogram", &open_size_, 70, 0);
//	cvCreateTrackbar( "Media Blur Size","Histogram", &median_blur_size_, 70, 0 );


	image_sub_=private_nh_.subscribe<sensor_msgs::Image>("/qbo_stereo_selector/object",1,&stereoSelectorCallback);

	ROS_INFO("Path to qbo_music_player: %s\n", path.c_str());

	char opt;

	bool calibration_interrupted = false;

	vector< vector<cv::Mat> > hand_gesture_images;

	cv::Mat image_to_store;


	capture_image = true;

	ROS_INFO("Waiting for first images of hands");

	while (input_image.cols == 0 || input_image.rows == 0)
	{
		ros::spinOnce();
	}



	for(unsigned int i = 0; i<hands.size();i++)
	{
		ROS_INFO("Storing images for gesture: %s", hands[i].c_str());
		ROS_INFO("Press 's' to store images.");

		vector<cv::Mat> temp_vector;

		temp_vector.clear();

		while((int)temp_vector.size() < images_per_gesture)
		{
			bool image_saved = false;

			ROS_INFO("\t -%s: Waiting for Image %u. %u more to go", hands[i].c_str(),
					(unsigned int)temp_vector.size()+1,
					images_per_gesture-((unsigned int)temp_vector.size()+1));

			while (!image_saved)
			{
				ros::spinOnce();

				if(input_image.cols != 0 && input_image.rows != 0)
				{
					image_to_store = input_image.clone();

					cv::imshow("Hand Gesture Image", image_to_store);
					//cv::imshow("Histogram", histogram_);
				}

				opt = cv::waitKey(10);

				if (!ros::ok())
				{
					calibration_interrupted = true;
					break;
				}
				else if(opt == 'S' || opt == 's')
				{
					image_saved = true;
					break;
				}
			}

			if(!ros::ok())
			{
				calibration_interrupted = true;
				break;
			}
			else if(image_saved)
			{
				temp_vector.push_back(image_to_store);
			}
		}

		if(!ros::ok())
		{
			calibration_interrupted = true;
			break;
		}

		hand_gesture_images.push_back(temp_vector);

		ROS_INFO("All images captured for %s. Press 'n' to continue", hands[i].c_str());

		char opt2='a';

		while(opt2 != 'n' && opt2 != 'N')
			opt2=cv::waitKey(10);


	}

	capture_image = false;

	if(calibration_interrupted) //Don't store images in path
	{
		ROS_INFO("Program ended without changes applied");
	}

	else //Store images in path
	{
		//Erase previous files if necessary
		if(replace_images)
		{
			boost::filesystem::remove_all(path);
			boost::filesystem::create_directory(path);
		}
		else if(!boost::filesystem::is_directory(path))
		{
			boost::filesystem::create_directory(path);
		}
		else
		{
			if(boost::filesystem::is_regular(path+"vocabulary.xml.gz"))
			{
				boost::filesystem::remove(path+"vocabulary.xml.gz");
			}
		}

		ROS_INFO("All images collected!");

		ROS_INFO("Saving images in %s...", path.c_str());

		unsigned int num_images_saved = 0;

		string time_now;
		stringstream out;
		out << time(NULL);
		time_now = out.str();

		for(unsigned int i = 0; i<hand_gesture_images.size();i++)
		{
			if(!boost::filesystem::is_directory(path+hands[i]))
				//Create the object's folder
				boost::filesystem::create_directory(path+hands[i]);
			else
			{
				if(boost::filesystem::is_regular_file(path+hands[i]+"/svm.xml.gz"))
				{
					boost::filesystem::remove(path+hands[i]+"/svm.xml.gz");
				}

				if(boost::filesystem::is_regular_file(path+hands[i]+"/descriptors.xml.gz"))
				{
					boost::filesystem::remove(path+hands[i]+"/descriptors.xml.gz");
				}
			}


			for(unsigned int j = 0; j<hand_gesture_images[i].size();j++)
			{
				string filename;
				stringstream out_2;

				out_2 <<path+"/"+hands[i].c_str()<<"/"<<time_now<<"_"<<j<<".jpg";
				filename = out_2.str();
				vector<int> params;
				params.push_back(CV_IMWRITE_JPEG_QUALITY);
				params.push_back(100);

				cv::imwrite(filename.c_str(), hand_gesture_images[i][j], params);
				num_images_saved++;
			}

		}



		ROS_INFO("All %u images successfully saved!", num_images_saved);

		cv::waitKey(2000);

		//Building orbit
		ROS_INFO("Launching gesture recognizer...");
		cv::Ptr<Orbit> orbit = new Orbit(path);
		//orbit->linear_kernel = true;
		ROS_INFO("Preparing gesture recognizer for BOW with SVN...");
		orbit->prepareOrbit(Orbit::BAG_OF_WORDS_SVM);
		ROS_INFO("Calibration completed and prepared for Gesture Recognition.");

	}

    int system_ret = system("rosnode kill qbo_stereo_selector");
    
	return system_ret;
}
