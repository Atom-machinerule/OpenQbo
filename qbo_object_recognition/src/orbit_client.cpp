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
#include <stdio.h>
#include <cxcore.h>
#include <climits>

#include "Orbit.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <qbo_talk/Text2Speach.h>

#include "qbo_object_recognition/LearnNewObject.h"
#include "qbo_object_recognition/RecognizeObject.h"
#include "qbo_object_recognition/Update.h"
#include "qbo_object_recognition/Teach.h"
#include "qbo_object_recognition/RecognizeObjectInputImage.h"

#include "boost/filesystem.hpp"   // includes all needed Boost.Filesystem declarations


cv::Ptr<Orbit> orbit;

ros::Subscriber image_sub_;

using namespace std;

ros::ServiceClient client_talker;

qbo_talk::Text2Speach srv_talker;

string recognized_object = "";

vector<cv::Mat> images_from_stereo_selector;
bool capture_image;

//Node ROS Parameters
int num_images_to_capture = 30;
string new_objects_path;
string update_path;
string init_orbit_path;
string input_image_topic;
double max_time_to_recognize;
double max_time_to_learn;
int stabilizer_threshold;
double certainty_threshold;

ros::NodeHandle * private_nh_;

void speak_this(string to_speak)
{
	srv_talker.request.command = to_speak;

	if (client_talker.call(srv_talker))
		ROS_INFO("Talked: %s", to_speak.c_str());
	else
		ROS_ERROR("Failed to call the service of qbo_talk");
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

    cv::Mat image_received;

    cv::cvtColor(cv_ptr->image, image_received, CV_BGR2RGB);

    images_from_stereo_selector.push_back(image_received);


}


bool learnNewObjectService(qbo_object_recognition::LearnNewObject::Request  &req, qbo_object_recognition::LearnNewObject::Response &res )
{

	string object_name = req.object_name;

	if(object_name.compare("") == 0) //If object's name is empty, return an error
	{
		res.learned = false;
		ROS_ERROR("Invalid object name");
		return true;
	}

	ROS_INFO("Trying to learn new object: %s", object_name.c_str());

	images_from_stereo_selector.clear();

	ROS_INFO("Waiting for %d images", num_images_to_capture);

	unsigned int last_num = 0;

	ros::Time t1 = ros::Time::now();
	ros::Time t2;
	ros::Duration diff_time;

	capture_image = true;
	while((int)images_from_stereo_selector.size()<num_images_to_capture)
	{
		usleep(200000);
		ros::spinOnce();

		if(last_num != images_from_stereo_selector.size())
		{
			last_num = images_from_stereo_selector.size();
			ROS_INFO("%u images received. %u more to go", last_num, num_images_to_capture-last_num);
		}


		t2 = ros::Time::now();
		diff_time = t2-t1;

		if(diff_time.toSec()>max_time_to_learn)
			break;
	}
	capture_image = false;

	if(images_from_stereo_selector.size() == 0)
	{
		ROS_WARN("TIMEOUT: No images were captured");
		ROS_INFO("Aborting object learn...\n");
		ROS_INFO("Ready to recognize or learn new objects");
		res.learned = false;
		return true;
	}

	ROS_INFO("All Images captured");

	if(!boost::filesystem::is_directory(new_objects_path))
	{
		//Create the main folder in the save path
		boost::filesystem::create_directory(new_objects_path);
	}

	if(!boost::filesystem::is_directory(new_objects_path+"/"+object_name))
		//Create the object's folder
		boost::filesystem::create_directory(new_objects_path+"/"+object_name);

	string time_now;
	stringstream out;
	out << time(NULL);
	time_now = out.str();

	//Save images
	int image_index = 0;
	for(unsigned int i = 0; i<images_from_stereo_selector.size();i++)
	{
		string filename;
		stringstream out_2;

		out_2 <<new_objects_path+"/"+object_name<<"/"<<time_now<<"_"<<image_index<<".jpg";
		filename = out_2.str();

		vector<int> params;

		params.push_back(CV_IMWRITE_JPEG_QUALITY);
		params.push_back(100);
		cv::imwrite(filename.c_str(), images_from_stereo_selector[i], params);
		image_index++;
	}




	ROS_INFO("%u images saved in %s\n", (unsigned int) images_from_stereo_selector.size(), (new_objects_path+"/"+object_name).c_str());


	res.learned = true;


	return true;
}

bool recognizeObjectService(qbo_object_recognition::RecognizeObject::Request  &req, qbo_object_recognition::RecognizeObject::Response &res )
{

	//Get image from Message

	images_from_stereo_selector.clear();

	float certainty = 0;

	string object_name = "";
	res.recognized = false;

	ROS_INFO("Trying to recognize incoming object");


	ros::Time t1 = ros::Time::now();
	ros::Time t2;
	ros::Duration diff_time;


	capture_image = true;

	while(certainty < (float)certainty_threshold)
	{
		usleep(400000);
		ros::spinOnce();

		if(images_from_stereo_selector.size()>0)
			certainty = orbit->recognizeObject(images_from_stereo_selector[images_from_stereo_selector.size()-1], object_name, Orbit::BAG_OF_WORDS_SVM);

		if(certainty>=(float)certainty_threshold)
			res.recognized = true;

		images_from_stereo_selector.clear();
		t2 = ros::Time::now();
		diff_time = t2-t1;

		if(diff_time.toSec()>max_time_to_recognize)
			break;
	}
	capture_image = false;



	res.object_name = object_name;


//	cout <<"Service call to recognize: "<< res.object_name<<endl;

	if(res.recognized)
	{
		ROS_INFO("Object recognized: %s\n", object_name.c_str());
		//speak_this("This is a "+object_name);
	}
	else
	{
		ROS_INFO("Could not recognize object\n");
		//speak_this("I don't know");

	}

	ROS_INFO("Ready to recognize or learn new objects");
	return true;
}


bool recognizeWithStabilizerService(qbo_object_recognition::RecognizeObject::Request  &req, qbo_object_recognition::RecognizeObject::Response &res )
{

	if(orbit->objects_.size()==0)
	{
		res.recognized = false;
		res.object_name = "";
	}
	else if(orbit->objects_.size()==1)
	{
		res.recognized = true;
		res.object_name = orbit->objects_[0].name_;
	}

	//Get image from Message

	images_from_stereo_selector.clear();

	float certainty = 0;

	string object_name = "";
	res.recognized = false;

	ROS_INFO("Trying to recognize incoming object with stabilizer");

	int stabilizer[orbit->objects_.size()+1];

	/*
	 * Stabilizer has N+1 elements, when there are N objects to recognized
	 * the N+1 corresponds to the "no object was found"
	 */
	for(unsigned int i = 0; i<=orbit->objects_.size(); i++)
		stabilizer[i] = 0;

	ros::Time t1 = ros::Time::now();
	ros::Time t2;
	ros::Duration diff_time;

	capture_image = true;

	while(1)
	{
		usleep(200000);
		ros::spinOnce();

		if(images_from_stereo_selector.size()>0)
		{

			certainty = orbit->recognizeObject(images_from_stereo_selector[images_from_stereo_selector.size()-1], object_name, Orbit::BAG_OF_WORDS_SVM);

			/**Update stabilizer**/

			if(certainty < (float)certainty_threshold) //No object was recognized
			{
				for(unsigned int i = 0; i<orbit->objects_.size(); i++)
				{
					if(stabilizer[i] > 0)
						stabilizer[i]--; //Decrement certainty of recognized objects
				}

				stabilizer[orbit->objects_.size()]++;
			}
			else
			{
				for(unsigned int i = 0; i<orbit->objects_.size(); i++)
				{
					if(object_name.compare(orbit->objects_[i].name_) == 0)
						stabilizer[i]+=2; //Increment certainty of recognized objects
					else if(stabilizer[i] > 0)
						stabilizer[i]--;
				}

				if(stabilizer[orbit->objects_.size()] > 0)
					stabilizer[orbit->objects_.size()]-=3;

				if(stabilizer[orbit->objects_.size()] < 0)
					stabilizer[orbit->objects_.size()]=0;
			}
		}

		/*
		 * Print stabilizer
		 */
		printf("Stabilizer: \n");
		for(unsigned int i = 0; i<=orbit->objects_.size(); i++)
		{
			if(i<orbit->objects_.size())
				printf("%s: %d,  ",orbit->objects_[i].name_.c_str(), stabilizer[i]);
			else
				printf("No Object: %d\n",stabilizer[i]);
		}
		printf("\n");

		/**Calc max value of stabilizer**/
		int max = stabilizer[0], max_index = 0;

		for(unsigned int i = 1; i<=orbit->objects_.size(); i++)
		{
			if(stabilizer[i]>max)
			{
				max = stabilizer[i];
				max_index = i;
			}
		}


		//Verify max stabilizer value
		if(max > stabilizer_threshold)
		{
			if(max_index<(int)orbit->objects_.size())
			{
				res.object_name = orbit->objects_[max_index].name_;
				res.recognized = true;
			}
			else
			{
				res.object_name = "";
				res.recognized = false;
			}


			break;
		}


		images_from_stereo_selector.clear();
		t2 = ros::Time::now();
		diff_time = t2-t1;

		if(diff_time.toSec()>2*max_time_to_recognize)
			break;

		/*
		if((diff_time.sec+1)%15 == 0)
		{

			speak_this("Wait a little bit");
			cv::waitKey(1500);
		}

		if((diff_time.sec+7)%15 == 0)
		{
			speak_this("It seems quite familiar");
			cv::waitKey(1500);
		}
		*/
	}

	capture_image = false;

	if(res.recognized)
	{
		ROS_INFO("Object recognized: %s\n", res.object_name.c_str());
		//speak_this("This is a "+res.object_name);
	}
	else
	{
		ROS_INFO("Could not recognize object\n");
		//speak_this("I don't know");

	}

	return true;
}

bool teachService(qbo_object_recognition::Teach::Request  &req, qbo_object_recognition::Teach::Response &res )
{

	int returned = orbit->teachOrbit(new_objects_path);


	if(returned>=0)
	{
		ROS_INFO("Orbit taught successfully !\n");
		res.taught = true;
	}
	else
	{
		ROS_ERROR("Error while teaching Orbit from %s\n", new_objects_path.c_str());
		res.taught = false;
		return true;
	}

	ROS_INFO("Orbit ready to recognize the following objects:");

	for(unsigned int i = 0; i<orbit->objects_.size(); i++)
	{
		ROS_INFO("\t - %s", orbit->objects_[i].name_.c_str() );
	}


	if(boost::filesystem::is_directory(new_objects_path))
	{
		//Delete folders
		for (boost::filesystem::directory_iterator itr(new_objects_path); itr!=boost::filesystem::directory_iterator(); ++itr)
		{
			if (boost::filesystem::is_directory(itr->status()))
			{
				std::string object_name=itr->path().filename().string();
				boost::filesystem::remove_all(new_objects_path+"/"+object_name);

			}
		}
	}

	return true;
}

bool updateService(qbo_object_recognition::Update::Request  &req, qbo_object_recognition::Update::Response &res )
{
    /*
        Get update path
    */
    private_nh_->getParam("/qbo_object_recognition/update_path",update_path);
	
    int returned = orbit->loadOrbit(update_path);

	if(returned>=0)
	{
		ROS_INFO("%d Objects Loaded from %s!\n", returned, update_path.c_str());
		res.updated = true;
	}
	else
	{
		ROS_ERROR("Error while loading Orbit from %s\n", update_path.c_str());
		res.updated = false;
		return true;
	}

	ROS_INFO("Orbit ready to recognize the following objects:");

	for(unsigned int i = 0; i<orbit->objects_.size(); i++)
	{
		ROS_INFO("\t - %s", orbit->objects_[i].name_.c_str() );
	}

	return true;
}

bool recognizeInputImageService(qbo_object_recognition::RecognizeObjectInputImage::Request  &req, qbo_object_recognition::RecognizeObjectInputImage::Response &res )
{

	//Get image from Message

	cv_bridge::CvImagePtr cv_ptr;

	try
	{
	  cv_ptr = cv_bridge::toCvCopy(req.input_image, sensor_msgs::image_encodings::RGB8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return false;
	}

	string object_name = "";


	float certainty = orbit->recognizeObject(cv_ptr->image, object_name, Orbit::BAG_OF_WORDS_SVM);

	res.recognized = false;

	if(certainty > 0)
	{
		res.object_name = object_name;
		res.recognized = true;
	}

	return true;
}


void terminate_func (int param)
{
  ROS_INFO("Terminating Object Recognizer\n");
  exit(1);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "orbit_client");

	//For signal handling
	void (*prev_fn)(int);
	prev_fn = signal (SIGINT,terminate_func);
	if (prev_fn==SIG_IGN) signal (SIGINT,SIG_IGN);

	private_nh_ = new ros::NodeHandle;

	string base_objects_path = ros::package::getPath("qbo_object_recognition")+"/objects";

	if(!boost::filesystem::is_directory(base_objects_path))
		boost::filesystem::create_directory(base_objects_path);
 
        if(!boost::filesystem::is_directory(base_objects_path+"/objects_db"))
		boost::filesystem::create_directory(base_objects_path+"/objects_db");

        if(!boost::filesystem::is_directory(base_objects_path+"/new_objects"))
		boost::filesystem::create_directory(base_objects_path+"/new_objects");

	string default_new_objects_path =base_objects_path + "/new_objects";
	string default_init_path = base_objects_path + "/objects_db";

	string default_update_path = default_init_path;
//	string default_init_path = "/opt/qbo_learn/orbit_db";

//	string default_input_image_topic = "/stereo_selector/object_image";
	string default_input_image_topic = "/qbo_stereo_selector/object";


	double default_certainty_threshold = 0.26;

	private_nh_->param("/qbo_object_recognition/num_images_to_capture", num_images_to_capture, 20);
	private_nh_->param("/qbo_object_recognition/new_object_images_path", new_objects_path, default_new_objects_path);
	private_nh_->param("/qbo_object_recognition/max_time_to_recognize", max_time_to_recognize, 30.0);
	private_nh_->param("/qbo_object_recognition/max_time_to_learn", max_time_to_learn,30.0);
	private_nh_->param("/qbo_object_recognition/update_path", update_path, default_update_path);
	private_nh_->param("/qbo_object_recognition/init_orbit_path", init_orbit_path, default_init_path);
	private_nh_->param("/qbo_object_recognition/stabilizer_threshold", stabilizer_threshold, 10);
	private_nh_->param("/qbo_object_recognition/certainty_threshold", certainty_threshold, default_certainty_threshold);
	private_nh_->param("/qbo_object_recognition/input_image_topic", input_image_topic, default_input_image_topic);

	ros::ServiceServer service_, service2_, service3_, service4_, service5_, service6_;

	image_sub_=private_nh_->subscribe<sensor_msgs::Image>(input_image_topic,1,&stereoSelectorCallback);

	client_talker = private_nh_->serviceClient<qbo_talk::Text2Speach>("/qbo_talk/festival_say_no_wait");

	//Advertise Services
	service_= private_nh_->advertiseService("qbo_object_recognition/learn", learnNewObjectService);
	service2_= private_nh_->advertiseService("qbo_object_recognition/recognize", recognizeObjectService);
	service3_= private_nh_->advertiseService("qbo_object_recognition/update", updateService);
	service4_= private_nh_->advertiseService("qbo_object_recognition/teach", teachService);
	service5_= private_nh_->advertiseService("qbo_object_recognition/recognize_with_stabilizer", recognizeWithStabilizerService);
	service6_= private_nh_->advertiseService("qbo_object_recognition/recognize_input_image", recognizeInputImageService);


	capture_image = false;

	//Building orbit
	ROS_INFO("Creating Orbit...");
	orbit = new Orbit(init_orbit_path);
	orbit->linear_kernel = true;
	ROS_INFO("Preparing Orbit for BOW with SVN...");
	orbit->prepareOrbit(Orbit::BAG_OF_WORDS_SVM);
	ROS_INFO("Orbit ready to recognize the following objects:");

	for(unsigned int i = 0; i<orbit->objects_.size(); i++)
	{
		ROS_INFO("\t - %s", orbit->objects_[i].name_.c_str() );
	}

	ros::spin();

  return 0;
}
