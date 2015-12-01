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
#include <map> 
#include <fstream>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>

#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxmisc.h>
#include <stdio.h>
#include <cxcore.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <qbo_talk/Text2Speach.h>

#include <qbo_listen/Listened.h>

#include <qbo_face_msgs/GetName.h>
#include <qbo_face_msgs/Train.h>
#include "qbo_face_msgs/LearnFaces.h"
#include <qbo_face_msgs/FacePosAndDist.h>



#include <boost/algorithm/string.hpp>
#include "boost/filesystem.hpp"   // includes all needed Boost.Filesystem declarations

using namespace std;

map<string, string> dictionary;

ros::NodeHandle * private_nh_;
ros::Subscriber listener_sub;
ros::Subscriber system_lang_sub;
ros::Subscriber image_sub_;
ros::Subscriber face_pos_sub_ ;
ros::ServiceClient client_talker;
qbo_talk::Text2Speach srv_talker;


ros::ServiceClient client_get_name_;
ros::ServiceClient client_train_;
ros::ServiceClient client_learn_faces_;

qbo_face_msgs::GetName srv_get_name_;
qbo_face_msgs::Train srv_train;
qbo_face_msgs::LearnFaces srv_learn_faces_;

vector<cv::Mat> received_faces_;

int num_images_to_hold_ = 20;
double wait_for_name_tolerance_ = 4.0;


string new_persons_path_ = "/opt/ros/electric/stacks/qbo_stack/qbo_face_vision/qbo_face_recognition/faces/new_faces";

bool learn_request = false;
string name_to_learn_ = "";

bool face_detected_ = false; /*Bool and indicates if a face hasn't been detected so as to ingnore listening */

void listenerCallback(const qbo_listen::ListenedConstPtr& msg);

/*
* Method that loads dictionary from a txt file into the map structure. It receives the language to be loaded
*/
int loadDictionary(string lang)
{
	string filename = "/opt/ros/hydro/stacks/qbo_face_vision/qbo_face_recognition/config/lang/"+lang+".txt";	

	string line;
	ifstream dict_file(filename.c_str());
	if (dict_file.is_open())
  	{
   		while (dict_file.good())
    		{
     			getline (dict_file,line);
			vector<string> words;
			boost::split(words, line, boost::is_any_of("="));
			if(words.size()>=2)
			{
				map<string, string>::iterator it = dictionary.find(words[0]);
				if(it != dictionary.end())
					dictionary[words[0]]=words[1];
				else
					dictionary.insert(make_pair(words[0], words[1]));
			}
	    	}	

	    	dict_file.close();
  	}
	else 
	{
		ROS_ERROR("Unable to open dictionary file [%s]",lang.c_str()); 
		return 1;
	}

	printf("Printing FULL DICTIONARY:\n");

	map<string,string>::iterator it;
	for(it=dictionary.begin(); it!=dictionary.end(); it++ )
    	cout << it->first << " corresponds to " << it->second << endl;

	string listener_topic = "/listen/"+lang+"_default";
	listener_sub = private_nh_->subscribe<qbo_listen::Listened>(listener_topic.c_str(),20,&listenerCallback);

	ROS_INFO("Subscribed to topic %s", listener_topic.c_str());

	return 0;
}

/*
 * Method that, by receiving a string, makes the robot talk that string
 */
void speak_this(string to_speak)
{
	srv_talker.request.command = to_speak;

	if (client_talker.call(srv_talker))
		ROS_INFO("Talked: %s", to_speak.c_str());
	else
		ROS_ERROR("Failed to call the service of qbo_talk");
}



/*

System Language callback
*/

void system_langCallback(const std_msgs::String::ConstPtr& language)
{
	string lang = language->data;
	ROS_INFO("Changing language to [%s]", language->data.c_str());
	
	if(lang != "es" && lang!="en")
	{
		ROS_ERROR("INVALID LANGUAGE. ONLY ENGLISH AND SPANISH ARE AVAILABLE.");
		return;
	}
	
	ROS_INFO("Loaded dictionary for %s", lang.c_str());

	loadDictionary(lang);
}

/*
*/
void facePosCallback(const qbo_face_msgs::FacePosAndDist::ConstPtr& face_pos)
{
	if(face_pos->face_detected)
	{
		face_detected_ = true;
	}
	else
	{
		face_detected_ = false;
	}
}

/*
 * Get the name recognizer by Qbo's face recognizer
 * If string is "", then the face has not been recognized
 */
string getNameFromFaceRecognizer()
{
	string name_recognized = "", str_detected;
	bool bool_detected = false;

	ros::Time time_saved = ros::Time::now();
	ros::Duration time_diff;

	while(1)
	{
		//Use the service
		if (client_get_name_.call(srv_get_name_))
		{
			str_detected = (string)(srv_get_name_.response.name);
			bool_detected = srv_get_name_.response.recognized;

			if(bool_detected)
			{
				name_recognized = str_detected;
				break;
			}
		}
		else
		{
			ROS_ERROR("Failed to call service get name from qbo_face_recognition");
			return "";
		}

		time_diff = ros::Time::now() - time_saved;

		if(time_diff.toSec()>=wait_for_name_tolerance_)
			break;
	}

	return name_recognized;
}


/*
 * Given the person's name, it stored the images received in the proper
 * folder and uses the qbo_face_recognition service to train to the recognizer the new person.
 */
bool learnPerson(string person_name)
{

	srv_learn_faces_.request.person_name = person_name;

	if (client_learn_faces_.call(srv_learn_faces_))
	{
		if(srv_learn_faces_.response.learned)
			ROS_INFO("Images of faces captured successfully!");
	}
	else
	{
		ROS_ERROR("Failed to call service learn faces");
		return false;
	}
	
	ROS_INFO("Faces images for %s saved. Calling the train service of Qbo face recognition node.", person_name.c_str());
	srv_train.request.update_path = "";

	speak_this(dictionary["I AM TRAINING MYSELF"]);
	if (client_train_.call(srv_train))
	{
		ROS_INFO("Learning DONE!");
	}
	else
	{
		ROS_ERROR("Failed to call service train new face");
		return false;
	}

	return true;
}

void listenerCallback(const qbo_listen::ListenedConstPtr& msg)
{

	string listened = msg->msg;

	ROS_INFO("Listened: %s", listened.c_str());

	
	if(! face_detected_)
	{
		ROS_INFO("A face have have not been detected. Ignoring all commands");
		return;
	}
		

	if(learn_request) //A name has been asked to be learned
	{
		if(string(listened) == dictionary["YES I DID"]) //Confirm the name
		{
			speak_this(dictionary["OK"]+" "+name_to_learn_+ ". "+dictionary["LET ME SEE HOW YOU LOOK"]);

			if(learnPerson(name_to_learn_))
				speak_this(dictionary["OK. I AM READY TO RECOGNIZE YOU"]);
			else
				speak_this(dictionary["OH OH. PROBLEM. I COULD NOT TRAIN MYSELF"]);

			learn_request = false;

		}
		else if(string(listened) == dictionary["NO I DID NOT"])
		{
			learn_request = false;
			speak_this(dictionary["IF YOUR NAME IS NOT"]+" "+name_to_learn_+". "+dictionary["WHAT IS YOUR NAME?"]);
			//cv::waitKey(5000);
		}
		else
			return;
	}

	/*
	 * Query sentences
	 */
	if(string(listened) == dictionary["WHAT IS MY NAME"] || string(listened) == dictionary["WHAT'S MY NAME"])
	{
		string rec_name;
		rec_name = getNameFromFaceRecognizer();

		if(rec_name != "")
			//speak_this("This is a "+recognized_person);
			speak_this(dictionary["YOUR NAME IS"]+" "+rec_name);
		else
			speak_this(dictionary["SORRY BUT I DON'T KNOW YOU"]);
	}
	
	else if(string(listened) == dictionary["ARE YOU SURE"])
	{
		string rec_name;
		rec_name = getNameFromFaceRecognizer();

		if(rec_name != "")
			speak_this(dictionary["YES. I AM SURE. YOUR NAME IS"]+" "+rec_name);
		else
			speak_this(dictionary["SORRY BUT I AM NOT SURE"]);
	}

	/*
	 * Learning faces
	 */
	else
	{
        	vector<string> words;
       		boost::split_regex(words, listened, boost::regex(dictionary["MY NAME IS"]+" "));

		if(words.size()>=2 ) //My name is #####
		{
			string person_name = words[1];

			speak_this(dictionary["DID YOU SAY"]+" "+person_name+"?");
			learn_request = true;
			name_to_learn_ = person_name;
		}

       		boost::split_regex(words, listened, boost::regex(dictionary["I AM"]+" "));
			
		if(words.size()>= 2) //I am #####
		{
			string person_name = words[1];

			speak_this(dictionary["DID YOU SAY"]+" "+person_name+"?");
			learn_request = true;
			name_to_learn_ = person_name;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "qbo_face_recognition_demo");

	private_nh_ = new ros::NodeHandle;


	string init_lang = "en";

	if( private_nh_->getParam("/system_lang", init_lang))
	{
		ROS_INFO("System language loaded -> %s", init_lang.c_str());
	}

	/*
	* Load dictionary
	*/
	loadDictionary(init_lang);

	/*
	* Subscribe to system languange topic 
	*/
	system_lang_sub= private_nh_->subscribe<std_msgs::String>("/system_lang",1,&system_langCallback);
	

	/*
	 * Set service client for qbo talker
	 */
	client_talker = private_nh_->serviceClient<qbo_talk::Text2Speach>("/qbo_talk/festival_say");

	/*
	 * Set service clients for face recognition
	 */
	client_get_name_ = private_nh_->serviceClient<qbo_face_msgs::GetName>("/qbo_face_recognition/get_name");
	client_train_ = private_nh_->serviceClient<qbo_face_msgs::Train>("/qbo_face_recognition/train");
	client_learn_faces_ = private_nh_->serviceClient<qbo_face_msgs::LearnFaces>("/qbo_face_recognition/learn_faces");


	/*
	* Callback for face tracking to check for faces
	*/

    face_pos_sub_ = private_nh_->subscribe<qbo_face_msgs::FacePosAndDist>("/qbo_face_tracking/face_pos_and_dist", 10, &facePosCallback);


	/*
	 * Set ROS parameters
	 */
	private_nh_->param<double>("/qbo_face_recognition_demo/wait_for_name_tolerance", wait_for_name_tolerance_, 4.0);

	/*
	 * Get update path parameter of the node
	 */
	private_nh_->getParam("/qbo_face_recognition/update_path", new_persons_path_);

	if(new_persons_path_ == "")
		new_persons_path_ = "/opt/qbo/ros_stacks/qbo_apps/qbo_face_recognition/faces/new_faces";

	ROS_INFO("Faced Recognition Demo Launched. Ready for incoming orders");
	
	ros::spin();

	private_nh_->deleteParam("/qbo_face_recognition_demo/wait_for_name_tolerance");

	return 0;
}
