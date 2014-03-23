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
#include <fstream>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>

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

#include <qbo_listen/Listened.h>

#include <std_msgs/String.h>

#include "qbo_object_recognition/LearnNewObject.h"
#include "qbo_object_recognition/RecognizeObject.h"
#include "qbo_object_recognition/Teach.h"
#include "qbo_object_recognition/Update.h"

#include <qbo_self_recognizer/QboRecognize.h>
#include <boost/algorithm/string.hpp>
#include "boost/filesystem.hpp"

using namespace std;

map<string, string> dictionary;

ros::Subscriber listener_sub;
ros::Subscriber system_lang_sub;

ros::ServiceClient client_talker;
ros::ServiceClient client_self_recognizer;

qbo_talk::Text2Speach srv_talker;
qbo_self_recognizer::QboRecognize srv_self_recognize;

string recognized_object = "";

ros::NodeHandle *private_nh_;

ros::ServiceClient client_learn;
ros::ServiceClient client_recognize;
ros::ServiceClient client_teach;
ros::ServiceClient client_update;

qbo_object_recognition::LearnNewObject srv_learn;
qbo_object_recognition::RecognizeObject srv_recognize;
qbo_object_recognition::Teach srv_teach;
qbo_object_recognition::Update srv_update;


ros::Time last_object_received_;


bool learn_request = false;
string object_to_learn = "";

string objects_path = "/opt/ros/electric/stacks/qbo_stack/qbo_object_recognition/objects/objects_db/";

void listenerCallback(const qbo_listen::ListenedConstPtr& msg);

void speak_this(string to_speak)
{
	srv_talker.request.command = to_speak;

	if (client_talker.call(srv_talker))
		ROS_INFO("Talked: %s", to_speak.c_str());
	else
		ROS_ERROR("Failed to call the service of qbo_talk");
}


/*
* Method that loads dictionary from a txt file into the map structure. It receives the language to be loaded
*/
int loadDictionary(string lang)
{
        string filename = ros::package::getPath("qbo_object_recognition") + "/config/lang/"+lang+".txt";

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
        cout << it->first << "=" << it->second << endl;

        string listener_topic = "/listen/"+lang+"_default";
        listener_sub = private_nh_->subscribe<qbo_listen::Listened>(listener_topic.c_str(),20,&listenerCallback);

        ROS_INFO("Subscribed to topic %s", listener_topic.c_str());

        return 0;
}


/*
 * System Language callback
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


void stereoSelectorCallback(const sensor_msgs::Image::ConstPtr& image_ptr)
{
	last_object_received_ = ros::Time::now();
}

bool self_recognize()
{
	if (client_self_recognizer.call(srv_self_recognize))
		return srv_self_recognize.response.recognized;
	else
		ROS_ERROR("Failed to call the service of qbo_self_recognizer");
		
	return false;
}

bool learnAndTeach(string object_name)
{
	srv_learn.request.object_name = object_name;

	if (client_learn.call(srv_learn))
	{
		if(srv_learn.response.learned)
			ROS_INFO("Images captured successfully!");
	}
	else
	{
		ROS_ERROR("Failed to call service learn object");
	}

	if(object_name == "MYSELF")
		speak_this(dictionary["NOW I GET THE PICTURE OF HOW I LOOK. I WILL TRAIN MYSELF"]);
	else
		speak_this(dictionary["NOW I GET THE PICTURE OF"]+" "+object_name+". "+dictionary["I WILL TRAIN MYSELF"]);

	if (client_teach.call(srv_teach))
	{
		if(srv_teach.response.taught)
			ROS_INFO("Orbit taught successfully!");
	}
	else
	{
		ROS_ERROR("Failed to call service teach object");
	}


	return srv_learn.response.learned;
}

bool forget_object(string object_name)
{
	speak_this(dictionary["FORGETTING OBJECT"]+". "+object_name+".");

	string object_to_forget_path = objects_path+"/"+object_name;

	if(!boost::filesystem::is_directory(object_to_forget_path))
	{
		speak_this(dictionary["SORRY BUT CANNOT FORGET THE OBJECT"]+". "+object_name+". "+dictionary["I DON'T KNOW ANY"]+" "+object_name);
		return true;
	}
	else
	{
		boost::filesystem::remove_all(object_to_forget_path);

		if(boost::filesystem::is_regular_file(objects_path+"/vocabulary.xml.gz"))
			boost::filesystem::remove(objects_path+"/vocabulary.xml.gz");

		speak_this(dictionary["I AM RE TRAINING MYSELF"]);

		if (client_update.call(srv_update))
		{
			if(srv_update.response.updated)
				ROS_INFO("Orbit updated successfully!");
		}
		else
		{
			ROS_ERROR("Failed to call service update orbit");
			return false;
		}

		speak_this(dictionary["OK. I HAVE FORGOTTEN OBJECT"]+" "+object_name);
	}

	return true;
}

string recognize()
{
	string name_detected = "";
	//Use the service
	if (client_recognize.call(srv_recognize))
	{
		if(srv_recognize.response.recognized)
			name_detected = (std::string)(srv_recognize.response.object_name);
	}
	else
	{
		ROS_ERROR("Failed to call service recognize object");
	}

	return name_detected;
}

void listenerCallback(const qbo_listen::ListenedConstPtr& msg)
{
	ros::Time time_now = ros::Time::now();
	
	ros::Duration  time_diff = time_now - last_object_received_;
	



	std::string listened = msg->msg;

	/*Added for mirror video 2*/
/*	if(listened =="HELLO CUBE E O")
		speak_this("Hello Arturo");
	else if(listened == "LETS MAKE AN EXPERT MEN")
		speak_this("Ok. Lets do it");
*/

	if(listened == "" and msg->not_msg == dictionary["WHAT IS THIS"])
		listened = dictionary["WHAT IS THIS"];
	else if(listened == "" and msg->not_msg == dictionary["WHAT IS THIS Q B O"])
		listened = dictionary["WHAT IS THIS"];
	else if(listened == "" and msg->not_msg == dictionary["Q B O WHAT IS THIS"])
		listened = dictionary["WHAT IS THIS"];
	else if(listened == "" and msg->not_msg == dictionary["Q B O THIS IS YOU"])
		listened = dictionary["Q B O THIS IS YOU"];

	ROS_INFO("Listened: %s", listened.c_str());

	if(time_diff.toSec()>0.3)
	{
		ROS_INFO("Ignoring last sentence because object is not spotted!!");
		return;
	}

	if(learn_request) //A name has been asked to be learned
	{
		if(string(listened) == dictionary["YES I DID"]) //Confirm the name
		{
			speak_this(dictionary["I'M LEARNING"]+" "+object_to_learn);
			//cv::waitKey(5000);
			learnAndTeach(object_to_learn);
			speak_this(dictionary["I'M READY TO RECOGNIZE"]+" "+object_to_learn);

			learn_request = false;
		}
		else if(string(listened) == dictionary["NO"] || string(listened) == dictionary["NO I DID NOT"])
		{
			learn_request = false;
			speak_this(dictionary["CAN YOU REPEAT THE NAME OF THE OBJECT PLEASE"]);
			//cv::waitKey(5000);
		}
		else
			return;
	}

	else if(string(listened) == dictionary["Q B O WHAT IS THIS"] || string(listened) == dictionary["DO YOU KNOW WHAT THIS IS"] ||
			string(listened) == dictionary["WHAT IS THIS Q B O"] || string(listened) == dictionary["WHAT IS THIS"] )
	{

		speak_this(dictionary["LET ME SEE IT"]);

		//cv::waitKey(3000);

		ROS_INFO("Trying to recognize");
		string recognized_object = recognize();

		if(recognized_object != "")
		{	
			if(recognized_object == "MYSELF")
				speak_this(dictionary["OH. THIS IS ME. NICE"]);
			else if(recognized_object =="QBO")
			{
				speak_this(dictionary["OH. IT'S A Q B O. COOL"]);
				
				/*
				speak_this("Interesting. It looks like a Q b o. Let me check who he is");
			
				//Kill qbo_stereo_selector node Because of the nose light on
				string command = "rosnode kill /qbo_stereo_selector";
		    		system(command.c_str());
			
				bool it_is_me = self_recognize();
			
				if(it_is_me)
					speak_this("Oh. It is me. I must have a mirror in front of me");
				else
				{	

				//listener_sub.shutdown();
			  	speak_this("Oh. It's another q b o.");
				//command = "rosrun qbo_questions questions2.py &";
		        //system(command.c_str());
				        
		                     //ignore = true;
				}
			//Re launch qbo_stereo_selector	
			command = "roslaunch qbo_stereo_selector qbo_stereo_selector.launch &";
		        system(command.c_str());
			*/
			}
		else
			speak_this(dictionary["I GOT IT. I THINK IT IS A"]+" "+recognized_object);
		}
		else
			speak_this(dictionary["I DON'T KNOW"]);
	}
	else if(string(listened) == dictionary["WHO IS THIS"])
	{
		speak_this(dictionary["LET ME SEE"]);

		//cv::waitKey(3000);

		ROS_INFO("Trying to recognize");
		string recognized_object = recognize();


		if(recognized_object == "MYSELF")
			speak_this(dictionary["OH. THIS IS ME. NICE"]);
		else if(recognized_object =="QBO")
		{
			speak_this(dictionary["OH. IT'S A Q B O. COOL"]);
		
			/*
				speak_this("Interesting. It looks like a Q b o. Let me check who he is");
			
			
			//Kill qbo_stereo_selector node Because of the nose light on
			string command = "rosnode kill /qbo_stereo_selector";
		    system(command.c_str());
			
			bool it_is_me = self_recognize();
			
			if(it_is_me)
				speak_this("Oh. It is me. I must have a mirror in front of me");
		    else
			{
			//	listener_sub.shutdown();
			    speak_this("Oh. It's another q b o.");
				//command = "rosrun qbo_questions questions2.py &";
		        //system(command.c_str());
		        
		        //ignore = true;
			}
			
			//Re launch qbo_stereo_selector	
			command = "roslaunch qbo_stereo_selector qbo_stereo_selector.launch &";
		        system(command.c_str());
                         
                       */
		}
		else
			speak_this(dictionary["I DON'T KNOW"]);
	}
	else if(string(listened) == dictionary["Q B O THIS IS YOU"] || string(listened) == dictionary["THIS IS YOU Q B O"])
	{
		speak_this(dictionary["OH. LET ME SEE HOW I LOOK."]);
		//cv::waitKey(5000);
		learnAndTeach("MYSELF");
		speak_this(dictionary["WO AW.. I AM READY TO RECOGNIZE MYSELF."]);
	}
	else
	{
                vector<string> words;
                boost::split_regex(words, listened, boost::regex(dictionary["Q B O THIS IS A"]+" "));

		if(words.size()>=2)
		{
			string object_name = words[1];

			speak_this(dictionary["DID YOU SAY"]+" "+object_name+"?");
			
			learn_request = true;
			object_to_learn = object_name;
		}
		else
		{	
                	boost::split_regex(words, listened, boost::regex(dictionary["THIS IS A"]+" "));

			if(words.size()>=2) //Q B O THIS IS A
			{	
				string object_name = words[1];

				speak_this(dictionary["DID YOU SAY"]+" "+object_name+"?");

				learn_request = true;
				object_to_learn = object_name;
			}
		}

		words.clear();
                boost::split_regex(words, listened, boost::regex(dictionary["PLEASE FOR GET"]+" "));

		if(words.size()>=2) //PLEASE FOR GET
		{
			string object_name = words[1];

			forget_object(object_name);
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "orbit_demo");

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

	ROS_INFO("Dictionary loaded");

	
        /*
        * Subscribe to system languange topic 
        */
        system_lang_sub= private_nh_->subscribe<std_msgs::String>("/system_lang",1,&system_langCallback);




	client_talker = private_nh_->serviceClient<qbo_talk::Text2Speach>("/qbo_talk/festival_say_no_wait");
	client_self_recognizer = private_nh_->serviceClient<qbo_self_recognizer::QboRecognize>("qbo_self_recognizer/recognize");
	client_learn = private_nh_->serviceClient<qbo_object_recognition::LearnNewObject>("/qbo_object_recognition/learn");
	client_teach = private_nh_->serviceClient<qbo_object_recognition::Teach>("/qbo_object_recognition/teach");
	client_recognize = private_nh_->serviceClient<qbo_object_recognition::RecognizeObject>("/qbo_object_recognition/recognize_with_stabilizer");
	client_update= private_nh_->serviceClient<qbo_object_recognition::Update>("/qbo_object_recognition/update");

	//listener_sub =private_nh_.subscribe<qbo_listen::Listened>("/listen/en_object_recog",1,&listenerCallback);
//	listener_sub =private_nh_->subscribe<qbo_listen::Listened>("/listen/en_default",1,&listenerCallback);

	//Subscribe to stereo selector images
        ros::Subscriber	image_sub=private_nh_->subscribe<sensor_msgs::Image>("/qbo_stereo_selector/object",1,&stereoSelectorCallback);
	
	ROS_INFO("Demo 2 Launched. Ready for incoming orders");

//	speak_this("I am ready to recognize objects");

	ros::spin();

  return 0;
}
