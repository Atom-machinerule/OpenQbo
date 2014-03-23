/*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2012 TheCorpora SL
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
 * Authors:    Sergio Merino <s.merino@openqbo.com>;
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <iostream>
#include <signal.h>
#include "qbo_video_record/StartRecord.h"
#include "qbo_video_record/StopRecord.h"
#include "qbo_video_record/StatusRecord.h"
#include <boost/thread.hpp>

ros::NodeHandle *pNh;
std::string tempDir= "/tmp/";
std::string defaultDirectory = "/home/qboblue/Videos/";
std::string defaultTopic="/stereo/left/image_raw";
std::string extension=".avi";
std::string soundRecorder="/usr/bin/arecord";
std::string sExtension=".wav";
std::string combinator="ffmpeg ";
std::string finalExtension=".ogv";
std::string deleter="rm";
int status = 0;
int isColor = 1;
int fps     = 30; 
int frameW  = 480; 
int frameH  = 320;

ros::Subscriber sub;
cv::VideoWriter *writer;
FILE* soundProc;
std::string directory;
std::string filename;
pid_t pID;

using namespace boost;

void recordCallback(const sensor_msgs::Image::ConstPtr& image_ptr)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    *writer <<  cv_ptr->image;
}

bool startServiceCallBack(qbo_video_record::StartRecord::Request  &req,
         qbo_video_record::StartRecord::Response &res )
{
  ROS_INFO("Service Called");
  ROS_INFO("Recived:Start");
  if (status==0){
    status=1;
    image_transport::ImageTransport it(*pNh);
    std::string topic = req.imageTopic;
    directory= req.directory;
    if (topic==""){
      topic=defaultTopic;
    }
    if (directory==""){
      directory=defaultDirectory;
    }
    else  {
      directory=directory+"/";
    }
    //Set Video file name and directory
    int now=time(0);
    std::stringstream sstr;
    sstr << now;
    filename=sstr.str();
    std::string file=tempDir+filename+extension;


    pID = vfork();
    if (pID == 0)                // child
    {
        std::string sound=soundRecorder+" "+tempDir+filename+sExtension;
        std::string params=tempDir+filename+sExtension;
        execl(soundRecorder.c_str(),soundRecorder.c_str(),params.c_str(), NULL);
        exit(1);
    }
    else
    {
      //Define Video file
      writer=new cv::VideoWriter(file, CV_FOURCC('D','I','V','X'), fps, cv::Size(frameW,frameH), true);
      //Defining imaging callback
      sub = pNh->subscribe<sensor_msgs::Image>(topic,1,&recordCallback); 
    }
    res.result=true;
  }
  else{
    res.result=false;
  }
  return true;
}

void combineAudioVideo()
{
      std::string tempFile=tempDir+filename+finalExtension;
      std::string finalFilename=directory+filename+finalExtension;
      std::string combi=combinator+" -i "+tempDir+filename+sExtension+" -i "+tempDir+filename+extension + " -vcodec libtheora -f ogg -acodec libvorbis -ar 22050 -y " + tempFile;
      const char * combiCommand=combi.c_str();

      //Set remove command
      std::string rm=deleter+" "+tempDir+filename+sExtension+" "+tempDir+filename+extension;
      const char * removeCommand=rm.c_str();

      FILE* combProc = popen(combiCommand, "r");
      char buffer[1028];
      while (fgets(buffer, 1028, combProc) != NULL)
      {
      }
      ROS_INFO("End Combining");
      pclose(combProc);
      //Removing audio and video files
      FILE* removeProc = popen(removeCommand, "r");
      while (fgets(buffer, 1028, removeProc) != NULL)
      {
      }
      pclose(removeProc);
      rename(tempFile.c_str(), finalFilename.c_str());
}

bool stopServiceCallBack(qbo_video_record::StopRecord::Request  &req,
         qbo_video_record::StopRecord::Response &res )
{
  ROS_INFO("Service Called");
  ROS_INFO("Recived:Stop");
//  cvReleaseVideoWriter(&writer);
  if (status==1)
  {
    status=0;
    kill( pID, SIGKILL );
    sub.shutdown();
    thread thread_1 = thread(combineAudioVideo);
    res.result=true;
  }
  else{
    res.result=false;
  }
  return true;
}

bool statusServiceCallBack(qbo_video_record::StatusRecord::Request  &req,
         qbo_video_record::StatusRecord::Response &res )
{
  ROS_INFO("Service Called");
  ROS_INFO("Recived:Status");
  res.status=status;
  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "qbo_record_video", ros::init_options::NoSigintHandler);
  pNh=new ros::NodeHandle;
  ros::ServiceServer startService = pNh->advertiseService("/qbo_video_record/start",startServiceCallBack);
  ros::ServiceServer stopService = pNh->advertiseService("/qbo_video_record/stop",stopServiceCallBack);
  ros::ServiceServer statusService = pNh->advertiseService("/qbo_video_record/status",statusServiceCallBack);
  ROS_INFO("Waiting Server");
  ros::spin();
  return 0;

}
