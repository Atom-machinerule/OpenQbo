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

#include "face_detector.h"


FaceDetector::FaceDetector() : it_(private_nh_)
{
	ROS_INFO("Initializing Qbo face tracking");
	onInit();
	ROS_INFO("Ready for face tracking");
}

FaceDetector::~FaceDetector() {

	ros::start();
	deleteROSParams();
	ROS_INFO("Qbo face tracking successfully ended");

	//Publisher of the nose color
	nose_color_pub_ = private_nh_.advertise<qbo_arduqbo::Nose>("/cmd_Nose", 4);

    /*
     * Creating structure to turn off the nose
     */
    qbo_arduqbo::Nose nose;
    nose.header.stamp = ros::Time::now();
    nose.color=0;


    ros::Time time_saved;
    time_saved = ros::Time::now();

    /*
     * Waiting for nose to turn off
     */
    while(1)
    {
    	ros::Duration time_diff = ros::Time::now() - time_saved;
        //Publish nose color to turn off the light
        nose_color_pub_.publish(nose);
    	if(time_diff.toSec()>2.0)
    		break;
    }
    printf("Qbo face tracking successfully ended\n");
}

void FaceDetector::setROSParams()
{
	//Setting default path of the Haar cascade classifier
	string default_classifier_path = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml";
        //string alternative_classifier_path = "/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_profileface.xml";
        string alternative_classifier_path = "none";

	//Set default parameter for face classifier path
	private_nh_.param("/qbo_face_tracking/face_classifier_path", face_classifier_path_, default_classifier_path);
        private_nh_.param("/qbo_face_tracking/alternative_face_classifier_path", alternative_face_classifier_path_, alternative_classifier_path);
	//default_pos refers to the head position when head is not found
	private_nh_.param<double>("/qbo_face_tracking/default_pos_x", default_pos_.x, double(0));
	private_nh_.param<double>("/qbo_face_tracking/default_pos_y", default_pos_.y, double(10));

	//Haar check period. If set as n, from n to n iterations, the face detector will perform a Haar Cascade.
	private_nh_.param<int>("check_Haar", check_Haar_, -1);


	//Track object period for CAM Shift. When track object is true, the CAM shift algorithm will refresh the color skin
	private_nh_.param<int>("/qbo_face_tracking/check_track_object", check_track_obj_, check_Haar_);

	//Undetected threshold determined the number of iterations the Kalman filter will make predictions after recognizing to have lost the face
	//from the image
	private_nh_.param<int>("/qbo_face_tracking/undetected_threshold", undetected_threshold_, 5);


	//The threshold distance, in meters, from which the nose will change the color from green to blue.
	private_nh_.param<double>("/qbo_face_tracking/distance_threshold", distance_threshold_, double(1.6));

	//If true, the messages will be sent to face recognizer to try to recognize the person, and the name will be seen in the viewer
	private_nh_.param("/qbo_face_tracking/send_to_recognizer", send_to_face_recognizer_, false);
	
	//If true, this parameter will print the recognized person in the viewer. Note: only applicable when send_to_recognizer is set as true 
	private_nh_.param("/qbo_face_tracking/print_recognized_face", print_recognized_face_, true);

}


void FaceDetector::deleteROSParams()
{
	private_nh_.deleteParam("/qbo_face_tracking/face_classifier_path");
        private_nh_.deleteParam("/qbo_face_tracking/alternative_face_classifier_path");
	private_nh_.deleteParam("/qbo_face_tracking/default_pos_x");
	private_nh_.deleteParam("/qbo_face_tracking/default_pos_y");
	private_nh_.deleteParam("/qbo_face_tracking/check_Haar");
	private_nh_.deleteParam("/qbo_face_tracking/check_track_object");
	private_nh_.deleteParam("/qbo_face_tracking/undetected_threshold");
	private_nh_.deleteParam("/qbo_face_tracking/distance_threshold");
	private_nh_.deleteParam("/qbo_face_tracking/send_to_recognizer");
	private_nh_.deleteParam("/qbo_face_tracking/print_recognized_face");
}

void FaceDetector::onInit()
{
	/*
	 * Setting ROS Parameters
	 */
	setROSParams();

	

	if(!face_classifier_.load(face_classifier_path_))
	{
		ROS_ERROR("Error importing face Haar cascade classifier from the specified path: %s", face_classifier_path_.c_str());

		deleteROSParams();
		exit(-1);
	}
	else
	{
		ROS_INFO("Haar cascade classifier successfully loaded from %s", face_classifier_path_.c_str());
	}

        if(!alternative_face_classifier_.load(alternative_face_classifier_path_))
        {
                ROS_WARN("Error importing alternative face Haar cascade classifier from the specified path: %s", alternative_face_classifier_path_.c_str());
                exist_alternative_=false;
        }
        else
        {
                ROS_INFO("Alternative Haar cascade classifier successfully loaded from %s", alternative_face_classifier_path_.c_str());
                exist_alternative_=true;
	}

	/*
	 * Subscribers of the node
	 */
	info_sub_=private_nh_.subscribe<sensor_msgs::CameraInfo>("/stereo/left/camera_info",10,&FaceDetector::infoCallback, this);


	/*
	 * Publishers of the node
	 */
	//Publisher of the face tracking position and size
	face_position_and_size_pub_=private_nh_.advertise<qbo_face_msgs::FacePosAndDist>("/qbo_face_tracking/face_pos_and_dist", 1);
	//Publisher of the face image
	face_pub_ = private_nh_.advertise<sensor_msgs::Image>("/qbo_face_tracking/face_image", 1);
	//Publisher of the viewer, using image transport for compression
	viewer_image_pub_ = it_.advertise("/qbo_face_tracking/viewer", 1);
	//Publisher of the nose color
	nose_color_pub_ = private_nh_.advertise<qbo_arduqbo::Nose>("/cmd_nose", 1);

	/*
	 * Initialize some internal parameters values
	 */
	face_detected_bool_ = false;
	image_size_ = cv::Size(0,0);

	//Initialize Kalman filter
	initializeKalmanFilter();

	loop_counter_ = 0;

	undetected_count_ = undetected_threshold_;


	/*
	 * Set dynamic Haar Cascade check
	 */
	cam_shift_detected_count_ = 0;
	dynamic_check_haar_ = false;
	if(check_Haar_<=0)
	{	dynamic_check_haar_ = true;
		check_Haar_=50;
		track_object_ = check_Haar_;
	}

	/*
	 * Service clients for face recognition
	 */
	client_recognize_ = private_nh_.serviceClient<qbo_face_msgs::RecognizeFace>("/qbo_face_recognition/recognize_with_stabilizer");
	client_get_name = private_nh_.serviceClient<qbo_face_msgs::GetName>("/qbo_face_recognition/get_name");


}

void FaceDetector::initializeKalmanFilter()
{
	kalman_filter_ = cv::KalmanFilter(4, 4);

	kalman_filter_.transitionMatrix = (cv::Mat_<float>(4,4) <<  1,0,1,0,
																0,1,0,1,
																0,0,1,0,
																0,0,0,1);

	kalman_filter_.errorCovPost = cv::Mat::eye(4,4, CV_32FC1);
	kalman_filter_.measurementNoiseCov = cv::Mat::eye(4,4,CV_32FC1) * 10;
	kalman_filter_.measurementMatrix = (cv::Mat_<float>(4,4) << 1,0,0,0,
																0,1,0,0,
																0,0,1,0,
																0,0,0,1);
}


void FaceDetector::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info)
{
	if(p_.data==NULL)
	{
		cv::Mat p=cv::Mat(3,4,CV_64F);
		for (int i=0;i<3;i++)
		{
			for (int j=0;j<4;j++)
			{
				p.at<double>(i,j)=info->P[4*i+j];
			}
		}
		p(cv::Rect(0,0,3,3)).convertTo(p_,CV_32F);

		/*
		 * Subscribing to image node
		 */
		image_sub_=private_nh_.subscribe<sensor_msgs::Image>("/stereo/left/image_rect_color",1,&FaceDetector::imageCallback, this);

		//TODO - Unsubscribe to the camera info topic
	}
}



void FaceDetector::imageCallback(const sensor_msgs::Image::ConstPtr& image_ptr)
{

	/*
	 * Cretate a CvImage pointer to get cv::Mat representation of image
	 */
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

    cv::Mat image_received = (cv_ptr->image).clone();
    image_size_ = cv_ptr->image.size();

    /*
     * Creating structure to publish nose color
     */
    qbo_arduqbo::Nose nose;
    nose.header.stamp = ros::Time::now();
    nose.color=1;


    string detection_type = "";

    if(face_detected_bool_) //If face was detected in previous iteration - use CAM shift
    {
    	if(detectFaceCamShift(image_received) != 0)
    	{
    		face_detected_bool_ = false;
    		//Reset cam_shift_detected
    		cam_shift_detected_count_ = 0;
    		cam_shift_undetected_count_++;
    	}
    	else
    	{
    		detection_type = "CAMSHIFT";

    		//Update cam_shift counter
    		cam_shift_detected_count_++;
    		cam_shift_undetected_count_ = 0;
    	}
    }


    if(!face_detected_bool_) //If face was not detected - use Haar Cascade
    {
        vector<cv::Rect> faces_roi;
    	detectFacesHaar(image_received, faces_roi);

    	if(faces_roi.size() > 0) //If Haar cascade classifier found a face
    	{
    		face_detected_bool_ = true;
    		track_object_ = false;

    		//Changed
    		detected_face_roi_ = faces_roi[0];
    		detected_face_ = cv_ptr->image(detected_face_roi_);

    		//Adjust face ratio because Haar Cascade always returns a square
    		face_ratio_ = 1.3*detected_face_roi_.height/detected_face_roi_.width;

    		detection_type = "HAAR";
    	}


    }



    if(!face_detected_bool_ && exist_alternative_) //If face was not detected - use Haar Cascade
    {
        vector<cv::Rect> faces_roi;
        detectFacesAltHaar(image_received, faces_roi);

        if(faces_roi.size() > 0) //If Haar cascade classifier found a face
        {
                face_detected_bool_ = true;
                track_object_ = false;

                //Changed
                detected_face_roi_ = faces_roi[0];
                detected_face_ = cv_ptr->image(detected_face_roi_);

                //Adjust face ratio because Haar Cascade always returns a square
                face_ratio_ = 1.3*detected_face_roi_.height/detected_face_roi_.width;

                detection_type = "HAAR";
        }


    }



	/*
	 * Kalman filter for Face pos estimation
	 */
    kalman_filter_.predict();
	if(face_detected_bool_) //IF face detected, use measured position to update kalman filter
	{
		if(undetected_count_>=undetected_threshold_)
		{
			cv::Mat new_state(4,1, CV_32FC1);
			new_state.at<float>(0,0) = float(detected_face_roi_.x+detected_face_roi_.width/2.);
			new_state.at<float>(1,0) = float(detected_face_roi_.y+detected_face_roi_.height/2.);
			new_state.at<float>(2,0) = 0.;
			new_state.at<float>(3,0) = 0.;

			new_state.copyTo(kalman_filter_.statePre);
			new_state.copyTo(kalman_filter_.statePost);

		}

		else
		{
			float u_vel = float(detected_face_roi_.x+detected_face_roi_.width/2.)-kalman_filter_.statePost.at<float>(0,0);
			float v_vel = float(detected_face_roi_.y+detected_face_roi_.height/2.)-kalman_filter_.statePost.at<float>(1,0);

			cv::Mat measures(4,1, CV_32FC1);
			measures.at<float>(0,0) = float(detected_face_roi_.x+detected_face_roi_.width/2.);
			measures.at<float>(1,0) = float(detected_face_roi_.y+detected_face_roi_.height/2.);
			measures.at<float>(2,0) = u_vel;
			measures.at<float>(3,0) = v_vel;

			kalman_filter_.correct(measures);
		}
	}
    else //If face haven't been found, use only Kalman prediction
    {
    	kalman_filter_.statePre.copyTo(kalman_filter_.statePost);
    	kalman_filter_.errorCovPre.copyTo(kalman_filter_.errorCovPost);
    }
    

	/*
	 * Compute head distance
	 */
    float head_distance;

	if(!face_detected_bool_)
    {
	  if(head_distances_.size()!=0)
			head_distance=head_distances_[0];//100;
		else
		  head_distance = 5;


	}
	else
	{
		head_distance=calcDistanceToHead(detected_face_, kalman_filter_);
	}
	
    if(head_distances_.size()==0)
    {
      for(int i=0;i<10;i++)
	    head_distances_.push_back(head_distance);
    }
    else
    {
      head_distances_.pop_back();
      head_distances_.insert(head_distances_.begin(),head_distance);
    }


    head_distance=0; //Reuse variable to compute mean head distance

    //Use mean distance of last measured head distances
    for(unsigned int i=0;i<head_distances_.size();i++)
    	head_distance+=head_distances_[i];

    head_distance=head_distance/head_distances_.size();



    //Update undetected count
	if(!face_detected_bool_)
	{	undetected_count_++;

	}
	else
	{
		undetected_count_ = 0;
	}

	//Create Face Pos and Size message
        qbo_face_msgs::FacePosAndDist message;
	message.image_width=cv_ptr->image.cols;
	message.image_height=cv_ptr->image.rows;
	message.type_of_tracking = detection_type;



	if(undetected_count_<undetected_threshold_) //If head have been recently detected, use Kalman filter prediction
	{
		message.face_detected = true;
		message.u = kalman_filter_.statePost.at<float>(0,0) - cv_ptr->image.cols/2.;
		message.v = kalman_filter_.statePost.at<float>(1,0) - cv_ptr->image.rows/2.;
		message.distance_to_head = float(head_distance);

	}
	else //If head has not been recently detected, face detection have failed
	{
		message.u = default_pos_.x;
		message.v = default_pos_.y;

		message.face_detected = false;

	}

    if(face_detected_bool_)
    {
    	//Publish head to topic
    	cv_ptr->image = detected_face_;
		face_pub_.publish(cv_ptr->toImageMsg());


		if(send_to_face_recognizer_)
			sendToRecognizer();


		//Change nose color
        nose.color=4; //If face detected - Blue
		if(head_distance<distance_threshold_)
        {
		 	nose.color=2; //If face is near - Green
        }
    }

    //Publish nose color
    nose_color_pub_.publish(nose);

    /*
     * Draws for the Viewer
     * Velocity vector, face rectangle and face center
     */
    int pred_x = int(message.u) + int(message.image_width)/2;
    int pred_y = int(message.v) + int(message.image_height)/2;

    cv::Point pred_kal = cv::Point(pred_x, pred_y);
    cv::Point tip_u_vel = cv::Point(pred_kal.x + (kalman_filter_.statePost.at<float>(2,0))/1., pred_kal.y);
    cv::Point tip_v_vel = cv::Point(pred_kal.x, pred_kal.y + (kalman_filter_.statePost.at<float>(3,0))/1.);
    //Draw face center and rectangle
    cv::circle(image_received, pred_kal,3,cv::Scalar(0,0,255), 3);
	cv::rectangle(image_received, detected_face_roi_, cv::Scalar(255,0,0), 2);
    if(tip_u_vel.x >= 0 && tip_u_vel.y >= 0 && tip_u_vel.x < image_received.cols && tip_u_vel.y < image_received.rows)
    {
    	cv::line(image_received, pred_kal, tip_u_vel, cv::Scalar(0,255,0), 2);
    }
    if(tip_v_vel.x >= 0 && tip_v_vel.y >= 0 && tip_v_vel.x < image_received.cols && tip_v_vel.y < image_received.rows)
    {
    	cv::line(image_received, pred_kal, tip_v_vel, cv::Scalar(255,0,0), 2);
    }

    //Draw name of person in image
	if(print_recognized_face_) 
		cv::putText(image_received, name_detected_, cv::Point(40,40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 3);

    
    /*
     * Publish Image viewer
     */
    cv_bridge::CvImagePtr cv_ptr2;
    try
    {
      cv_ptr2 = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
    }
    
    cv_ptr2->image = image_received;  
    viewer_image_pub_.publish(cv_ptr2->toImageMsg());
    

    /*
     * Publish face position and size
     */
    face_position_and_size_pub_.publish(message);


    /*
     * Update Haar cascade use frequency for Dynamic Haar Cascade
     */
    if(dynamic_check_haar_)
    {
	if(undetected_count_ > 10)
	{
		check_Haar_ = 2;
		cam_shift_detected_count_ = 0;
		cam_shift_undetected_count_ = 0;
	}
	
    	else if(cam_shift_undetected_count_>3)
    	{
    		check_Haar_/=2;
    		if(check_Haar_<2)
    			check_Haar_ = 2;
    	}

    	if(cam_shift_detected_count_>10)
    	{
    		check_Haar_+=5;
    		if(check_Haar_>100)
    			check_Haar_ = 100;

    	}
	

    	track_object_= check_Haar_ ;
    }

    /*
     * ROS_INFO
     */
    if(face_detected_bool_)
    {
    	if(dynamic_check_haar_)
    		ROS_INFO("FACE DETECTED -> Head Pos :(%d, %d), Head Distance: %lg, Dynamic check Haar: %u, Det type: %s, Alt: %s",
    				(int)message.u, (int)message.v, head_distance, check_Haar_, detection_type.c_str(), (exist_alternative_)?"true":"false");
    	else
    		ROS_INFO("FACE DETECTED -> Head Pos :(%d, %d), Head Distance: %lg, Check Haar: %u, Detection type: %s, Alt: %s",
    				(int)message.u, (int)message.v, head_distance, check_Haar_, detection_type.c_str(), (exist_alternative_)?"true":"false");
    }
    else
    {
    	if(dynamic_check_haar_)
    		ROS_INFO("NO FACE DETECTED. Using Haar Cascade Classifier to find one. Dynamic Check Haar: %u, Alt: %s", check_Haar_, (exist_alternative_)?"true":"false");
    	else
    		ROS_INFO("NO FACE DETECTED. Using Haar Cascade Classifier to find one. Check Haar: %u, Alt: %s", check_Haar_, (exist_alternative_)?"true":"false");
    }
    
	/*
	 * Refresh face_detected bool to use Haar Cascade once in a while
	 */
    if(loop_counter_%check_Haar_==0)
    	face_detected_bool_ = false;

    if(loop_counter_%check_track_obj_==0)
    	track_object_ = false;

    loop_counter_++;
    
}

void FaceDetector::sendToRecognizer()
{
	cv::Mat face_gray;

	cv::cvtColor(detected_face_, face_gray, CV_BGR2GRAY);

	cv_bridge::CvImage cv_image;
	cv_image.image = face_gray;

	cv_image.header.stamp=ros::Time::now();
	cv_image.header.frame_id="cara";
	cv_image.encoding = "mono8";

	cv_image.toImageMsg(srv.request.face);

	name_detected_ = "";
	string str_detected = "";
	bool bool_detected = false;

	//Use the service
	if (client_recognize_.call(srv))
	{
		//ROS_INFO("Sum: %ld and %s", (long int)srv.response.sum, ((std::string)srv.response.name).c_str());

		str_detected = (std::string)(srv.response.name);
		bool_detected = srv.response.recognized;

		if(bool_detected && str_detected == "")
			name_detected_ = "UNKOWN";
		else
			name_detected_ = str_detected;
	}
	else
	{
		ROS_ERROR("Failed to call service recognize face");
		return;
	}

}

void FaceDetector::sendToRecognizerGetName()
{


	string str_detected = "";
	bool bool_detected = false;

	name_detected_ = "";


	//Use the service
	if (client_get_name.call(srv_get_name))
	{
		//ROS_INFO("Sum: %ld and %s", (long int)srv.response.sum, ((std::string)srv.response.name).c_str());

		str_detected = (std::string)(srv_get_name.response.name);
		bool_detected = srv_get_name.response.recognized;

		if(bool_detected && str_detected == "")
			name_detected_ = "UNKOWN";
		else
			name_detected_ = str_detected;
	}
	else
	{
		ROS_ERROR("Failed to call service get name from recognize");
		return;
	}

}



unsigned int FaceDetector::detectFaceCamShift(cv::Mat img)
{
	/* Variables for CAM Shift */
	cv::Mat hsv = cv::Mat(img.size(), CV_8UC3 );
	cv::Mat mask = cv::Mat(img.size(), CV_8UC1);
	cv::Mat grayscale;
	cv::Mat backproject = cv::Mat( img.size(), CV_8UC1 );
	cv::Mat histimg = cv::Mat::zeros( img.size(), CV_8UC3);
	cv::Mat hue[hsv.channels()];
	cv::MatND hist;
	int vmin = 10, vmax = 256, smin = 30;
	int n = 16;
	float hranges_arr[] = {0,180};
	const float* hranges = hranges_arr;
	int channels[]={0};
	cv::Rect track_window, face_roi;
	cv::RotatedRect track_box;

	int mean_score_threshold = 70; //Minimum value of mean score of the CAM Shift. Value between 0 and 100
	float max_face_deslocation = detected_face_roi_.height/3.;

	unsigned int max_distance_width = detected_face_roi_.width/3.;
	unsigned int max_distance_height = detected_face_roi_.height/3.;

	/*********************/


	cv::cvtColor( img, hsv, CV_RGB2HSV );
	cv::cvtColor(img, grayscale, CV_RGB2GRAY);
	cv::inRange(hsv, cv::Scalar(0,smin,MIN(vmin,vmax),0),cv::Scalar(180,256,MAX(vmin,vmax),0), mask );
	split( hsv,hue);
	face_roi = detected_face_roi_;


	if(!track_object_) //Select a region with the face color skin and calc histogram
	{
		double max_val = 0.0;

		//ROI selection is a bit different now and uses operator ()

		/*
		 * little_face is a ROI that contains exclusively the color skin
		 */
		cv::Rect little_face;

		little_face = face_roi;

		little_face.width = face_roi.width -face_roi.width/3;
		little_face.height = face_roi.height -face_roi.height/3;
		little_face.x += face_roi.width/(2.*3.);
		little_face.y += face_roi.height/(2.*3.);

		cv::Mat hueRoi=hue[0](little_face);
		cv::Mat maskRoi=mask(little_face);
		cv::calcHist( &hueRoi,1,channels, maskRoi , hist, 1 ,  &n  ,  &hranges, true, 0 );
		cv::minMaxLoc(hist, 0, &max_val, 0, 0);

		float scale =max_val ? 255. / max_val : 0.;
		hist.convertTo(hist,hist.type(),scale,0);
		track_window = face_roi;
		histimg.setTo(0);
	}


	cv::calcBackProject(hue,1,channels,hist,backproject,&hranges,1,true);
	cv::bitwise_and( backproject, mask, backproject);

	//Use camshift over computed histogram
	track_box = cv::CamShift( backproject, track_window, cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));


	if(track_box.size.height>0 && track_box.size.width > 0
			&& track_box.center.x>0 && track_box.center.y >0)
		face_roi = track_box.boundingRect();

	face_roi.height = face_ratio_*face_roi.width;

	face_roi.x = min(face_roi.x, img.cols);
	face_roi.y = min(face_roi.y, img.rows);

	face_roi.width = min(img.cols-face_roi.x-3, face_roi.width);
	face_roi.height = min(img.rows-face_roi.y-3, face_roi.height);


	face_roi.height = max(face_roi.height, 0);
	face_roi.width = max(face_roi.width, 0);

	face_roi.x = max(face_roi.x, 0);
	face_roi.y = max(face_roi.y, 0);


	if(face_roi.height <= 0 || face_roi.width <= 0) //If face ROI has invalid dimensions
		return 1;

	/*
	 * This is used to check mean score of CAM shift selected ROI
	 */
	double mean_score = 0;

	if(face_roi.x > 0 && face_roi.y > 0 && face_roi.width>20 && face_roi.height>20)
	{
		cv::Mat temp = backproject(face_roi);
		for(int r = 0; r < temp.rows; r++)
			for(int c = 0; c < temp.cols; c++)
				mean_score+= temp.at<unsigned char>(r,c);

		mean_score = mean_score/double(temp.rows*temp.cols);
	}


	if(mean_score<mean_score_threshold) //Let's see if CAM Shift respects mean score threshold
	{
		face_detected_bool_ = false;
		return 1;
	}

	//If face position have moved considerably, ignore CAM shift
	if(euclidDist(cv::Point2f(face_roi.x + face_roi.width/2.,face_roi.y+face_roi.height/2.),
			cv::Point2f(detected_face_roi_.x+detected_face_roi_.width/2,detected_face_roi_.y+detected_face_roi_.height/2.)) > max_face_deslocation)
	{
		face_detected_bool_ = false;

		return 1;
	}


	//This is to avoid big increases in the size of the detected_face_roi
	if(abs(face_roi.width - detected_face_roi_.width)> max_distance_width
			|| abs(face_roi.height - detected_face_roi_.height)> max_distance_height)
	{
		face_roi.x = face_roi.x + face_roi.width/2. -detected_face_roi_.width/2.;
		face_roi.y = face_roi.y + face_roi.height/2. -detected_face_roi_.height/2.;

		face_roi.width = detected_face_roi_.width;
		face_roi.height = detected_face_roi_.height;

	}

	//Check if Face ROI respects image's dimensions
	if(face_roi.x<0 || face_roi.y < 0
	|| (face_roi.x+face_roi.width)>= img.cols || (face_roi.y+face_roi.height)>= img.rows)
		return 1;


	//CAM Shift have succesfully found face
	detected_face_roi_ = face_roi;
	detected_face_ = img(detected_face_roi_);
	face_detected_bool_ = true;
	return 0;
}



double FaceDetector::euclidDist(cv::Point2f pt1, cv::Point2f pt2)
{
	return sqrt(pow(pt1.x-pt2.x, 2) + pow(pt1.y-pt2.y, 2));
}


void FaceDetector::setFaceClassifierPath(std::string face_classifier_path)
{
	face_classifier_.load(face_classifier_path);
}


void FaceDetector::classifierDetect(cv::Mat image, std::vector<cv::Rect>& detections, cv::CascadeClassifier classifier,int flag, cv::Size size)
{
	classifier.detectMultiScale(image, detections, 1.1, 2,  flag, size);//, cv::Size(100,100));

}

unsigned int FaceDetector::detectFacesHaar(cv::Mat image, std::vector<cv::Rect> &faces)
{
	classifierDetect(image,faces,face_classifier_);
	return faces.size();
}

unsigned int FaceDetector::detectFacesAltHaar(cv::Mat image, std::vector<cv::Rect> &faces)
{
        classifierDetect(image,faces,alternative_face_classifier_);
        return faces.size();
}

float FaceDetector::calcDistanceToHead(cv::Mat& head, cv::KalmanFilter& kalman_filter)
{
	float u_left=kalman_filter.statePost.at<float>(0,0)-head.cols/2;
	cv::Mat uv_left(3,1,CV_32F);
	uv_left.at<float>(0,0)=u_left;
	uv_left.at<float>(1,0)=kalman_filter.statePost.at<float>(1,0);
	uv_left.at<float>(2,0)=1;
	float u_right=u_left+head.cols;
	cv::Mat uv_right(3,1,CV_32F);
	uv_right.at<float>(0,0)=u_right;
	uv_right.at<float>(1,0)=kalman_filter.statePost.at<float>(1,0);
	uv_right.at<float>(2,0)=1;


	cv::Mat cart_left=p_.inv()*uv_left;
	cv::Mat cart_right=p_.inv()*uv_right;


	cart_left=cart_left/sqrt(cart_left.dot(cart_left));
	cart_right=cart_right/sqrt(cart_right.dot(cart_right));

	float theta=acos(cart_left.dot(cart_right));
	float l=(HEAD_SIZE/2)/tan(theta/2);
	return l;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "qbo_face_tracking");

	FaceDetector face_detector;

	ros::spin();

	return 0;
}
