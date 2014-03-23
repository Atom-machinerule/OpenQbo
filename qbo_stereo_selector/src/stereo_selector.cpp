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

#include "stereo_selector.h"

StereoSelector::StereoSelector()  : it_(private_nh_)
{
	ROS_INFO("Initializing Qbo stereo selector");
	onInit();
	ROS_INFO("Ready to select incoming objects");
}

StereoSelector::~StereoSelector() {
	deleteROSParams();
	printf("Qbo face tracking successfully ended\n");

}

void StereoSelector::setROSParams()
{
	private_nh_.param("/qbo_stereo_selector/move_head", move_head_bool_, true);

	/*
	 * Search values when looking for objects
	 */
	private_nh_.param("/qbo_stereo_selector/search_min_pan", search_min_pan_, -0.7);
	private_nh_.param("/qbo_stereo_selector/search_max_pan", search_max_pan_, 0.7);
	private_nh_.param("/qbo_stereo_selector/search_pan_vel", search_pan_vel_, 0.3);
	private_nh_.param("/qbo_stereo_selector/search_max_tilt", search_max_tilt_, -0.7);
	private_nh_.param("/qbo_stereo_selector/search_min_tilt", search_min_tilt_, -0.5);
	private_nh_.param("/qbo_stereo_selector/search_tilt_vel", search_tilt_vel_, 0.3);

	/*
	 * Mean threshold used in find ROI to find the roi with the best mean.
	 * Value between 0 and 255
	 */
	private_nh_.param("/qbo_stereo_selector/roi_mean_threshold", mean_threshold_, 60);

	/*
	 * Mean threshold used in CAM Shift to discriminate bad values of bounding rect found
	 * Value between 0 and 255
	 */
	private_nh_.param("/qbo_stereo_selector/match_threshold_cam_shift", match_threshold_cam_shift_, 70);


	/*
	 * This consists in the number of iterations we will keep moving the head
	 * after have lost the object. It is useful to keep seeking for the object when
	 * it gets out of the image
	 */
	private_nh_.param("/qbo_stereo_selector/undetected_threshold", undetect_threshold_, 10);

	/*
	 * The distance_filter_threshold corresponds to the maximum distance in which incoming objects
	 * are considered. If a point distance surpasses this value, it will not be considered
	 * The value is in meters
	 */
	private_nh_.param("/qbo_stereo_selector/distance_filter_threshold", distance_filter_threshold_, 1.0);

}

void StereoSelector::deleteROSParams()
{
	private_nh_.deleteParam("/qbo_stereo_selector/move_head");
	private_nh_.deleteParam("/qbo_stereo_selector/search_min_pan");
	private_nh_.deleteParam("/qbo_stereo_selector/search_max_pan");
	private_nh_.deleteParam("/qbo_stereo_selector/search_pan_vel");
	private_nh_.deleteParam("/qbo_stereo_selector/search_max_tilt");
	private_nh_.deleteParam("/qbo_stereo_selector/search_min_tilt");
	private_nh_.deleteParam("/qbo_stereo_selector/search_tilt_vel");

	private_nh_.deleteParam("/qbo_stereo_selector/roi_mean_threshold");
	private_nh_.deleteParam("/qbo_stereo_selector/match_threshold_cam_shift");
	private_nh_.deleteParam("/qbo_stereo_selector/undetected_threshold");
	private_nh_.deleteParam("/qbo_stereo_selector/distance_filter_threshold");

}

void StereoSelector::onInit()
{
	/*
	 * Initialize ROS parameters
	 */
	setROSParams();

	//ROS Subscribers
	info_sub_=private_nh_.subscribe<sensor_msgs::CameraInfo>("/stereo/left/camera_info",10,&StereoSelector::infoCallback, this);
	joint_states_=private_nh_.subscribe<sensor_msgs::JointState>("/joint_states",10,&StereoSelector::newJointState, this);


	//ROS Publishers - images
	viewer_image_pub_ = it_.advertise("/qbo_stereo_selector/viewer", 1);
	viewer_object_image_pub_ = it_.advertise("/qbo_stereo_selector/object", 1);

	object_mask_pub_ = private_nh_.advertise<sensor_msgs::Image>("/qbo_stereo_selector/object_mask", 1);

	//ROS Publishers - nose color and head movement
	joint_pub_ = private_nh_.advertise<sensor_msgs::JointState>("/cmd_joints", 1);
	nose_color_pub_ = private_nh_.advertise<qbo_arduqbo::Nose>("/cmd_nose", 1);

	//Initializing values
	image_size_ = cv::Size(320,240);
	object_detected_bool_ = false;

	init_roi_width_=80;
	init_roi_height_=80;

	object_roi_.x = image_size_.width/2-init_roi_width_/2;
	object_roi_.y = image_size_.height/2-init_roi_height_/2;
	object_roi_.width = init_roi_width_;
	object_roi_.height = init_roi_height_;

    undetected_count_ = 0;
    yaw_from_joint_ = 0;
    tilt_from_joint_ = 0;

    u_act_=0;
    u_prev_=0;
    diff_u_=0;
    kp_u_=0.0030; //0.0050
    ki_u_=0.00;
    kd_u_=0.02;

    v_act_=0;
    v_prev_=0;
    diff_v_=0;
    kp_v_=0.0035;
    ki_v_=0.00;
    kd_v_=0.00;
}


void StereoSelector::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info)
{
  if(p_.data==NULL)
  {
    cv::Mat p=cv::Mat(3,4,CV_64F);
    for (int i=0;i<3;i++)
      for (int j=0;j<4;j++)
      {
        p.at<double>(i,j)=info->P[4*i+j];
      }
    p(cv::Rect(0,0,3,3)).convertTo(p_,CV_32F);

	image_sub_=private_nh_.subscribe<sensor_msgs::Image>("/stereo/left/image_rect_color",1,&StereoSelector::imageCallback, this);
	disparity_img_sub_ = private_nh_.subscribe<stereo_msgs::DisparityImage>("/stereo/disparity", 1, &StereoSelector::disparityImageCallback, this);
  }
}

void StereoSelector::newJointState(const sensor_msgs::JointStateConstPtr& msg)
{
	if(msg->position.size()!=msg->name.size())
    {
        ROS_ERROR("Malformed JointState message has arrived. Names size and positions size do not match");
        return;
    }
    for (unsigned int i=0;i<msg->name.size();i++)
    {
        if (msg->name[i].compare("head_pan_joint")==0)
        {
        	yaw_from_joint_=msg->position[i];
        }

        if (msg->name[i].compare("head_tilt_joint")==0)
        {
        	tilt_from_joint_=msg->position[i];
        }
    }
}

void StereoSelector::imageCallback(const sensor_msgs::Image::ConstPtr& image_ptr)
{
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
    image_size_ = cv_ptr->image.size();
    cv::cvtColor(cv_ptr->image, image_received, CV_BGR2RGB);

    qbo_arduqbo::Nose nose;
    nose.header.stamp = ros::Time::now();
    nose.color=0;

    if(object_detected_bool_)
    {
	    /*
	     * Object mask
	     */
		cv_bridge::CvImage cv_image2;
		cv_image2.header.stamp=ros::Time::now();
		cv_image2.header.frame_id="object_mask";
		cv_image2.encoding = "mono8";
		cv_image2.image = object_mask_(object_roi_);
		object_mask_pub_.publish(cv_image2.toImageMsg());

    	/*
    	 * Object Image
    	 */
		cv_bridge::CvImage cv_image;
		cv_image.header.stamp = ros::Time::now();
		cv_image.header.frame_id = "selected_object";
		cv_image.encoding = sensor_msgs::image_encodings::RGB8;
		cv_image.image = cv_ptr->image(object_roi_);
		viewer_object_image_pub_.publish(cv_image.toImageMsg());

		nose.color=2;
    }

    if(move_head_bool_)
    	nose_color_pub_.publish(nose);

    /*
     * Create viewer image
     */
    cv::rectangle(image_received, object_roi_, cv::Scalar(0,255,0), 2);
    cv::Mat image_received_BGR;
    cv::cvtColor(image_received, image_received_BGR, CV_BGR2RGB);
    cv_ptr->image = image_received_BGR;

    //Publish viewer image
    viewer_image_pub_.publish(cv_ptr->toImageMsg());

}

/*
 * The main method of this node, which receives a disparity image and computes the selected ROI of the
 * object
 */
void StereoSelector::disparityImageCallback(const stereo_msgs::DisparityImage::ConstPtr& disp_image_ptr)
{


	//Get last value of the head movement
	private_nh_.getParam("/qbo_stereo_selector/move_head", move_head_bool_);


	sensor_msgs::Image image = disp_image_ptr->image;
	cv_bridge::CvImagePtr cv_ptr;

    try
    {
    	cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
		ROS_ERROR("Error receiving the disparity image. cv_bridge exception: %s", e.what());
		return;
    }

    cv::Mat disparity_image = cv_ptr->image;

    /*
     * Initializing image in which each pixel correspond to the distance of that point
     */
    cv::Mat point_distances = cv::Mat::ones(disparity_image.size(), CV_32FC1);

    //Calculate distances for each point
    point_distances = point_distances*disp_image_ptr->f * disp_image_ptr->T;
    point_distances = point_distances/disparity_image;

    for(int i = 0; i<disparity_image.rows;i++)
    {
    	for(int j = 0; j<disparity_image.cols;j++)
    	{
    		if(disparity_image.at<float>(i,j)<=disp_image_ptr->min_disparity || disparity_image.at<float>(i,j)>disp_image_ptr->max_disparity)
    		{
    			//If point is irregular, consider it to be very far away
    			point_distances.at<float>(i,j) = distance_filter_threshold_*10;

    		}
    	}
    }


    /*
     * Calculating max distance point
     */
    double minVal, maxVal;
    cv::Point minPt, maxPt;
    cv::minMaxLoc(point_distances, &minVal, &maxVal, &minPt, &maxPt);


    /*
     * Normalized point distance image where values are from 0-255, in which closer points
     * have higher values and further points have lower values
     */

    cv::Mat point_distances_normalized = 255-point_distances*255/maxVal;

    //Applying filter using the distance filter
	for(int i = 0; i<point_distances.rows;i++)
	   for(int j = 0; j<point_distances.cols;j++)
			if(point_distances.at<float>(i,j)<=0 || point_distances.at<float>(i,j)>distance_filter_threshold_)
				point_distances_normalized.at<float>(i,j) = 0;

	/*
	 * Distance image representation with 1byte per pixel
	 */
    cv::Mat distance_image_8bit;
    point_distances_normalized.convertTo(distance_image_8bit, CV_8UC1);



    //Computing selection
    if(object_detected_bool_ == false)
    {
    	ROS_INFO("Searching for object using FIND ROI. Best object mean till now: %lg. Distance threshold: %lg",object_roi_mean_, distance_filter_threshold_);
    	object_detected_bool_ = findRoi(point_distances_normalized);

    }
    else
    {
    	ROS_INFO("Selecting object using CAM Shift");
    	detectCamShift(distance_image_8bit);
    }


    //If object found, calculate closest point
    if(object_detected_bool_)
    {
    	object_mask_ = distance_image_8bit;

    	cv::minMaxLoc(point_distances(object_roi_), &minVal, &maxVal, &minPt, &maxPt);
    	object_distance_ = minVal;

    	object_pos_ = cv::Point(object_roi_.x+object_roi_.width/2 - image_size_.width/2,
    			object_roi_.y+object_roi_.height/2-image_size_.height/2);

    	ROS_INFO("OBJECT FOUND -> Position (%d, %d), Distance: %lg", object_pos_.x, object_pos_.y, object_distance_);

    	//Update undetected_count counter
    	undetected_count_ = 0;
    }
    else
    	undetected_count_++;

    if(move_head_bool_)
    	moveHead();

//    cv::imshow("Stereo Selector Image",distance_image_8bit);
//    cv::waitKey(10);
}


/*
 * Given an image, calculates the roi of size (roi_width_,roi_height_) with the best mean values
 * Returns true if it founds a roi with mean value superior to mean_threshold
 */
bool StereoSelector::findRoi(cv::Mat image)
{
	cv::Rect temp_roi;

	double best_mean = 0;

	cv::Scalar mean_scalar;

	temp_roi.x = 0;
	temp_roi.y = 0;
	temp_roi.width = init_roi_width_;
	temp_roi.height = init_roi_height_;

	cv::Point best_location = cv::Point(0,0);

	for(int i = 0; i+init_roi_height_<image.rows; i++)
	{
		temp_roi.y = i;

		for(int j = 0; j+init_roi_width_<image.cols; j++)
		{
			temp_roi.x = j;
			mean_scalar = cv::mean(image(temp_roi));

			if(mean_scalar.val[0]>best_mean)
			{
				best_mean = mean_scalar.val[0];
				best_location = cv::Point(j,i);
			}
		}
	}

	object_roi_mean_ = best_mean;

	if(best_mean>=(double)mean_threshold_)
	{
		temp_roi.x = best_location.x;
		temp_roi.y = best_location.y;

		object_roi_ = temp_roi;
		object_detected_bool_ = true;
		return true;
	}

	object_detected_bool_ = false;
	return false;
}

void StereoSelector::detectCamShift(cv::Mat image)
{
	cv::Rect temp_roi;

	cv::RotatedRect track_box;

	track_box = cv::CamShift( image, object_roi_, cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

	temp_roi = track_box.boundingRect();

	/*Adjusting roi*/
	temp_roi.x = max(0, temp_roi.x);
	temp_roi.y = max(0, temp_roi.y);

	temp_roi.x = min(temp_roi.x, image.cols-1);
	temp_roi.y = min(temp_roi.y, image.rows-1);


	temp_roi.width = min(image.cols-temp_roi.x-2, temp_roi.width);
	temp_roi.height = min(image.rows-temp_roi.y-2, temp_roi.height);

	temp_roi.height = max(0, temp_roi.height);
	temp_roi.width = max(0, temp_roi.width);

	cv::Scalar mean_scalar = cv::mean(image(temp_roi));

	unsigned int max_distance_width_inc = 10;//object_roi_.width/5.;
	unsigned int max_distance_height_inc = 10;//object_roi_.height/5.;


	if(mean_scalar.val[0]<match_threshold_cam_shift_)
	{
		object_detected_bool_ = false;
		return;
	}

	//This is to avoid big increases in the size of the detected_face_roi
	if(abs(temp_roi.width - object_roi_.width)> max_distance_width_inc
			|| abs(temp_roi.height - object_roi_.height)> max_distance_height_inc)
	{
		temp_roi.x = temp_roi.x + temp_roi.width/2. - object_roi_.width/2.;
		temp_roi.y = temp_roi.y + temp_roi.height/2. - object_roi_.height/2.;

		temp_roi.width = object_roi_.width;
		temp_roi.height = object_roi_.height;

	}

	if(temp_roi.x<0 || temp_roi.y < 0
	|| (temp_roi.x+temp_roi.width)>= image.cols ||
	(temp_roi.y+temp_roi.height)>= image.rows)
	{
		object_detected_bool_ = false;
		return;
	}


	object_detected_bool_ = true;
	object_roi_ = temp_roi;
}


void StereoSelector::moveHead()
{
	if(!move_head_bool_) 
		return;

	float pan_velocity, tilt_velocity;

	u_act_ = object_pos_.x;  //The -20 is because we consider the left eye, thus we need a small offset
									 //to keep the head centered in respect to the object
	v_act_ = object_pos_.y;  
	/*
	 * This threshold is used to avoid small movements of the head.
	 * Thus, if the object's position is near to the center of the image, then the head movement is zero
	 */
	float u_threshold = 3;
	float v_threshold = 3;

	if(abs(u_act_) < u_threshold)
		u_act_ = 0;
	else if(u_act_<-1*u_threshold)
		u_act_ = u_act_ +u_threshold;
	else if(u_act_>u_threshold)
		u_act_ = u_act_-u_threshold;

	if(abs(v_act_) < v_threshold)
		v_act_ = 0;
	else if(v_act_<-1*v_threshold)
		v_act_ = v_act_ +v_threshold;
	else if(v_act_>v_threshold)
		v_act_ = v_act_-v_threshold;



    diff_u_=u_act_-u_prev_;
    pan_velocity=controlPID(u_act_,0,diff_u_,kp_u_,ki_u_,kd_u_);
    u_prev_=u_act_;


    diff_v_=v_act_-v_prev_;
    tilt_velocity =controlPID(v_act_,0,diff_v_,kp_v_,ki_v_,kd_v_);
    v_prev_=v_act_;

    setHeadPosition(v_act_, u_act_, tilt_velocity, pan_velocity);
}


void StereoSelector::setHeadPosition(float pos_tilt, float pos_pan, float vel_updown, float vel_leftright)
{
	if(p_.data == NULL)
		return;

	float left_right = 0,up_down = 0;

	/*
	 * Setting maximum speed for pan and tilt movement
	 */
	if(vel_leftright>3.1)
	  vel_leftright = 3.1;
	else if(vel_leftright<-3.1)
	  vel_leftright = -3.1;

	vel_leftright = abs(vel_leftright);
	vel_updown = abs(vel_updown);

	/*
	 * Computing the head position we need to center the object in the image
	 */
	left_right = pos_pan+ image_size_.width/2;
	up_down = pos_tilt+ image_size_.height/2;
	left_right = atan2((left_right - p_.at<float>(0,2)),p_.at<float>(0,0));
	up_down = atan2((up_down - p_.at<float>(1,2)),p_.at<float>(1,1));

	left_right = yaw_from_joint_ - left_right;
	up_down = up_down + tilt_from_joint_;

	sensor_msgs::JointState joint_state;
	int servos_count=2;
	joint_state.name.resize(servos_count);
	joint_state.position.resize(servos_count);
	joint_state.velocity.resize(servos_count);

	joint_state.name[0]="head_pan_joint";

	if(undetected_count_<undetect_threshold_) //If object was detected
	{
		if(pos_pan>0)	
			joint_state.position[0]=-1.7;
		else	
			joint_state.position[0]=1.7; 
		
		//joint_state.position[0]=left_right;
		joint_state.velocity[0]= vel_leftright;
	}
	else //Object was not detected
	{
		srand(time(NULL));
		float rand_pan = search_min_pan_+((search_max_pan_-search_min_pan_)/20.0) * double(rand()%20);
		
		joint_state.position[0]=rand_pan;
		joint_state.velocity[0]= search_pan_vel_;
	}


	joint_state.name[1]="head_tilt_joint";	//If object was detected

	if(undetected_count_ < undetect_threshold_)
	{
		if(pos_tilt>0)	
			joint_state.position[1]=1.5;  //TODO remove the increment
		else	
			joint_state.position[1]=-1.5;  //TODO remove the increment
		//joint_state.position[1]=up_down;  //TODO remove the increment
		joint_state.velocity[1]=vel_updown;
	}
	else //Object was not detected
	{
		srand(time(NULL));
		float rand_tilt = search_min_tilt_+((search_max_tilt_-search_min_tilt_)/20.0) * double(rand()%20);
		joint_state.position[1]=rand_tilt;
		joint_state.velocity[1]=search_tilt_vel_;
	}

	joint_state.header.stamp = ros::Time::now();
	//Publishing to topic to move head
	joint_pub_.publish(joint_state);

	/*
	 * Printing ROS_INFO
	 */
	if(undetected_count_ < undetect_threshold_)
	{
		ROS_INFO("Moving head to object. Pan/Tilt: (%lg, %lg), Pan/Tilt vel: (%lg, %lg)",
				joint_state.position[0], joint_state.position[1], joint_state.velocity[0], joint_state.velocity[1]);
	}
	else
	{
		ROS_INFO("Randomly moving head. Pan/Tilt: (%lg, %lg), Pan/Tilt vel: (%lg, %lg)",
				joint_state.position[0], joint_state.position[1], joint_state.velocity[0], joint_state.velocity[1]);
	}
}

void StereoSelector::headToZeroPosition()
{
	if(!move_head_bool_)
		return;

	ros::start();

	sensor_msgs::JointState joint_state;
	int servos_count=2;
	joint_state.name.resize(servos_count);
	joint_state.position.resize(servos_count);
	joint_state.velocity.resize(servos_count);

	joint_state.name[0]="head_pan_joint";

	joint_state.position[0]=0.;
	joint_state.velocity[0]= 0.2;


	joint_state.name[1]="head_tilt_joint";
	joint_state.position[1]=0.;
	joint_state.velocity[1]=0.2;

	joint_state.header.stamp = ros::Time::now();
	ros::NodeHandle private_nh_2;
	yaw_from_joint_ = 1.0;
	tilt_from_joint_= 1.0;

	ros::Publisher joint_pub_2 = private_nh_2.advertise<sensor_msgs::JointState>("/cmd_joints", 1);
	ros::Subscriber joint_states_=private_nh_2.subscribe<sensor_msgs::JointState>("/joint_states",10,&StereoSelector::newJointState, this);
	ros::Publisher nose_pub = private_nh_2.advertise<qbo_arduqbo::Nose>("/cmd_nose", 1);

    qbo_arduqbo::Nose nose;
    nose.header.stamp = ros::Time::now();
    nose.color=0;

	printf("\nPublishing center head\n");

	ros::Time time_saved = ros::Time::now();
	ros::Duration time_diff;

	while(ros::ok())
	{
		ros::spinOnce();
		nose.header.stamp = ros::Time::now();
		joint_state.header.stamp = ros::Time::now();

		//Publishing nose color and head to zero position
		joint_pub_2.publish(joint_state);
		nose_pub.publish(nose);
		sleep(1);
		if(yaw_from_joint_ == 0 && tilt_from_joint_ == 0)
			break;

		time_diff = ros::Time::now()- time_saved;
		if(time_diff.toSec() >= 6.0) break;

	}
	printf("Stereo Selector Node Successfully Ended!\n");
}



float StereoSelector::controlPID(float x, float ix, float dx, float Kp, float Ki, float Kd)
{
	float result;
	result=Kp*x+Ki*ix+Kd*dx;
	return result;
}


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "qbo_stereo_selector");

	StereoSelector stereo_selector;
	ros::spin();
	//stereo_selector.headToZeroPosition();

	return 0;
}
