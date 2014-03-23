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

#include "face_follower.h"

FaceFollower::FaceFollower()
{
	ROS_INFO("Initializing Qbo face following");
	onInit();
	ROS_INFO("Ready for face following. Waiting for face positions");
}

FaceFollower::~FaceFollower()
{
	deleteROSParams();
	printf("Qbo face tracking successfully ended\n");
}



void FaceFollower::setROSParams()
{
	//Set default value for move base boolean
	private_nh_.param("/qbo_face_following/move_base", move_base_bool_, false);
	
	//set default value for move head boolean
	private_nh_.param("/qbo_face_following/move_head", move_head_bool_, true);

	//Parameters that define the movement when Qbo is searching for faces
	private_nh_.param("/qbo_face_following/search_min_pan", search_min_pan_, -0.3);
	private_nh_.param("/qbo_face_following/search_max_pan", search_max_pan_, 0.3);
	private_nh_.param("/qbo_face_following/search_pan_vel", search_pan_vel_, 0.3);
	private_nh_.param("/qbo_face_following/search_max_tilt", search_max_tilt_, 0.7);
	private_nh_.param("/qbo_face_following/search_min_tilt", search_min_tilt_, 0.7);
	private_nh_.param("/qbo_face_following/search_tilt_vel", search_tilt_vel_, 0.3);
	private_nh_.param("/qbo_face_following/desired_distance", desired_distance_, 1.0);
	private_nh_.param("/qbo_face_following/send_stop", send_stop_, true);
}

void FaceFollower::deleteROSParams()
{
	private_nh_.deleteParam("/qbo_face_following/move_base");
	private_nh_.deleteParam("/qbo_face_following/move_head");
	private_nh_.deleteParam("/qbo_face_following/search_min_pan");
	private_nh_.deleteParam("/qbo_face_following/search_max_pan");
	private_nh_.deleteParam("/qbo_face_following/search_pan_vel");
	private_nh_.deleteParam("/qbo_face_following/search_max_tilt");
	private_nh_.deleteParam("/qbo_face_following/search_min_tilt");
	private_nh_.deleteParam("/qbo_face_following/search_tilt_vel");
	private_nh_.deleteParam("/qbo_face_following/desired_distance");
	private_nh_.deleteParam("/qbo_face_following/send_stop");

}

void FaceFollower::onInit()
{
	/*
	 * Set node's subscriptions
	 */
	camera_info_sub_=private_nh_.subscribe<sensor_msgs::CameraInfo>("/stereo/left/camera_info",10,&FaceFollower::cameraInfoCallback, this);

	/*
	 * Set node's publishers
	 */
	joint_pub_ = private_nh_.advertise<sensor_msgs::JointState>("/cmd_joints", 1); //To move the head
	base_control_pub_=private_nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1); //to move robot's base

	setROSParams();

	yaw_from_joint_=0;
	pitch_from_joint_=0;

	//TODO - Read from rosparam server this value
	min_pitch_ = -0.5;

	//TODO - Should be ROS parameters
	/*
	 * Setting PIDs values
	 */
	//For head's pan movement
	u_act_=0;
	u_prev_=0;
	diff_u_=0;
	kp_u_=0.0040; //0.0066
	ki_u_=0;
	kd_u_=0.001;

	//For head's tilt movement
	v_act_=0;
	v_prev_=0;
	diff_v_=0;
	kp_v_=0.0040;
	ki_v_=0;
	kd_v_=0.001;

	//For base's linear movement
	distance_act_=0;
	distance_prev_=0;
	diff_distance_=0;
	kp_distance_=0.3;
	ki_distance_=0;
	kd_distance_=0.1;

	//For base's angular movement
	yaw_act_=0;
	yaw_prev_=0;
	diff_yaw_=0;
	kp_yaw_=1.3;
	ki_yaw_=0;
	kd_yaw_=0.0;

	/*
	 * Initialize values
	 */
	image_width_ = 320;
	image_height_ = 240;
}

void FaceFollower::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
	if(joint_state->position.size()!=joint_state->name.size())
    {
        ROS_ERROR("Malformed JointState message has arrived. Names size and positions size do not match");
        return;
    }
    for (int i=0;i<(int)joint_state->name.size();i++)
    {
        if (joint_state->name[i].compare("head_pan_joint")==0)
        {
        	yaw_from_joint_=joint_state->position[i];
        	//std::cout << "Mensaje pan: " <<  yaw_from_joint_ << std::endl;
        }
        if (joint_state->name[i].compare("head_tilt_joint")==0)
        {
        	pitch_from_joint_=joint_state->position[i];
        	//std::cout << "Mensaje tilt: " <<  pitch_from_joint_ << std::endl;
        }
    }
}

void FaceFollower::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info)
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

		joint_states_sub_=private_nh_.subscribe<sensor_msgs::JointState>("/joint_states",10,&FaceFollower::jointStateCallback, this);
		face_pos_sub_ = private_nh_.subscribe<qbo_face_msgs::FacePosAndDist>("/qbo_face_tracking/face_pos_and_dist", 10, &FaceFollower::facePositionCallback, this);
	}

}

/*
 * Callback of the face position and size
 * This is the main callback of the node that moves the robot's head and base
 * to follow the face
 */
void FaceFollower::facePositionCallback(const qbo_face_msgs::FacePosAndDistConstPtr& head_pos_size)
{
	image_width_ = head_pos_size->image_width;
	image_height_ = head_pos_size->image_height;

	//Refresh status of move_base
	private_nh_.getParam("/qbo_face_following/move_base", move_base_bool_);
	private_nh_.getParam("/qbo_face_following/move_head", move_head_bool_);

	if(head_pos_size->face_detected) //If a face was detected by the face detector
	{
		/*
		 * Velocities for head movement
		 */
		float pan_vel;
		float tilt_vel;


		/*
		 * HEAD MOVEMENT
		 */
		u_act_=head_pos_size->u;
		diff_u_=u_act_-u_prev_;
		pan_vel=controlPID(u_act_,0,diff_u_,kp_u_,ki_u_,kd_u_);
		u_prev_=u_act_;

		v_act_=head_pos_size->v;
		diff_v_=v_act_-v_prev_;
		tilt_vel=controlPID(v_act_,0,diff_v_,kp_v_,ki_v_,kd_v_);
		v_prev_=v_act_;

		ROS_INFO("Moving head: pos(%lg, %lg) and vel(%lg, %lg)", v_act_,u_act_,tilt_vel,pan_vel);

		if(move_head_bool_)	
			setHeadPositionToFace(v_act_,u_act_,tilt_vel,pan_vel);

		if(!move_base_bool_)
			return;

		/*
		 * BASE MOVEMENT
		 */

		/*
		 * Velocities for base movement
		 */
		float linear_vel = 0;
		float angular_vel = 0;

		distance_act_ = head_pos_size->distance_to_head-desired_distance_;
		diff_distance_=distance_act_-distance_prev_;
		linear_vel=controlPID(distance_act_,0,diff_distance_,kp_distance_,ki_distance_,kd_distance_);
		distance_prev_=distance_act_;

		bool head_near_to_border = ((head_pos_size->v)+(head_pos_size->image_height/2))<100;

		if(pitch_from_joint_ <= min_pitch_ && head_near_to_border && linear_vel>0.0)
		{
			ROS_INFO("Head near border\n");
			linear_vel = 0.0;
		}

//		if (linear_vel>-0.1 && linear_vel < 0.1) linear_vel=0;
//		else if (linear_vel>3.0) linear_vel=2.0;
//		else if (linear_vel<-3.0) linear_vel=-2.0;
//
//		else if (linear_vel<-0.6) linear_vel=-1.3;
//		else if (linear_vel>0.6) linear_vel=1.3;

		//TODO - Use PID control
		yaw_act_ = yaw_from_joint_;
		diff_yaw_=yaw_act_-yaw_prev_;
		angular_vel=controlPID(yaw_act_,0,diff_yaw_,kp_yaw_,ki_yaw_,kd_yaw_);
		yaw_prev_=yaw_act_;

//		if(yaw_from_joint_<-0.3)
//		  angular_vel = -1.1;
//		else if(yaw_from_joint_>0.3)
//		  angular_vel = 1.1;
//		else if(yaw_from_joint_>0.1 && yaw_from_joint_<=0.3)
//			angular_vel = 0.9;
//		else if(yaw_from_joint_<-0.1 && yaw_from_joint_>=-0.3)
//			angular_vel = -0.9;
//		else
//			angular_vel=0;


		//ROS_INFO("Moving base: linear velocity: %lg, angular vel: %lg",linear_vel,angular_vel);
		sendVelocityBase(linear_vel,angular_vel);


	}
	else //Qbo didn't detect any face and it's searching for one
	{
		srand(time(NULL));
		float rand_tilt = search_min_tilt_+((search_max_tilt_-search_min_tilt_)/20.0) * double(rand()%20);
		float rand_pan = search_min_pan_+((search_max_pan_-search_min_pan_)/20.0) * double(rand()%20);


		ROS_INFO("Randomly moving head: pos(%lg, %lg) and vel(%lg, %lg)", rand_tilt, rand_pan, search_tilt_vel_, search_pan_vel_);
		
		if(move_head_bool_)
			setHeadPositionGlobal(rand_tilt, rand_pan, 0.3, 0.3);

		//TODO - Analyse this
		if(move_base_bool_ && send_stop_)
			sendVelocityBase(0,0);
	}

}
/*
 * Given the head position in the image and the velocities to move the head, this will move the head with the
 * given velocity to the destinated position
 */
void FaceFollower::setHeadPositionToFace(float pos_updown, float pos_leftright, float vel_updown, float vel_leftright)
{
	if(p_.data == NULL)
		return;

//	printf("Pos_left_right: %lg\n",pos_leftright);
	
	float pan_pos,tilt_pos;

	pan_pos = pos_leftright+ image_width_/2;
	tilt_pos = pos_updown+ image_height_/2;

	pan_pos = atan2((pan_pos - p_.at<float>(0,2)),p_.at<float>(0,0));
	tilt_pos = atan2((tilt_pos - p_.at<float>(1,2)),p_.at<float>(1,1));
	
//	printf("Angle pos: %lg. Yaw from joint: %lg \n", pan_pos, yaw_from_joint_);


	pan_pos = yaw_from_joint_-pan_pos;
	tilt_pos = tilt_pos + pitch_from_joint_;

//	printf("SUM: %lg\n", pan_pos);
	//printf("Yaw from joint: %lg\n", yaw_from_joint_);
	//printf("Image width: %d\n", image_width_);
	//printf("Image height: %d\n", image_height_);

	if(vel_leftright<0)
		vel_leftright=-vel_leftright;

	if(vel_updown<0)
		vel_updown=-vel_updown;

	

	ROS_INFO("Moving head to face: pos(%lg, %lg) and vel(%lg, %lg)", tilt_pos, pan_pos, vel_updown, vel_leftright);
	
	sensor_msgs::JointState joint_state;
	int servos_count=2;
	joint_state.name.resize(servos_count);
	joint_state.position.resize(servos_count);
	joint_state.velocity.resize(servos_count);

	joint_state.name[0]="head_pan_joint";	//izquierda-derecha

	if(pos_leftright>0)
		joint_state.position[0]=-1.7;
	else
		joint_state.position[0]=1.7;

//	joint_state.position[0]= pan_pos;
	joint_state.velocity[0]=vel_leftright;

	joint_state.name[1]="head_tilt_joint";	//arriba-abajo

	if(pos_updown>0)
		joint_state.position[1]=1.5;
	else
		joint_state.position[1]=-1.5;


//	joint_state.position[1]=tilt_pos;

	joint_state.velocity[1]=vel_updown;

	joint_state.header.stamp = ros::Time::now();

	//publish
	joint_pub_.publish(joint_state);
}


void FaceFollower::setHeadPositionGlobal(float pos_updown, float pos_leftright, float vel_updown, float vel_leftright)
{
    if(pos_leftright<0)
        pos_leftright=-1.5;
    else if(pos_leftright>0)
        pos_leftright=1.5;
/*
    if(pos_updown<0)
        pos_updown=-1.5;
    else if(pos_updown>0)
        pos_updown=1.5;
*/
	if(vel_leftright<0)
		vel_leftright=-vel_leftright;
	if(vel_updown<0)
		vel_updown=-vel_updown;

	sensor_msgs::JointState joint_state;

	int servos_count=2;

	joint_state.name.resize(servos_count);
	joint_state.position.resize(servos_count);
	joint_state.velocity.resize(servos_count);

	joint_state.name[0]="head_pan_joint";
	joint_state.position[0]=pos_leftright;
	joint_state.velocity[0]=vel_leftright;

	joint_state.name[1]="head_tilt_joint";
	joint_state.position[1]=pos_updown;
	joint_state.velocity[1]=vel_updown;

	joint_state.header.stamp = ros::Time::now();
	//publish
	joint_pub_.publish(joint_state);
}

/*
 * Move Qbo's base according to the given linear and angular velocity
 */
void FaceFollower::sendVelocityBase(float linear_vel, float angular_vel)
{
	geometry_msgs::Twist velocidad_base;
	velocidad_base.linear.x=linear_vel;
	velocidad_base.linear.y=0;
	velocidad_base.linear.z=0;

	velocidad_base.angular.x=0;
	velocidad_base.angular.y=0;
	velocidad_base.angular.z=angular_vel;
	//publish

	ROS_INFO("Linear vel: %lg, Angular vel: %lg", linear_vel, angular_vel);

	base_control_pub_.publish(velocidad_base);

}

float FaceFollower::controlPID(float x, float ix, float dx, float Kp, float Ki, float Kd)
{
	float result;
	result=Kp*x+Ki*ix+Kd*dx;
	return result;
}

/*
 * This function is executed when the node is killed
 * It restores the head to its default position
 */
void FaceFollower::headToZeroPosition()
{
	printf("Moving head to its default position\n");

	ros::start();

	sensor_msgs::JointState joint_state;
	int servos_count=2;
	joint_state.name.resize(servos_count);
	joint_state.position.resize(servos_count);
	joint_state.velocity.resize(servos_count);

	joint_state.name[0]="head_pan_joint";	//izquierda-derecha

	joint_state.position[0]=0.;
	joint_state.velocity[0]= 0.2;


	joint_state.name[1]="head_tilt_joint";	//arriba-abajo
	joint_state.position[1]=0.;
	joint_state.velocity[1]=0.2;

	joint_state.header.stamp = ros::Time::now();
	ros::NodeHandle private_nh_2;
	yaw_from_joint_ = 1.0;
	pitch_from_joint_= 1.0;

	ros::Publisher joint_pub_2 = private_nh_2.advertise<sensor_msgs::JointState>("/cmd_joints", 1);
	ros::Subscriber joint_states_=private_nh_2.subscribe<sensor_msgs::JointState>("/joint_states",10,&FaceFollower::jointStateCallback, this);

	ros::Time time_saved = ros::Time::now();
	ros::Duration time_diff;

	while(1 && ros::ok())
	{
		ros::spinOnce();
		//printf("Publishing\n");
		joint_state.header.stamp = ros::Time::now();
		joint_pub_2.publish(joint_state);

		//printf("Published\n");
		if(yaw_from_joint_ == 0 && pitch_from_joint_ == 0)
			break;

		time_diff = ros::Time::now()- time_saved;

		if(time_diff.toSec() >= 4.0)
			break;

	}
	printf("Face Following Node Successfully Ended!\n");

	printf("Qbo face following node successfully ended\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qbo_face_following");


  FaceFollower ff;
  ros::spin();
  ff.headToZeroPosition();

  return 0;
}
