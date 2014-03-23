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

#include "qbo_detector.h"


QboDetector::QboDetector() : it_(private_nh_)
{
	ROS_INFO("Initializing Qbo detector");
	onInit();
	ROS_INFO("Ready to attend the Service for Qbo Recognition");
}

void QboDetector::onInit()
{
	/*
	 * Setting ROS parameters
	 */
	codeword_size_ = 3;
	codeword_time_diff_ = 5;
	codeword_checks_num_ = 5;
	good_ratio_threshold_ = 0.6;
	num_of_samples_ = 5;

	service_= private_nh_.advertiseService("qbo_self_recognizer/recognize", &QboDetector::recognizeQboService, this);

}

void QboDetector::imageCallback(const sensor_msgs::Image::ConstPtr& image_ptr)
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
    cvtColor(image_received, image_received, CV_BGR2RGB);

    /*
     * Get circles from image
     */
    cv::Mat gray;

    cvtColor(image_received, gray, CV_BGR2GRAY);
   // cv::equalizeHist(gray, gray);
    cv::GaussianBlur( gray,gray, cv::Size(3, 3), 1, 1 );
    vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, gray.rows/4, 200, 100, 30 );

    for (unsigned int i = 0; i < circles.size(); i++)
    {
         cv::circle( image_received, cvPoint(cvRound(circles[i][0]),cvRound(circles[i][1])),3, CV_RGB(0,255,0), -1, 8, 0 );
         cv::circle( image_received, cvPoint(cvRound(circles[i][0]),cvRound(circles[i][1])), cvRound(circles[i][2]), CV_RGB(255,0,0), 3, 8, 0 );
    }

    /*
     * If a circle was detected in the image
     */
    if(circles.size()>0)
    {
        index_count_++;
    	/*
    	 * Calc the largest square within circle length value
    	 */
		float square_side = 2*circles[0][2]*sqrt(2)/2;  //2*r*sin(45 degrees)


		/*
		 * Define square of the nose zone
		 */
		cv::Rect nose_roi;
		nose_roi.width = square_side/2;
		nose_roi.height = square_side/2;
		nose_roi.x = circles[0][0] - nose_roi.width/2;
		nose_roi.y = circles[0][1] - nose_roi.height/2 + nose_roi.height/4;


		/*
		 * Adjust square to image dimentions
		 */
		if(nose_roi.width>nose_roi.x + nose_roi.width) //Square is in the left border
		{
			nose_roi.width = nose_roi.x + nose_roi.width;
			nose_roi.x = 0;
		}
		if(nose_roi.width>gray.cols - (nose_roi.x +1)) //Square is in the right border
		{
			nose_roi.width = gray.cols - (nose_roi.x +1);
			nose_roi.x = gray.cols-nose_roi.width;
		}
		if(nose_roi.height>nose_roi.y + nose_roi.height) //Square is in the bottom border
		{
			nose_roi.height = nose_roi.y + nose_roi.height;
			nose_roi.y = 0;
		}
		if(nose_roi.height>gray.rows - (nose_roi.y +1)) //Square is in the top border
		{
			nose_roi.height = gray.rows - (nose_roi.y +1);
			nose_roi.y = gray.rows-nose_roi.height;
		}

	    double mean_value = cv::mean(gray(nose_roi)).val[0];

	    if(state_ == QboDetector::CALIBRATING_OFF)
	    {

	    	if(index_count_<5) //Ignore first 5 images
			{
				ROS_INFO("Accessing first images from camera");
				
				
				if(index_count_<=1)
				{
					/*
					 * Turn off nose
					 */
					qbo_arduqbo::Nose nose;
					nose.header.stamp = ros::Time::now();
			
					nose.color=0;
					ros::Time saved_time = ros::Time::now();
					ros::Duration diff_t;
					/*
					* Publishing nose to turn off
					*/
					while(diff_t.toSec()<0.2)
					{
						nose_color_pub_.publish(nose);
						diff_t = ros::Time::now() - saved_time;				
					}
					
					cv::waitKey(500);
				}
				
				cv::waitKey(200);
			}
			else if(nose_off_sample_.rows<num_of_samples_) //Collect sample data from the first 50 nose values
			{
				nose_off_sample_.push_back(mean_value);

				ROS_INFO("Collecting sample %d", nose_off_sample_.rows);

				cv::Scalar mean, stddev;
				cv::meanStdDev(nose_off_sample_, mean, stddev);

				/*
				 * Calc mean value of the brightness value
				 */
				nose_off_mean_ = mean.val[0];
				nose_off_stddev_ = stddev.val[0];

				ROS_INFO("Mean value for Nose OFF: %lg, Std_dev = %lg\n", nose_off_mean_, nose_off_stddev_);
			}
			else
			{
				if(nose_off_stddev_ > 4)
				{
					nose_off_sample_ = cv::Mat::zeros(0,1,CV_64FC1);
					
					ROS_INFO("RE-CALIBRATING NOSES BRIGTHNESS - NOSE OFF");
				}
				else
				{
					state_ = QboDetector::CALIBRATING_ON;
					ROS_INFO("CALIBRATING NOSES BRIGTHNESS - NOSE ON");
				}
				index_count_ = 0;
				
			}
	    }
	    else if(state_ == QboDetector::CALIBRATING_ON)
	    {

			if(index_count_<5) //Ignore first 5 images
			{

				ROS_INFO("Accessing first images from camera");

				if(index_count_<=1)
				{
					/*
					 * Turn off nose
					 */
					qbo_arduqbo::Nose nose;
					nose.header.stamp = ros::Time::now();

					nose.color=2;
					ros::Time saved_time = ros::Time::now();
					ros::Duration diff_t;
					/*
					* Publishing nose to turn ON
					*/
					while(diff_t.toSec()<0.2)
					{
						nose_color_pub_.publish(nose);
						diff_t = ros::Time::now() - saved_time;
					}
					
					cv::waitKey(500);
				}
			}
			else if(nose_on_sample_.rows<num_of_samples_) //Collect sample data from the first 50 nose values
			{
				nose_on_sample_.push_back(mean_value);

				ROS_INFO("Collecting sample %d", nose_on_sample_.rows);

				cv::Scalar mean, stddev;
				cv::meanStdDev(nose_on_sample_, mean, stddev);

				/*
				 * Calc mean value of the brightness value
				 */
				nose_on_mean_ = mean.val[0];
				nose_on_stddev_ = stddev.val[0];

				ROS_INFO("Mean value for Nose On: %lg, Std_dev = %lg\n", nose_on_mean_, nose_on_stddev_);
			}
			else
			{
			
			 	if(nose_on_stddev_ > 4)
				{
					nose_on_sample_ = cv::Mat::zeros(0,1,CV_64FC1);
					index_count_=0;
					
					ROS_INFO("RE-CALIBRATING NOSES BRIGTHNESS - NOSE ON");
					return;
				}

				/*
				 * Compute Codeword
				 */
				ROS_INFO("Generating CODEWORD...");

				srand((unsigned)time(0));

				bool on_exists = false;
				bool off_exists = false;

				while(!on_exists || !off_exists)
				{
					codeword_.clear();
					for(int i = 0; i<codeword_size_; i++)
					{
						bool random_bool = (rand()%2 == 0);

						if(random_bool)
							on_exists = true;
						else
							off_exists = true;

						codeword_.push_back(random_bool);
						cv::waitKey(100);
					}
				}

				/*
				 * Printing Code Word
				 */

				std::stringstream ss;

				for(int i = 0; i<codeword_size_; i++)
				{
					if(codeword_[i])
						ss << "1";
					else
						ss << "0";

					if(i<codeword_size_-1)
						ss<<", ";
				}

				ROS_INFO("CODEWORD GENERATED: %s", ss.str().c_str());

				current_word_idx_ = 0;

				/*
				 * Change nose color according to the first word
				 */
				qbo_arduqbo::Nose nose;
				nose.header.stamp = ros::Time::now();

				if(codeword_[current_word_idx_])
					nose.color=2;
				else
					nose.color=0;

			    ros::Time saved_time = ros::Time::now();
				ros::Duration diff_t;
				/*
				* Publishing nose to turn off
				*/
				while(1)
				{
					nose_color_pub_.publish(nose);
					diff_t = ros::Time::now() - saved_time;
				 	if(diff_t.toSec()>0.3)
				 		break;
					
				}

				//Wait a little bit for the nose color to be set
				cv::waitKey(500);

				word_time_ = ros::Time::now();
				state_ = QboDetector::PROCESSING_CODEWORD;

			}
	    }
	    /*
	     * Process Codeword one by one
	     */
	    else if(state_ == QboDetector::PROCESSING_CODEWORD)
	    {
	    	ros::Duration diff_time = ros::Time::now() - word_time_;

	    	if(diff_time.toSec()>codeword_time_diff_ || curr_codeword_values_.size()>=codeword_checks_num_) //Check if need to change Word
	    	{
	    		/*
	    		 * Evaluation
	    		 */

	    		//Count good values
	    		double good_ratio = 0;

	    		for(unsigned int i = 0; i<curr_codeword_values_.size();i++)
	    		{
	    			if(curr_codeword_values_[i] == codeword_[current_word_idx_])
	    				good_ratio++;
	    		}

	    		good_ratio = good_ratio/curr_codeword_values_.size();

	    		ROS_INFO("Evaluation for Word %d: %lg", current_word_idx_+1, good_ratio);

	    		if(good_ratio<good_ratio_threshold_)
	    		{
	    			recognized_ = false;
	    			state_ = QboDetector::FINISHED;

	    			ROS_INFO("Evaluation has been REFUSED for threshold %lg. Current value was %lg", good_ratio_threshold_, good_ratio);
	    			return;
	    		}
	    		else if(current_word_idx_ == (unsigned int)(codeword_size_)-1) //If this is last word
	    		{
		    		ROS_INFO("Evaluation for last word was ACCEPTED for threshold %lg. Current value was %lg", good_ratio_threshold_, good_ratio);
	    			recognized_ = true;
	    			state_ = QboDetector::FINISHED;
	    			return;
	    		}

	    		ROS_INFO("Evaluation has been ACCEPTED for threshold %lg. Current value was %lg", good_ratio_threshold_, good_ratio);


	    		ROS_INFO("CHANGING WORD...\n");

	    		current_word_idx_++;
	    		curr_codeword_values_.clear();
				/*
				 * Change nose color according to the next codeword
				 */
				qbo_arduqbo::Nose nose;
				nose.header.stamp = ros::Time::now();

				if(codeword_[current_word_idx_])
					nose.color=2;
				else
					nose.color=0;

			    ros::Time saved_time = ros::Time::now();
				ros::Duration diff_t;
				/*
				* Publishing nose to turn off
				*/
				while(1)
				{
					nose_color_pub_.publish(nose);
					diff_t = ros::Time::now() - saved_time;
				 	if(diff_t.toSec()>0.3)
				 		break;

				}

				//Wait a little bit for the nose color to be set
				cv::waitKey(800);

				word_time_ = ros::Time::now();


	    	}


	    	/*
	    	 * Check if seen Nose is turn on or off
	    	 */
	    	double off_diff = abs(nose_off_mean_-mean_value);
	    	double on_diff = abs(nose_on_mean_-mean_value);

			printf("Mean value: %lg, Diff Off= %lg, Diff On= %lg, Nose Off Mean=%lg, Nose On Mean=%lg\n",
		    		mean_value, off_diff, on_diff, nose_off_mean_, nose_on_mean_); 
            
  

		    if(on_diff<=off_diff)
		    {
		    	ROS_INFO("Word %d -> Expected Nose Value: %s, Detected Nose: true", current_word_idx_+1, (codeword_[current_word_idx_])?"true":"false");

		    	curr_codeword_values_.push_back(true);
		    }
		    else
		    {
		    	ROS_INFO("Word %d -> Expected Nose Value: %s, Detected Nose: false",current_word_idx_+1, (codeword_[current_word_idx_])?"true":"false");
		    	curr_codeword_values_.push_back(false);


		    	/*
				 * Update mean value of the brightness value
				 */
		    	nose_off_sample_.push_back(mean_value);
		    	cv::Scalar mean, stddev;
		    	cv::meanStdDev(nose_off_sample_, mean, stddev);

		    	nose_off_mean_ = mean.val[0];
		    	nose_off_stddev_ = stddev.val[0];

		    }
	    }

		/*
		 * Drawing
		 */
		cv::rectangle(image_received, nose_roi, cv::Scalar(255,0,0), 2);
    }


    //cv::imshow( "Qbo Detector", image_received );
    //cv::imshow( "Gray", gray );
    //cv::waitKey(100);

}

bool QboDetector::recognizeQboService(qbo_self_recognizer::QboRecognize::Request  &req, qbo_self_recognizer::QboRecognize::Response &res )
{

	ROS_INFO("Qbo Recognize Service Start!");

    /*
     * Publishers
     */
	//Publisher of the nose color
	nose_color_pub_ = private_nh_.advertise<qbo_arduqbo::Nose>("/cmd_nose", 4);



    nose_off_sample_ = cv::Mat::zeros(0,1,CV_64FC1);
    nose_on_sample_ = cv::Mat::zeros(0,1,CV_64FC1);
    index_count_ = 0;
    current_word_idx_ = 0;
    codeword_.clear();
    recognized_ = false;
    state_ = QboDetector::CALIBRATING_OFF;
    curr_codeword_values_.clear();

	/*
	 * Subscribe to topics
	 */

    ROS_INFO("CALIBRATING NOSES BRIGTHNESS - NOSE OFF");

    image_sub_ = private_nh_.subscribe<sensor_msgs::Image>("/stereo/left/image_color",1,&QboDetector::imageCallback, this);



	while(state_ != QboDetector::FINISHED && ros::ok())
	{
		ros::spinOnce();
	}


	image_sub_.shutdown();

	res.recognized = recognized_;

	if(recognized_)
		ROS_INFO("Qbo reflection was RECOGNIZED!");
	else
		ROS_INFO("Qbo reflection was NOT RECOGNIZED");

	ROS_INFO("Qbo Recognize Service Ended!");
	return true;
}

QboDetector::~QboDetector()
{
	printf("Qbo detector successfully ended\n");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "qbo_detector");

	QboDetector qbo_detector;

	ros::spin();

	return 0;
}
