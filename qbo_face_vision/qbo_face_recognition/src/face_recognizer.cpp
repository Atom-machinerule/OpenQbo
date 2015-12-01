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

#include "face_recognizer.h"

FaceRecognizer::FaceRecognizer() {
	
	ROS_INFO("Initializing Qbo face recognition");
	onInit();
	ROS_INFO("Ready to recognize faces. Awaiting incoming images");

}

FaceRecognizer::~FaceRecognizer() {
	deleteROSParams();
}

void FaceRecognizer::setROSParams()
{

	string base_faces_path = "/opt/ros/hydro/qbo_face_vision/qbo_face_recognition/faces";

	//Make sure the default folders exist
	if(!boost::filesystem::is_directory(base_faces_path))
		boost::filesystem::create_directory(base_faces_path);

	if(!boost::filesystem::is_directory(base_faces_path+"/faces_db"))
		boost::filesystem::create_directory(base_faces_path+"/faces_db");
		
	if(!boost::filesystem::is_directory(base_faces_path+"/new_faces"))
		boost::filesystem::create_directory(base_faces_path+"/new_faces");

	//Setting default path for faces database
	string default_faces_db_path = "/opt/ros/hydro/qbo_face_vision/qbo_face_recognition/faces/faces_db/";

	//Set default parameter for faces database
	private_nh_.param("/qbo_face_recognition/faces_path", faces_db_path_, default_faces_db_path);

	//Setting default path for the update path, i.e., the folder where the temporary images to be learned
	//are going to be stored
	string default_update_path = "/opt/ros/hydro/qbo_face_vision/qbo_face_recognition/faces/new_faces/";
	//Set default parameter for faces database
	private_nh_.param("/qbo_face_recognition/uptate_path", update_path_, default_update_path);


	int default_recog_type = BAG_OF_WORDS_SVM;
	//Type of recognition: 0 - PCA with SVM,  1 - BAG of Words with SVM. Default is Bags of Word with SVM
	private_nh_.param<int>("/qbo_face_recognition/recognition_type", recog_type_, default_recog_type);

	/*
	 * Stabilizer parameters
	 */
	private_nh_.param<int>("/qbo_face_recognition/stabilizer_threshold", stabilizer_threshold_, 4);
	private_nh_.param<int>("/qbo_face_recognition/stabilizer_max", stabilizer_max_, 7);

	/*
	 * For Bags of Words Approach
	 */
	//Number of SURF descriptors to be extracted per face image
	private_nh_.param<int>("/qbo_face_recognition/num_of_desc_per_face_", num_of_desc_per_face_, 50);

	//Certainty of the SVM classifiers. This values must be in the interval (0,1]
	private_nh_.param<double>("/qbo_face_recognition/bow_certainty_threshold", bow_certainty_threshold_, 0.05);
	//SURF descriptors match threshold
	private_nh_.param<double>("/qbo_face_recognition/descriptors_match_threshold", descriptors_match_threshold_, 0.23);

	//Linear kernel for the SVM classifiers?
	private_nh_.param("/qbo_face_recognition/linear_kernel", linear_kernel_, false);

	/*
	 * PCA parameters
	 */
	private_nh_.param<int>("/qbo_face_recognition/pca_dimension", pca_dimension_, 40);
	private_nh_.param<int>("/qbo_face_recognition/pca_image_height", image_size_.height, 100);
	private_nh_.param<int>("/qbo_face_recognition/pca_image_width", image_size_.width, 100);

	
	/*
	* Parameters for the service learn_faces
	*/
	
	//Number of images to store when learning faces
	private_nh_.param<int>("/qbo_face_recognition/num_images_to_hold", num_images_to_hold_, 20);
	
	string default_face_topic = "/qbo_face_tracking/face_image";
	
	private_nh_.param("/qbo_face_recognition/face_topic", face_topic_, default_face_topic);

	//Maximum allowed time to capture images. In seconds.
	private_nh_.param<double>("/qbo_face_recognition/max_time_to_learn", max_time_to_learn, 30.0);

}

void FaceRecognizer::deleteROSParams()
{
	private_nh_.deleteParam("/qbo_face_recognition/faces_path");
	private_nh_.deleteParam("/qbo_face_recognition/uptate_path");
	private_nh_.deleteParam("/qbo_face_recognition/recognition_type");
	private_nh_.deleteParam("/qbo_face_recognition/stabilizer_threshold");
	private_nh_.deleteParam("/qbo_face_recognition/stabilizer_max");
	private_nh_.deleteParam("/qbo_face_recognition/num_of_desc_per_face_");
	private_nh_.deleteParam("/qbo_face_recognition/bow_certainty_threshold");
	private_nh_.deleteParam("/qbo_face_recognition/descriptors_match_threshold");
	private_nh_.deleteParam("/qbo_face_recognition/linear_kernel");
	private_nh_.deleteParam("/qbo_face_recognition/pca_dimension");
	private_nh_.deleteParam("/qbo_face_recognition/pca_image_height");
	private_nh_.deleteParam("/qbo_face_recognition/pca_image_width");
}

void FaceRecognizer::onInit()
{
	/*
	 * Initialize ROS parameters
	 */
	setROSParams();

	//Initialize Feature Detector and descriptor extractor for BOW approach
	descriptor_detector_ = new cv::GridAdaptedFeatureDetector(new cv::SurfFeatureDetector, num_of_desc_per_face_, 1, 1);
	descriptor_extractor_ = new cv::SurfDescriptorExtractor;

    //Initialize Bow Img Descriptor Extractor
    bowExtractor_ = new cv::BOWImgDescriptorExtractor(descriptor_extractor_,
    						new cv::BruteForceMatcher<cv::L2<float> >);

	ROS_INFO("Loading persons' names...");
	//Preparing face recognizer
	loadPersonsNames();
	ROS_INFO("%u persons' names collected!\n", (unsigned int)persons_.size());

	switch(recog_type_)
	{
		case BAG_OF_WORDS_SVM:

			ROS_INFO("-->Using Bags of Words with SURF descriptors and SVMs approach\n");

			if(persons_.size() == 0)
			{
				ROS_INFO("No images collected yet because there are no persons in the data base");
			}
			else if(loadVocabulary() == false || loadBOW_SVMClassifiers() == false)
			{
				ROS_INFO("Vocabulary and SVM not loaded successfully\n");

				ROS_INFO("Loading persons' images...");
				int num_img = loadAllPersonsImages();
				ROS_INFO("Persons' images loaded: %d!", num_img);

				if(num_img>0)
				{
					ROS_INFO("Loading persons' descriptors...");
					loadAllPersonsDescriptors();
					ROS_INFO("Persons' descriptors loaded!\n");
				}

				if(persons_.size()>=2)
				{
					ROS_INFO("Training Vocabulary");
					trainVocabulary();
					ROS_INFO("Vocabulary trained and stored in %s!\n", faces_db_path_.c_str());

					ROS_INFO("Training BOW SVM classifiers");
					trainBOW_SVMClassifiers();
					ROS_INFO("BOW SVM classifiers trained\n");
				}
			}
			else
				ROS_INFO("Vocabulary and BOW SVM loaded successfully!\n");

			break;

		case PCA_SVM:

			ROS_INFO("-->Using PCA with SVM approach\n");

			

			if(persons_.size() == 0)
			{
				ROS_INFO("No images collected yet because there are no persons in the data base");
			}
			else if(loadPCA() == false || loadPCA_SVMClassifiers() == false)
			{
				ROS_INFO("PCA and SVMs not loaded successfully\n");

				ROS_INFO("Loading persons' images...");
				int num_img = loadAllPersonsImages();
				ROS_INFO("Persons' images loaded: %d!\n", num_img);

				if(persons_.size()>=2)
				{
					ROS_INFO("Building PCA from %d images", num_img);
					buildPCA();
					ROS_INFO("PCA built with a dimensionality of %d!", pca_dimension_);
					ROS_INFO("Storing new PCA...");
					storePCA();
					ROS_INFO("New PCA stored in %s!\n", faces_db_path_.c_str());


					ROS_INFO("Training SVM classifiers");
					trainPCA_SVMClassifiers();
					ROS_INFO("SVM classifiers trained and stored!\n");
				}
			}
			else
				ROS_INFO("PCA and SVMs loaded successfully!");

			break;

		default:
			break;
	};


	/*
	 * Advertise ROS services
	 */
	service_= private_nh_.advertiseService("/qbo_face_recognition/recognize", &FaceRecognizer::recognizeService, this);
	service3_= private_nh_.advertiseService("/qbo_face_recognition/recognize_with_stabilizer", &FaceRecognizer::recognizeStabilizerService, this);
	service2_= private_nh_.advertiseService("/qbo_face_recognition/get_name", &FaceRecognizer::getNameService, this);
	service4_= private_nh_.advertiseService("/qbo_face_recognition/train", &FaceRecognizer::trainService, this);
	service5_= private_nh_.advertiseService("/qbo_face_recognition/learn_faces", &FaceRecognizer::learnFaces, this);
}

/*
 * Load the persons, given the faces db path. Fills the vector of Persons
 *
 */
int FaceRecognizer::loadPersonsNames()
{

	if(faces_db_path_.compare("") == 0) //Check if faces_db_path_ contains a path
	{
		ROS_ERROR("Error: Faces data base folder was not specified");
		exit(-1); //Return error
	}

	if(!boost::filesystem::is_directory(faces_db_path_))
	{
		ROS_ERROR("Error: Faces data base folder is not valid");
		exit(-1);
	}

	//Get the folders names that represent the different person in our database
	for (boost::filesystem::directory_iterator itr(faces_db_path_); itr!=boost::filesystem::directory_iterator(); ++itr)
	{
		if (boost::filesystem::is_directory(itr->status()))
		{

			std::string person_name=itr->path().filename().string();

			if(person_name.at(0)=='.')
				continue;

			Person person(person_name);

			person.images_dir_path_ = faces_db_path_+"/"+person_name;
			persons_.push_back(person);
			ROS_INFO("\tName collected: %s", person_name.c_str());
		}
	}

	return 0;
}

bool FaceRecognizer::storePCA()
{
	string pca_file_name = faces_db_path_+"/"+"pca.xml.gz";

	cv::FileStorage fs(pca_file_name, cv::FileStorage::WRITE);

	if( !fs.isOpened() )
	{
		//If I cannot open the file for writing, send an error
		ROS_ERROR("Failed to save PCA to file");
		return false;
	}

	fs << "PCA_mean" << pca_.mean;
	fs << "PCA_eigenvalues" << pca_.eigenvalues;
	fs << "PCA_eigenvectors" << pca_.eigenvectors;

	return true;
}

bool FaceRecognizer::loadPCA(string path)
{

	if(path.compare("") == 0)
		path = faces_db_path_;

	if(path.compare("") == 0)
	{
		ROS_ERROR("Fail loading PCA: invalid path");
		return false;
	}

	string file_name = path+"/"+"pca.xml.gz";

    cv::FileStorage fs(file_name, cv::FileStorage::READ);

    if(fs.isOpened())
    {
        fs["PCA_mean"] >> pca_.mean;
    	fs["PCA_eigenvalues"] >> pca_.eigenvalues;
    	fs["PCA_eigenvectors"] >> pca_.eigenvectors;

    	pca_dimension_ = pca_.eigenvalues.rows;

    	stabilizer_states_.clear();
    	for(unsigned int i = 0 ; i<=persons_.size();i++)
    	{
    		stabilizer_states_.push_back(0);
    	}

        return true;
    }

    return false;
}

/*
 * Load svm.xml.gz classifier files. Returns true if succesfully loaded all svm files.
 * Return false if for at least one person, an svm file has not been found
 */
bool FaceRecognizer::loadPCA_SVMClassifiers()
{
	/*
	 *
	 * OBSOLETE FOR NEW PCA
	 */
	/*
	for(unsigned int i = 0; i < persons_.size();i++)
	{
	    // first check if a previously trained svm for the current class has been saved to file
	    string svmFilename = persons_[i].images_dir_path_+"/pca_svm.xml.gz";

	    cv::FileStorage fs(svmFilename, cv::FileStorage::READ);
	    if( fs.isOpened() )
	    {
	    	persons_[i].pca_svm_ = new CvSVM;
	    	persons_[i].pca_svm_->load(svmFilename.c_str() );
	    }
	    else
	    	return false;
	}
	*/

	for(unsigned int i = 0; i < persons_.size();i++)
	{
	    // first check if a previously trained svm for the current class has been saved to file
	    string projFilename = persons_[i].images_dir_path_+"/pca_proj.xml.gz";

	    cv::FileStorage fs(projFilename, cv::FileStorage::READ);

	    if( fs.isOpened() )
	    {
	    	fs["pca_proj"] >> persons_[i].pca_proj_;

	    	if(persons_[i].pca_proj_.cols != pca_dimension_)
	    		return false;

	    }
	    else
	    	return false;
	}


	return true;
}

/*
 * Trains the SVM Classifiers for each Person, using the PCA
 */
void FaceRecognizer::trainPCA_SVMClassifiers()
{
	for(unsigned int i = 0; i< persons_.size(); i++) //For each person
	{
		//Projections matrix
		cv::Mat train_data(0, pca_dimension_, CV_32FC1);

		for(unsigned int j = 0; j<persons_[i].images_.size(); j++) //For each person's image
		{
				cv::Mat temp, temp2;
				equalizeFace(persons_[i].images_[j]).convertTo(temp, CV_32FC1);
				temp.reshape(1,1).copyTo(temp2);
				cv::Mat coeffs = pca_.project(temp2);
				train_data.push_back(coeffs);
		}
		
		//Assign projections to person
		persons_[i].pca_proj_ = train_data;

		string file_name = persons_[i].images_dir_path_+"/"+"pca_proj.xml.gz";

	    cv::FileStorage fs( file_name, cv::FileStorage::WRITE );
	    if( !fs.isOpened() )
	    {
	    	//If I cannot open the file for writing, send an error
	    	return;
	    }

	    //Save PCA projections
	    fs << "pca_proj" << train_data;

	    ROS_INFO("\t PCA Projections stored for %s ... ",persons_[i].name_.c_str());
	}



	/*
	* OBSOLETE FOR NEW PCA
	for(unsigned int i = 0; i< persons_.size(); i++) //For each person
	{
		//Build SVM classifier for person i
		persons_[i].pca_svm_ = new CvSVM;
		//Preparing data to train its SVM
		cv::Mat train_data(0, pca_dimension_, CV_32FC1);
		cv::Mat responses(0, 1,  CV_32FC1);
		ROS_INFO("\t Building SVM Classifier for %s", persons_[i].name_.c_str());

		for(unsigned int i2 = 0; i2<persons_.size(); i2++) //For each person in which we will get image
		{
			for(unsigned int j = 0; j<persons_[i2].images_.size(); j++) //For eachs person's image
			{
				cv::Mat temp, temp2;
				equalizeFace(persons_[i2].images_[j]).convertTo(temp, CV_32FC1);
				temp.reshape(1,1).copyTo(temp2);
				cv::Mat coeffs = pca_.project(temp2);

				train_data.push_back(coeffs);
				cv::Mat response_mat= cv::Mat::ones(1,1,  CV_32FC1);

				if(i!=i2) //person present at this image?
					response_mat*=-1.; //If not

				responses.push_back(response_mat);
			}
		}


        ROS_INFO("\t Training classifier for %s ... ",persons_[i].name_.c_str());
		//SVM Training
        CvSVMParams svmParams;
        setSVMParams( svmParams, responses);
        CvParamGrid c_grid, gamma_grid, p_grid, nu_grid, coef_grid, degree_grid;
        setSVMTrainAutoParams( c_grid, gamma_grid,  p_grid, nu_grid, coef_grid, degree_grid );

        persons_[i].pca_svm_->train_auto(train_data, responses, cv::Mat(), cv::Mat(), svmParams, 10,
        		c_grid, gamma_grid, p_grid, nu_grid, coef_grid, degree_grid );

        //svms_[i]->train(train_data, responses);
        string svm_file = persons_[i].images_dir_path_+"/pca_svm.xml.gz";
        //Saving svm to a file
        persons_[i].pca_svm_->save(svm_file.c_str());

        ROS_INFO("\t Classifier trained and stored in %s!\n",svm_file.c_str());
//        ROS_INFO("Testing classifier for %s", person_names_[i].c_str());
//
//		for(unsigned int i2 = 0; i2<person_imgs_.size(); i2++) //For each person in which we will get image
//		{
//			for(unsigned int j = 0; j<person_imgs_[i2].size(); j++) //For eachs person's image
//			{
//
//				cv::Mat temp, temp2;
//
//				person_imgs_[i2][j].convertTo(temp, CV_32FC1);
//
//				temp.reshape(1,1).copyTo(temp2);
//
//				cv::Mat coeffs = pcaBW_.project(temp2);
//
//				float confidence = -1.f *svm->predict(coeffs, true);
//
//				cout<<"Confidence for "<<person_names_[i]<<" appearance: "<<confidence<<endl;
//				cv::imshow("Foto", person_imgs_[i2][j]);
//
//				cv::waitKey(0);
//
//
//			}
//		}
	}
	*/
}


/*
 * Given the persons, load their images. Returns number of images loaded
 */
int FaceRecognizer::loadAllPersonsImages()
{

	int num_images = 0;


	//Loop to get the images for each person and store them in person_imgs
	for(unsigned int i = 0 ; i<persons_.size();i++)
	{
		int num_images_temp = loadOnePersonImages(persons_[i]);
		ROS_INFO("\tImages loaded from %s: %d", persons_[i].name_.c_str(),num_images_temp);
		num_images+=num_images_temp;
	}

	return num_images;
}


int FaceRecognizer::loadOnePersonImages(Person &person)
{

	int num_images = 0;

	if(person.images_.size()>0)
		return person.images_.size();


	//Access to each image of the directory of the person
	for (boost::filesystem::directory_iterator itr(person.images_dir_path_); itr!=boost::filesystem::directory_iterator(); ++itr)
	{
		//If the element is a regular file
		if (boost::filesystem::is_regular_file(itr->status()))
		{
			std::string extension=boost::filesystem::extension(itr->path());

			//If the files extension is an image
			if((extension==std::string(".jpg")) || (extension==std::string(".pgm")))
			{
				std::string imgFilename=person.images_dir_path_+"/"+itr->path().filename().string();
				cv::Mat img =cv::imread(imgFilename);
				person.images_.push_back(img);
				num_images++;
			}
		}
	}

	return num_images;
}

void FaceRecognizer::buildPCA()
{


	//Assign the default image_size as the first image of the first person
//	image_size_.width = persons_[0].images_[0].cols;
//	image_size_.height = persons_[0].images_[0].cols;

	cv::Mat pca_set(0, image_size_.width*image_size_.height, CV_8UC1);

	//If param for pca_dimension is set
	if (private_nh_.hasParam("/qbo_face_recognition/pca_dimension"))
		private_nh_.getParam("/qbo_face_recognition/pca_dimension", pca_dimension_);
	

	for(unsigned int i = 0 ; i<persons_.size();i++)
		for(unsigned int j = 0 ; j<persons_[i].images_.size();j++)
		{
			cv::Mat temp_mat = equalizeFace(persons_[i].images_[j]);
			temp_mat = temp_mat.reshape(1,1);
			//Push to PCA the vector image
			pca_set.push_back(temp_mat);

		}

	

	if(pca_dimension_<1)
		pca_dimension_ = pca_set.rows -1 ;
	else
		pca_dimension_ = min(pca_dimension_, pca_set.rows);

	pca_(pca_set, cv::Mat(), CV_PCA_DATA_AS_ROW, pca_dimension_);


	stabilizer_states_.clear();
	for(unsigned int i = 0 ; i<=persons_.size();i++)
	{
		stabilizer_states_.push_back(0);
	}
}



string FaceRecognizer::recognizePCA_SVM(cv::Mat img)
{

	string returned= "";

	if(persons_.size()==0)
		return returned;
	else if(persons_.size()==1)
	{
		return persons_[0].name_;
	}


	/*
	 * Get PCA projection for incoming image
	 */

	cv::Mat temp, temp2;
	cv::Mat grey = img.clone();
	equalizeFace(grey).copyTo(temp);
	cv::imshow("To recognized",temp);
	cv::waitKey(5);

	temp.reshape(1,1).convertTo(temp2, CV_32FC1);
	cv::Mat coeffs = pca_.project(temp2);

	/*
	 * Find the projection with the least Euclidian distance
	 */
	double leastDistSq = DBL_MAX;
	string recognized_person;

	double num_of_coeffs_ = 0;

	for(unsigned int i=0; i<persons_.size(); i++) //For each person
	{
		double distSq=0;

		for(int j=0; j<persons_[i].pca_proj_.rows; j++) //For each projection of a person
		{
			//Get projection
		 	cv::Mat temp = persons_[i].pca_proj_.row(j);

		 	cv::Mat dist = coeffs - temp;

			dist = dist.mul(dist);

		 	for(int l = 0; l<dist.cols;l++) //Calc sum of the vector
				distSq+=dist.at<float>(0,l);
		
			
			if(distSq < leastDistSq)
			{
				leastDistSq = distSq;
				returned = persons_[i].name_;
			}

			num_of_coeffs_+=1.0;
		}
	}
	
	printf("leastDistSq %lg\n", leastDistSq);

	// Return the confidence level based on the Euclidean distance,
	// so that similar images should give a confidence between 0.5 to 1.0,
	// and very different images should give a confidence between 0.0 to 0.5.
	double pConfidence = 1.0 - sqrt( leastDistSq / (double)(num_of_coeffs_ * pca_dimension_) ) / 255.0;

	printf("Recognized %s with confidence %lg, dim = %d\n", returned.c_str(), pConfidence, pca_dimension_);



	return returned;

	/*
	 * COMMENTED FOR NEW PCA


	cv::Mat temp, temp2;
	cv::Mat grey = img.clone();

	equalizeFace(grey).copyTo(temp);

	//cv::imshow("To recognized",temp);

	//cv::waitKey(5);

	temp.reshape(1,1).convertTo(temp2, CV_32FC1);

	cv::Mat coeffs = pca_.project(temp2);

	float confidences[persons_.size()];

	cv::Mat projected = pca_.backProject(coeffs);
	cv::Mat good_projected;
	projected.reshape(1, image_size_.height).convertTo(good_projected,CV_8SC1);
	//cv::imshow("Projected", good_projected);

	//Calculate confidences
	for(unsigned int i = 0; i<persons_.size();i++)
	{
		confidences[i] = -1.f *persons_[i].pca_svm_->predict(coeffs, true);
		//confidences[i] = svms_[i]->predict(coeffs, true);
	}


	float max_confidence = 0;
	unsigned int max_index = 0;

	//Print confidences
	for(unsigned int i = 0; i<persons_.size();i++)
	{
		if(confidences[i]>max_confidence)
		{
			max_confidence = confidences[i];
			max_index = i;
		}

		printf("%s: %2.5lg,  \t", persons_[i].name_.c_str(), confidences[i]);
	}

	cout<<endl;

	if(confidences[max_index] > 0.1)
	{
		return persons_[max_index].name_;
	}
	else
		return "";

	return "";


	*/
}



bool FaceRecognizer::recognizeService(qbo_face_msgs::RecognizeFace::Request  &req, qbo_face_msgs::RecognizeFace::Response &res )
{

	if(persons_.size() == 0) //If there are no persons in Data Base
	{
		res.name = string("");
		res.recognized = true;
		return true;
	}
	else if(persons_.size() == 1) //If there is only one person in Data Base
	{
		res.name = persons_[0].name_;
		res.recognized = true;
		return true;
	}


	//Get image from Message

	cv_bridge::CvImagePtr cv_ptr;

	try
	{
	  cv_ptr = cv_bridge::toCvCopy(req.face, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return false;
	}

	string name_recognized;

	switch(recog_type_)
	{
	case PCA_SVM:
		name_recognized = recognizePCA_SVM(cv_ptr->image);
		break;
	case BAG_OF_WORDS_SVM:
		name_recognized = recognizeBOW_SVM(cv_ptr->image);
		break;
	}



	res.name = name_recognized;

	return true;
}


bool FaceRecognizer::recognizeStabilizerService(qbo_face_msgs::RecognizeFace::Request  &req, qbo_face_msgs::RecognizeFace::Response &res )
{

	if(persons_.size() == 0) //If there are no persons in Data Base
	{
		res.name = string("");
		res.recognized = true;
		return true;
	}
	else if(persons_.size() == 1) //If there is only one person in Data Base
	{
		res.name = persons_[0].name_;
		res.recognized = true;
		return true;
	}



	//Get image from Message

	cv_bridge::CvImagePtr cv_ptr;



	try
	{
	  cv_ptr = cv_bridge::toCvCopy(req.face, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return false;
	}



	string name_recognized = "";



	switch(recog_type_)
	{
	case PCA_SVM:
		ROS_INFO("Recognizing using PCA and SVM");
		name_recognized = recognizePCA_SVM(cv_ptr->image);
		break;
	case BAG_OF_WORDS_SVM:
		ROS_INFO("Recognizing using Bags of Words and SVM");
		name_recognized = recognizeBOW_SVM(cv_ptr->image);
		break;
	}



	unsigned int index = persons_.size();

	for(unsigned int i = 0; i<persons_.size();i++)
	{
		if(persons_[i].name_.compare(name_recognized) == 0)
		{
			index = i;
			break;
		}
	}


//	if(index < persons_.size())
//	{

	/**Update stabilizer**/
	for(unsigned int i = 0; i<stabilizer_states_.size(); i++)
	{
		if(i == index)
		{
			if(stabilizer_states_[i] < stabilizer_max_)
			{
				if(i<stabilizer_states_.size()-1)
					stabilizer_states_[i]+=2; //TODO - Change to +=2
				else
					stabilizer_states_[i]+=1;
			}
		}

		else
		{	if(stabilizer_states_[i]>0)
			{
				if(i<stabilizer_states_.size()-1)
					stabilizer_states_[i]--;
				else
					stabilizer_states_[i]-= 2; //TODO - Change to -=1
			}
		}
	}
//	}


	//Print stabilizer
	for(unsigned int i = 0; i<stabilizer_states_.size(); i++)
	{
		if(i!= stabilizer_states_.size()-1)
			printf("%s: %d    |   ", persons_[i].name_.c_str(), stabilizer_states_[i]);
		else
			printf("Not Known: %d\n",stabilizer_states_[i]);
	}


	/**Calc max value of stabilizer**/
	int max_state = stabilizer_states_[0];
	unsigned int max_index = 0;

	for(unsigned int i = 1; i<stabilizer_states_.size(); i++)
	{
		if(stabilizer_states_[i]>max_state)
		{
			max_state = stabilizer_states_[i];
			max_index = i;
		}
	}

	bool recognized = false;

	if(max_state > stabilizer_threshold_)
		recognized = true;

	if(recognized)
	{
		if(max_index < persons_.size())
			res.name = persons_[max_index].name_;
		else
			res.name = "";

		res.recognized = true;
	}
	else
	{
		res.name = "";
		res.recognized = false;
	}



	return true;
}




bool FaceRecognizer::getNameService(qbo_face_msgs::GetName::Request  &req, qbo_face_msgs::GetName::Response &res )
{
	/**Calc max value of stabilizer**/

	if(persons_.size() == 0) //If there are no persons in Data Base
	{
		res.name = string("");
		res.recognized = true;
		return true;
	}
	else if(persons_.size() == 1) //If there is only one person in Data Base
	{
		res.name = persons_[0].name_;
		res.recognized = true;
		return true;
	}



	int max = stabilizer_states_[0];
	unsigned int max_index = 0;

	for(unsigned int i = 1; i<stabilizer_states_.size(); i++)
	{
		if(stabilizer_states_[i]>max)
		{
			max = stabilizer_states_[i];
			max_index = i;
		}
	}

	bool recognized = false;


	if(max > stabilizer_threshold_)
		recognized = true;


	if(recognized)
	{
		if(max_index == stabilizer_states_.size()-1)
			res.name = "";
		else
			res.name = persons_[max_index].name_;

		res.recognized = true;
	}

	else
	{	res.name = string("");
		res.recognized = false;
	}

	return true;
}

bool FaceRecognizer::trainService(qbo_face_msgs::Train::Request  &req, qbo_face_msgs::Train::Response &res )
{
	int returned = true;

	switch(recog_type_)
	{
	case PCA_SVM:
		returned = trainPCA_Recognizer(req.update_path);
		break;
	case BAG_OF_WORDS_SVM:
		returned = trainBOW_Recognizer(req.update_path);
		break;
	}


	if(returned == 0)
	{
		string path;
		if(req.update_path.compare("") == 0)
			path = update_path_;
		else
			path = req.update_path;
			
		//Delete all folders in new persons path, so as to not repeat these images in the next learning phase
		for (boost::filesystem::directory_iterator itr(path); itr!=boost::filesystem::directory_iterator(); ++itr)
		{
			if (boost::filesystem::is_directory(itr->status()))
			{
				std::string person_name=itr->path().filename().string();
				boost::filesystem::remove_all(path+"/"+person_name);
			}
		}
		res.taught = true;
	}
	else
		res.taught = false;








	return true;
}



bool FaceRecognizer::learnFaces(qbo_face_msgs::LearnFaces::Request  &req, qbo_face_msgs::LearnFaces::Response &res )
{	
	string person_name = req.person_name;

	if(person_name.compare("") == 0) //If object's name is empty, return an error
	{
		res.learned = false;
		ROS_ERROR("Invalid person name");
		return true;
	}

	ROS_INFO("Trying to learn faces of person %s", person_name.c_str());

	/*
	 * Clear vector of received faces
	 */
	received_faces_.clear();

	//Subscribe to face image topic
	ros::Subscriber image_sub=private_nh_.subscribe<sensor_msgs::Image>(face_topic_,1,&FaceRecognizer::faceImageCallback, this);


	//Defining time variables
	ros::Time t1 = ros::Time::now();
	ros::Time t2;
	ros::Duration diff_time;
	
	//Last image index stored
	unsigned int last_num = 0;
	
	//Waiting for a minimum number of face images to arrive
	while((int)received_faces_.size()<num_images_to_hold_)
	{	
		usleep(200000);
		ros::spinOnce();

		if(last_num != received_faces_.size())
		{
			last_num = received_faces_.size();
			ROS_INFO("%u faces received. %u more to go", last_num, num_images_to_hold_-last_num);
		}

		t2 = ros::Time::now();
		diff_time = t2-t1;
		
		//Check if max time to store images has been reached
		if(diff_time.toSec()>max_time_to_learn)
			break;
	}
		
	//Unsubscribe to topic of faces
	image_sub.shutdown();
	
	
	//Check if at least one face has been captured. If not, abort the service.
	if(received_faces_.size() == 0)
	{
		ROS_WARN("TIMEOUT: No images were captured");
		ROS_INFO("Aborting object learn...\n");
		ROS_INFO("Ready to recognize or learn new objects");
		res.learned = false;
		return true;
	}


	/*
		Storing captured images into the filesystem
	*/

	
	if(!boost::filesystem::is_directory(update_path_))
	{
		//Create the main folder in the save path
		boost::filesystem::create_directory(update_path_);
	}

	if(!boost::filesystem::is_directory(update_path_+"/"+person_name))
		//Create the person's folder
		boost::filesystem::create_directory(update_path_+"/"+person_name);

	string time_now;
	stringstream out;
	out << time(NULL);
	time_now = out.str();


	ROS_INFO("Saving face images in a image files...");
	//Save images
	int image_index = 0;
	for(unsigned int i = 0; i<received_faces_.size();i++)
	{
		string filename;
		stringstream out_2;

		out_2 <<update_path_+"/"+person_name<<"/"<<time_now<<"_"<<image_index<<".jpg";
		filename = out_2.str();

		vector<int> params;

		params.push_back(CV_IMWRITE_JPEG_QUALITY);
		params.push_back(100);
		cv::imwrite(filename.c_str(), received_faces_[i], params);
		image_index++;
	}
	
	

	ROS_INFO("Learn service successfully completed: %u faces has been stored for person %s", (unsigned int)received_faces_.size(), person_name.c_str());
	
	res.learned = true;

	return true;
}



/*
* Faces callback used to store the face's images in the learn_faces service
*/
void FaceRecognizer::faceImageCallback(const sensor_msgs::Image::ConstPtr& image_ptr)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
		ROS_ERROR("Cv_bridge exception by receiving image of faces: %s", e.what());
        return;
    }

    cv::Mat image_received;

    //Erase vector if it is full
        while((int)received_faces_.size()>=num_images_to_hold_)
                received_faces_.erase(received_faces_.begin());

        //Append it to vector of received messages
        received_faces_.push_back(cv_ptr->image);

        ROS_INFO("Face's images received");
}




int FaceRecognizer::trainPCA_Recognizer(string update_path)
{

	string path;

	if(update_path.compare("") == 0)
		path = update_path_;
	else
		path = update_path;

	vector<Person> temp_persons;

	ROS_INFO("\nLoading names from update path: %s...", path.c_str());

	//Get folders' names
	for (boost::filesystem::directory_iterator itr(path); itr!=boost::filesystem::directory_iterator(); ++itr)
	{
		if (boost::filesystem::is_directory(itr->status()))
		{
			std::string person_name=itr->path().filename().string();

			if(person_name.at(0)=='.')
				continue;

			Person person(person_name);

			person.images_dir_path_ = path+"/"+person_name;
			temp_persons.push_back(person);
		}
	}

	ROS_INFO("%u names loaded!\n", (unsigned int)temp_persons.size());


	ROS_INFO("Loading images from update path...\n");
	unsigned int num_imgs = 0;
	//Load images for each person
	for(unsigned int i = 0 ; i<temp_persons.size();i++)
		num_imgs+=loadOnePersonImages(temp_persons[i]);
	printf("%u images loaded from update path!\n", num_imgs);


	ROS_INFO("Loading images from data_base\n");
	int num_img = 0;
	//Load all database persons descriptors
	num_img = loadAllPersonsImages();
	ROS_INFO("%d images loaded from data base!\n", num_img);


	for(unsigned int i = 0; i<temp_persons.size();i++) //For each new person
	{
		bool person_exists = false;

		for(unsigned int j = 0; j<persons_.size();j++) //For each database person
		{

			if( temp_persons[i].name_.compare(persons_[j].name_) == 0) //If person exists
			{
				person_exists = true;

				if(temp_persons[i].images_.size()>0)
				{
					printf("\nUpdating person %s with %u new images...\n", temp_persons[i].name_.c_str(), (unsigned int)temp_persons[i].images_.size());

					unsigned int image_index = persons_[j].images_.size();

					for(unsigned int im = 0; im<temp_persons[i].images_.size();im++) //For each new image
					{
						//Append image to the person in data base
						persons_[j].images_.push_back(temp_persons[i].images_[im]);
					}

					//Save new images to the data base folder of the person
					for(unsigned int im = 0; im<temp_persons[i].images_.size();im++)
					{
						string filename;
						stringstream out;

						out <<persons_[j].images_dir_path_<<"/image_"<<image_index<<".jpg";
						filename = out.str();

						vector<int> params;

						params.push_back(CV_IMWRITE_JPEG_QUALITY);
						params.push_back(100);
						cv::imwrite(filename.c_str(), temp_persons[i].images_[im], params);
						image_index++;
					}

					printf("Person %s updated!\n", temp_persons[i].name_.c_str());
				}
			}
		}

		//If person doesn't exist, we need to append it to the list of persons
		if(!person_exists)
		{
			printf("\nAdding new person to data base: %s\n", temp_persons[i].name_.c_str());

			//Create folder of the person in data base path
			boost::filesystem::create_directory(faces_db_path_+"/"+temp_persons[i].name_);

			unsigned int image_index = 0;

			//Save all images in that path
			for(unsigned int im = 0; im<temp_persons[i].images_.size();im++)
			{
				string filename;
				stringstream out;

				out<<faces_db_path_+"/"+temp_persons[i].name_<<"/image_"<<image_index<<".jpg";
				filename = out.str();

				vector<int> params;

				params.push_back(CV_IMWRITE_JPEG_QUALITY);
				params.push_back(100);

				cv::imwrite(filename.c_str(), temp_persons[i].images_[im], params);

				image_index++;
			}

			Person person_to_append(temp_persons[i].name_);
			person_to_append.images_dir_path_ = faces_db_path_+"/"+temp_persons[i].name_;

			//Load new person images
			loadOnePersonImages(person_to_append);


			//Add new person to the person list
			persons_.push_back(person_to_append);

			printf("Number of images of new person: %u\n", (unsigned int)persons_[persons_.size()-1].images_.size());

			printf("Person %s added\n", person_to_append.name_.c_str());
		}
	}


	if(persons_.size()<2)
		return 0;


	printf("Rebuilding PCA\n");
	//Retrain vocabulary
	buildPCA();
	printf("PCA rebuilt!\n");

	printf("Storing new PCA\n");
	//Retrain vocabulary
	storePCA();
	printf("New PCA built!\n");


	printf("Retraining SVM Classifiers\n");
	//Retrain all persons SVM classifiers
	trainPCA_SVMClassifiers();
	printf("SVM Classifiers trained\n");

	return 0;
}


// FOR BOW Approach
/*
 * Given the persons, load their descriptors file from the images folder
 * If a descriptors file isn't found for an person, then it extracts descriptors from
 * all images of that person
 */
int FaceRecognizer::loadAllPersonsDescriptors()
{
	int num_desc = 0;
	//Loop to get the images for each person and store them in person_imgs
	for(unsigned int i = 0 ; i<persons_.size();i++)
	{

		cout<<" - For person "<<persons_[i].name_.c_str()<<":"<<endl;

		//Load descriptors
		num_desc+=loadOnePersonDescriptors(persons_[i]);
	}
	return num_desc;
}
/*
 * Given an person, load its descriptors from the images folder.
 * Return true if succeeded
 */
int FaceRecognizer::loadOnePersonDescriptors(Person &person)
{
	string file_name = person.images_dir_path_+"/"+"descriptors.xml.gz";

	cv::FileStorage fs(file_name, cv::FileStorage::READ);

	bool loaded_from_file = false;;

	if( fs.isOpened() )
	{
		fs["Descriptors"] >> person.descriptors_;
		loaded_from_file = true;
	}
	else
		loaded_from_file = false;

	if(loaded_from_file)
	{
		printf("\t %d descriptors successfully loaded from descriptors.xml.gz in %s\n", person.descriptors_.rows, person.images_dir_path_.c_str());
		return person.descriptors_.rows;
	}

	printf("\t descritors.xml.gz not found in %s\n", person.images_dir_path_.c_str());

	printf("\t Loading images in %s...\n", person.images_dir_path_.c_str());
	int num_images = loadOnePersonImages(person);
	printf("\t Loaded %d images \n", num_images);

	printf("\t Extracting descriptors ...\n");
	int num_desc = extractOnePersonDescriptors(person);
	printf("\t Extracted %d descriptors \n", num_desc);

	return num_desc;
}

int FaceRecognizer::extractOnePersonDescriptors(Person &person)
{
	int num_descriptors = 0;

	for(unsigned int i = 0; i<person.images_.size();i++)
	{
		num_descriptors+=addDescriptorsToPerson(person.images_[i], person);
	}


	//Store descriptors file

	string file_name = person.images_dir_path_+"/"+"descriptors.xml.gz";

    cv::FileStorage fs( file_name, cv::FileStorage::WRITE );
    if( !fs.isOpened() )
    {
    	//If I cannot open the file for writing, send an error
    	return -1;
    }

    fs << "Descriptors" << person.descriptors_;

	return num_descriptors;
}




/*
 * Load bow_svm.xml classifier files. Returns true if succesfully loaded all svm files.
 * Return false if for at least one person, an svm file has not been found
 */
bool FaceRecognizer::loadBOW_SVMClassifiers()
{
	for(unsigned int i = 0; i < persons_.size();i++)
	{
	    /* first check if a previously trained svm for the current class has been saved to file */
	    string svmFilename = persons_[i].images_dir_path_+"/bow_svm.xml.gz";

	    cv::FileStorage fs(svmFilename, cv::FileStorage::READ);
	    if( fs.isOpened() )
	    {
	    	persons_[i].bow_svm_ = new CvSVM;
	    	persons_[i].bow_svm_->load(svmFilename.c_str() );
	    }
	    else
	    	return false;
	}

	return true;
}

/*
 * Trains the SVM Classifiers used for the Bags of Words Approach
 */
int FaceRecognizer::trainBOW_SVMClassifiers()
{

	if(persons_.size()==0)
		return 0;

	//Data structures for descriptors extraction
	vector<cv::KeyPoint> extracted_keypoints;
	//cv::GridAdaptedFeatureDetector descriptor_detector_2(new cv::SurfFeatureDetector, 50, 1, 1);


	//Load images for training
	loadAllPersonsImages();

	for(unsigned int i = 0; i<persons_.size(); i++) //For each person
	{
		//Build SVM classifier for person i
		persons_[i].bow_svm_ = new CvSVM;

		//Preparing data to train its SVM
		cv::Mat train_data(0, persons_vocabulary_.rows, CV_32FC1);
		cv::Mat responses(0, 1,  CV_32SC1);

		for(unsigned int i2 = 0; i2<persons_.size(); i2++) //For each person in which we will get image
		{
			for(unsigned int j = 0; j<persons_[i2].images_.size(); j++) //For eachs person's image
			{
				extracted_keypoints.clear();

				cv::Mat temp_img;
				cv::cvtColor(persons_[i2].images_[j], temp_img, CV_RGB2GRAY);

				cv::equalizeHist(temp_img, temp_img);
				//Detect keypoints from grayscale image
				cv::Mat mask = cv::Mat::zeros(persons_[i2].images_[j].size(), CV_8UC1);
				cv::RotatedRect rot_rect;
				rot_rect.angle = 0;
				rot_rect.size = cv::Size(persons_[i2].images_[j].cols*0.85,persons_[i2].images_[j].rows*0.95);
				rot_rect.center = cv::Point(persons_[i2].images_[j].cols/2, persons_[i2].images_[j].rows/2 );

				cv::ellipse(mask, rot_rect, cv::Scalar(255,255,255), -1);


				descriptor_detector_->detect(temp_img, extracted_keypoints, mask);


				if(extracted_keypoints.size()==0)
					continue;

				cv::Mat words;
				bowExtractor_->compute(temp_img, extracted_keypoints, words);

				train_data.push_back(words);

				cv::Mat response_mat= cv::Mat::ones(1,1,  CV_32SC1);

				if(i!=i2) //Person present at this image?
					response_mat*=-1.; //If not

				responses.push_back(response_mat);
			}
		}


		//If training data or responses are empty
		if(train_data.rows == 0 || responses.rows == 0)
			continue;

		//SVM Training
        CvSVMParams svmParams;


        setSVMParams( svmParams, responses);

        CvParamGrid c_grid, gamma_grid, p_grid, nu_grid, coef_grid, degree_grid;
        setSVMTrainAutoParams( c_grid, gamma_grid,  p_grid, nu_grid, coef_grid, degree_grid );


        ROS_INFO("\t Training BOW classifier for %s...",persons_[i].name_.c_str());

        persons_[i].bow_svm_->train_auto(train_data, responses, cv::Mat(), cv::Mat(), svmParams, 10,
        		c_grid, gamma_grid, p_grid, nu_grid, coef_grid, degree_grid );
		//persons_[i].svm_->train(train_data, responses);

        string svm_file = persons_[i].images_dir_path_+"/bow_svm.xml.gz";
        //Saving svm to a file
        persons_[i].bow_svm_->save(svm_file.c_str());

        ROS_INFO("\t BOW classifier for %s trained and stored",persons_[i].name_.c_str());

	}
	return 0;
}

/*
 *	Given a path, it extracts the images of each's person's folder,  copy the files to the respective folders in
 *	persons's main path, add's it to the model, generate the vocabulary, retrain the classifiers.
 *	Return 0 if succeeded, and != 0 if not succeeded.
 */
int FaceRecognizer::trainBOW_Recognizer(string update_path)
{

	string path = update_path_;



	if(path.compare("") == 0) //Check if faces_db_path_ contains a path
	{
		ROS_ERROR("Error: Update folder folder was not specified");
		exit(-1); //Return error
	}

	if(!boost::filesystem::is_directory(path))
	{
		ROS_ERROR("Error: Update folder folder is not valid");
		exit(-1);
	}

	vector<Person> temp_persons;

	ROS_INFO("Loading names from update path: %s...", path.c_str());

	//Get folders' names
	for (boost::filesystem::directory_iterator itr(path); itr!=boost::filesystem::directory_iterator(); ++itr)
	{
		if (boost::filesystem::is_directory(itr->status()))
		{
			std::string person_name=itr->path().filename().string();

			if(person_name.at(0)=='.')
				continue;

			Person person(person_name);


			person.images_dir_path_ = path+"/"+person_name;
			temp_persons.push_back(person);
		}
	}

	ROS_INFO("%u names loaded!\n", (unsigned int)temp_persons.size());


	printf("\nLoading images from update path...\n");
	unsigned int num_imgs = 0;
	//Load images for each person
	for(unsigned int i = 0 ; i<temp_persons.size();i++)
		num_imgs+=loadOnePersonImages(temp_persons[i]);
	printf("%u images loaded from update path!\n", num_imgs);



	printf("\nLoading descriptors from data_base\n");
	int num_desc = 0;
	//Load all database persons descriptors

	num_desc = loadAllPersonsDescriptors();
	printf("%d descriptors loaded from data base!\n", num_desc);


	printf("\nLoading images from data_base\n");
	int num_img = 0;
	//Load all database persons descriptors
	num_img = loadAllPersonsImages();
	printf("%d images loaded from data base!\n", num_img);


	for(unsigned int i = 0; i<temp_persons.size();i++) //For each new person
	{
		bool person_exists = false;

		for(unsigned int j = 0; j<persons_.size();j++) //For each database person
		{

			if( temp_persons[i].name_.compare(persons_[j].name_) == 0) //If person exists
			{
				person_exists = true;

				if(temp_persons[i].images_.size()>0)
				{
					printf("\nUpdating person %s with %u new images...\n", temp_persons[i].name_.c_str(), (unsigned int)temp_persons[i].images_.size());

					unsigned int image_index = persons_[j].images_.size();

					for(unsigned int im = 0; im<temp_persons[i].images_.size();im++) //For each new image
					{
						//Extract descriptors of new image and append them to the list of descriptors
						addDescriptorsToPerson(temp_persons[i].images_[im], persons_[j]);

						//Append image to the person in data base
						persons_[j].images_.push_back(temp_persons[i].images_[im]);
					}

					//Save new descriptors in descriptors.xml.gz
					string file_name = persons_[j].images_dir_path_+"/"+"descriptors.xml.gz";

					cv::FileStorage fs( file_name, cv::FileStorage::WRITE );
					if( !fs.isOpened() )
					{
						//If I cannot open the file for writing, send an error
						return -1;
					}

					//Saving descriptors
					fs << "Descriptors" << persons_[j].descriptors_;

					//Save new images to the data base folder of the person
					for(unsigned int im = 0; im<temp_persons[i].images_.size();im++)
					{
						string filename;
						stringstream out;

						out <<persons_[j].images_dir_path_<<"/image_"<<image_index<<".jpg";
						filename = out.str();

						vector<int> params;

						params.push_back(CV_IMWRITE_JPEG_QUALITY);
						params.push_back(100);
						cv::imwrite(filename.c_str(), temp_persons[i].images_[im], params);
						image_index++;
					}

					printf("Person %s updated!\n", temp_persons[i].name_.c_str());
				}
			}
		}

		//If person doesn't exist, we need to append it to the list of persons
		if(!person_exists )
		{
			ROS_INFO("\nAdding new person to data base: %s\n", temp_persons[i].name_.c_str());

			//Create folder of the person in data base path
			boost::filesystem::create_directory(faces_db_path_+"/"+temp_persons[i].name_);

			unsigned int image_index = 0;

			//Save all images in that path
			for(unsigned int im = 0; im<temp_persons[i].images_.size();im++)
			{
				string filename;
				stringstream out;

				out<<faces_db_path_+"/"+temp_persons[i].name_<<"/image_"<<image_index<<".jpg";
				filename = out.str();

				vector<int> params;

				params.push_back(CV_IMWRITE_JPEG_QUALITY);
				params.push_back(100);

				cv::imwrite(filename.c_str(), temp_persons[i].images_[im], params);

				image_index++;
			}

			Person person_to_append(temp_persons[i].name_);
			person_to_append.images_dir_path_ = faces_db_path_+"/"+temp_persons[i].name_;


			//Load new person images
			loadOnePersonImages(person_to_append);

			//Extract new person descriptors
			extractOnePersonDescriptors(person_to_append);

			//Add new person to the person list
			persons_.push_back(person_to_append);

			ROS_INFO("Number of images of new person %s: %u", persons_[persons_.size()-1].name_.c_str(), (unsigned int)persons_[persons_.size()-1].images_.size());
			ROS_INFO("Number of descriptors extracted for new person %s: %d", persons_[persons_.size()-1].name_.c_str(), persons_[persons_.size()-1].descriptors_.rows);
			ROS_INFO("Person %s added\n", person_to_append.name_.c_str());
		}
	}


	if(persons_.size()<2)
		return 0;


	ROS_INFO("Retraining vocabulary for BOW");
	//Retrain vocabulary
	trainVocabulary();
	ROS_INFO("Vocabulary trained!\n");


	ROS_INFO("Retraining BOW SVM Classifiers");
	//Retrain all persons SVM classifiers
	trainBOW_SVMClassifiers();
	ROS_INFO("BOW SVM Classifiers trained\n");

	return 0;
}


/*
 * Loads the vocabulary.xml from persons_path and returns true. If file is not found, then
 * it will return false;
 */
bool FaceRecognizer::loadVocabulary(string input_path)
{

	if(input_path.compare("") == 0)
		input_path = faces_db_path_;

	string file_name = input_path+"/"+"vocabulary.xml.gz";

    cv::FileStorage fs(file_name, cv::FileStorage::READ);

    if( fs.isOpened() )
    {
        fs["Vocabulary"] >> persons_vocabulary_;

    	//Assign vocabulary to Bow Image Descriptor Extractor
    	bowExtractor_->setVocabulary(persons_vocabulary_);

    	stabilizer_states_.clear();
    	for(unsigned int i = 0 ; i<=persons_.size();i++)
    	{
    		stabilizer_states_.push_back(0);
    	}

        return true;
    }


    return false;
}

/*
 * Used to train the vocabulary for BOW. Returns the number of descriptors used to train the vocabulary
 */
int FaceRecognizer::trainVocabulary()
{
	if(persons_.size()==0)
			return 0;

	int num_descriptors = 0;

	for(unsigned int ob = 0; ob < persons_.size();ob++)
	{
		//Add extracted descriptors to the BOW Trainer
		num_descriptors+=persons_[ob].descriptors_.rows;
	}


	//BoW KMeans Trainer Parameters
	cv::TermCriteria terminate_criterion;
	terminate_criterion.epsilon = FLT_EPSILON;
	int number_of_clusters = min(1500, num_descriptors/2);
	int k_means_attempts = 3;
	int k_means_flags=cv::KMEANS_PP_CENTERS;

	ROS_INFO("\t Generating vocabulary of %d clusters from %d descriptors...", number_of_clusters, num_descriptors);

	//initialize BoW trainer
	bowTrainer_ = new cv::BOWKMeansTrainer( number_of_clusters, terminate_criterion, k_means_attempts, k_means_flags);


	for(unsigned int ob = 0; ob < persons_.size();ob++)
	{
		//Add extracted descriptors to the BOW Trainer
		bowTrainer_->add(persons_[ob].descriptors_);
	}

	//Store vocabulary
	persons_vocabulary_ = bowTrainer_->cluster();

	//Assign vocabulary to Bow Image Descriptor Extractor
	bowExtractor_->setVocabulary(persons_vocabulary_);


	//Store vocabulary file
	string file_name = faces_db_path_+"/"+"vocabulary.xml.gz";

	cv::FileStorage fs( file_name, cv::FileStorage::WRITE );
	if( !fs.isOpened() )
	{
		//If I cannot open the file for writing, send an error
		return -1;
	}

	fs << "Vocabulary" << persons_vocabulary_;

	stabilizer_states_.clear();
	for(unsigned int i = 0 ; i<=persons_.size();i++)
	{
		stabilizer_states_.push_back(0);
	}


	return num_descriptors;

}


/*
 * Recognize using the BOW SVM classifiers
 */
string FaceRecognizer::recognizeBOW_SVM(cv::Mat img)
{
	//output_person = persons_[0];

	string returned= "";

	if(persons_.size()==0)
		return returned;
	else if(persons_.size()==1)
	{
		return persons_[0].name_;
	}

	//Data structures for descriptors extraction
	vector<cv::KeyPoint> extracted_keypoints;
	//cv::GridAdaptedFeatureDetector descriptor_detector_2(new cv::SurfFeatureDetector, 50, 1, 1);


	//Detect keypoints from grayscale image
	cv::Mat temp_img = img.clone();
	cv::equalizeHist(img, temp_img);

	cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
	cv::RotatedRect rot_rect;
	rot_rect.angle = 0;
	rot_rect.size = cv::Size(img.cols*0.85,img.rows*0.95);
	rot_rect.center = cv::Point(img.cols/2, img.rows/2 );
	cv::ellipse(mask, rot_rect, cv::Scalar(255,255,255), -1);

	descriptor_detector_->detect(temp_img, extracted_keypoints, mask);

	//TODO - To remove later
/*	cv::Mat to_show;
	cv::bitwise_and(temp_img, mask, to_show);
	cv::imshow("Show", to_show);
	cv::waitKey(10);
*/

	if(extracted_keypoints.size()==0)
		return returned;

	cv::Mat words;
	bowExtractor_->compute(temp_img, extracted_keypoints, words);

	float confidences[persons_.size()];

	//Calculate confidences
	for(unsigned int i = 0; i<persons_.size();i++)
		confidences[i] = -1.f *persons_[i].bow_svm_->predict(words, true);

	float max_confidence = 0;
	unsigned int max_index = 0;

	//Print confidences
	for(unsigned int i = 0; i<persons_.size();i++)
	{
		if(confidences[i]>max_confidence)
		{
			max_confidence = confidences[i];
			max_index = i;
		}

		printf("%s: %2.5f,  \t", persons_[i].name_.c_str(), confidences[i]);
	}

	cout<<endl;

	if(confidences[max_index]>bow_certainty_threshold_)
		returned = persons_[max_index].name_;

	return returned;
}


void FaceRecognizer::setSVMParams( CvSVMParams& svmParams, cv::Mat responses)
{
    int pos_ex = cv::countNonZero(responses == 1);
    int neg_ex = cv::countNonZero(responses == -1);

	svmParams.svm_type = CvSVM::C_SVC;

    if(linear_kernel_)
    	svmParams.kernel_type = CvSVM::LINEAR;

    else
    	svmParams.kernel_type = CvSVM::RBF;

	CvMat *class_wts = cvCreateMat(2, 1, CV_32FC1);

	cvmSet(class_wts, 0, 0, static_cast<float>(neg_ex)/static_cast<float>(pos_ex+neg_ex));
	cvmSet(class_wts, 1, 0, static_cast<float>(pos_ex)/static_cast<float>(pos_ex+neg_ex));

	svmParams.class_weights = class_wts;
}

void FaceRecognizer::setSVMTrainAutoParams( CvParamGrid& c_grid, CvParamGrid& gamma_grid,
                            CvParamGrid& p_grid, CvParamGrid& nu_grid,
                            CvParamGrid& coef_grid, CvParamGrid& degree_grid )
{
    c_grid = CvSVM::get_default_grid(CvSVM::C);

    gamma_grid = CvSVM::get_default_grid(CvSVM::GAMMA);

    p_grid = CvSVM::get_default_grid(CvSVM::P);
    p_grid.step = 0;

    nu_grid = CvSVM::get_default_grid(CvSVM::NU);
    nu_grid.step = 0;

    coef_grid = CvSVM::get_default_grid(CvSVM::COEF);
    coef_grid.step = 0;

    degree_grid = CvSVM::get_default_grid(CvSVM::DEGREE);
    degree_grid.step = 0;
}


/*Equalizes image of the face in size (provided by width and height) and in brightness
 * Returns the equalized image**/

cv::Mat FaceRecognizer::equalizeFace(cv::Mat face)
{
	if(face.channels()!=1)
	{
		cv::Mat old_face = face.clone();
		cv::cvtColor(old_face, face, CV_RGB2GRAY);
	}


	float hw_ratio = float(image_size_.height)/float(image_size_.width);

	cv::Rect new_dim_rect;

	new_dim_rect.height= min(float(face.rows), face.cols*hw_ratio);
	new_dim_rect.width= min(float(face.cols), face.rows/hw_ratio);


	if(new_dim_rect.height<face.rows)
		new_dim_rect.y += (face.rows-new_dim_rect.height)/2.;

	if(new_dim_rect.width<face.cols)
		new_dim_rect.x += (face.cols-new_dim_rect.width)/2.;


	cv::Mat equalized_face = cv::Mat(image_size_, CV_8UC1);

	cv::resize(face(new_dim_rect),equalized_face, equalized_face.size(), 0, 0);

	cv::equalizeHist(equalized_face, equalized_face);

	//Apply ellipse mask
        /*	
	cv::Mat mask = cv::Mat::zeros(equalized_face.size(), equalized_face.type());
	cv::Mat mask2 = cv::Mat::ones(equalized_face.size(), equalized_face.type())*255/2.;

	cv::RotatedRect rot_rect;
	rot_rect.angle = 0;
	rot_rect.size = cv::Size(equalized_face.cols*0.85,equalized_face.rows*0.95);
	rot_rect.center = cv::Point(equalized_face.cols/2, equalized_face.rows/2 );

	cv::ellipse(mask, rot_rect, cv::Scalar(255,255,255), -1);
	cv::ellipse(mask2, rot_rect, cv::Scalar(0,0,0), -1);

	cv::bitwise_and(equalized_face, mask, equalized_face);
	cv::bitwise_or(equalized_face, mask2, equalized_face);
	*/

	return equalized_face;
}

/*
 * Given an image, it extracts the good SURF descriptors from it and add it to the person's model
 * Returns the number of descriptors added to the model. If <0, then an error has occurred
 */
int FaceRecognizer::addDescriptorsToPerson(cv::Mat image, Person &person)
{
	int num_descriptors = 0;

	//Convert image to grayscale
	cv::Mat grayscale;
	cv::cvtColor(image, grayscale, CV_RGB2GRAY);


	//Data structures for descriptors extraction
	vector<cv::KeyPoint> extracted_keypoints;
	cv::Mat extracted_descriptors;

	//Detect keypoints from grayscale image
	cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
	cv::RotatedRect rot_rect;
	rot_rect.angle = 0;
	rot_rect.size = cv::Size(image.cols*0.85,image.rows*0.95);
	rot_rect.center = cv::Point(image.cols/2, image.rows/2 );
	cv::ellipse(mask, rot_rect, cv::Scalar(255,255,255), -1);

	cv::Mat temp_img;
	cv::cvtColor(image, temp_img, CV_RGB2GRAY);
	cv::equalizeHist(temp_img, temp_img);


	descriptor_detector_->detect(temp_img, extracted_keypoints, mask);
	//Extract descriptors from grayscale image
	descriptor_extractor_->compute(temp_img, extracted_keypoints, extracted_descriptors);

	if(person.descriptors_.rows == 0) //If person doesn't have any descriptor in model
	{
		person.descriptors_ = extracted_descriptors;
		return extracted_descriptors.rows;
	}
	else //If person has at leat onde descriptor in model
	{
		vector<vector<cv::DMatch> > matches;
		//Get matches of new descriptors with stored descriptors so as to don't add similar descriptors to person model
		desc_matcher_.radiusMatch(extracted_descriptors, person.descriptors_, matches, 1.5*descriptors_match_threshold_);

		for(unsigned int i = 0; i<extracted_keypoints.size();i++)
		{
			if(matches[i].size()==0) //Add different to the rest descriptors
			{
				person.descriptors_.push_back(extracted_descriptors.row(i));
				//cv::circle(img_BGR,extracted_keypoints[i].pt,2,cv::Scalar(0,0,255),2);
				num_descriptors++;
			}
		}
	}

	return num_descriptors;
}


int main(int argc, char **argv)
{
	
	
	ros::init(argc, argv, "qbo_face_recognition");

	FaceRecognizer face_recognizer;

	ros::spin();

  return 0;
}
