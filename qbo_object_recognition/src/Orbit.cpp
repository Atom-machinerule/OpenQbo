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

#include "Orbit.h"


Orbit::Orbit(string images_path, string update_path)
//descriptor_detector_(new cv::SurfAdjuster(),5, 3, 1000)
//descriptor_detector_(new cv::SurfFeatureDetector, 10, 1, 1)
{
	objects_main_path_ = images_path;
	update_path_ = update_path;

	//For equalize image
	patch_size_ = cv::Size(40,40);

	/*
	 * Basic SURF Count approach
	 */
	descriptor_detector_ = new cv::GridAdaptedFeatureDetector(new cv::SurfFeatureDetector, 50, 1, 1);
	descriptor_extractor_ = new cv::SurfDescriptorExtractor;
	descriptors_match_threshold_ = 0.23; //SURF Descriptors match threshold


	/*
	 * For Bags of Words Approach
	 */
    //Initialize Bow Img Descriptor Extractor
    bowExtractor_ = new cv::BOWImgDescriptorExtractor(descriptor_extractor_,
    						new cv::BruteForceMatcher<cv::L2<float> >);


    linear_kernel = false;
}



Orbit::~Orbit() {
	// TODO Auto-generated destructor stub
}

/*
 * Load the objects, given the object image path. Fills the vector of OrbitObjects
 *
 * Return number of images collected. If < 0, then an error ocurred
 */
int Orbit::loadObjectsNames(string input_path)
{

	if(input_path.compare("") == 0)
		input_path = objects_main_path_;

	if(objects_main_path_.compare("") == 0) //Check if object_images_path_ contains a path
		return -1; //Return error



	//Get the folders names that represent the different object in our database
	for (boost::filesystem::directory_iterator itr(input_path); itr!=boost::filesystem::directory_iterator(); ++itr)
	{
		if (boost::filesystem::is_directory(itr->status()))
		{
			std::string object_name=itr->path().filename().string();
			OrbitObject object(object_name);

			object.images_dir_path_ = input_path+"/"+object_name;
			objects_.push_back(object);
		}
	}

	return 0;
}



/*
 * Given the objects, load their images
 */
int Orbit::loadAllObjectsImages()
{

	int num_images = 0;


	//Loop to get the images for each person and store them in person_imgs
	for(unsigned int i = 0 ; i<objects_.size();i++)
	{

		num_images+=loadOneObjectImages(objects_[i]);
	}

	return num_images;
}

/*
 * Given one objects, load their images
 * Returns the number of loaded images
 */
int Orbit::loadOneObjectImages(OrbitObject &object)
{

	int num_images = 0;

	if(object.images_.size()>0)
		return object.images_.size();


	//Access to each image of the directory of the person
	for (boost::filesystem::directory_iterator itr(object.images_dir_path_); itr!=boost::filesystem::directory_iterator(); ++itr)
	{
		//If the element is a regular file
		if (boost::filesystem::is_regular_file(itr->status()))
		{
			std::string extension=boost::filesystem::extension(itr->path());

			//If the files extension is an image
			if((extension==std::string(".jpg")) || (extension==std::string(".pgm")))
			{
				std::string imgFilename=object.images_dir_path_+"/"+itr->path().filename().string();
				cv::Mat img =cv::imread(imgFilename);
				object.images_.push_back(img);
				num_images++;
			}
		}
	}

	return num_images;
}



/*
 * Given the objects, load their descriptors file from the images folder
 * Returns the number of descriptors loaded. If <0, then an error ocurred
 */
int Orbit::loadAllObjectsDescriptors()
{
	int num_desc = 0;
	//Loop to get the images for each person and store them in person_imgs
	for(unsigned int i = 0 ; i<objects_.size();i++)
	{

		cout<<" - For Object "<<objects_[i].name_.c_str()<<":"<<endl;

		//Load descriptors
		num_desc+=loadOneObjectsDescriptors(objects_[i]);
	}


	return num_desc;
}

/*
 * Given an object, load its descriptors file from the images folder.
 * Return true if succeeded
 */
int Orbit::loadOneObjectsDescriptors(OrbitObject &object)
{
	string file_name = object.images_dir_path_+"/"+"descriptors.xml.gz";

    cv::FileStorage fs(file_name, cv::FileStorage::READ);

    bool loaded_from_file = false;;

    if( fs.isOpened() )
    {
        fs["Descriptors"] >> object.descriptors_;
        loaded_from_file = true;
    }
    else
    	loaded_from_file = false;

    if(loaded_from_file)
    {
    	printf("\t %d descriptors successfully loaded from descriptors.xml.gz in %s\n", object.descriptors_.rows, object.images_dir_path_.c_str());
    	return object.descriptors_.rows;
    }

	printf("\t descritors.xml.gz not found in %s\n", object.images_dir_path_.c_str());

	printf("\t Loading images in %s...\n", object.images_dir_path_.c_str());
	int num_images = loadOneObjectImages(object);
	printf("\t Loaded %d images \n", num_images);

	printf("\t Extracting descriptors ...\n");
	int num_desc = extractOneObjectDescriptors(object);
	printf("\t Extracted %d descriptors \n", num_desc);

	return num_desc;
}


/*
 * Prepare orbit to use the input type method. For eg., for BASIC_SURF_COUNT,
 * it loads images and extract descriptors of the objects loaded
 * If < 0, then an error ocurre
 */
int Orbit::prepareOrbit(RECOGNITION_TYPE type)
{
	int returned = 0;

	recog_type_ = type;

	switch(type)
	{
	case BASIC_SURF_COUNT:
		loadAllObjectsImages();
		extractAllObjectDescriptors();
		break;
	case BASIC_NCC:
		loadAllObjectsImages();
		break;
	case BAG_OF_WORDS_NAIVE_BAYES:
		loadAllObjectsImages();
		extractAllObjectDescriptors();
		trainVocabulary();
		trainNaiveBayesClassifier();
		break;

	case BAG_OF_WORDS_SVM:
		loadObjectsNames();

		if(loadVocabulary() == false || loadSVMClassifiers() == false)
		{
			printf("Vocabulary and SVM not loaded successfully\n");

			printf("\nLoading objects' images...\n");
			int num_img = loadAllObjectsImages();
			printf("Objects' images loaded: %d!\n", num_img);

			if(objects_.size()>=2)
			{

                printf("\nLoading objects' descriptors...\n");
                loadAllObjectsDescriptors();
                printf("Objects' descriptors loaded!\n");

				printf("\nTraining Vocabulary\n");
				trainVocabulary();
				printf("Vocabulary trained and stored in %s!\n", objects_main_path_.c_str());

				printf("\nTraining SVM classifiers\n");
				trainSVMClassifiers();
				printf("SVM classifiers trained\n");
			}
		}
		else
			printf("Vocabulary and SVM loaded successfully!\n");

		break;

	default:
		break;
	};

	return returned;
}

int Orbit::loadOrbit(string load_path)
{
	//Clear previous objects
	objects_.clear();

	printf("\nLoading names from %s...\n", load_path.c_str());

    objects_main_path_=load_path; 
        
	prepareOrbit(recog_type_);



	return objects_.size();
}

int Orbit::storeOrbit(string save_path, string objects_folder_name)
{

	boost::filesystem::remove_all(save_path+"/"+objects_folder_name+"/");

	string objects_path = save_path+"/"+objects_folder_name;

	//Create the main folder in the save path
	boost::filesystem::create_directory(objects_path);


	if(objects_vocabulary_.rows!=0)
	{
		//Save vocabulary in the created folder
		string vocabulary_file_name = objects_path+"/"+"vocabulary.xml.gz";

		cv::FileStorage fs( vocabulary_file_name, cv::FileStorage::WRITE );
		if( !fs.isOpened() )
		{
			//If I cannot open the file for writing, send an error
			return -1;
		}

		fs << "Vocabulary" << objects_vocabulary_;
	}


    for(unsigned int i = 0; i<objects_.size();i++) //For each object
    {
    	//Create a folder with the name of the object
    	boost::filesystem::create_directory(objects_path+"/"+objects_[i].name_);

    	if(objects_.size()>1)
    	{
			//Store its SVM in the created folder
			string svm_file = objects_path+"/"+objects_[i].name_+"/svm.xml.gz";
			//Saving svm to a file
			objects_[i].svm_->save(svm_file.c_str());
    	}
    }


	return 0;
}



/*
 *	Given the input_image, tries to recognize the object in the image.
 *	input_image - image of the object to recognize
 *	output_name - name of the recognized object
 *	type - specifies the type of recognition used to find the object
 *	(see enum types)
 *
 *	Return:
 *		A float, in the interval [0-1], indicating the certainty of the object found. If it is 0,
 *		then no object is detected
 */
float Orbit::recognizeObject(cv::Mat input_image, string& output_name, RECOGNITION_TYPE type)
{

	OrbitObject matched_object;



	float certainty = recognizeObject(input_image, matched_object, type);


	output_name = matched_object.name_;

	return certainty;
}

/*
 *	Given the input_image, tries to recognize the object in the image.
 *	input_image - image of the object to recognize
 *	output_object - recognized object (if possible)
 *	type - specifies the type of recognition used to find the object
 *	(see enum types)
 *
 *	Return:
 *		A float, in the interval [0-1], indicating the certainty of the object found. If it is 0,
 *		then no object is detected
 */
float Orbit::recognizeObject(cv::Mat input_image, OrbitObject &output_object, RECOGNITION_TYPE type)
{

	float certainty = 0;


	switch(type)
	{
	case BASIC_SURF_COUNT:
		certainty = recognizeObjectWithBasicSURFCount(input_image, output_object);
		break;
	case BASIC_NCC:
		certainty = recognizeObjectWithNCC(input_image, output_object);
		break;
	case BAG_OF_WORDS_NAIVE_BAYES:
		certainty = recognizeObjectWithBOW_NaiveBayes(input_image, output_object);
		break;
	case BAG_OF_WORDS_SVM:
		certainty = recognizeObjectWithBOW_SVM(input_image, output_object);
		break;

	default:
		break;
	};


	return certainty;
}

/*
 * Basic SURF Count that counts the number of descriptors matched for each group of descriptors associated with
 * the data_base objects
 */
float Orbit::recognizeObjectWithBasicSURFCount(cv::Mat input_image, OrbitObject& output_object)
{



	//Convert image to grayscale
	cv::Mat grayscale;
	cv::cvtColor(input_image, grayscale, CV_RGB2GRAY);


	//Data structures for descriptors extraction
	vector<cv::KeyPoint> extracted_keypoints;
	cv::Mat extracted_descriptors;

	//cv::GridAdaptedFeatureDetector descriptor_detector_2(new cv::SurfFeatureDetector, 50, 1, 1);

	//cv::SurfFeatureDetector descriptor_detector_2;
    //Detect keypoints from grayscale image
	descriptor_detector_->detect(input_image, extracted_keypoints);
    //Extract descriptors from grayscale image
	descriptor_extractor_->compute(input_image, extracted_keypoints, extracted_descriptors);

    vector<vector<cv::DMatch> > matches;

    unsigned int max_index = 0;
    unsigned int max_matches = 0, temp_matches;

	vector<unsigned int> matches_per_object;

    for(unsigned int i = 0; i<objects_.size();i++) //For each object
    {

    	matches.clear();

    	temp_matches=0;

        //Get matches of the extracted descriptors and stored object descriptors
    	desc_matcher_.radiusMatch(extracted_descriptors, objects_[i].descriptors_, matches, descriptors_match_threshold_);


    	//Calc the number of matches between input image and object[i]
    	for(unsigned int k = 0; k<extracted_keypoints.size();k++)
    	{
    		if(matches[k].size()!=0)
    		{
    			temp_matches++;
    		}
    	}

    	matches_per_object.push_back(temp_matches);

    	if(temp_matches>max_matches)
    	{
    		max_matches = temp_matches;
    		max_index = i;
    	}
   }

    for(unsigned int k = 0; k<matches_per_object.size();k++)
    	printf("%u, ", matches_per_object[k]);

    printf("\n");

    sort(matches_per_object.begin(), matches_per_object.end());


    //Assign more correlated object
    output_object = objects_[max_index];
    float certainty;

    if(extracted_keypoints.size()==0)
    	certainty = 0;
    else if((matches_per_object[matches_per_object.size()-1]-matches_per_object[matches_per_object.size()-2])<3)
    {
    	certainty = 0;
    }
    else
    	certainty = (float)max_matches/float(extracted_keypoints.size());
    	//certainty =1-float(matches_per_object[matches_per_object.size()-2]/((float)matches_per_object[matches_per_object.size()-1]));



    return certainty;
}

/*
 * The recognized object is the one that has the most correlated image with the input image
 */
float Orbit::recognizeObjectWithNCC(cv::Mat input_image, OrbitObject& output_object)
{

	float best_ncc[objects_.size()];

	float temp_best_ncc;

	cv::Mat grey_input_image;
	cv::cvtColor(input_image, grey_input_image, CV_RGB2GRAY);

	cv::Mat normalized_input_image = equalizeImage(grey_input_image);

	for(unsigned int i = 0; i<objects_.size();i++)
	{

		temp_best_ncc = 0;
		for(unsigned int im = 0; im<objects_[i].images_.size();im++)
		{
			cv::Mat result;
			cv::Mat grey_object_image;
			cv::cvtColor(objects_[i].images_[im], grey_object_image, CV_RGB2GRAY);
			cv::Mat normalized_object_image = equalizeImage(grey_object_image);



			//Calc Normalized Corr Coeff
			cv::matchTemplate(normalized_input_image, normalized_object_image,result, CV_TM_CCOEFF_NORMED);

			//Insert ncc value and index to vector
			if(result.at<float>(0,0)>temp_best_ncc)
				temp_best_ncc = result.at<float>(0,0);
		}

		best_ncc[i] = temp_best_ncc;
	}


	float max_ncc = 0;
	unsigned int max_index = 0;

	for(unsigned int i = 0; i<objects_.size();i++)
	{
		printf("%lg   -  ", best_ncc[i]);

		if(best_ncc[i]>max_ncc)
		{
			max_index = i;
			max_ncc = best_ncc[i];
		}
	}

	printf("\n");

	output_object = objects_[max_index];

	return 1;
}

/*
 * Basic SURF Count that counts the number of descriptors matched for each group of descriptors associated with
 * the data_base objects
 */
float Orbit::recognizeObjectWithBOW_NaiveBayes(cv::Mat input_image, OrbitObject& output_object)
{

	//Data structures for descriptors extraction
	vector<cv::KeyPoint> extracted_keypoints;
	//cv::GridAdaptedFeatureDetector descriptor_detector_2(new cv::SurfFeatureDetector, 50, 1, 1);

	//Convert image to grayscale
	cv::Mat grayscale;
	cv::cvtColor(input_image, grayscale, CV_RGB2GRAY);

    //Detect keypoints from grayscale image
	descriptor_detector_->detect(input_image, extracted_keypoints);

	if(extracted_keypoints.size()==0)
		return 0;

	cv::Mat words;
	bowExtractor_->compute(input_image, extracted_keypoints, words);



	cv::Mat resp = cv::Mat(1,1,CV_32FC1);


	classifier_model_->predict(words, &resp);

	float index = resp.at<float>(0);


	output_object = objects_[index];

	return 1;
}

/*
 *  Recognition using Bag of Words method and SVM classifiers
 */
float Orbit::recognizeObjectWithBOW_SVM(cv::Mat input_image, OrbitObject& output_object)
{
	//output_object = objects_[0];

	if(objects_.size()==0)
		return 0;
	else if(objects_.size()==1)
	{
		output_object = objects_[0];
		return 1;
	}

	//Data structures for descriptors extraction
	vector<cv::KeyPoint> extracted_keypoints;
	//cv::GridAdaptedFeatureDetector descriptor_detector_2(new cv::SurfFeatureDetector, 50, 1, 1);

	//Convert image to grayscale
	cv::Mat grayscale;
	cv::cvtColor(input_image, grayscale, CV_RGB2GRAY);

    //Detect keypoints from grayscale image
	descriptor_detector_->detect(input_image, extracted_keypoints);

	if(extracted_keypoints.size()==0)
		return 0;

	cv::Mat words;
	bowExtractor_->compute(input_image, extracted_keypoints, words);


	float certainty = 0;

	float confidences[objects_.size()];

	//Calculate confidences
	for(unsigned int i = 0; i<objects_.size();i++)
		confidences[i] = -1.f *objects_[i].svm_->predict(words, true);

	float max_confidence = 0;
	unsigned int max_index = 0;

	//Print confidences
	for(unsigned int i = 0; i<objects_.size();i++)
	{
		if(confidences[i]>max_confidence)
		{
			max_confidence = confidences[i];
			max_index = i;
		}

		printf("%s: %2.5f,  \t", objects_[i].name_.c_str(), confidences[i]);
	}

	cout<<endl;


	output_object = objects_[max_index];
	certainty = confidences[max_index];

	return certainty;
}


/*
 * This functions extract descriptors from the images of the objects and assigns
 * these descriptors to the respective objects. Returns the number of total
 * descriptors added and <0 if a failure occur
 */
int Orbit::extractAllObjectDescriptors()
{
	int num_descriptors = 0;

	for(unsigned int i = 0; i<objects_.size();i++)
  	{
		int temp;

		if((temp = extractOneObjectDescriptors(objects_[i]))<0)
			return temp;

		num_descriptors+= temp;
  	}

	return num_descriptors;
}

int Orbit::extractOneObjectDescriptors(OrbitObject &object)
{
	int num_descriptors = 0;

	for(unsigned int i = 0; i<object.images_.size();i++)
	{
		num_descriptors+=addDescriptorsToObject(object.images_[i], object);
	}


	//Store descriptors file

	string file_name = object.images_dir_path_+"/"+"descriptors.xml.gz";

    cv::FileStorage fs( file_name, cv::FileStorage::WRITE );
    if( !fs.isOpened() )
    {
    	//If I cannot open the file for writing, send an error
    	return -1;
    }

    fs << "Descriptors" << object.descriptors_;

	return num_descriptors;
}



/*
 * Loads the vocabulary.xml from objects_path and returns true. If file is not found, then
 * it will return false;
 */
bool Orbit::loadVocabulary(string input_path)
{

	if(input_path.compare("") == 0)
		input_path = objects_main_path_;

	string file_name = input_path+"/"+"vocabulary.xml.gz";

    cv::FileStorage fs(file_name, cv::FileStorage::READ);

    if( fs.isOpened() )
    {
        fs["Vocabulary"] >> objects_vocabulary_;

    	//Assign vocabulary to Bow Image Descriptor Extractor
    	bowExtractor_->setVocabulary(objects_vocabulary_);

        return true;
    }


    return false;

}



/*
 * Load svm.xml.gz classifier files. Returns true if succesfully loaded all svm files.
 * Return false if for at least one object, an svm file has not been found
 */
bool Orbit::loadSVMClassifiers()
{

	for(unsigned int i = 0; i < objects_.size();i++)
	{
	    /* first check if a previously trained svm for the current class has been saved to file */
	    string svmFilename = objects_[i].images_dir_path_+"/svm.xml.gz";

	    cv::FileStorage fs(svmFilename, cv::FileStorage::READ);
	    if( fs.isOpened() )
	    {
	    	objects_[i].svm_ = new CvSVM;
	    	objects_[i].svm_->load(svmFilename.c_str() );
	    }
	    else
	    	return false;
	}

	return true;
}


/*
 * Used to train the vocabulary for BOW. Returns the number of descriptors used to train the vocabulary
 */
int Orbit::trainVocabulary()
{
	if(objects_.size()==0)
		return 0;

	int num_descriptors = 0;

	for(unsigned int ob = 0; ob < objects_.size();ob++)
	{
		//Add extracted descriptors to the BOW Trainer
		num_descriptors+=objects_[ob].descriptors_.rows;
	}


	printf("\t Generating vocabulary from %d descriptors...\n", num_descriptors);

	//BoW KMeans Trainer Parameters
    cv::TermCriteria terminate_criterion;
    terminate_criterion.epsilon = FLT_EPSILON;
    int number_of_clusters = min(1500, num_descriptors/2);
    int k_means_attempts = 3;
    int k_means_flags=cv::KMEANS_PP_CENTERS;

    //initialize BoW trainer
    bowTrainer_ = new cv::BOWKMeansTrainer( number_of_clusters, terminate_criterion, k_means_attempts, k_means_flags);


	for(unsigned int ob = 0; ob < objects_.size();ob++)
	{
		//Add extracted descriptors to the BOW Trainer
		bowTrainer_->add(objects_[ob].descriptors_);
	}

	//Store vocabulary
	objects_vocabulary_ = bowTrainer_->cluster();

	//Assign vocabulary to Bow Image Descriptor Extractor
	bowExtractor_->setVocabulary(objects_vocabulary_);


	//Store vocabulary file
	string file_name = objects_main_path_+"/"+"vocabulary.xml.gz";

    cv::FileStorage fs( file_name, cv::FileStorage::WRITE );
    if( !fs.isOpened() )
    {
    	//If I cannot open the file for writing, send an error
    	return -1;
    }

    fs << "Vocabulary" << objects_vocabulary_;



	return num_descriptors;
}

/*
 * Trains the classifier used for the Bags of Words Approach
 */
int Orbit::trainNaiveBayesClassifier()
{
	//Data structures for descriptors extraction
	vector<cv::KeyPoint> extracted_keypoints;
	//cv::GridAdaptedFeatureDetector descriptor_detector_2(new cv::SurfFeatureDetector, 50, 1, 1);



	//Initialize classifier for BOW
	classifier_model_ = new CvNormalBayesClassifier;


	//Preparing data to train classifier model
	cv::Mat data(0, objects_vocabulary_.rows, CV_32FC1);
	cv::Mat responses(0, 1, CV_32FC1);

	for(unsigned int i = 0; i<objects_.size(); i++)
	{

		for(unsigned int j = 0; j<objects_[i].images_.size(); j++)
		{


			//Convert image to grayscale
			cv::Mat grayscale;
			cv::cvtColor(objects_[i].images_[j], grayscale, CV_RGB2GRAY);

			extracted_keypoints.clear();

		    //Detect keypoints from grayscale image
			descriptor_detector_->detect(objects_[i].images_[j], extracted_keypoints);

			if(extracted_keypoints.size()==0)
				continue;

			cv::Mat words;
			bowExtractor_->compute(objects_[i].images_[j], extracted_keypoints, words);

//			printf("Words dimention: %d, %d, of type %d = %d?\n", words.rows, words.cols, words.type(), CV_32FC1);
//
//			for(int r = 0; r<words.rows; r++)
//				for(int c = 0; c<words.cols; c++)
//					cout<<words.at<float>(r,c)<<" ";
//
//			printf("\n");
//			cv::waitKey(0);

			//printf("coeffs size (%d, %d)\n ", coeffs.rows, coeffs.cols);

			data.push_back(words);

			cv::Mat idx_matrix = cv::Mat::ones(1,1, CV_32FC1) * i;



			responses.push_back(idx_matrix);

		}
	}


	classifier_model_->train(data, responses);

	return 0;
}



/*
 * Trains the SVM Classifiers used for the Bags of Words Approach
 */
int Orbit::trainSVMClassifiers()
{

	if(objects_.size()==0)
		return 0;


	//Data structures for descriptors extraction
	vector<cv::KeyPoint> extracted_keypoints;
	//cv::GridAdaptedFeatureDetector descriptor_detector_2(new cv::SurfFeatureDetector, 50, 1, 1);


	//Load images for training
	loadAllObjectsImages();

	//Preparing data to train its SVM
	cv::Mat train_data(0, objects_vocabulary_.rows, CV_32FC1);
	
	
	unsigned int imgs_index;
	vector<unsigned int> bad_imgs;

	for(unsigned int i = 0; i<objects_.size(); i++) //For each object
	{

		imgs_index = 0;

		//Build SVM classifier for object i
		objects_[i].svm_ = new CvSVM;

		cv::Mat responses(0, 1,  CV_32SC1);

		for(unsigned int i2 = 0; i2<objects_.size(); i2++) //For each object in which we will get image
		{
			for(unsigned int j = 0; j<objects_[i2].images_.size(); j++) //For eachs object's image
			{

				if(i == 0) //Only extract descriptors in the iteration of the first object
				{	extracted_keypoints.clear();
					//Detect keypoints from grayscale image
					descriptor_detector_->detect(objects_[i2].images_[j], extracted_keypoints);

					if(extracted_keypoints.size()==0)
					{	
						bad_imgs.push_back(imgs_index);
						imgs_index++;
						continue;
					}
					cv::Mat words;

					if(extracted_keypoints.size()!=0)
						bowExtractor_->compute(objects_[i2].images_[j], extracted_keypoints, words);
					else
						words = cv::Mat::zeros(1, objects_vocabulary_.rows, CV_32F);


					train_data.push_back(words);
				}

				cv::Mat response_mat= cv::Mat::ones(1,1,  CV_32SC1);
				
				bool is_bad_image = false;
				
				for(unsigned int k = 0; k<bad_imgs.size(); k++)
				{
					if(bad_imgs[k] == imgs_index)
					{
						is_bad_image = true;
						
						break;
					}	
				} 
				
				if(!is_bad_image)
				{				
					if(i!=i2) //Object present at this image?
						response_mat*=-1.; //If not

					responses.push_back(response_mat);
					
				}
				
				imgs_index++;
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


        cout<<"\t Training classifier for "<<objects_[i].name_<<"..."<<endl;
        //cout<<"\t Train data="<<train_data.rows<<", Responses="<<responses.rows<<endl;

        objects_[i].svm_->train_auto(train_data, responses, cv::Mat(), cv::Mat(), svmParams, 10,
        		c_grid, gamma_grid, p_grid, nu_grid, coef_grid, degree_grid );
		//objects_[i].svm_->train(train_data, responses);

        string svm_file = objects_[i].images_dir_path_+"/svm.xml.gz";
        //Saving svm to a file
        objects_[i].svm_->save(svm_file.c_str());

        cout<<"\tClassifier for "<<objects_[i].name_<<" trained and stored!"<<endl;

	}


	return 0;
}


int Orbit::teachOrbit(string update_path)
{

	string path;

	if(update_path.compare("") == 0)
		path = update_path_;
	else
		path = update_path;

	vector<OrbitObject> temp_objects;

	printf("\nLoading names from update path: %s...\n", path.c_str());

	//Get folders' names
	for (boost::filesystem::directory_iterator itr(path); itr!=boost::filesystem::directory_iterator(); ++itr)
	{
		if (boost::filesystem::is_directory(itr->status()))
		{
			std::string object_name=itr->path().filename().string();
			OrbitObject object(object_name);

			object.images_dir_path_ = path+"/"+object_name;
			temp_objects.push_back(object);
		}
	}

	printf("%u names loaded!\n", (unsigned int)temp_objects.size());


	printf("\nLoading images from update path...\n");
	unsigned int num_imgs = 0;
	//Load images for each object
	for(unsigned int i = 0 ; i<temp_objects.size();i++)
		num_imgs+=loadOneObjectImages(temp_objects[i]);
	printf("%u images loaded from update path!\n", num_imgs);



	printf("\nLoading descriptors from data_base\n");
	int num_desc = 0;
	//Load all database objects descriptors

	num_desc = loadAllObjectsDescriptors();
	printf("%d descriptors loaded from data base!\n", num_desc);


	printf("\nLoading images from data_base\n");
	int num_img = 0;
	//Load all database objects descriptors
	num_img = loadAllObjectsImages();
	printf("%d images loaded from data base!\n", num_img);

    string time_now;
    stringstream out;
    out << time(NULL);
    time_now = out.str();

	for(unsigned int i = 0; i<temp_objects.size();i++) //For each new object
	{
		bool object_exists = false;

		for(unsigned int j = 0; j<objects_.size();j++) //For each database object
		{


			if( temp_objects[i].name_.compare(objects_[j].name_) == 0) //If object exists
			{
				object_exists = true;

				if(temp_objects[i].images_.size()>0)
				{
					printf("\nUpdating object %s with %u new images...\n", temp_objects[i].name_.c_str(), (unsigned int)temp_objects[i].images_.size());

					unsigned int image_index = objects_[j].images_.size();

					for(unsigned int im = 0; im<temp_objects[i].images_.size();im++) //For each new image
					{
						//Extract descriptors of new image and append them to the list of descriptors
						addDescriptorsToObject(temp_objects[i].images_[im], objects_[j]);

						//Append image to the object in data base
						objects_[j].images_.push_back(temp_objects[i].images_[im]);
					}

					//Save new descriptors in descriptors.xml.gz
					string file_name = objects_[j].images_dir_path_+"/"+"descriptors.xml.gz";

					cv::FileStorage fs( file_name, cv::FileStorage::WRITE );
					if( !fs.isOpened() )
					{
						//If I cannot open the file for writing, send an error
						return -1;
					}

					//Saving descriptors
					fs << "Descriptors" << objects_[j].descriptors_;




					//Save new images to the data base folder of the object
					for(unsigned int im = 0; im<temp_objects[i].images_.size();im++)
					{
						string filename;
    						stringstream out;

						out <<objects_[j].images_dir_path_<<"/"<<time_now<<"_"<<image_index<<".jpg";
						filename = out.str();

						vector<int> params;

						params.push_back(CV_IMWRITE_JPEG_QUALITY);
						params.push_back(100);
						cv::imwrite(filename.c_str(), temp_objects[i].images_[im], params);
						image_index++;
					}

					printf("Object %s updated!\n", temp_objects[i].name_.c_str());
				}
			}
		}

		//If object doesn't exist, we need to append it to the list of objects
		if(!object_exists )
		{
			printf("\nAdding new object to data base: %s\n", temp_objects[i].name_.c_str());

			//Create folder of the object in data base path
			boost::filesystem::create_directory(objects_main_path_+"/"+temp_objects[i].name_);

			unsigned int image_index = 0;

			//Save all images in that path
			for(unsigned int im = 0; im<temp_objects[i].images_.size();im++)
			{
				string filename;
				stringstream out;

				out<<objects_main_path_+"/"+temp_objects[i].name_<<"/"<<time_now<<"_"<<image_index<<".jpg";
				filename = out.str();

				vector<int> params;

				params.push_back(CV_IMWRITE_JPEG_QUALITY);
				params.push_back(100);

				cv::imwrite(filename.c_str(), temp_objects[i].images_[im], params);

				image_index++;
			}

			OrbitObject object_to_append(temp_objects[i].name_);
			object_to_append.images_dir_path_ = objects_main_path_+"/"+temp_objects[i].name_;


			//Load new object images
			loadOneObjectImages(object_to_append);

			//Extract new object descriptors
			extractOneObjectDescriptors(object_to_append);

			//Add new object to the object list
			objects_.push_back(object_to_append);

			printf("Number of images of new object: %u\n", (unsigned int)objects_[objects_.size()-1].images_.size());

			printf("Object %s added\n", object_to_append.name_.c_str());
		}
	}


	if(objects_.size()<2)
		return 0;


	printf("Retraining vocabulary for BOW\n");
	//Retrain vocabulary
	trainVocabulary();
	printf("Vocabulary trained!\n");


	printf("Retraining SVM Classifiers\n");
	//Retrain all objects SVM classifiers
	trainSVMClassifiers();
	printf("SVM Classifiers trained\n");




	return 0;
}




void Orbit::setSVMParams( CvSVMParams& svmParams, cv::Mat responses)
{
    int pos_ex = cv::countNonZero(responses == 1);
    int neg_ex = cv::countNonZero(responses == -1);



    svmParams.svm_type = CvSVM::C_SVC;

    if(linear_kernel)
    	svmParams.kernel_type = CvSVM::LINEAR;
    else
    	svmParams.kernel_type = CvSVM::RBF;


	//cv::Mat class_wts( 2, 1, CV_32FC1 );
	CvMat *class_wts = cvCreateMat(2, 1, CV_32FC1);

	//class_wts.at<float>(0) = static_cast<float>(neg_ex)/static_cast<float>(pos_ex+neg_ex); // weighting for costs of positive class + 1 (i.e. cost of false positive - larger gives greater cost)
	//class_wts.at<float>(1) = static_cast<float>(pos_ex)/static_cast<float>(pos_ex+neg_ex); // weighting for costs of negative class - 1 (i.e. cost of false negative)

	cvmSet(class_wts, 0, 0, static_cast<float>(neg_ex)/static_cast<float>(pos_ex+neg_ex));
	cvmSet(class_wts, 1, 0, static_cast<float>(pos_ex)/static_cast<float>(pos_ex+neg_ex));


	svmParams.class_weights = class_wts;

}

void Orbit::setSVMTrainAutoParams( CvParamGrid& c_grid, CvParamGrid& gamma_grid,
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

cv::Mat Orbit::equalizeImage(cv::Mat input_image)
{
	float hw_ratio = float(patch_size_.height)/float(patch_size_.width);

	cv::Rect new_dim_rect;

	new_dim_rect.height= min(float(input_image.rows), input_image.cols*hw_ratio);
	new_dim_rect.width= min(float(input_image.cols), input_image.rows/hw_ratio);

	/*Centralize patch*/
	if(new_dim_rect.height<input_image.rows)
		new_dim_rect.y += (input_image.rows-new_dim_rect.height)/2.;

	if(new_dim_rect.width<input_image.cols)
		new_dim_rect.x += (input_image.cols-new_dim_rect.width)/2.;


	cv::Mat equalized_patch = cv::Mat(patch_size_, CV_8UC1);

	cv::resize(input_image(new_dim_rect),equalized_patch, equalized_patch.size(), 0, 0);

	cv::equalizeHist(equalized_patch, equalized_patch);

	return equalized_patch;
}



int Orbit::addDescriptorsToObject(cv::Mat image, OrbitObject &object)
{
	int num_descriptors = 0;

	//Convert image to grayscale
	cv::Mat grayscale;
	cv::cvtColor(image, grayscale, CV_RGB2GRAY);


	//Data structures for descriptors extraction
	vector<cv::KeyPoint> extracted_keypoints;
	cv::Mat extracted_descriptors;

    //Detect keypoints from grayscale image
	descriptor_detector_->detect(image, extracted_keypoints);
    //Extract descriptors from grayscale image
    descriptor_extractor_->compute(image, extracted_keypoints, extracted_descriptors);

    if(object.descriptors_.rows == 0) //If object doesn't have any descriptor in model
    {
    	object.descriptors_ = extracted_descriptors;
    	return extracted_descriptors.rows;
    }
    else //If object has at leat onde descriptor in model
    {
        vector<vector<cv::DMatch> > matches;
        //Get matches of new descriptors with stored descriptors so as to don't add similar descriptors to object model
    	desc_matcher_.radiusMatch(extracted_descriptors, object.descriptors_, matches, 1.5*descriptors_match_threshold_);

    	for(unsigned int i = 0; i<extracted_keypoints.size();i++)
    	{
    		if(matches[i].size()==0) //Add different to the rest descriptors
    		{
    			object.descriptors_.push_back(extracted_descriptors.row(i));
    			//cv::circle(img_BGR,extracted_keypoints[i].pt,2,cv::Scalar(0,0,255),2);
    			num_descriptors++;
    		}
    	}
    }

	return num_descriptors;
}
