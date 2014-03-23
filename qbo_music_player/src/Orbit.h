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
 * Authors: Arturo Bajuelos <arturo@openqbo.com>
 */

#ifndef ORBIT_H_
#define ORBIT_H_

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <set>
#include <cmath>
#include <string>

#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxmisc.h>
#include <stdio.h>
#include <cxcore.h>
#include <climits>
#include <cfloat>
#include <ctype.h>
#include <ml.h>





#include "boost/filesystem.hpp"   // includes all needed Boost.Filesystem declarations

#include "OrbitObject.h"

using namespace std;

class Orbit {
public:
	Orbit(string images_path = "", string update_path = "");

	virtual ~Orbit();

	enum RECOGNITION_TYPE {
	      BASIC_SURF_COUNT,
	      BASIC_NCC,
	      BAG_OF_WORDS_NAIVE_BAYES,
	      BAG_OF_WORDS_SVM
	};

	bool linear_kernel;


	string objects_main_path_;

	string update_path_;

	vector<OrbitObject> objects_;

	cv::Size patch_size_;



	/**
	 * For recognition using SURF Descriptors
	 */
	//cv::SurfFeatureDetector descriptor_detector_;
	//cv::DynamicAdaptedFeatureDetector descriptor_detector_;
	//cv::GridAdaptedFeatureDetector descriptor_detector_;

	cv::Ptr<cv::FeatureDetector> descriptor_detector_;

	cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_;


	cv::BruteForceMatcher<cv::L2<float> > desc_matcher_;
	float descriptors_match_threshold_;


	//Variables for Bag of Words approach
	cv::Ptr<cv::BOWKMeansTrainer> bowTrainer_;
	cv::Ptr<cv::BOWImgDescriptorExtractor> bowExtractor_;
	cv::Mat objects_vocabulary_;
	cv::Ptr<CvNormalBayesClassifier> classifier_model_; //Classifier used for BOW

	/*
	 * Load the objects, given the object image path. Fills the vector of OrbitObjects
	 *
	 * Return number of images collected. If < 0, then an error ocurred
	 */
	int loadObjectsNames(string input_path="");



	/*
	 * Given all objects, load their images. Uses loadOneObjectImages
	 */
	int loadAllObjectsImages();

	/*
	 * Given one objects, load their images
	 * Returns the number of loaded images
	 */
	int loadOneObjectImages(OrbitObject &object);

	/*
	 * Given the objects, load their descriptors file from the images folder
	 * If a descriptors file isn't found for an object, then it extracts descriptors from
	 * all images of that object
	 */
	int loadAllObjectsDescriptors();


	/*
	 * Given an object, load its descriptors from the images folder.
	 * Return true if succeeded
	 */
	int loadOneObjectsDescriptors(OrbitObject &object);


	/*
	 * Prepare orbit to use the input type method. For eg., for BASIC_SURF_COUNT,
	 * it loads images and extract descriptors of the objects loaded
	 * If < 0, then an error ocurre
	 */
	int prepareOrbit(RECOGNITION_TYPE type = BAG_OF_WORDS_SVM);


	/*
	 * Load Orbit's files necessary to recognize objects. In the case of BOW with SVM, it loads
	 * the SVM classifiers for each object, and the vocabulary necessary to generate the codewords
	 * Returns the number of objects loaded. If <0, then an error occurred.
	 *
	 */
	int loadOrbit(string load_path);

	/*
	 * Stores Orbit's files necessary to recognize object. In the case of BOW with SVM, it stores
	 * the SVM classifiers for each object, and the vocabulary necessary to generate the codewords
	 * Returns 0 if succeeded and <0 if not.
	 *
	 */
	int storeOrbit(string save_path, string objects_folder_name = "orbit_store");


	/*
	 *	Given the input_image, tries to recognize the object in the image.
	 *	input_image - image of the object to recognize
	 *	output_name - name of the recognized object (if possible)
	 *	type - specifies the type of recognition used to find the object
	 *	(see enum types)
	 *
	 *	Return:
	 *		A float, in the interval [0-1], indicating the certainty of the object found. If it is 0,
	 *		then no object is detected
	 */
	float recognizeObject(cv::Mat input_image, string& output_name, RECOGNITION_TYPE type = BAG_OF_WORDS_SVM);

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
	float recognizeObject(cv::Mat input_image, OrbitObject &output_object, RECOGNITION_TYPE type = BAG_OF_WORDS_SVM);




	/*
	 * Basic SURF Count that counts the number of descriptors matched for each group of descriptors associated with
	 * the data_base objects
	 */
	float recognizeObjectWithBasicSURFCount(cv::Mat input_image, OrbitObject& output_object);



	/*
	 * The recognized object is the one that has the most correlated image with the input image
	 */
	float recognizeObjectWithNCC(cv::Mat input_image, OrbitObject& output_object);

	/*
	 *  Recognition using Bag of Words method and a Naive Bayes Classifier
	 */
	float recognizeObjectWithBOW_NaiveBayes(cv::Mat input_image, OrbitObject& output_object);


	/*
	 *  Recognition using Bag of Words method and SVM classifiers
	 */
	float recognizeObjectWithBOW_SVM(cv::Mat input_image, OrbitObject& output_object);



	int extractAllObjectDescriptors();

	int extractOneObjectDescriptors(OrbitObject &object);


	//
	// For Bags of Words DESCRIPTORS APPROACH
	//

	/*
	 * Loads the vocabulary.xml from objects_path and returns true. If file is not found, then
	 * it will return false;
	 */
	bool loadVocabulary(string input_path = "");

	/*
	 * Used to train the vocabulary for BOW. Returns the number of descriptors used to train the vocabulary
	 */
	int trainVocabulary();


	/*
	 * Trains the Naive Bayes Classifier used for the Bags of Words Approach
	 */
	int trainNaiveBayesClassifier();



	/*
	 * Load svm.xml classifier files. Returns true if succesfully loaded all svm files.
	 * Return false if for at least one object, an svm file has not been found
	 */
	bool loadSVMClassifiers();

	/*
	 * Trains the SVM Classifiers used for the Bags of Words Approach
	 */
	int trainSVMClassifiers();



	/*
	 *	Given a path, it extracts the images of each's object's folder,  copy the files to the respective folders in
	 *	objects's main path, add's it to the model, generate the vocabulary, retrain the classifiers.
	 *	Return 0 if succeeded, and != 0 if not succeeded.
	 */
	int teachOrbit(string update_path = "");


private:
	cv::Mat equalizeImage(cv::Mat input_image);


	//Auxiliary functions to get SVM params
	void setSVMParams( CvSVMParams& svmParams, cv::Mat responses);

	void setSVMTrainAutoParams( CvParamGrid& c_grid, CvParamGrid& gamma_grid,
	                            CvParamGrid& p_grid, CvParamGrid& nu_grid,
	                            CvParamGrid& coef_grid, CvParamGrid& degree_grid );


	/*
	 * Given an image, it extracts the good SURF descriptors from it and add it to the object's model
	 * Returns the number of descriptors added to the model. If <0, then an error has occurred
	 */
	int addDescriptorsToObject(cv::Mat image, OrbitObject &object);

};

#endif /* ORBIT_H_ */
