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

#ifndef PERSON_H_
#define PERSON_H_

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>

#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxmisc.h>
#include <stdio.h>
#include <cxcore.h>
#include <ml.h>

using std::string;
using std::vector;

class Person {
public:
	Person(string name="");

	vector<cv::Mat> images_;

	string name_;

	string images_dir_path_;

	cv::Ptr<CvSVM> pca_svm_;

	cv::Mat pca_proj_;

	cv::Mat descriptors_;

	cv::Ptr<CvSVM> bow_svm_; //Used for Bag of Words Approach


	virtual ~Person();
};

#endif /* PERSON_H_ */
