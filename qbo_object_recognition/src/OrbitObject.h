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

#ifndef CORBITOBJECT_H_
#define CORBITOBJECT_H_

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
#include <ml.h>

using namespace std;

class OrbitObject {
public:
	OrbitObject(string name="");
	virtual ~OrbitObject();

	vector<cv::Mat> images_;

	string name_;

	string images_dir_path_;

	cv::Mat descriptors_;

	cv::Ptr<CvSVM> svm_; //Used for Bag of Words Approach




};

#endif /* CORBITOBJECT_H_ */
