/*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2012 Thecorpora, Inc.
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
 * Authors: Miguel Angel Julian <miguel.a.j@openqbo.org>;
 * 
 */

#ifndef _STEREOVISION_H
#define	_STEREOVISION_H

#include "cv.h"
#include "cxcore.h"
#include "cvaux.h"
#include "cxmisc.h"
#include "highgui.h"

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/SetCameraInfo.h"
#include "image_transport/image_transport.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "cv_bridge/CvBridge.h"
#include <boost/signals2/mutex.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <semaphore.h>

class StereoVision {
private:

    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber left_image_sub_;
    image_transport::CameraSubscriber right_image_sub_;
    std::string stereo_ns_;

    //Cliente para guardar los datos
    sensor_msgs::CvBridge bridge_;

    cv::Size2i image_size_;
    cv::Size2i left_image_size_;
    cv::Size2i right_image_size_;

    //Variables para realizar la rectificación de las imágenes
    cv::Mat mtx1_;
    cv::Mat mty1_;
    cv::Mat mtx2_;
    cv::Mat mty2_;

    cv::Mat intrinsic_matrix_left_;
    cv::Mat intrinsic_matrix_right_;
    cv::Mat distorsion_matrix_left_;
    cv::Mat distorsion_matrix_right_;
    cv::Mat rotation_matrix_left_;
    cv::Mat rotation_matrix_right_;
    cv::Mat proyection_matrix_left_;
    cv::Mat proyection_matrix_right_;
    cv::Mat f_matrix_;
    cv::Mat t_matrix_;
    cv::Mat q_matrix_;

    //Distancia focal de las cámaras (con las imágenes rectificadas)
    double focal_distance_;
    //Distancia real entre las cámaras (en mm)
    double camera_distance_;
    
    //Variables para guardar las imagenes capturadas. Zona de exclusion mutua
    cv::Mat left;
    cv::Mat right;

    //Variable para crear el mapa de disparidad
    cv::Mat disp;
    cv::Mat vdisp;

    //Variables de las imagenes capturadas en blanco y negro
    cv::Mat left_bn_;
    cv::Mat right_bn_;

    boost::mutex left_mutex_;
    boost::mutex right_mutex_;

    bool calibrated;
    bool mostrar_imagen_left_;
    bool mostrar_imagen_right_;

    ros::Time left_image_time_;
    ros::Time right_image_time_;

    void leftImageCallback(const sensor_msgs::ImageConstPtr& msg_ptr, const sensor_msgs::CameraInfoConstPtr& info_ptr);
    void rightImageCallback(const sensor_msgs::ImageConstPtr& msg_ptr, const sensor_msgs::CameraInfoConstPtr& info_ptr);


public:
    StereoVision ();
    ~StereoVision ();
    int rectificarImagen(int iCamera, cv::Mat& i);  //Esta es la que hay que controlar
    int calibrateStereo(int nx, int ny, int n_boards, float squareSize=1);
    int calibrateSingleCamara(int cam, int board_w, int board_h, int n_boards, cv::Mat& m, cv::Mat& d, float squareSize=1);
    int clickToCapture(cv::Mat color[]);
    int clickToCaptureSingle(int cam, cv::Mat& color);
    int mostrarCamaras();
    int mostrarCamara(int cam);

};

#endif	/* _STEREOVISION_H */

