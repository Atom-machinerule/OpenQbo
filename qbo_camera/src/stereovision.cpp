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

#include "stereovision.h"
#include "camera_calibration_parsers/parse_yml.h"
#include <dirent.h>

#define LEFT 0
#define RIGHT 1

using namespace std;
using namespace cv;

//Constructor de la clase
StereoVision::StereoVision () : n_("~"), it_(n_),
                                mostrar_imagen_left_(false), mostrar_imagen_right_(false)
{ //Inicializacion de todas las variables que se necesitan

    //Subscribirse a las subscripciones de las imagenes y al servidor
    stereo_ns_ = ros::names::resolve(std::string("stereo"));
    std::string left_topic = ros::names::clean(stereo_ns_ + "/left/image_raw");
    std::string right_topic = ros::names::clean(stereo_ns_ + "/right/image_raw");

    left_image_sub_ = it_.subscribeCamera(left_topic, 1, &StereoVision::leftImageCallback, this);
    right_image_sub_ = it_.subscribeCamera(right_topic, 1, &StereoVision::rightImageCallback, this);

    //Obtenemos el tamaño de las imagenes de los topics. Para ello esperamos a que entren imágenes
    image_size_=Size2i(0,0);  //Tamaño de las imágenes que capturamos
    while(image_size_.width==0 || image_size_.height==0)
    {
        ros::spinOnce();
        if((left_image_size_.width == right_image_size_.width ) && (left_image_size_.height == right_image_size_.height))
          image_size_=left_image_size_;
    }

}

//Destructor de la clase para liberar la memoria reservada
StereoVision::~StereoVision() {
}

//Rectifica la imagen que le pasemos y la guarda dentro de la clase como la imagen de la camara correspondiente
int StereoVision::rectificarImagen(int iCamera, Mat& i) {
    Mat mx;
    Mat my;
    if(iCamera==2) {    //Camara derecha==2
        mx=mtx2_;
        my=mty2_;
    }
    else {    //Camara izquierda==1
        mx=mtx1_;
        my=mty1_;
    }
    if(calibrated) {
        Mat imagen;
        i.copyTo(imagen);
        remap(imagen,i,mx,my,INTER_LINEAR);
    }
}

//Realiza el calibrado estereo de las camaras
//nx es el numero de esquinas en el eje x del grid de calibracion
//ny es el numero de esquinas en el eje y del grid de calibracion
//n_boards es el numero de imagenes con el que se hace cada calibracion
//c son las camaras
int StereoVision::calibrateStereo(int nx, int ny, int n_boards, float squareSize) {

    bool singleCamara=true;    //true hace el calibrado de las camaras por separado y luego el estereo. False solo el estereo
    Size2i patternSize(nx,ny);

    Mat img(image_size_,CV_8UC3);
    Mat c_color_l(image_size_,CV_8UC3);
    Mat c_color_r(image_size_,CV_8UC3);
    Mat color[]={c_color_l,c_color_r};

    cout << "Welcome to the camera calibration process" << endl << endl;

    if(singleCamara)
    {
        //Primero calibrar las camaras por separado para obtener _M1, _M2, _D1 y _D2

        cout << "Left camera calibration process stars" << endl;
        calibrateSingleCamara(LEFT,nx,ny,n_boards,intrinsic_matrix_left_,distorsion_matrix_left_,squareSize);

        cout << endl << "Right camera calibration process stars" << endl;
        calibrateSingleCamara(RIGHT,nx,ny,n_boards,intrinsic_matrix_right_,distorsion_matrix_right_,squareSize);
    }

    int salida=-1;
    do {

        cout << endl << "Stereo camera calibration process starts" << endl;
        cout << n_boards << " images are going to be picked." << endl;
        cout << "The calibration pattern must be seen by the two cameras at the same time."
                << "Push a key to capture the image" << endl;

        //const float squareSize = 30.f; //Dimension de las casillas del patrón de calibrado en mm. Importante para el cálculo de las distancias. Quiero buscarle un sitio a la variable para que se pueda modificar
        int i, j, lr, n = nx*ny;

        vector<vector<Point3f> > objectPoints;
        vector<vector<Point2f> > points[2];
        vector<Point2f> temp;

        int success1=0;
        bool fallo[]={true,true};
        namedWindow( "Left camera capture result" );
        cvMoveWindow("Left camera capture result", 0, image_size_.height+30);
        namedWindow( "Right camera capture result" );
        cvMoveWindow("Right camera capture result", image_size_.width, image_size_.height+30);
        for(i=0;success1<n_boards;i++)
        {
            bool result=false;
            lr = i % 2;
            vector<vector<Point2f> >& pts = points[lr]; //pts apunta a los puntos de la camara con la que se trabaja (left-right)

            //Adquirimos las dos imagenes a la vez (una vez cada 2 pasadas)
            if(!lr)
                clickToCapture(color);  //guarda en color la captura simultanea de las 2 camaras. color es un vector con las dos imagenes

            //FIND CHESSBOARDS AND CORNERS THEREIN:
            result=findChessboardCorners(color[lr],patternSize,temp,CV_CALIB_CB_ADAPTIVE_THRESH+CV_CALIB_CB_FILTER_QUADS);

            if( result )    //Si el resultado de la busqueda de esquinas es bueno
            {
                // Get subpixel accuracy on those corners
                cvtColor(color[lr], img, CV_BGR2GRAY);
                cornerSubPix(img,temp,Size(11,11),Size(-1,-1),TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01));

                // Draw it
                drawChessboardCorners(color[lr],patternSize,Mat_<Point2f>(temp),result);
                if(lr)
                    imshow( "Right camera capture result", color[lr] );
                else
                    imshow( "Left camera capture result", color[lr] );

                pts.push_back(temp);
                fallo[lr]=false;
            }
            else {
                fallo[lr]=true;
            }

            if(lr==1) {
                if(fallo[0]||fallo[1])
                {
                    if (!fallo[0])
                      points[0].pop_back();
                    if (!fallo[1])
                      points[1].pop_back();
                    cout << "Fail. An additional image is going to be taken" << endl;
                }
                else
                {
                    success1++;
                    cout << n_boards-success1 << "  images left" << endl;
                }

            }

        }   //Fin del procesamiento de todas las imagenes


        printf("\n");
        // HARVEST CHESSBOARD 3D OBJECT POINT LIST:
        vector<Point3f> objectPointsCoordenates(n);
        for( i = 0; i < ny; i++ )
            for( j = 0; j < nx; j++ )
                objectPointsCoordenates[i*nx + j] = Point3f(i*squareSize, j*squareSize, 0); //Coordenadas de los puntos en un eje real
        for( i = 0; i < success1; i++ )
            objectPoints.push_back(objectPointsCoordenates);

        // CALIBRATE THE STEREO CAMERAS
        printf("Running stereo calibration ...");
        fflush(stdout);

        Mat e;
        Mat r;
        if(singleCamara)
            stereoCalibrate(objectPoints,points[0],points[1],intrinsic_matrix_left_,distorsion_matrix_left_,intrinsic_matrix_right_,distorsion_matrix_right_,image_size_,r,t_matrix_,e,f_matrix_);
        else
            stereoCalibrate(objectPoints,points[0],points[1],intrinsic_matrix_left_,distorsion_matrix_left_,intrinsic_matrix_right_,distorsion_matrix_right_,image_size_,r,t_matrix_,e,f_matrix_,TermCriteria(TermCriteria::COUNT+  TermCriteria::EPS, 30, 1e-6),NULL);

        printf(" done\n");

        //COMPUTE RECTIFICATION
        stereoRectify(intrinsic_matrix_left_,distorsion_matrix_left_,intrinsic_matrix_right_,distorsion_matrix_right_,image_size_,r,t_matrix_,rotation_matrix_left_,rotation_matrix_right_,proyection_matrix_left_,proyection_matrix_right_,q_matrix_,-1);
        initUndistortRectifyMap(intrinsic_matrix_left_,distorsion_matrix_left_,rotation_matrix_left_,proyection_matrix_left_,image_size_,CV_32FC1,mtx1_,mty1_);
        initUndistortRectifyMap(intrinsic_matrix_right_,distorsion_matrix_right_,rotation_matrix_right_,proyection_matrix_right_,image_size_,CV_32FC1,mtx2_,mty2_);

        calibrated=true;

        // CALIBRATION QUALITY CHECK
        // because the output fundamental matrix implicitly
        // includes all the output information,
        // we can check the quality of calibration using the
        // epipolar geometry constraint: m2^t*F*m1=0
        vector<Point2f> imagePoints1;
        vector<Point2f> imagePoints2;
        for(int i=0;i<points[0].size();i++) {
            imagePoints1.insert(imagePoints1.begin(),points[0][i].begin(),points[0][i].end());
            imagePoints2.insert(imagePoints2.begin(),points[1][i].begin(),points[1][i].end());
        }
        Mat rot;
        undistortPoints(Mat_<Point2f>(imagePoints1),imagePoints1,intrinsic_matrix_left_,distorsion_matrix_left_,rot,intrinsic_matrix_left_);
        undistortPoints(Mat_<Point2f>(imagePoints2),imagePoints2,intrinsic_matrix_right_,distorsion_matrix_right_,rot,intrinsic_matrix_right_);

        vector<Vec3f> lines1;
        vector<Vec3f> lines2;

        computeCorrespondEpilines(Mat_<Point2f>(imagePoints1),1,f_matrix_,lines1);
        computeCorrespondEpilines(Mat_<Point2f>(imagePoints2),2,f_matrix_,lines2);

        vector<Point2f> _points[]={imagePoints1,imagePoints2};
        vector<Vec3f> lines[]={lines1,lines2};

        double avgErr = 0;
        for( i = 0; i < lines1.size(); i++ )
        {
            double err = fabs(_points[0][i].x*lines[1][i][0] +
                _points[0][i].y*lines[1][i][1] + lines[1][i][2])
                + fabs(_points[1][i].x*lines[0][i][0] +
                _points[1][i].y*lines[0][i][1] + lines[0][i][2]);
            avgErr += err;
        }
        cout << "avg err = " << avgErr/lines1.size() << endl;
        

        cout << "Calibration result is being showed. If you want to repeat it press 'r'" << endl;
        cout << "Press enter to continue" << endl;

        salida=mostrarCamaras();

        if(salida!='r')
        {
            //Enviamos los datos de calibrado al servicio de recalibrado
            cv::Mat l_i=intrinsic_matrix_left_.reshape(0,1);
            cv::Mat r_i=intrinsic_matrix_right_.reshape(0,1);
            cv::Mat l_d=distorsion_matrix_left_.reshape(0,1);
            cv::Mat r_d=distorsion_matrix_right_.reshape(0,1);
            cv::Mat l_r=rotation_matrix_left_.reshape(0,1);
            cv::Mat r_r=rotation_matrix_right_.reshape(0,1);
            cv::Mat l_p=proyection_matrix_left_.reshape(0,1);
            cv::Mat r_p=proyection_matrix_right_.reshape(0,1);

            sensor_msgs::CameraInfo left_info;
            sensor_msgs::CameraInfo right_info;
            left_info.height=image_size_.height;
            right_info.height=image_size_.height;
            left_info.width=image_size_.width;
            right_info.width=image_size_.width;

            for (int i=0; i<9; i++) {
              left_info.K[i] = l_i.at<double>(0,i);
              right_info.K[i] = r_i.at<double>(0,i);
              left_info.R[i] = l_r.at<double>(0,i);
              right_info.R[i] = r_r.at<double>(0,i);
            }
            left_info.distortion_model= "plumb_bob";
            right_info.distortion_model="plumb_bob";
            for (int i=0; i<5; i++) {
            /*
              left_info.D[i] = l_d.at<double>(0,i);
              right_info.D[i] = r_d.at<double>(0,i);
            */
              left_info.D.push_back(l_d.at<double>(0,i));
              right_info.D.push_back(r_d.at<double>(0,i));
            }
            for (int i=0; i<12; i++) {
              left_info.P[i] = l_p.at<double>(0,i);
              right_info.P[i] = r_p.at<double>(0,i);
            }

            sensor_msgs::SetCameraInfo setInfoMsg;

            setInfoMsg.request.camera_info=right_info;
            if (!ros::service::call(stereo_ns_ + "/right/set_camera_info", setInfoMsg))
            {
              ROS_ERROR("Failed to call service set_camera_info. The calibration result is going to be saved to a file");
            }
            setInfoMsg.request.camera_info=left_info;
            if (!ros::service::call(stereo_ns_ + "/left/set_camera_info", setInfoMsg))
            {
              ROS_ERROR("Failed to call service set_camera_info. The calibration result is going to be saved to a file");
            }
            char * pPath;
            pPath = getenv ("HOME");

            //Path donde están los archivos de calibración
            std::string home_path(pPath);

            //camera_calibration_parsers::writeCalibrationYml(home_path+"/right_camera_calibration_result.yml", std::string("right_camera"), right_info);
            camera_calibration_parsers::writeCalibrationYml(home_path+"/.ros/camera_info/right_camera.yaml", std::string("right_camera"), right_info);
            //camera_calibration_parsers::writeCalibrationYml(home_path+"/left_camera_calibration_result.yml", std::string("left_camera"), left_info);
            camera_calibration_parsers::writeCalibrationYml(home_path+"/.ros/camera_info/left_camera.yaml", std::string("left_camera"), left_info);
      }


    }while(salida=='r');
    cvDestroyWindow("Right camera capture result");
    cvDestroyWindow("Left camera capture result");
}

//Realiza la calibración de una camara
//Se le pasan como parámetros
//cam: la camara que vamos a calibrar
//board_w y board_h los tamaños del patron de calibración
//n_boards: número de imágenes con el que se hará la calibración
//m y d las matrices donde se guardarán los parámetros intrísecos y de distorsión de la camara
int StereoVision::calibrateSingleCamara(int cam,int board_w, int board_h, int n_boards, Mat& m, Mat& d, float squareSize) {

    Size2i patternSize(board_w,board_h);
    cout << "The complete calibration pattern must be seen by the camera when capturing the images" << endl << "Change the calibration pattern position between captures" << endl;
    int c='r';
    while(c=='r')
    {
        int i, j, nframes, n = board_w*board_h;
        vector<vector<Point3f> > objectPoints;
        vector<vector<Point2f> > points;
        vector<Point2f> temp;

        int successes=0;

	      namedWindow( "Capture result" );
        cvMoveWindow("Capture result", image_size_.width, 0);

        Mat image(image_size_,CV_8UC3);
        Mat gray_image(image_size_,CV_8UC1);

        cout << n_boards << " images are going to be taken" << endl;
        cout << "Press a key to capture the image" << endl;

	      while( successes < n_boards ){
            clickToCaptureSingle(cam,image);
            bool result=findChessboardCorners(image,patternSize,temp,CV_CALIB_CB_ADAPTIVE_THRESH+CV_CALIB_CB_FILTER_QUADS);

            if( result )    //Si el resultado de la busqueda de esquinas es bueno
            {
                // Get subpixel accuracy on those corners
                cvtColor(image, gray_image, CV_BGR2GRAY);
                cornerSubPix(gray_image,temp,Size(11,11),Size(-1,-1),TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01));

                // Draw it
                drawChessboardCorners(image,patternSize,Mat_<Point2f>(temp),result);
                imshow( "Capture result", image );

                points.push_back(temp);
                successes++;
                cout << n_boards-successes << " images left" << endl;
            }
            else {
                cout << "This image is not going to be used." << endl << "An additional image is going to be taken" << endl;
            }
	      } // End collection while loop

        nframes = successes;//Number of good chessboads found
        vector<Point3f> objectPointsCoordenates(n);
        for( i = 0; i < board_h; i++ )
            for( j = 0; j < board_w; j++ )
                objectPointsCoordenates[i*board_w + j] = Point3f(i*squareSize, j*squareSize, 0); //Coordenadas de los puntos en un eje real
        for( i = 0; i < nframes; i++ )
            objectPoints.push_back(objectPointsCoordenates);
        
        m=Mat(3,3,CV_64F);
        setIdentity(m);
        d=Mat::zeros(1,4,CV_64F);

        vector<Mat> r;
        vector<Mat> t;
        calibrateCamera(objectPoints,points,image_size_,m,d, r, t);

        // Build the undistort map that we will use for all subsequent frames
	      Mat mapx;
	      Mat mapy;
        Mat R(3,3,CV_64F);
        setIdentity(R);
        initUndistortRectifyMap(m,d,R,m,image_size_,CV_32FC1,mapx,mapy);

        cvDestroyWindow("Capture result");

        namedWindow( "Undistorted image" );
        cvMoveWindow("Undistorted image", 0, 0);
        namedWindow( "Raw image" );
        cvMoveWindow("Raw image", image_size_.width, 0);

        cout << "To repeat the calibration process press 'r'" << endl;
        cout << "Press enter to continue" << endl;
	      while( 1 )
        {
		        ros::spinOnce();
            //Pillar imagen left y guardarla en image
			      if(cam==LEFT)
            {
                left_mutex_.lock();
                if(mostrar_imagen_left_)
                {
                  	if(!left.empty())
                    {
                        imshow( "Raw image", left ); // Show raw image
                        remap(left,image,mapx,mapy,INTER_LINEAR);
                        imshow( "Undistorted image", image ); // Show corrected image
                  	}
                    mostrar_imagen_left_=false;
                }
                left_mutex_.unlock();
            }
            else
            {
                right_mutex_.lock();
                if(mostrar_imagen_right_)
                {
                   	if(!right.empty())
                    {
                     		imshow( "Raw image", right ); // Show raw image
		                    remap(right,image,mapx,mapy,INTER_LINEAR);
		                    imshow( "Undistorted image", image ); // Show corrected image
                   	}
                    mostrar_imagen_right_=false;
                }
                right_mutex_.unlock();
            }


            c=waitKey(15);
            c=c&0xFF;
            if(c=='\n'||c=='r'||c=='n')
            {
                break;
            }
	      }
        cvDestroyWindow("Undistorted image");
        cvDestroyWindow("Raw image");
    }

    return 0;
}

//Muestra por pantalla las imágenes de las 2 camaras
//cuando se pulse una tecla (la que sea) guarda la captura en las matrices color[0] y color[1] y retorna
int StereoVision::clickToCapture(Mat color[]) {
    namedWindow( "Left" );
    cvMoveWindow("Left", 0, 0);
    namedWindow( "Right" );
    cvMoveWindow("Right", image_size_.width, 0);


    while(1) {
        ros::spinOnce();
        //Pillamos las imagenes derecha e izquierda y las guardamos en color[0] y color[1]
        left_mutex_.lock();
        if(mostrar_imagen_left_==true)
        {
       	  if(left.cols>0&&left.rows>0)
          {
         		left.copyTo(color[0]);
          	imshow( "Left", color[0] );
          }
          mostrar_imagen_left_=false;
        }
        left_mutex_.unlock();
        right_mutex_.lock();
        if(mostrar_imagen_right_==true)
        {
          if(right.cols>0&&right.rows>0)
          {
        	  right.copyTo(color[1]);
            imshow( "Right", color[1] );
          }
          mostrar_imagen_right_=false;
        }
        right_mutex_.unlock();

        int c=waitKey(15);
        if(c!=-1)
        {
            break;
        }

    }
    cvDestroyWindow("Left");
    cvDestroyWindow("Right");
}

//Muestra por pantalla la camara que se le pasa como parámetro
//cuando se pulse una tecla (la que sea) guarda la captura en las matriz color y retorna
int StereoVision::clickToCaptureSingle(int c,Mat& color) {

    cvNamedWindow( "Camara" );
    cvMoveWindow("Camara", 0, 0);

    while(1)
    {
        ros::spinOnce();
        if(c==LEFT)
        {
		        left_mutex_.lock();
		        if(mostrar_imagen_left_==true)
            {
			          if(left.cols>0&&left.rows>0)
                {
				            left.copyTo(color);
				            imshow( "Camara", color );
			          }
			          mostrar_imagen_left_=false;
		        }
		        left_mutex_.unlock();
		    }
        else
        {
            right_mutex_.lock();
		        if(mostrar_imagen_right_==true)
            {
			          if(right.cols>0&&right.rows>0)
                {
			            right.copyTo(color);
			            imshow( "Camara", color );
			          }
			          mostrar_imagen_right_=false;
		        }
		        right_mutex_.unlock();
        }
        int c=waitKey(15);
        if(c!=-1)
        {
            break;
        }

    }

    cvDestroyWindow("Camara");
}

//Muestra por pantalla las 2 camaras
//Retorna al pulsar 'r' o enter
//Devuelve el valor de la tecla pulsada
int StereoVision::mostrarCamaras() {
    //Mat image(imageSize,CV_8UC3);
    //Mat image;
    namedWindow("Left");
    cvMoveWindow("Left", 0, 0);
    namedWindow("Right");
    cvMoveWindow("Right", image_size_.width, 0);
    int c=0;
    while( 1 )
    {
        ros::spinOnce();
        left_mutex_.lock();
        if(mostrar_imagen_left_) {
      	if(left.cols>0&&left.rows>0) {
      		//Mat temp;
      		//left.copyTo(temp);
      		rectificarImagen(1, left);
      		imshow( "Left", left );
      	}
          mostrar_imagen_left_=false;
        }
        left_mutex_.unlock();
        right_mutex_.lock();
        if(mostrar_imagen_right_) {
         	if(right.cols>0&&right.rows>0) {
         		//Mat temp;
         		//right.copyTo(temp);
         	  rectificarImagen(2, right);
         	  imshow( "Right", right );
         	}
          mostrar_imagen_right_=false;
        }
        right_mutex_.unlock();

        c=waitKey(15);
        c=c&0xFF;
        if(c=='\n'||c=='r'||c=='n')
            break;
    }
    cvDestroyWindow("Left");
    cvDestroyWindow("Right");
    return c;
}

//Muestra por pantalla la camara que se le pasa como parámetro
//Retorna al pulsar 'r' o enter
//Devuelve el valor de la tecla pulsada
int StereoVision::mostrarCamara(int cam) {
    int c=0;
    Mat image(image_size_,CV_8UC3);
    namedWindow("Camara");
    cvMoveWindow("Camara", 0, 0);
    while( 1 )
    {
        ros::spinOnce();
        if(cam==LEFT) {
          left_mutex_.lock();
          if(mostrar_imagen_left_) {
        	if(!left.empty()) {
        		rectificarImagen(1, left);
        		imshow( "Camara", left );
        	}
            mostrar_imagen_left_=false;
          }
          left_mutex_.unlock();
        }
        else {
          right_mutex_.lock();
          if(mostrar_imagen_right_) {
           	if(!right.empty()) {
           	  rectificarImagen(2, right);
           	  imshow( "Camara", right );
           	}
            mostrar_imagen_right_=false;
          }
          right_mutex_.unlock();
        }

        c=waitKey(15);
        c=c&0xFF;
        if(c=='\n'||c=='r')
            break;
    }
    cvDestroyWindow("Camara");
    return c;
}

void StereoVision::leftImageCallback(const sensor_msgs::ImageConstPtr& msg_ptr, const sensor_msgs::CameraInfoConstPtr& info_ptr) {
	left_mutex_.lock();
  try
  {
	  Mat temp(bridge_.imgMsgToCv(msg_ptr, "bgr8"));
    temp.copyTo(left);
    mostrar_imagen_left_=true;
    left_image_size_=left.size();
    left_mutex_.unlock();

  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("error");
    mostrar_imagen_left_=false;
	  left_mutex_.unlock();
  }
}

void StereoVision::rightImageCallback(const sensor_msgs::ImageConstPtr& msg_ptr, const sensor_msgs::CameraInfoConstPtr& info_ptr) {
  right_mutex_.lock();
  try
  {
	  Mat temp(bridge_.imgMsgToCv(msg_ptr, "bgr8"));
    temp.copyTo(right);
    mostrar_imagen_right_=true;
    right_image_size_=right.size();
    right_mutex_.unlock();
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("error");
    mostrar_imagen_right_=false;
    right_mutex_.unlock();
  }
}
