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
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_calibration");

  ros::NodeHandle nh("~");
  ros::names::remap("stereo");

  int board_w; // Board width in squares  nº de casillas-1 (nº de esquinas)
  int board_h; // Board height    nº de casillas-1 (nº de esquinas)
  int n_boards; // Number of boards. Se obtiene este numero de imagenes para calcular los parámetros
  double square_dimension; //Tamaño en m de las casillas del patron de calibrado. Si no se pone se supone que es 1
  nh.param("board_width", board_w, 6);
  nh.param("board_height", board_h, 8);
  nh.param("boards_number", n_boards, 9);
  nh.param("square_dimension", square_dimension, 1.0);

  system("mkdir -p ~/.ros/camera_info");

  StereoVision stereoV;
  stereoV.calibrateStereo(board_w,board_h,n_boards,(float)square_dimension);
  
  return 0;
}
