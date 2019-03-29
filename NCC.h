//
// Created by gintoki on 19-3-28.
//
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;
using namespace cv;
#ifndef LIDAR_RECONSTRUCTION_CAMERA_NCC_H
#define LIDAR_RECONSTRUCTION_CAMERA_NCC_H
double NCC( const Mat& ref, const Mat& curr, const Vector2d& pt_ref, const Vector2d& pt_curr );
#endif //LIDAR_RECONSTRUCTION_CAMERA_NCC_H
