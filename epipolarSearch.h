//
// Created by gintoki on 19-3-28.
//

// for sophus
#include <sophus/se3.h>
using Sophus::SE3;

// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

#include "NCC.h"
#ifndef LIDAR_RECONSTRUCTION_CAMERA_EPIPOLARSEARCH_H
#define LIDAR_RECONSTRUCTION_CAMERA_EPIPOLARSEARCH_H
bool epipolarSearch(
    const Mat& ref, const Mat& curr,
    const SE3& T_C_R, const Vector2d& pt_ref,
    const double& depth_mu, const double& depth_cov,
    Vector2d& pt_curr );
#endif //LIDAR_RECONSTRUCTION_CAMERA_EPIPOLARSEARCH_H
