//
// Created by gintoki on 19-3-27.
//
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace cv;


#ifndef LIDAR_RECONSTRUCTION_CAMERA_FIND_FEATURE_MATCHES_H
#define LIDAR_RECONSTRUCTION_CAMERA_FIND_FEATURE_MATCHES_H
void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches );
#endif //LIDAR_RECONSTRUCTION_CAMERA_FIND_FEATURE_MATCHES_H
