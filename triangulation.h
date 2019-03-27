//
// Created by gintoki on 19-3-27.
//
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace cv;


#ifndef LIDAR_RECONSTRUCTION_CAMERA_TRIANGULATION_H
#define LIDAR_RECONSTRUCTION_CAMERA_TRIANGULATION_H
void triangulation (
    const vector<Point3d>& keypoint_1,
    const vector<Point3d>& keypoint_2,
    const Mat& R, const Mat& t,
    vector<Point3d>& points
);

void triangulation (
    const vector< KeyPoint >& keypoint_1,
    const vector< KeyPoint >& keypoint_2,
    const std::vector< DMatch >& matches,
    const Mat& R, const Mat& t,
    vector< Point3d >& points );
#endif //LIDAR_RECONSTRUCTION_CAMERA_TRIANGULATION_H
