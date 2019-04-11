//
// Created by gintoki on 19-4-2.
//
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;
#ifndef LIDAR_RECONSTRUCTION_CAMERA_READ_LIDAR_DATA_H
#define LIDAR_RECONSTRUCTION_CAMERA_READ_LIDAR_DATA_H
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr read_lidar_data(const char* file_name);
#endif //LIDAR_RECONSTRUCTION_CAMERA_READ_LIDAR_DATA_H
