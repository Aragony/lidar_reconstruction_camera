//
// Created by gintoki on 19-3-28.
//

#include "pcl_cloud.h"
void pcl_cloud(vector<Point3d>& points){
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloud;

  PointCloud::Ptr pointcloud(new PointCloud);

  for (auto &i:points) {
    PointT p;
    p.x = i.x;
    p.y = i.y;
    p.z = i.z;
    p.b = 0;
    p.g = 0;
    p.r = 255;
    pointcloud->points.push_back(p);
  }

  pointcloud->is_dense = true;
  pcl::io::savePCDFileBinary("pointcloud.pcd", *pointcloud);
}