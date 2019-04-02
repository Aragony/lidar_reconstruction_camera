//
// Created by gintoki on 19-4-2.
//

#include "read_lidar_data.h"
PointCloud::Ptr read_lidar_data(const char* file_name){

  //分配4M缓存（只有 ～130*4*4 KB是必需的）
  int32_t num= 1000000;
  float *data=(float*)malloc(num*sizeof(float));

  //指针
  float *px=data+0;
  float *py=data+1;
  float *pz=data+2;
  float *pr=data+3;//载入点云
  FILE *stream;
  stream=fopen(file_name,"rb");
  num=fread(data, sizeof(float),num,stream)/4;
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloud;
  PointCloud::Ptr pointcloud(new PointCloud);

  for(int32_t i=0;i<num;i++){
    PointT p;
    p.x=*px;
    p.y=*py;
    p.z=*pz;
    p.b = 0;
    p.g = 255;
    p.r = 255;
    px+=4;
    py+=4;
    pz+=4;
    pointcloud->points.push_back(p);
  }
  fclose(stream);

  return pointcloud;
}