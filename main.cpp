#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "find_feature_matches.h"
#include "pose_estimation_2d2d .h"
#include "triangulation.h"
#include "pcl_cloud.h"
#include "epipolarSearch.h"
#include "read_lidar_data.h"

using namespace std;
using namespace cv;

int main() {
  const int boarder = 20;    // 边缘宽度
  const int width = 1242;    // 宽度
  const int height = 375;    // 高度
  const double fx = 9.597910e+02;    // 相机内参
  const double fy = 9.569251e+02;
  const double cx = 6.960217e+02;
  const double cy = 2.241806e+02;

  const double min_cov = 0.1;    // 收敛判定：最小方差
  const double max_cov = 10;    // 发散判定：最大方差

  //相机内参
  Mat K = (Mat_<double>(3, 3)
      << 9.597910e+02, 0.000000e+00, 6.960217e+02, 0.000000e+00, 9.569251e+02, 2.241806e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00);
  //读取图像
  Mat img_1 = imread("0000000000.png", CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread("0000000001.png", CV_LOAD_IMAGE_COLOR);

  vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;
  find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);

  //估计两张图像间运动
  Mat R = Mat::ones(3, 3, CV_64FC1);
  Mat t = Mat::ones(3, 1, CV_64FC1);
  pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);
  //std::cout<<R<<std::endl<<t<<std::endl;

  //0328极线搜索
  Mat &ref = img_1;
  Mat &curr = img_2;
  double init_depth = 3.0;    // 深度初始值
  double init_cov2 = 3.0;    // 方差初始值
  Mat depth(height, width, CV_64F, init_depth);             // 深度图
  Mat depth_cov(height, width, CV_64F, init_cov2);          // 深度图方差
  vector<Point3d> lp;
  vector<Point3d> rp;

  //Eigen::Vector3d t2(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0));
  //Eigen::Matrix3d R2;
  //R2 << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
 //     R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
  //    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
  //std::cout<<R2<<std::endl<<t2<<std::endl;

/*
//#pragma omp parallel for
  for (int x = boarder; x < width - boarder; x++) {
//#pragma omp parallel for
    for (int y = boarder; y < height - boarder; y++) {
      // 遍历每个像素
      if (depth_cov.ptr<double>(y)[x] < min_cov || depth_cov.ptr<double>(y)[x] > max_cov) // 深度已收敛或发散
        continue;
      // 在极线上搜索 (x,y) 的匹配
      Vector2d pt_curr;
      bool ret = epipolarSearch(
          ref,
          curr,
          R,
          t,
          Vector2d(x, y),
          depth.ptr<double>(y)[x],
          sqrt(depth_cov.ptr<double>(y)[x]),
          pt_curr
      );

      if (ret==false) {// 匹配失败
        continue;
      } else {
        lp.emplace_back(Point3d(x, y, 1));
        rp.emplace_back(Point3d(pt_curr[0], pt_curr[1], 1));
      }
    }
  }*/

  //三角化
  //-- 三角化
  vector<Point3d> points;
  //triangulation(lp, rp, R, t, points);//稠密版
  triangulation(keypoints_1, keypoints_2, matches, R, t, points);//稀疏版

  pcl_cloud(points);

  ///以下为激光点云处理
  //读取数据
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloud;
  PointCloud::Ptr pointcloud0(new PointCloud);
  PointCloud::Ptr pointcloud1(new PointCloud);

  pointcloud0=read_lidar_data("0000000000.bin");
  pointcloud1=read_lidar_data("0000000001.bin");
  //pointcloud0->is_dense = true;
  //pcl::io::savePCDFileBinary("pointcloud0.pcd", *pointcloud0);


}