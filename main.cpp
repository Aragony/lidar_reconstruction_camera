#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>



#include "find_feature_matches.h"
#include "pose_estimation_2d2d .h"
#include "triangulation.h"
#include "pcl_cloud.h"

using namespace std;
using namespace cv;

int main() {
  //相机内参
  Mat K = ( Mat_<double> ( 3,3 ) << 9.842439e+02, 0, 6.900000e+02, 0, 9.808141e+02, 2.331966e+02, 0, 0, 1 );
  //读取图像
  Mat img_1 = imread("0000000000.png", CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread("0000000001.png", CV_LOAD_IMAGE_COLOR);

  vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;
  find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);

  //估计两张图像间运动
  Mat R=Mat::ones(3,3,CV_64FC1);
  Mat t=Mat::ones(3,1,CV_64FC1);
  pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

  vector<Point3d> lp, rp;
  for (int i = 0; i < img_1.rows; i++) {
    for (int j = 0; j < img_1.cols; j++) {
      Mat A = Mat::ones(3, 1, CV_64FC1);
      Mat B = Mat::ones(3, 1, CV_64FC1);
      A.at<float>(0, 0) = i;
      A.at<float>(1, 0) = j;
      A.at<float>(2, 0) = 1;
      B = K*R*K.inv()*A + K*t;
      B = B/B.at<float>(2, 0);
      if (B.at<float>(0, 0) > 0 && B.at<float>(0, 0) < 375 && B.at<float>(1, 0) > 0 && B.at<float>(1, 0) < 1247) {
        Point3d pl(A.at<float>(0, 0),
                   A.at<float>(1, 0),
                   A.at<float>(2, 0));
        Point3d pr(B.at<float>(0, 0),
                   B.at<float>(1, 0),
                   B.at<float>(2, 0));
        lp.push_back(pl);
        rp.push_back(pr);
      }
    }
  }

  //三角化
  //-- 三角化
  vector<Point3d> points;
  //triangulation(lp, rp,  R, t, points);//稠密版
  triangulation( keypoints_1, keypoints_2, matches, R, t, points );//稀疏版

  pcl_cloud(points);

}