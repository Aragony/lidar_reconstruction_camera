//
// Created by gintoki on 19-3-27.
//

#include <opencv/cv.hpp>
#include "triangulation.h"
#include "pixel2cam.h"
void triangulation(
    const vector<Point3d> &keypoint_1,
    const vector<Point3d> &keypoint_2,
    const Mat &R, const Mat &t,
    vector<Point3d> &points) {
  Mat T1 = (Mat_<float>(3, 4) <<
                              1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0);
  Mat T2 = (Mat_<float>(3, 4) <<
                              R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
      R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
      R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
  );

  Mat K = (Mat_<double>(3, 3)
      << 9.597910e+02, 0.000000e+00, 6.960217e+02, 0.000000e+00, 9.569251e+02, 2.241806e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00);
  vector<Point2d> pts_1, pts_2;
  for (int i = 0; i < keypoint_1.size(); ++i) {
    // 将像素坐标转换至相机坐标
    pts_1.push_back(pixel2cam(Point2d(keypoint_1[i].x, keypoint_1[i].y), K));
    pts_2.push_back(pixel2cam(Point2d(keypoint_2[i].x, keypoint_2[i].y), K));
  }

  Mat pts_4d;
  cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

  // 转换成非齐次坐标
  for (int i = 0; i < pts_4d.cols; i++) {
    Mat x = pts_4d.col(i);
    x /= x.at<float>(3, 0); // 归一化
    Point3d p(
        x.at<float>(0, 0),
        x.at<float>(1, 0),
        x.at<float>(2, 0)
    );
    points.push_back(p);
  }
}

void triangulation(
    const vector<KeyPoint> &keypoint_1,
    const vector<KeyPoint> &keypoint_2,
    const std::vector<DMatch> &matches,
    const Mat &R, const Mat &t,
    vector<Point3d> &points) {
  Mat T1 = (Mat_<float>(3, 4) <<
                              1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0);
  Mat T2 = (Mat_<float>(3, 4) <<
                              R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
      R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
      R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
  );

  Mat K = (Mat_<double>(3, 3)
      << 9.597910e+02, 0.000000e+00, 6.960217e+02, 0.000000e+00, 9.569251e+02, 2.241806e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00);
  vector<Point2f> pts_1, pts_2;
  for (DMatch m:matches) {
    // 将像素坐标转换至相机坐标
    pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
    pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
  }

  Mat pts_4d;
  cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

  // 转换成非齐次坐标
  for (int i = 0; i < pts_4d.cols; i++) {
    Mat x = pts_4d.col(i);
    x /= x.at<float>(3, 0); // 归一化
    Point3d p(
        x.at<float>(0, 0),
        x.at<float>(1, 0),
        x.at<float>(2, 0)
    );
    points.push_back(p);
  }
}