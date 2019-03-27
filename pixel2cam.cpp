//
// Created by gintoki on 19-3-27.
//

#include "pixel2cam.h"
Point2f pixel2cam ( const Point2d& p, const Mat& K )
{
  return Point2f
      (
          ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0),
          ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1)
      );
}