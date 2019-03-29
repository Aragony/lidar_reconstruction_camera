//
// Created by gintoki on 19-3-28.
//

#include "epipolarSearch.h"
const int boarder = 20; 	// 边缘宽度
const int width = 1242;  	// 宽度
const int height = 375;  	// 高度
const double fx = 9.842439e+02;	// 相机内参
const double fy = 9.808141e+02;
const double cx = 6.900000e+02;
const double cy = 2.331966e+02;

// 像素到相机坐标系
inline Vector3d px2cam ( const Vector2d px ) {
  return Vector3d (
      (px(0,0) - cx)/fx,
      (px(1,0) - cy)/fy,
      1
  );
}

// 相机坐标系到像素
inline Vector2d cam2px ( const Vector3d p_cam ) {
  return Vector2d (
      p_cam(0,0)*fx/p_cam(2,0) + cx,
      p_cam(1,0)*fy/p_cam(2,0) + cy
  );
}

// 检测一个点是否在图像边框内
inline bool inside( const Vector2d& pt ) {
  return pt(0,0) >= boarder && pt(1,0)>=boarder
      && pt(0,0)+boarder<width && pt(1,0)+boarder<=height;
}

bool epipolarSearch(
    const Mat& ref, const Mat& curr,
    const SE3& T_C_R, const Vector2d& pt_ref,
    const double& depth_mu, const double& depth_cov,
    Vector2d& pt_curr )
{
  Vector3d f_ref = px2cam( pt_ref );
  f_ref.normalize();
  Vector3d P_ref = f_ref*depth_mu;	// 参考帧的 P 向量

  Vector2d px_mean_curr = cam2px( T_C_R*P_ref ); // 按深度均值投影的像素
  double d_min = depth_mu-3*depth_cov, d_max = depth_mu+3*depth_cov;
  if ( d_min<0.1 ) d_min = 0.1;
  Vector2d px_min_curr = cam2px( T_C_R*(f_ref*d_min) );	// 按最小深度投影的像素
  Vector2d px_max_curr = cam2px( T_C_R*(f_ref*d_max) );	// 按最大深度投影的像素

  Vector2d epipolar_line = px_max_curr - px_min_curr;	// 极线（线段形式）
  Vector2d epipolar_direction = epipolar_line;		// 极线方向
  epipolar_direction.normalize();
  double half_length = 0.5*epipolar_line.norm();	// 极线线段的半长度
  if ( half_length>100 ) half_length = 100;   // 我们不希望搜索太多东西

  // 取消此句注释以显示极线（线段）
  // showEpipolarLine( ref, curr, pt_ref, px_min_curr, px_max_curr );

  // 在极线上搜索，以深度均值点为中心，左右各取半长度
  double best_ncc = -1.0;
  Vector2d best_px_curr;
  for ( double l=-half_length; l<=half_length; l+=0.7 )  // l+=sqrt(2)
  {
    Vector2d px_curr = px_mean_curr + l*epipolar_direction;  // 待匹配点
    if ( !inside(px_curr) )
      continue;
    // 计算待匹配点与参考帧的 NCC
    double ncc = NCC( ref, curr, pt_ref, px_curr );
    if ( ncc>best_ncc )
    {
      best_ncc = ncc;
      best_px_curr = px_curr;
    }
  }
  if ( best_ncc < 0.85f )      // 只相信 NCC 很高的匹配
    return false;
  pt_curr = best_px_curr;
  return true;
}