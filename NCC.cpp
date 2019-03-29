//
// Created by gintoki on 19-3-28.
//

#include "NCC.h"
using namespace std;
const int ncc_window_size = 2;	// NCC 取的窗口半宽度
const int ncc_area = (2*ncc_window_size+1)*(2*ncc_window_size+1); // NCC窗口面积
// 双线性灰度插值
inline double getBilinearInterpolatedValue( const Mat& img, const Vector2d& pt ) {
  uchar* d = & img.data[ int(pt(1,0))*img.step+int(pt(0,0)) ];
  double xx = pt(0,0) - floor(pt(0,0));
  double yy = pt(1,0) - floor(pt(1,0));
  return  (( 1-xx ) * ( 1-yy ) * double(d[0]) +
      xx* ( 1-yy ) * double(d[1]) +
      ( 1-xx ) *yy* double(d[img.step]) +
      xx*yy*double(d[img.step+1]))/255.0;
}
// 计算 NCC 评分
double NCC (
    const Mat& ref, const Mat& curr,
    const Vector2d& pt_ref, const Vector2d& pt_curr
)
{
  // 零均值-归一化互相关
  // 先算均值
  double mean_ref = 0, mean_curr = 0;
  vector<double> values_ref, values_curr; // 参考帧和当前帧的均值
  for ( int x=-ncc_window_size; x<=ncc_window_size; x++ )
    for ( int y=-ncc_window_size; y<=ncc_window_size; y++ )
    {
      double value_ref = double(ref.ptr<uchar>( int(y+pt_ref(1,0)) )[ int(x+pt_ref(0,0)) ])/255.0;
      mean_ref += value_ref;

      double value_curr = getBilinearInterpolatedValue( curr, pt_curr+Vector2d(x,y) );
      mean_curr += value_curr;

      values_ref.push_back(value_ref);
      values_curr.push_back(value_curr);
    }

  mean_ref /= ncc_area;
  mean_curr /= ncc_area;

  // 计算 Zero mean NCC
  double numerator = 0, demoniator1 = 0, demoniator2 = 0;
  for ( int i=0; i<values_ref.size(); i++ )
  {
    double n = (values_ref[i]-mean_ref) * (values_curr[i]-mean_curr);
    numerator += n;
    demoniator1 += (values_ref[i]-mean_ref)*(values_ref[i]-mean_ref);
    demoniator2 += (values_curr[i]-mean_curr)*(values_curr[i]-mean_curr);
  }
  return numerator / sqrt( demoniator1*demoniator2+1e-10 );   // 防止分母出现零
}

