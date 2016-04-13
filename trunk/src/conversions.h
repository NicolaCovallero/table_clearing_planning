#ifndef _CONVERSIONS_H
#define _CONVERSIONS_H

#include <pcl/common/eigen.h>

namespace conv
{

template <typename PointT>
Eigen::Vector3f  pointT2vector3f(PointT point)
{
  Eigen::Vector3f vec;
  vec[0] = point.x;
  vec[1] = point.y;
  vec[2] = point.z;
  return vec;
}

/**
 * @brief Return the Eigen vector in pcl format, only the XYZ fields are set.
 * @details [long description]
 * 
 * @param vec [description]
 * @return [description]
 */
template <typename PointT>
PointT vector3f2pointT(Eigen::Vector3f vec)
{
  PointT point;
  point.x = vec[0];
  point.y = vec[1];
  point.z = vec[2];
  return point;
}


}
#endif
