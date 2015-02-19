#ifndef OSGDART_UTILS_H
#define OSGDART_UTILS_H

#include <Eigen/Geometry>

#include <osg/Matrix>

template<typename Scalar>
osg::Matrix eigToOsg(const Eigen::Transform<Scalar,3,Eigen::Isometry>& tf)
{
  // TODO(MXG): See if this can be made more efficient. osg::Matrix is
  // automatically initialized to Identity, which is a waste.
  osg::Matrix output;
  for(size_t i=0; i<4; ++i)
    for(size_t j=0; j<4; ++j)
      output(i,j) = tf(j,i);
  return output;
}

//==============================================================================
template<typename Scalar>
osg::Vec3d eigToOsg(const Eigen::Matrix<Scalar, 3, 1>& vec)
{
  return osg::Vec3d(vec[0], vec[1], vec[2]);
}

#endif // OSGDART_UTILS_H
