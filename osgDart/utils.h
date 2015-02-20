/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *            Pete Vieira <pete.vieira@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OSGDART_UTILS_H
#define OSGDART_UTILS_H

#include <Eigen/Geometry>

#include <osg/Matrix>

template<typename Scalar>
osg::Matrix eigToOsgMatrix(const Eigen::Transform<Scalar,3,Eigen::Isometry>& tf)
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
template<typename Derived>
osg::Matrix eigToOsgMatrix(const Eigen::MatrixBase<Derived>& M)
{
  osg::Matrix output;
  for(size_t i=0; i<4; ++i)
    for(size_t j=0; j<4; ++j)
      output(i,j) = M(j,i);
  return output;
}

//==============================================================================
template<typename Derived>
osg::Vec3d eigToOsgVec3(const Eigen::MatrixBase<Derived>& vec)
{
  return osg::Vec3d(vec[0], vec[1], vec[2]);
}

//==============================================================================
template<typename Derived>
osg::Vec4d eigToOsgVec4(const Eigen::MatrixBase<Derived>& vec)
{
  return osg::Vec4d(vec[0], vec[1], vec[2], vec[3]);
}

#endif // OSGDART_UTILS_H
