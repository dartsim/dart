/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 06/12/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#ifndef MATH_EIGEN_HELPER_H
#define MATH_EIGEN_HELPER_H

#include <Eigen/Dense>
#include <Eigen/StdVector>

#define EIGEN_V_VEC2D std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
#define EIGEN_V_VEC3D std::vector< Eigen::Vector3d >
#define EIGEN_V_VEC4D std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >

#define EIGEN_V_MAT2D std::vector< Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> >
#define EIGEN_V_MAT3D std::vector< Eigen::Matrix3d >
#define EIGEN_V_MAT4D std::vector< Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >

#define EIGEN_VV_VEC2D std::vector< std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > >
#define EIGEN_VV_VEC3D std::vector< std::vector< Eigen::Vector3d > >
#define EIGEN_VV_VEC4D std::vector< std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > >

#define EIGEN_VV_MAT2D std::vector< std::vector< Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> > >
#define EIGEN_VV_MAT3D std::vector< std::vector< Eigen::Matrix3d > >
#define EIGEN_VV_MAT4D std::vector< std::vector< Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > >

#define EIGEN_V_QUATD std::vector< Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond > >
#define EIGEN_VV_QUATD std::vector< std::vector< Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > >

#endif // #ifndef UTILS_EIGEN_HELPER_H

