/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef UTILS_EIGEN_HELPER_H
#define UTILS_EIGEN_HELPER_H

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

