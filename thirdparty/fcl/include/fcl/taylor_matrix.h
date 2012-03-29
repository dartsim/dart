/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jia Pan */

#ifndef FCL_TAYLOR_MATRIX_H
#define FCL_TAYLOR_MATRIX_H


#include "fcl/matrix_3f.h"
#include "fcl/taylor_vector.h"
#include "fcl/interval_matrix.h"

namespace fcl
{

struct TMatrix3
{
  TaylorModel i_[3][3];

  TMatrix3();
  TMatrix3(TaylorModel m[3][3]);
  TMatrix3(const Matrix3f& m);

  TVector3 getColumn(size_t i) const;
  TVector3 getRow(size_t i) const;

  const TaylorModel& operator () (size_t i, size_t j) const;
  TaylorModel& operator () (size_t i, size_t j);

  TVector3 operator * (const Vec3f& v) const;
  TVector3 operator * (const TVector3& v) const;
  TMatrix3 operator * (const Matrix3f& m) const;
  TMatrix3 operator * (const TMatrix3& m) const;
  TMatrix3 operator * (const TaylorModel& d) const;
  TMatrix3 operator + (const TMatrix3& m) const;
  TMatrix3& operator += (const TMatrix3& m);

  IMatrix3 getBound() const;
  void print() const;
  void setIdentity();
  void setZero();
  BVH_REAL diameter() const;
};

}

#endif
