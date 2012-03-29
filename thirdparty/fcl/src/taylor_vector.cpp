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

#include "fcl/taylor_vector.h"

namespace fcl
{

TVector3::TVector3() {}
TVector3::TVector3(TaylorModel v[3])
{
  i_[0] = v[0];
  i_[1] = v[1];
  i_[2] = v[2];
}

TVector3::TVector3(const TaylorModel& v1, const TaylorModel& v2, const TaylorModel& v3)
{
  i_[0] = v1;
  i_[1] = v2;
  i_[2] = v3;
}

TVector3::TVector3(const Vec3f& v)
{
  i_[0] = TaylorModel(v[0]);
  i_[1] = TaylorModel(v[1]);
  i_[2] = TaylorModel(v[2]);
}

void TVector3::setZero()
{
  i_[0] = TaylorModel(0);
  i_[1] = TaylorModel(0);
  i_[2] = TaylorModel(0);
}

TVector3 TVector3::operator + (const TVector3& other) const
{
  TaylorModel res[3];
  res[0] = i_[0] + other.i_[0];
  res[1] = i_[1] + other.i_[1];
  res[2] = i_[2] + other.i_[2];

  return TVector3(res);
}

TVector3 TVector3::operator + (BVH_REAL d) const
{
  TaylorModel res[3];
  res[0] = i_[0];
  res[1] = i_[1];
  res[2] = i_[2];
  res[0].coeffs_[0] += d;
  return TVector3(res);
}

TVector3 TVector3::operator - (const TVector3& other) const
{
  TaylorModel res[3];
  res[0] = i_[0] - other.i_[0];
  res[1] = i_[1] - other.i_[1];
  res[2] = i_[2] - other.i_[2];

  return TVector3(res);
}

TVector3& TVector3::operator += (const TVector3& other)
{
  i_[0] += other.i_[0];
  i_[1] += other.i_[1];
  i_[2] += other.i_[2];
  return *this;
}

TVector3& TVector3::operator -= (const TVector3& other)
{
  i_[0] -= other.i_[0];
  i_[1] -= other.i_[1];
  i_[2] -= other.i_[2];
  return *this;
}

TVector3& TVector3::operator = (const Vec3f& other)
{
  i_[0] = TaylorModel(other[0]);
  i_[1] = TaylorModel(other[1]);
  i_[2] = TaylorModel(other[2]);
  return *this;
}

const TaylorModel& TVector3::operator [] (size_t i) const
{
  return i_[i];
}

TaylorModel& TVector3::operator [] (size_t i)
{
  return i_[i];
}

TaylorModel TVector3::dot(const TVector3& other) const
{
  return i_[0] * other.i_[0] + i_[1] * other.i_[1] + i_[2] * other.i_[2];
}

TVector3 TVector3::cross(const TVector3& other) const
{
  TaylorModel res[3];
  res[0] = i_[1] * other.i_[2] - i_[2] * other.i_[1];
  res[1] = i_[2] * other.i_[0] - i_[0] * other.i_[2];
  res[2] = i_[0] * other.i_[1] - i_[1] * other.i_[0];

  return TVector3(res);
}

BVH_REAL TVector3::volumn() const
{
  return i_[0].getBound().diameter() * i_[1].getBound().diameter() * i_[2].getBound().diameter();
}

IVector3 TVector3::getBound() const
{
  Interval res[3];
  res[0] = i_[0].getBound();
  res[1] = i_[1].getBound();
  res[2] = i_[2].getBound();

  return IVector3(res);
}

void TVector3::print() const
{
  i_[0].print();
  i_[1].print();
  i_[2].print();
}

IVector3 TVector3::getBound(BVH_REAL t) const
{
  return IVector3(i_[0].getBound(t), i_[1].getBound(t), i_[2].getBound(t));
}

TaylorModel TVector3::squareLength() const
{
  return i_[0] * i_[0] + i_[1] * i_[1] + i_[2] * i_[2];
}

void generateTVector3ForLinearFunc(TVector3& v, const Vec3f& position, const Vec3f& velocity)
{
  generateTaylorModelForLinearFunc(v.i_[0], position[0], velocity[0]);
  generateTaylorModelForLinearFunc(v.i_[1], position[1], velocity[1]);
  generateTaylorModelForLinearFunc(v.i_[2], position[2], velocity[2]);
}





}
