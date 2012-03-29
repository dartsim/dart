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

#ifndef FCL_TAYLOR_MODEL_H
#define FCL_TAYLOR_MODEL_H

#include "fcl/interval.h"

namespace fcl
{

/** \brief TaylorModel implements a third order Taylor model, i.e., a cubic approximation of a function
 * over a time interval, with an interval remainder.
 * All the operations on two Taylor models assume their time intervals are the same.
 */
struct TaylorModel
{
  /** \brief Coefficients of the cubic polynomial approximation */
  BVH_REAL coeffs_[4];

  /** \brief interval remainder */
  Interval r_;

  void setTimeInterval(BVH_REAL l, BVH_REAL r);

  TaylorModel();
  TaylorModel(BVH_REAL coeff);
  TaylorModel(BVH_REAL coeffs[3], const Interval& r);
  TaylorModel(BVH_REAL c0, BVH_REAL c1, BVH_REAL c2, BVH_REAL c3, const Interval& r);

  TaylorModel operator + (const TaylorModel& other) const;
  TaylorModel operator - (const TaylorModel& other) const;
  TaylorModel& operator += (const TaylorModel& other);
  TaylorModel& operator -= (const TaylorModel& other);
  TaylorModel operator * (const TaylorModel& other) const;
  TaylorModel operator * (BVH_REAL d) const;
  TaylorModel operator - () const;

  void print() const;

  Interval getBound() const;
  Interval getBound(BVH_REAL l, BVH_REAL r) const;

  Interval getTightBound() const;
  Interval getTightBound(BVH_REAL l, BVH_REAL r) const;

  Interval getBound(BVH_REAL t) const;

  void setZero();

  /** \brief time interval and different powers */
  Interval t_; // [t1, t2]
  Interval t2_; // [t1, t2]^2
  Interval t3_; // [t1, t2]^3
  Interval t4_; // [t1, t2]^4
  Interval t5_; // [t1, t2]^5
  Interval t6_; // [t1, t2]^6

  static const BVH_REAL PI_;
};

void generateTaylorModelForCosFunc(TaylorModel& tm, BVH_REAL w, BVH_REAL q0);
void generateTaylorModelForSinFunc(TaylorModel& tm, BVH_REAL w, BVH_REAL q0);
void generateTaylorModelForLinearFunc(TaylorModel& tm, BVH_REAL p, BVH_REAL v);

}

#endif
