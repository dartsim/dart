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

#ifndef COLLISION_CHECKING_CONSERVATIVE_ADVANCEMENT_H
#define COLLISION_CHECKING_CONSERVATIVE_ADVANCEMENT_H

#include <limits>
#include "BVH_defs.h"
#include "BVH_model.h"
#include "BVH_front.h"

/** \brief Main namespace */

namespace collision_checking
{

/** \brief Linear interpolation motion
 * Each Motion is assumed to have constant linear velocity and angular velocity
 */
class InterpMotion
{
  /** \brief Quaternion used locally by InterpMotion */
  struct SimpleQuaternion
  {
    /** \brief Default quaternion is identity rotation */
    SimpleQuaternion()
    {
      data[0] = 1;
      data[1] = 0;
      data[2] = 0;
      data[3] = 0;
    }

    SimpleQuaternion(BVH_REAL a, BVH_REAL b, BVH_REAL c, BVH_REAL d)
    {
      data[0] = a; // w
      data[1] = b; // x
      data[2] = c; // y
      data[3] = d; // z
    }

    /** \brief Matrix to quaternion */
    void fromRotation(const Vec3f R[3]);

    /** \brief Quaternion to matrix */
    void toRotation(Vec3f R[3]) const;

    /** \brief Axis and angle to quaternion */
    void fromAxisAngle(const Vec3f& axis, BVH_REAL angle);

    /** \brief Quaternion to axis and angle */
    void toAxisAngle(Vec3f& axis, BVH_REAL& angle) const;

    /** \brief Dot product between quaternions */
    BVH_REAL dot(const SimpleQuaternion& other) const;

    /** \brief addition */
    SimpleQuaternion operator + (const SimpleQuaternion& other) const;

    /** \brief minus */
    SimpleQuaternion operator - (const SimpleQuaternion& other) const;

    /** \brief multiplication */
    SimpleQuaternion operator * (const SimpleQuaternion& other) const;

    /** \brief division */
    SimpleQuaternion operator - () const;

    /** \brief scalar multiplication */
    SimpleQuaternion operator * (BVH_REAL t) const;

    /** \brief conjugate */
    SimpleQuaternion conj() const;

    /** \brief inverse */
    SimpleQuaternion inverse() const;

    /** \brief rotate a vector */
    Vec3f transform(const Vec3f& v) const;

    BVH_REAL data[4];
  };

  /** \brief Simple transform class used locally by InterpMotion */
  struct SimpleTransform
  {
    /** \brief Rotation matrix and translation vector */
    Vec3f R[3];
    Vec3f T;

    /** \brief Quaternion representation for R */
    SimpleQuaternion q;

    /** \brief Default transform is no movement */
    SimpleTransform()
    {
      R[0][0] = 1; R[1][1] = 1; R[2][2] = 1;
    }

    SimpleTransform(const Vec3f R_[3], const Vec3f& T_)
    {
      for(int i = 0; i < 3; ++i)
        R[i] = R_[i];
      T = T_;

      q.fromRotation(R_);
    }
  };

public:
  /** Default transformations are all identities */
  InterpMotion()
  {
    /** Default angular velocity is zero */
    angular_axis = Vec3f(1, 0, 0);
    angular_vel = 0;

    /** Default linear velocity is zero */
  }

  /** \brief Construct motion from the initial rotation/translation and goal rotation/translation */
  InterpMotion(const Vec3f R1[3], const Vec3f& T1,
               const Vec3f R2[3], const Vec3f& T2)
  {
    t1 = SimpleTransform(R1, T1);
    t2 = SimpleTransform(R2, T2);

    /** Current time is zero, so the transformation is t1 */
    t = t1;

    /** Compute the velocities for the motion */
    computeVelocity();
  }

  /** \brief Construct motion from the initial rotation/translation and goal rotation/translation related to some rotation center
   */
  InterpMotion(const Vec3f R1[3], const Vec3f& T1,
               const Vec3f R2[3], const Vec3f& T2,
               const Vec3f& O)
  {
    t1 = SimpleTransform(R1, T1 - MxV(R1, O));
    t2 = SimpleTransform(R2, T2 - MxV(R2, O));
    t = t1;

    /** Compute the velocities for the motion */
    computeVelocity();
  }


  /** \brief Integrate the motion from 0 to dt
   * We compute the current transformation from zero point instead of from last integrate time, for precision.
   */
  bool integrate(double dt)
  {
    if(dt > 1) dt = 1;

    t.T = t1.T + linear_vel * dt;

    t.q = absoluteRotation(dt);
    t.q.toRotation(t.R);

    return true;
  }

  /** \brief Compute the motion bound for a bounding volume, given the closest direction n between two query objects
   * according to mu < |v * n| + ||w x n||(r + max(||ci*||)) where ||ci*|| = ||ci x w||. w is the angular axis (normalized)
   * and ci are the endpoints of the generator primitives of RSS.
   * Notice that all bv parameters are in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
   */
  BVH_REAL computeMotionBound(const RSS& bv, const Vec3f& n) const;

  /** \brief Compute the motion bound for a triangle, given the closest direction n between two query objects
   * according to mu < |v * | + ||w x n||(max||ci*||) where ||ci*|| = ||ci x w||. w is the angular axis (normalized)
   * and ci are the triangle vertex coordinates.
   * Notice that the triangle is in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
   */
  BVH_REAL computeMotionBound(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& n) const;

  /** \brief Get the rotation and translation in current step */
  void getCurrentTransformation(Vec3f R[3], Vec3f& T) const
  {
    for(int i = 0; i < 3; ++i)
    {
      R[i] = t.R[i];
    }

    T = t.T;
  }

  void computeVelocity()
  {
    linear_vel = t2.T - t1.T;
    SimpleQuaternion deltaq = t2.q * t1.q.inverse();
    deltaq.toAxisAngle(angular_axis, angular_vel);
  }

  SimpleQuaternion deltaRotation(BVH_REAL t) const
  {
    SimpleQuaternion res;
    res.fromAxisAngle(angular_axis, (BVH_REAL)(t * angular_vel));
    return res;
  }

  SimpleQuaternion absoluteRotation(BVH_REAL t) const
  {
    SimpleQuaternion delta_t = deltaRotation(t);
    return delta_t * t1.q;
  }

  /** \brief The transformation at time 0 */
  SimpleTransform t1;

  /** \brief The transformation at time 1 */
  SimpleTransform t2;

  /** \brief The transformation at current time t */
  SimpleTransform t;

  /** \brief Linear velocity */
  Vec3f linear_vel;

  /** \brief Angular speed */
  BVH_REAL angular_vel;

  /** \brief Angular velocity axis */
  Vec3f angular_axis;
};

struct BVH_CAResult
{
  /** \brief Number of BV collision test performed */
  int num_bv_tests;

  /** \brief Number of triangle collision test performed */
  int num_tri_tests;

  /** \brief Query time used */
  BVH_REAL query_time_seconds;

  /** \brief relative and absolute error, default value is 0.01 for both terms */
  BVH_REAL rel_err;
  BVH_REAL abs_err;

  /** \brief distance and points establishing the minimum distance for the models, within the relative and absolute error bounds specified.
   * p1 is in model1's local coordinate system while p2 is in model2's local coordinate system
   */
  BVH_REAL distance;
  Vec3f p1, p2;

  /** \brief Optional parameter controlling the size of a priority queue used to direct the search for closest points. A larger
   * queue can help the algorithm discover  the minimum with fewer steps, but will increase the cost of each step.
   * It is not beneficial to increase qsize if the application has frame-to-frame coherence, i.e., the pairs of models
   * taks small steps between each call,since the 'last tri_id' trick already accelerates this situtation with no overhead.
   * However, a queue size of 100 to 200 has been seen to save time in a motion planning application with "non-coherent" placements of models.
   */
  int qsize;

  /** \brief Remember the nearest neighbor points */
  int last_tri_id1;
  int last_tri_id2;

  /** \brief CA controlling variable: early stop for the early iterations of CA */
  BVH_REAL w;

  /** \brief The time from beginning point */
  BVH_REAL toc;
  BVH_REAL t_err;

  /** \brief The delta_t each step */
  BVH_REAL delta_t;

  /** \brief Motions for the two objects in query */
  InterpMotion motion1;
  InterpMotion motion2;

  BVH_CAResult(BVH_REAL w_ = 1)
  {
    num_bv_tests = 0;
    num_tri_tests = 0;

    query_time_seconds = 0;

    /* default queue size is 2 */
    qsize = 2;

    /* default last_tris are the first triangle on each model */
    last_tri_id1 = 0;
    last_tri_id2 = 0;

    /** default relative and absolute error */
    rel_err = (BVH_REAL)0.01;
    abs_err = (BVH_REAL)0.01;

    distance = std::numeric_limits<BVH_REAL>::max();

    delta_t = 1;
    toc = 0;
    t_err = (BVH_REAL)0.001;

    w = w_;
  }

  ~BVH_CAResult() {}

  /** \brief Reset the distance query statistics */
  void resetRecord()
  {
    num_bv_tests = 0;
    num_tri_tests = 0;

    delta_t = 1;
    toc = 0;
  }
};



/** \brief Recursive conservative advancement kernel between two RSS trees */
void conservativeAdvancementRecurse(BVNode<RSS>* tree1, BVNode<RSS>* tree2,
                                    const Vec3f R[3], const Vec3f& T,
                                    int b1, int b2,
                                    Vec3f* vertices1, Vec3f* vertices2,
                                    Triangle* tri_indices1, Triangle* tri_indices2,
                                    BVH_CAResult* res, BVHFrontList* front_list = NULL);


/** \brief Continuous collision detection query between two RSS models based on conservative advancement. */
void continuousCollide_CA(const BVHModel<RSS>& model1, const Vec3f R1_1[3], const Vec3f& T1_1, const Vec3f R1_2[3], const Vec3f& T1_2,
                          const BVHModel<RSS>& model2, const Vec3f R2_1[3], const Vec3f& T2_1, const Vec3f R2_2[3], const Vec3f& T2_2,
                          BVH_CAResult* res, BVHFrontList* front_list = NULL);


}

#endif
