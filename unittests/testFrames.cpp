
/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#include <gtest/gtest.h>

#include "dart/dynamics/SimpleFrame.h"
#include "dart/math/Helpers.h"

#include "TestHelpers.h"

using namespace dart;
using namespace dynamics;

void randomize_transforms(std::vector<Eigen::Isometry3d>& tfs)
{
  for(size_t i=0; i<tfs.size(); ++i)
  {
    Eigen::Vector3d v;
    for(size_t i=0; i<3; ++i)
      v[i] = math::random(-100, 100);

    Eigen::Vector3d theta;
    for(size_t i=0; i<3; ++i)
      theta[i] = math::random(-100, 100);

    Eigen::Isometry3d& tf = tfs[i];
    tf.setIdentity();
    tf.translate(v);

    if(theta.norm()>0)
      tf.rotate(Eigen::AngleAxisd(theta.norm(), theta.normalized()));
  }
}

template<int N>
Eigen::Matrix<double,N,1> random_vec()
{
  Eigen::Matrix<double,N,1> v;
  for(size_t i=0; i<N; ++i)
    v[i] = math::random(-100, 100);
  return v;
}

void compute_spatial_velocity(const Eigen::Vector6d& v_parent,
                              const Eigen::Vector6d& v_relative,
                              const Eigen::Isometry3d& tf_rel,
                              Eigen::Vector6d& v_child)
{
  v_child = math::AdInvT(tf_rel, v_parent) + v_relative;
}

void compute_velocity(const Eigen::Vector3d& v_parent,
                      const Eigen::Vector3d& w_parent,
                      const Eigen::Vector3d& v_rel,
                      const Eigen::Vector3d& w_rel,
                      const Eigen::Vector3d& offset,
                      const Eigen::Isometry3d& tf_parent,
                      Eigen::Vector3d& v_child,
                      Eigen::Vector3d& w_child)
{
  const Eigen::Matrix3d& R = tf_parent.rotation();

  v_child = v_parent + R*v_rel + w_parent.cross(R*offset);

  w_child = w_parent + R*w_rel;
}

void compute_acceleration(const Eigen::Vector3d& a_parent,
                          const Eigen::Vector3d& alpha_parent,
                          const Eigen::Vector3d& w_parent,
                          const Eigen::Vector3d& a_rel,
                          const Eigen::Vector3d& alpha_rel,
                          const Eigen::Vector3d& v_rel,
                          const Eigen::Vector3d& offset,
                          const Eigen::Isometry3d& tf_parent,
                          Eigen::Vector3d& a_child,
                          Eigen::Vector3d& alpha_child)
{
  const Eigen::Matrix3d& R = tf_parent.rotation();

  a_child = a_parent + R*a_rel
          + alpha_parent.cross(R*offset)
          + 2*w_parent.cross(R*v_rel)
          + w_parent.cross(w_parent.cross(R*offset));

  alpha_child = alpha_parent + R*alpha_rel + alpha_parent.cross(R*alpha_rel);
}

TEST(FRAMES, FORWARD_KINEMATICS_CHAIN)
{
  std::vector<SimpleFrame*> frames;

  double tolerance = 1e-8;

  SimpleFrame A(Frame::World(), "A");
  frames.push_back(&A);
  SimpleFrame B(&A, "B");
  frames.push_back(&B);
  SimpleFrame C(&B, "C");
  frames.push_back(&C);
  SimpleFrame D(&C, "D");
  frames.push_back(&D);

  // -- Test Position --------------------------------------------------------
  EXPECT_TRUE( equals(D.getTransform().matrix(),
                      Eigen::Isometry3d::Identity().matrix(),
                      tolerance));

  std::vector<Eigen::Isometry3d> tfs;
  tfs.resize(frames.size(), Eigen::Isometry3d::Identity());

  randomize_transforms(tfs);

  for(size_t i=0; i<frames.size(); ++i)
  {
    SimpleFrame* F = frames[i];
    F->setRelativeTransform(tfs[i]);
  }

  for(size_t i=0; i<frames.size(); ++i)
  {
    Frame* F = frames[i];
    Eigen::Isometry3d expectation(Eigen::Isometry3d::Identity());
    for(size_t j=0; j<=i; ++j)
    {
      expectation = expectation * tfs[j];
    }

    Eigen::Isometry3d actual = F->getTransform();
    EXPECT_TRUE( equals(actual.matrix(), expectation.matrix(), tolerance));
  }

  randomize_transforms(tfs);
  for(size_t i=0; i<frames.size(); ++i)
  {
    SimpleFrame* F = frames[i];
    F->setRelativeTransform(tfs[i]);
  }

  Eigen::Isometry3d expectation(Eigen::Isometry3d::Identity());
  for(size_t j=0; j<frames.size(); ++j)
    expectation = expectation * tfs[j];

  EXPECT_TRUE( equals(frames.back()->getTransform().matrix(),
                      expectation.matrix(),
                      tolerance) );


  // -- Test Velocity --------------------------------------------------------

  // Basic forward spatial velocity computation
  { // The brackets are to recycle variable names
    std::vector<Eigen::Vector6d> v_rels(frames.size());
    std::vector<Eigen::Vector6d> v_total(frames.size());

    for(size_t i=0; i<frames.size(); ++i)
    {
      v_rels[i] = random_vec<6>();

      SimpleFrame* F = frames[i];
      F->setRelativeSpatialVelocity(v_rels[i]);
    }

    v_total[0] = v_rels[0];
    for(size_t i=1; i<frames.size(); ++i)
    {
      SimpleFrame* F = frames[i];
      compute_spatial_velocity(v_total[i-1], v_rels[i],
          F->getRelativeTransform(), v_total[i]);
    }

    for(size_t i=0; i<frames.size(); ++i)
    {
      SimpleFrame* F = frames[i];

      Eigen::Vector6d v_actual = F->getSpatialVelocity();

      EXPECT_TRUE( equals(v_total[i], v_actual) );
    }
  }

  // Testing conversion beteween spatial and classical velocities
//  {
//    std::vector<Eigen::Vector3d> v_rels(frames.size());
//    std::vector<Eigen::Vector3d> w_rels(frames.size());

//    std::vector<Eigen::Vector3d> v_total(frames.size());
//    std::vector<Eigen::Vector3d> w_total(frames.size());

//    for(size_t i=0; i<frames.size(); ++i)
//    {
//      v_rels[i] = random_vec<3>();
//      w_rels[i] = random_vec<3>();
//      Eigen::Vector6d spatial_v;
//      spatial_v << w_rels[i], v_rels[i];
//      SimpleFrame* F = frames[i];
//      F->setRelativeSpatialVelocity(spatial_v, F->getParentFrame());
//    }

//    v_total[0] = v_rels[0];
//    w_total[0] = w_rels[0];
//    for(size_t i=1; i<frames.size(); ++i)
//    {
//      SimpleFrame* F = frames[i];
//      Eigen::Vector3d offset = F->getRelativeTransform().translation();
//      Eigen::Isometry3d tf = frames[i-1]->getTransform();
//      compute_velocity(v_total[i-1], w_total[i-1], v_rels[i], w_rels[i],
//          offset, tf, v_total[i], w_total[i]);
//    }

//    for(size_t i=0; i<frames.size(); ++i)
//    {
//      std::cout << "Trial #" << i << "\n";
//      SimpleFrame* F = frames[i];
//  //    Eigen::Vector3d v_actual = F->getLinearVelocity();
//  //    Eigen::Vector3d w_actual = F->getAngularVelocity();

//      Eigen::Vector6d v = F->getSpatialVelocity(Frame::World());
//      Eigen::Vector3d v_actual = v.tail<3>();
//      Eigen::Vector3d w_actual = v.head<3>();

//      EXPECT_TRUE( equals(v_total[i], v_actual, tolerance) );
//      EXPECT_TRUE( equals(w_total[i], w_actual, tolerance) );

//      std::cout << "expected: " << v_total[i].transpose()
//                << "\nactual:   " << v_actual.transpose() << "\n";
//    }
//  }

}

int main(int argc, char* argv[])
{
  math::seedRand();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
