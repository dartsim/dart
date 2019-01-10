/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/math/Random.hpp"

#include "TestHelpers.hpp"

using namespace dart;
using namespace dynamics;

template<int N>
Eigen::Matrix<double,N,1> random_vec(double limit=100)
{
  return Random::uniform<Eigen::Matrix<double,N,1>>(-limit, limit);
}

void randomize_transform(Eigen::Isometry3d& tf,
                         double translation_limit=100,
                         double rotation_limit=100)
{
  Eigen::Vector3d r = random_vec<3>(translation_limit);
  Eigen::Vector3d theta = random_vec<3>(rotation_limit);

  tf.setIdentity();
  tf.translate(r);

  if(theta.norm()>0)
    tf.rotate(Eigen::AngleAxisd(theta.norm(), theta.normalized()));
}

void randomize_transforms(common::aligned_vector<Eigen::Isometry3d>& tfs)
{
  for(std::size_t i=0; i<tfs.size(); ++i)
  {
    randomize_transform(tfs[i]);
  }
}

void compute_spatial_velocity(const Eigen::Vector6d& v_parent,
                              const Eigen::Vector6d& v_relative,
                              const Eigen::Isometry3d& tf_rel,
                              Eigen::Vector6d& v_child)
{
  v_child = math::AdInvT(tf_rel, v_parent) + v_relative;
}

void compute_spatial_acceleration(const Eigen::Vector6d& a_parent,
                                  const Eigen::Vector6d& a_relative,
                                  const Eigen::Vector6d& v_child,
                                  const Eigen::Vector6d& v_rel,
                                  const Eigen::Isometry3d& tf_rel,
                                  Eigen::Vector6d& a_child)
{
  a_child = math::AdInvT(tf_rel, a_parent) + a_relative
          + math::ad(v_child, v_rel);
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
                          const Eigen::Vector3d& w_rel,
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

  alpha_child = alpha_parent + R*alpha_rel + w_parent.cross(R*w_rel);
}

TEST(FRAMES, FORWARD_KINEMATICS_CHAIN)
{
  std::vector<SimpleFrame*> frames;

  double tolerance = 1e-6;

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

  common::aligned_vector<Eigen::Isometry3d> tfs;
  tfs.resize(frames.size(), Eigen::Isometry3d::Identity());

  randomize_transforms(tfs);

  for(std::size_t i=0; i<frames.size(); ++i)
  {
    SimpleFrame* F = frames[i];
    F->setRelativeTransform(tfs[i]);
  }

  for(std::size_t i=0; i<frames.size(); ++i)
  {
    Frame* F = frames[i];
    Eigen::Isometry3d expectation(Eigen::Isometry3d::Identity());
    for(std::size_t j=0; j<=i; ++j)
    {
      expectation = expectation * tfs[j];
    }

    Eigen::Isometry3d actual = F->getTransform();
    EXPECT_TRUE( equals(actual.matrix(), expectation.matrix(), tolerance));
  }

  randomize_transforms(tfs);
  for(std::size_t i=0; i<frames.size(); ++i)
  {
    SimpleFrame* F = frames[i];
    F->setRelativeTransform(tfs[i]);
  }

  Eigen::Isometry3d expectation(Eigen::Isometry3d::Identity());
  for(std::size_t j=0; j<frames.size(); ++j)
    expectation = expectation * tfs[j];

  EXPECT_TRUE( equals(frames.back()->getTransform().matrix(),
                      expectation.matrix(),
                      tolerance) );


  // -- Test Velocity --------------------------------------------------------

  // Basic forward spatial velocity propagation
  { // The brackets are to allow reusing variable names
    common::aligned_vector<Eigen::Vector6d> v_rels(frames.size());
    common::aligned_vector<Eigen::Vector6d> v_total(frames.size());

    for(std::size_t i=0; i<frames.size(); ++i)
    {
      v_rels[i] = random_vec<6>();

      SimpleFrame* F = frames[i];
      F->setRelativeSpatialVelocity(v_rels[i]);

      if(i>0)
        compute_spatial_velocity(v_total[i-1], v_rels[i],
            F->getRelativeTransform(), v_total[i]);
      else
        compute_spatial_velocity(Eigen::Vector6d::Zero(), v_rels[i],
                                 F->getRelativeTransform(), v_total[i]);
    }

    for(std::size_t i=0; i<frames.size(); ++i)
    {
      SimpleFrame* F = frames[i];

      Eigen::Vector6d v_actual = F->getSpatialVelocity();

      EXPECT_TRUE( equals(v_total[i], v_actual) );
    }
  }

  // Testing conversion beteween spatial and classical velocities
  {
    std::vector<Eigen::Vector3d> v_rels(frames.size());
    std::vector<Eigen::Vector3d> w_rels(frames.size());

    std::vector<Eigen::Vector3d> v_total(frames.size());
    std::vector<Eigen::Vector3d> w_total(frames.size());

    for(std::size_t i=0; i<frames.size(); ++i)
    {
      v_rels[i] = random_vec<3>();
      w_rels[i] = random_vec<3>();

      SimpleFrame* F = frames[i];
      F->setClassicDerivatives(v_rels[i], w_rels[i]);

      Eigen::Vector3d offset = F->getRelativeTransform().translation();

      Eigen::Isometry3d tf = i>0? frames[i-1]->getTransform() :
          Eigen::Isometry3d::Identity();

      if(i>0)
        compute_velocity(v_total[i-1], w_total[i-1], v_rels[i], w_rels[i],
            offset, tf, v_total[i], w_total[i]);
      else
        compute_velocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                         v_rels[i], w_rels[i],
                         offset, tf, v_total[i], w_total[i]);
    }

    for(std::size_t i=0; i<frames.size(); ++i)
    {
      SimpleFrame* F = frames[i];
      Eigen::Vector3d v_actual = F->getLinearVelocity();
      Eigen::Vector3d w_actual = F->getAngularVelocity();

      EXPECT_TRUE( equals(v_total[i], v_actual, tolerance) );
      EXPECT_TRUE( equals(w_total[i], w_actual, tolerance) );
    }
  }

  // -- Acceleration ---------------------------------------------------------

  // Basic forward spatial acceleration propagation
  {
    common::aligned_vector<Eigen::Vector6d> v_rels(frames.size());
    common::aligned_vector<Eigen::Vector6d> a_rels(frames.size());

    common::aligned_vector<Eigen::Vector6d> v_total(frames.size());
    common::aligned_vector<Eigen::Vector6d> a_total(frames.size());


    for(std::size_t i=0; i<frames.size(); ++i)
    {
      v_rels[i] = random_vec<6>();
      a_rels[i] = random_vec<6>();

      SimpleFrame* F = frames[i];
      F->setRelativeSpatialVelocity(v_rels[i]);
      F->setRelativeSpatialAcceleration(a_rels[i]);
      Eigen::Isometry3d tf = F->getRelativeTransform();

      if(i>0)
      {
        compute_spatial_velocity(v_total[i-1], v_rels[i], tf, v_total[i]);
        compute_spatial_acceleration(a_total[i-1], a_rels[i], v_total[i],
            v_rels[i], tf, a_total[i]);
      }
      else
      {
        compute_spatial_velocity(Eigen::Vector6d::Zero(), v_rels[i],
                                 tf, v_total[i]);
        compute_spatial_acceleration(Eigen::Vector6d::Zero(), a_rels[i],
                                     v_total[i], v_rels[i], tf, a_total[i]);
      }
    }

    for(std::size_t i=0; i<frames.size(); ++i)
    {
      SimpleFrame* F = frames[i];

      Eigen::Vector6d a_actual = F->getSpatialAcceleration();

      EXPECT_TRUE( equals(a_total[i], a_actual) );
    }

    // Test relative computations
    for(std::size_t i=0; i<frames.size(); ++i)
    {
      Frame* F = frames[i];
      Frame* P = F->getParentFrame();

      Eigen::Vector6d v_rel = F->getSpatialVelocity(P, F);
      Eigen::Vector6d a_rel = F->getSpatialAcceleration(P, F);

      EXPECT_TRUE( equals(v_rels[i], v_rel, tolerance) );
      EXPECT_TRUE( equals(a_rels[i], a_rel, tolerance) );
    }

    // Test offset computations
    for(std::size_t i=0; i<frames.size(); ++i)
    {
      Eigen::Vector3d offset = random_vec<3>();
      Frame* F = frames[i];

      Eigen::Vector3d v_actual = F->getLinearVelocity(offset);
      Eigen::Vector3d w_actual = F->getAngularVelocity();
      Eigen::Vector3d v_expect, w_expect;
      compute_velocity(F->getLinearVelocity(), F->getAngularVelocity(),
                       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       offset, F->getWorldTransform(), v_expect, w_expect);

      EXPECT_TRUE( equals( v_expect, v_actual) );
      EXPECT_TRUE( equals( w_expect, w_actual) );

      Eigen::Vector3d a_actual = F->getLinearAcceleration(offset);
      Eigen::Vector3d alpha_actual = F->getAngularAcceleration();
      Eigen::Vector3d a_expect, alpha_expect;
      compute_acceleration(F->getLinearAcceleration(),
                           F->getAngularAcceleration(), F->getAngularVelocity(),
                           Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                           offset, F->getWorldTransform(),
                           a_expect, alpha_expect);

      EXPECT_TRUE( equals( a_expect, a_actual) );
      EXPECT_TRUE( equals( alpha_expect, alpha_actual) );
    }
  }

  // Testing conversion between spatial and classical accelerations
  {
    std::vector<Eigen::Vector3d> v_rels(frames.size());
    std::vector<Eigen::Vector3d> w_rels(frames.size());
    std::vector<Eigen::Vector3d> a_rels(frames.size());
    std::vector<Eigen::Vector3d> alpha_rels(frames.size());

    std::vector<Eigen::Vector3d> v_total(frames.size());
    std::vector<Eigen::Vector3d> w_total(frames.size());
    std::vector<Eigen::Vector3d> a_total(frames.size());
    std::vector<Eigen::Vector3d> alpha_total(frames.size());

    for(std::size_t i=0; i<frames.size(); ++i)
    {
      v_rels[i] = random_vec<3>();
      w_rels[i] = random_vec<3>();
      a_rels[i] = random_vec<3>();
      alpha_rels[i] = random_vec<3>();

      SimpleFrame* F = frames[i];
      Eigen::Isometry3d rel_tf;
      randomize_transform(rel_tf);
      F->setRelativeTransform(rel_tf);
      F->setClassicDerivatives(v_rels[i], w_rels[i], a_rels[i], alpha_rels[i]);

      Eigen::Vector3d offset = F->getRelativeTransform().translation();

      Eigen::Isometry3d tf = i>0? frames[i-1]->getTransform() :
          Eigen::Isometry3d::Identity();

      if(i>0)
      {
        compute_velocity(v_total[i-1], w_total[i-1], v_rels[i], w_rels[i],
            offset, tf, v_total[i], w_total[i]);
        compute_acceleration(a_total[i-1], alpha_total[i-1], w_total[i-1],
            a_rels[i], alpha_rels[i], v_rels[i], w_rels[i], offset, tf,
            a_total[i], alpha_total[i]);
      }
      else
      {
        compute_velocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                         v_rels[i], w_rels[i], offset, tf,
                         v_total[i], w_total[i]);
        compute_acceleration(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                             Eigen::Vector3d::Zero(), a_rels[i], alpha_rels[i],
                             v_rels[i], w_rels[i], offset, tf,
                             a_total[i], alpha_total[i]);
      }
    }

    for(std::size_t i=0; i<frames.size(); ++i)
    {
      SimpleFrame* F = frames[i];
      Eigen::Vector3d v_actual = F->getLinearVelocity();
      Eigen::Vector3d w_actual = F->getAngularVelocity();
      Eigen::Vector3d a_actual = F->getLinearAcceleration();
      Eigen::Vector3d alpha_actual = F->getAngularAcceleration();

      EXPECT_TRUE( equals(v_total[i], v_actual, tolerance) );
      EXPECT_TRUE( equals(w_total[i], w_actual, tolerance) );
      EXPECT_TRUE( equals(a_total[i], a_actual, tolerance) );
      EXPECT_TRUE( equals(alpha_total[i], alpha_actual, tolerance) );
    }

    // Test relative computations
    for(std::size_t i=0; i<frames.size(); ++i)
    {
      SimpleFrame* F = frames[i];
      Frame* P = F->getParentFrame();
      Eigen::Vector3d v_rel = F->getLinearVelocity(P, P);
      Eigen::Vector3d w_rel = F->getAngularVelocity(P, P);
      Eigen::Vector3d a_rel = F->getLinearAcceleration(P, P);
      Eigen::Vector3d alpha_rel = F->getAngularAcceleration(P, P);

      EXPECT_TRUE( equals(v_rels[i], v_rel, tolerance) );
      EXPECT_TRUE( equals(w_rels[i], w_rel, tolerance) );
      EXPECT_TRUE( equals(a_rels[i], a_rel, tolerance) );
      EXPECT_TRUE( equals(alpha_rels[i], alpha_rel, tolerance) );
    }
  }
}

void randomize_target_values(const std::vector<SimpleFrame*>& targets,
                                bool spatial)
{
  for(std::size_t i=0; i<targets.size(); ++i)
  {
    SimpleFrame* T = targets[i];

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    randomize_transform(tf, 1, 2*M_PI);

    if(spatial)
    {
      T->setRelativeTransform(tf);
      T->setRelativeSpatialVelocity(random_vec<6>(1));
      T->setRelativeSpatialAcceleration(random_vec<6>(1));
    }
    else
    {
      T->setRelativeTransform(tf);
      T->setClassicDerivatives(random_vec<3>(1), random_vec<3>(1),
                               random_vec<3>(1), random_vec<3>(1));
    }
  }
}

void set_relative_values(const std::vector<SimpleFrame*>& targets,
                         const std::vector<SimpleFrame*>& followers,
                         bool spatial)
{
  for(std::size_t i=0; i<targets.size(); ++i)
  {
    Frame* T = targets[i];
    SimpleFrame* F = followers[i];
    Frame* P = F->getParentFrame();

    F->setRelativeTransform(T->getTransform(P));
    if(spatial)
    {
      F->setRelativeSpatialVelocity(T->getSpatialVelocity(P, F));
      F->setRelativeSpatialAcceleration(T->getSpatialAcceleration(P, F));
    }
    else
    {
      F->setClassicDerivatives(T->getLinearVelocity(P, P),
                               T->getAngularVelocity(P, P),
                               T->getLinearAcceleration(P, P),
                               T->getAngularAcceleration(P, P));
    }
  }
}

void check_world_values(const std::vector<SimpleFrame*>& targets,
                        const std::vector<SimpleFrame*>& followers,
                        double tolerance)
{
  for(std::size_t i=0; i<targets.size(); ++i)
  {
    Frame* T = targets[i];
    Frame* F = followers[i];

    const Eigen::Isometry3d& tf_error =
        T->getWorldTransform()*F->getWorldTransform().inverse();
    Eigen::Vector6d error;
    Eigen::AngleAxisd rot_error(tf_error.rotation());
    error.block<3,1>(0,0) = rot_error.angle()*rot_error.axis();
    error.block<3,1>(3,0) = tf_error.translation();

    EXPECT_TRUE( error.norm() < tolerance );

    EXPECT_TRUE( equals(T->getSpatialVelocity(),
                        F->getSpatialVelocity(), tolerance) );

    EXPECT_TRUE( equals(T->getLinearVelocity(),
                        F->getLinearVelocity(), tolerance) );

    EXPECT_TRUE( equals(T->getAngularVelocity(),
                        F->getAngularVelocity(), tolerance) );

    EXPECT_TRUE( equals(T->getSpatialAcceleration(),
                        F->getSpatialAcceleration(), tolerance) );

    EXPECT_TRUE( equals(T->getLinearAcceleration(),
                        F->getLinearAcceleration(), tolerance) );

    EXPECT_TRUE( equals(T->getAngularAcceleration(),
                        F->getAngularAcceleration(), tolerance) );
  }
}

void check_values(const std::vector<SimpleFrame*>& targets,
                  const std::vector<SimpleFrame*>& followers,
                  const Frame* relativeTo,
                  const Frame* inCoordinatesOf,
                  double tolerance)
{
  for(std::size_t i=0; i<targets.size(); ++i)
  {
    Frame* T = targets[i];
    Frame* F = followers[i];

    const Eigen::Isometry3d& tf_error =
        T->getTransform(relativeTo)*F->getTransform(relativeTo).inverse();
    Eigen::Vector6d error;
    Eigen::AngleAxisd rot_error(tf_error.rotation());
    error.block<3,1>(0,0) = rot_error.angle()*rot_error.axis();
    error.block<3,1>(3,0) = tf_error.translation();

    EXPECT_TRUE( error.norm() < tolerance );

    EXPECT_TRUE( equals(T->getSpatialVelocity(relativeTo, inCoordinatesOf),
                        F->getSpatialVelocity(relativeTo, inCoordinatesOf),
                        tolerance) );

    EXPECT_TRUE( equals(T->getLinearVelocity(relativeTo, inCoordinatesOf),
                        F->getLinearVelocity(relativeTo, inCoordinatesOf),
                        tolerance) );

    EXPECT_TRUE( equals(T->getAngularVelocity(relativeTo, inCoordinatesOf),
                        F->getAngularVelocity(relativeTo, inCoordinatesOf),
                        tolerance) );

    EXPECT_TRUE( equals(T->getSpatialAcceleration(relativeTo, inCoordinatesOf),
                        F->getSpatialAcceleration(relativeTo, inCoordinatesOf),
                        tolerance) );

    EXPECT_TRUE( equals(T->getLinearAcceleration(relativeTo, inCoordinatesOf),
                        F->getLinearAcceleration(relativeTo, inCoordinatesOf),
                        tolerance) );

    EXPECT_TRUE( equals(T->getAngularAcceleration(relativeTo, inCoordinatesOf),
                        F->getAngularAcceleration(relativeTo, inCoordinatesOf),
                        tolerance) );
  }
}

void check_offset_computations(const std::vector<SimpleFrame*>& targets,
                               const std::vector<SimpleFrame*>& followers,
                               const Frame* relativeTo,
                               const Frame* inCoordinatesOf,
                               double tolerance)
{
  for(std::size_t i=0; i<targets.size(); ++i)
  {
    Frame* T = targets[i];
    Frame* F = followers[i];

    Eigen::Isometry3d coordTf = relativeTo->getTransform(inCoordinatesOf);

    Vector3d offset_T = random_vec<3>();
    Vector3d offset_F = T->getTransform(F) * offset_T;

    Vector3d v_TO, w_TO, v_FO, w_FO, a_TO, alpha_TO, a_FO, alpha_FO;

    // Compute velocity of the offfset in the relative frame
    compute_velocity(T->getLinearVelocity(relativeTo, relativeTo),
                     T->getAngularVelocity(relativeTo, relativeTo),
                     Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                     offset_T, T->getTransform(relativeTo), v_TO, w_TO);
    // Convert to the desired coordinate system
    v_TO = coordTf.linear() * v_TO;
    w_TO = coordTf.linear() * w_TO;

    // Compute acceleration of the offset in the relative frame
    compute_acceleration(T->getLinearAcceleration(relativeTo, relativeTo),
                         T->getAngularAcceleration(relativeTo, relativeTo),
                         T->getAngularVelocity(relativeTo, relativeTo),
                         Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                         Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                         offset_T, T->getTransform(relativeTo), a_TO, alpha_TO);
    // Convert to the desired coordinate system
    a_TO = coordTf.linear() * a_TO;
    alpha_TO = coordTf.linear() * alpha_TO;

    compute_velocity(F->getLinearVelocity(relativeTo, relativeTo),
                     F->getAngularVelocity(relativeTo, relativeTo),
                     Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                     offset_F, F->getTransform(relativeTo), v_FO, w_FO);
    v_FO = coordTf.linear() * v_FO;
    w_FO = coordTf.linear() * w_FO;

    compute_acceleration(F->getLinearAcceleration(relativeTo, relativeTo),
                         F->getAngularAcceleration(relativeTo, relativeTo),
                         F->getAngularVelocity(relativeTo, relativeTo),
                         Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                         Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                         offset_F, F->getTransform(relativeTo), a_FO, alpha_FO);
    a_FO = coordTf.linear() * a_FO;
    alpha_FO = coordTf.linear() * alpha_FO;

    Eigen::Vector3d v_TO_actual = T->getLinearVelocity(offset_T, relativeTo,
                                                       inCoordinatesOf);
    Eigen::Vector3d v_FO_actual = F->getLinearVelocity(offset_F, relativeTo,
                                                       inCoordinatesOf);

    EXPECT_TRUE( equals(v_TO, v_FO, tolerance) );
    EXPECT_TRUE( equals(v_TO, v_TO_actual, tolerance) );
    EXPECT_TRUE( equals(v_FO, v_FO_actual, tolerance) );

    Eigen::Vector3d a_TO_actual = T->getLinearAcceleration(offset_T, relativeTo,
                                                           inCoordinatesOf);
    Eigen::Vector3d a_FO_actual = F->getLinearAcceleration(offset_F, relativeTo,
                                                           inCoordinatesOf);

    EXPECT_TRUE( equals(a_TO, a_FO, tolerance) );
    EXPECT_TRUE( equals(a_TO, a_TO_actual, tolerance) );
    EXPECT_TRUE( equals(a_FO, a_FO_actual, tolerance) );
  }
}

void test_relative_values(bool spatial_targets, bool spatial_followers)
{
  const double tolerance = 1e-8;

  // These frames will form a chain
  SimpleFrame A(Frame::World(), "A");
  SimpleFrame B(&A, "B");
  SimpleFrame C(&B, "C");
  SimpleFrame D(&C, "D");

  std::vector<SimpleFrame*> targets;
  targets.push_back(&A);
  targets.push_back(&B);
  targets.push_back(&C);
  targets.push_back(&D);

  // R will be an arbitrary reference frame
  SimpleFrame R(Frame::World(), "R");
  targets.push_back(&R);

  // Each of these frames will attempt to track one of the frames in the chain
  // with respect to the frame R.
  SimpleFrame AR(&R, "AR");
  SimpleFrame BR(&R, "BR");
  SimpleFrame CR(&R, "CR");
  SimpleFrame DR(&R, "DR");
  SimpleFrame RR(&R, "RR");

  std::vector<SimpleFrame*> followers;
  followers.push_back(&AR);
  followers.push_back(&BR);
  followers.push_back(&CR);
  followers.push_back(&DR);
  followers.push_back(&RR);

  assert( targets.size() == followers.size() );

  randomize_target_values(targets, spatial_targets);
  set_relative_values(targets, followers, spatial_followers);
  check_world_values(targets, followers, tolerance);

  // Check every combination of relative values
  for(std::size_t i=0; i<targets.size(); ++i)
  {
    Frame* T = targets[i];
    for(std::size_t j=0; j<followers.size(); ++j)
    {
      Frame* F = followers[j];

      check_values(targets, followers, T, F, tolerance);
      check_values(targets, followers, F, T, tolerance);
      check_offset_computations(targets, followers, T, F, tolerance);
    }
  }

  // Test SimpleFrame::setTransform()
  for(std::size_t i=0; i<followers.size(); ++i)
  {
    for(std::size_t j=0; j<followers.size(); ++j)
    {
      SimpleFrame* F = followers[i];
      SimpleFrame* T = followers[j];
      Eigen::Isometry3d tf;
      randomize_transform(tf, 1, 2*M_PI);
      T->setTransform(tf, F);
      if(i != j)
      {
        EXPECT_TRUE( equals(T->getTransform(F).matrix(), tf.matrix(), 1e-10));
      }
    }
  }
}

// Test different combinations of using spatial and classical derivative terms

TEST(RELATIVE_FRAMES, SPATIAL_SPATIAL)
{
  test_relative_values(true, true);
}

TEST(RELATIVE_FRAMES, CLASSIC_CLASSIC)
{
  test_relative_values(false, false);
}

TEST(RELATIVE_FRAMES, SPATIAL_CLASSIC)
{
  test_relative_values(true, false);
}

TEST(RELATIVE_FRAMES, CLASSIC_SPATIAL)
{
  test_relative_values(false, true);
}

TEST(FRAMES, CHILDHOOD)
{
  SimpleFrame F1(Frame::World(), "F1");
  SimpleFrame F2(&F1, "F2");

  EXPECT_TRUE(F1.getNumChildFrames() == 1);
}
