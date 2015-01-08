
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

template<int N>
Eigen::Matrix<double,N,1> random_vec(double limit=100)
{
  Eigen::Matrix<double,N,1> v;
  for(size_t i=0; i<N; ++i)
    v[i] = math::random(-fabs(limit), fabs(limit));
  return v;
}

void randomize_transform(Eigen::Isometry3d& tf)
{
  Eigen::Vector3d v = random_vec<3>();
  Eigen::Vector3d theta = random_vec<3>();

  tf.setIdentity();
  tf.translate(v);

  if(theta.norm()>0)
    tf.rotate(Eigen::AngleAxisd(theta.norm(), theta.normalized()));
}

void randomize_transforms(std::vector<Eigen::Isometry3d>& tfs)
{
  for(size_t i=0; i<tfs.size(); ++i)
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

  double tolerance = 1e-5;

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

  // Basic forward spatial velocity propagation
  { // The brackets are to allow reusing variable names
    std::vector<Eigen::Vector6d> v_rels(frames.size());
    std::vector<Eigen::Vector6d> v_total(frames.size());

    for(size_t i=0; i<frames.size(); ++i)
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

    for(size_t i=0; i<frames.size(); ++i)
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

    for(size_t i=0; i<frames.size(); ++i)
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

    for(size_t i=0; i<frames.size(); ++i)
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
    std::vector<Eigen::Vector6d> v_rels(frames.size());
    std::vector<Eigen::Vector6d> a_rels(frames.size());

    std::vector<Eigen::Vector6d> v_total(frames.size());
    std::vector<Eigen::Vector6d> a_total(frames.size());

    for(size_t i=0; i<frames.size(); ++i)
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

    for(size_t i=0; i<frames.size(); ++i)
    {
      SimpleFrame* F = frames[i];

      Eigen::Vector6d a_actual = F->getSpatialAcceleration();

      EXPECT_TRUE( equals(a_total[i], a_actual) );
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

    for(size_t i=0; i<frames.size(); ++i)
    {
      v_rels[i] = random_vec<3>();
      w_rels[i] = random_vec<3>();
      a_rels[i] = random_vec<3>();
      alpha_rels[i] = random_vec<3>();

      SimpleFrame* F = frames[i];
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

    for(size_t i=0; i<frames.size(); ++i)
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
  }
}

typedef Eigen::Matrix<double,12,1> FrameDerivative;

class FrameState
{
public:
  Eigen::Isometry3d x;
  Eigen::Vector6d v;

  FrameState& integrate(const FrameDerivative& f_dt)
  {
    Eigen::Isometry3d xi(Eigen::Isometry3d::Identity());
    xi.translate(x.translation());
    xi.translate(f_dt.block<3,1>(3,0));
    Eigen::Vector3d theta = f_dt.block<3,1>(0,0);
    if(theta.norm() > 0)
      xi.rotate(Eigen::AngleAxisd(theta.norm(), theta.normalized()));
    xi.rotate(x.rotation());

    x = xi;

    v += f_dt.block<6,1>(6,0);

    return *this;
  }
};

typedef std::vector<FrameState> StateVector;
typedef std::vector<FrameDerivative> DerivVector;

DerivVector operator+(const DerivVector& dv1, const DerivVector& dv2)
{
  assert( dv1.size() == dv2.size() );
  DerivVector result(dv1.size());

  for(size_t i=0; i<dv1.size(); ++i)
    result[i] = dv1[i]+dv2[i];

  return result;
}

DerivVector operator*(const DerivVector& dv, double dt)
{
  DerivVector result(dv.size());
  for(size_t i=0; i<dv.size(); ++i)
    result[i] = dv[i]*dt;

  return result;
}

StateVector integrate(const StateVector& sv, const DerivVector& f_dt)
{
  assert( sv.size() == f_dt.size() );

  StateVector result = sv;

  for(size_t i=0; i<result.size(); ++i)
    result[i].integrate(f_dt[i]);

  return result;
}

void setSpatialStates(const std::vector<SimpleFrame*>& targets,
                      const StateVector& sv)
{
  assert( targets.size() == sv.size() );
  for(size_t i=0; i<sv.size(); ++i)
  {
    SimpleFrame* F = targets[i];
    F->setRelativeTransform(sv[i].x);
    F->setRelativeSpatialVelocity(sv[i].v);
  }
}

void getSpatialStates(const std::vector<SimpleFrame*>& targets, StateVector& sv)
{
  assert( targets.size() == sv.size() );
  for(size_t i=0; i<sv.size(); ++i)
  {
    Frame* F = targets[i];
    sv[i].x = F->getRelativeTransform();
    sv[i].v = F->getRelativeSpatialVelocity();
  }
}


DerivVector getRelativeSpatialDerivatives(
    const std::vector<SimpleFrame*>& targets,
    const std::vector<SimpleFrame*>& followers,
    const Frame* R,
    const StateVector& sv_targets,
    const StateVector& sv_followers)
{
  assert( followers.size() == targets.size() );
  assert( sv_targets.size() == targets.size() );
  assert( sv_followers.size() == targets.size() );

  DerivVector dv(targets.size());

  setSpatialStates(targets, sv_targets);
  setSpatialStates(followers, sv_followers);

  for(size_t i=0; i<dv.size(); ++i)
  {
    Frame* F = followers[i];
    dv[i].block<6,1>(0,0) = F->getSpatialVelocity(R, R);
    Frame* T = targets[i];
    dv[i].block<6,1>(6,0) = T->getSpatialAcceleration(R, R);
  }

  return dv;
}

DerivVector getSpatialDerivatives(const std::vector<SimpleFrame*>& targets,
                                  const StateVector& sv)
{
  assert( targets.size() == sv.size() );
  DerivVector dv(sv.size());

  setSpatialStates(targets, sv);

  for(size_t i=0; i<dv.size(); ++i)
  {
    Frame* T = targets[i];
    dv[i].block<6,1>(0,0) = T->getRelativeSpatialVelocity();
    dv[i].block<6,1>(6,0) = T->getRelativeSpatialAcceleration();
  }

  return dv;
}

void RK4(const std::vector<SimpleFrame*>& targets,
         const std::vector<SimpleFrame*>& followers,
         StateVector& sv_targets, StateVector& sv_followers,
         const Frame* R, double dt)
{
  DerivVector k_targets[4];
  DerivVector k_followers[4];

  k_targets[0] = getSpatialDerivatives(targets, sv_targets);
  k_followers[0] = getRelativeSpatialDerivatives(
        targets, followers, R, sv_targets, sv_followers);

  k_targets[1] = getSpatialDerivatives(targets,
        integrate(sv_targets, k_targets[0]*(dt/2)) );
  k_followers[1] = getRelativeSpatialDerivatives(
        targets, followers, R,
        integrate(sv_targets, k_targets[0]*(dt/2)),
        integrate(sv_followers, k_followers[0]*(dt/2)) );

  k_targets[2] = getSpatialDerivatives(targets,
        integrate(sv_targets, k_targets[1]*(dt/2)) );
  k_followers[2] = getRelativeSpatialDerivatives(
        targets, followers, R,
        integrate(sv_targets, k_targets[1]*(dt/2)),
        integrate(sv_followers, k_followers[1]*(dt/2)) );

  k_targets[3] = getSpatialDerivatives(targets,
        integrate(sv_targets, k_targets[2]*dt));
  k_followers[3] = getRelativeSpatialDerivatives(
        targets, followers, R,
        integrate(sv_targets, k_targets[2]*dt),
        integrate(sv_followers, k_followers[2]*dt) );

  DerivVector dx_targets = ( k_targets[0]
                           + k_targets[1]*2
                           + k_targets[2]*2
                           + k_targets[3] )*(dt/6);
  DerivVector dx_followers = ( k_followers[0]
                             + k_followers[1]*2
                             + k_followers[2]*2
                             + k_followers[3] )*(dt/6);

  sv_targets = integrate(sv_targets, dx_targets);
  sv_followers = integrate(sv_followers, dx_followers);
}

TEST(FRAMES, SPATIAL_INTEGRATION)
{
  const double dt = 0.001;
  const double final_time = 0.5;

  // These frames will form a moving chain
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
  // Each of these frames will attempt to track one of the frames in the chain
  // with respect to the frame R by mimicking their their accelerations. If they
  // each end with the same world pose as the frame they are tracking, then our
  // library's calculations for relative velocity and acceleration are correct.
  SimpleFrame RA(&R, "RA");
  SimpleFrame RB(&R, "RB");
  SimpleFrame RC(&R, "RC");
  SimpleFrame RD(&R, "RD");
  SimpleFrame RR(&R, "RR");

  targets.push_back(&R);

  std::vector<SimpleFrame*> followers;
  followers.push_back(&RA);
  followers.push_back(&RB);
  followers.push_back(&RC);
  followers.push_back(&RD);
  followers.push_back(&RR);

  assert( targets.size() == followers.size() );

  for(size_t i=0; i<targets.size(); ++i)
  {
    SimpleFrame* T = targets[i];
    SimpleFrame* F = followers[i];

    Eigen::Isometry3d tf;
    randomize_transform(tf);

    T->setRelativeTransform(tf);
    T->setRelativeSpatialVelocity(random_vec<6>(1));
    T->setRelativeSpatialAcceleration(random_vec<6>(1));

    F->setRelativeTransform(T->getTransform(&R));
    F->setRelativeSpatialVelocity(T->getSpatialVelocity(&R, F));
    F->setRelativeSpatialAcceleration(T->getSpatialAcceleration(&R, F));
  }

  StateVector sv_targets(targets.size()), sv_followers(followers.size());
  double elapsed_time = 0;
  while(elapsed_time < final_time)
  {
    elapsed_time += dt;

    getSpatialStates(targets, sv_targets);
    getSpatialStates(followers, sv_followers);

    RK4(targets, followers, sv_targets, sv_followers, &R, dt);

    setSpatialStates(targets, sv_targets);
    setSpatialStates(followers, sv_followers);
  }

  for(size_t i=0; i<targets.size(); ++i)
  {
    Frame* T = targets[i];
    Frame* F = followers[i];
//    EXPECT_TRUE( equals(T->getTransform().matrix(),
//                        F->getTransform().matrix()) );

    std::cout << "Trial " << T->getName() << "\n";
    std::cout << T->getTransform().matrix() << "\n";
    std::cout << F->getTransform().matrix() << "\n";
  }
}

int main(int argc, char* argv[])
{
  math::seedRand();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
