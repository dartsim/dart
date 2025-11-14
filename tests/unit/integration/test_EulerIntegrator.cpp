/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "../../helpers/GTestUtils.hpp"
#include "dart/integration/EulerIntegrator.hpp"

#include <gtest/gtest.h>

using namespace dart::integration;

namespace {

class RecordingSystem : public IntegrableSystem
{
public:
  RecordingSystem()
  {
    mConfigs = Eigen::VectorXd::Zero(1);
    mGenVels = Eigen::VectorXd::Zero(1);
    mAccels = Eigen::VectorXd::Zero(1);
  }

  void setConfigs(const Eigen::VectorXd& configs) override
  {
    mConfigs = configs;
  }

  void setGenVels(const Eigen::VectorXd& genVels) override
  {
    mGenVels = genVels;
  }

  Eigen::VectorXd getConfigs() const override
  {
    return mConfigs;
  }

  Eigen::VectorXd getGenVels() const override
  {
    ++mGetVelCalls;
    return mGenVels;
  }

  Eigen::VectorXd evalGenAccs() override
  {
    ++mEvalAccCalls;
    return mAccels;
  }

  void integrateConfigs(const Eigen::VectorXd& genVels, double dt) override
  {
    ++mConfigIntegrations;
    mConfigs += genVels * dt;
    mLastConfigDt = dt;
  }

  void integrateGenVels(const Eigen::VectorXd& accels, double dt) override
  {
    ++mVelIntegrations;
    mGenVels += accels * dt;
    mLastVelDt = dt;
  }

  void setAcceleration(const Eigen::VectorXd& accels)
  {
    mAccels = accels;
  }

  void resetCounters()
  {
    mConfigIntegrations = 0;
    mVelIntegrations = 0;
    mGetVelCalls = 0;
    mEvalAccCalls = 0;
    mLastConfigDt = 0.0;
    mLastVelDt = 0.0;
  }

  int mConfigIntegrations = 0;
  int mVelIntegrations = 0;
  int mGetVelCalls = 0;
  int mEvalAccCalls = 0;
  double mLastConfigDt = 0.0;
  double mLastVelDt = 0.0;

private:
  Eigen::VectorXd mConfigs;
  Eigen::VectorXd mGenVels;
  Eigen::VectorXd mAccels;
};

} // namespace

//==============================================================================
TEST(EulerIntegratorTests, IntegrateUpdatesConfigsThenVels)
{
  EulerIntegrator integrator;
  RecordingSystem system;

  Eigen::VectorXd configs(1);
  configs << 1.0;
  Eigen::VectorXd velocities(1);
  velocities << 2.0;
  Eigen::VectorXd accels(1);
  accels << -3.0;

  system.setConfigs(configs);
  system.setGenVels(velocities);
  system.setAcceleration(accels);

  const double dt = 0.1;
  integrator.integrate(&system, dt);

  EXPECT_NEAR(system.getConfigs()[0], 1.0 + 2.0 * dt, 1e-12);
  EXPECT_NEAR(system.getGenVels()[0], 2.0 + (-3.0) * dt, 1e-12);
  EXPECT_EQ(system.mConfigIntegrations, 1);
  EXPECT_EQ(system.mVelIntegrations, 1);
  EXPECT_EQ(system.mGetVelCalls, 1);
  EXPECT_EQ(system.mEvalAccCalls, 1);
  EXPECT_DOUBLE_EQ(system.mLastConfigDt, dt);
  EXPECT_DOUBLE_EQ(system.mLastVelDt, dt);
}

//==============================================================================
TEST(EulerIntegratorTests, IntegratePosUsesVelocitiesOnly)
{
  EulerIntegrator integrator;
  RecordingSystem system;
  system.resetCounters();

  Eigen::VectorXd configs(1);
  configs << -5.0;
  Eigen::VectorXd velocities(1);
  velocities << 4.0;
  system.setConfigs(configs);
  system.setGenVels(velocities);

  const double dt = 0.25;
  integrator.integratePos(&system, dt);

  EXPECT_NEAR(system.getConfigs()[0], -5.0 + 4.0 * dt, 1e-12);
  EXPECT_NEAR(system.getGenVels()[0], 4.0, 1e-12);
  EXPECT_EQ(system.mConfigIntegrations, 1);
  EXPECT_EQ(system.mVelIntegrations, 0);
  EXPECT_EQ(system.mGetVelCalls, 1);
  EXPECT_EQ(system.mEvalAccCalls, 0);
}

//==============================================================================
TEST(EulerIntegratorTests, IntegrateVelUsesAccelerationsOnly)
{
  EulerIntegrator integrator;
  RecordingSystem system;
  system.resetCounters();

  Eigen::VectorXd velocities(1);
  velocities << -1.0;
  Eigen::VectorXd accels(1);
  accels << 0.5;
  system.setGenVels(velocities);
  system.setAcceleration(accels);

  const double dt = 0.4;
  integrator.integrateVel(&system, dt);

  EXPECT_NEAR(system.getGenVels()[0], -1.0 + 0.5 * dt, 1e-12);
  EXPECT_EQ(system.mConfigIntegrations, 0);
  EXPECT_EQ(system.mVelIntegrations, 1);
  EXPECT_EQ(system.mGetVelCalls, 0);
  EXPECT_EQ(system.mEvalAccCalls, 1);
  EXPECT_DOUBLE_EQ(system.mLastVelDt, dt);
}
