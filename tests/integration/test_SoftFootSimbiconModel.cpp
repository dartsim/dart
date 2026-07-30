/*
 * Copyright (c) 2011, The DART development contributors
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
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided with the
 *     distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// GUI-free numerical oracle for the soft-foot SIMBICON biped (Jain/Liu 2011
// "biped push recovery" parity row). The reusable model + the reused
// atlas_simbicon SIMBICON controller stay GUI-free so the paper's soft-vs-rigid
// contact claim can be exercised directly here.
//
// The three gates below reproduce the paper's row:
//  1. The soft-foot biped stays finite and standing under the controller and is
//     bit-for-bit deterministic across runs.
//  2. Soft feet maintain at least as many ground contact points as rigid feet
//     over a settled window (the paper's headline: "maintaining more ground
//     contact points") -- here a large, robust margin.
//  3. Soft feet withstand at least as large a recoverable pelvis push as rigid
//     feet (the paper's "withstands larger perturbations").
//
// A fourth gate covers the scene's Reset action, which has to restore the soft
// feet's independent point-mass state and the gait phase, not just the
// skeleton's generalized coordinates.

#include "examples/demos/scenes/soft_foot_simbicon/SoftFootSimbiconModel.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <iomanip>
#include <iostream>

#include <cstddef>

namespace sfs = dart_demos::soft_foot_simbicon_model;

namespace {

constexpr std::size_t kStandSteps = 1000;
constexpr std::size_t kContactWindowStart = 500;

struct StandOutcome
{
  bool finiteThroughout = true;
  bool uprightThroughout = true;
  std::size_t failedStep = 0;
  double minUprightGap = 1e9;
  double finalPelvisY = 0.0;
  Eigen::VectorXd state;
  std::size_t contactSum = 0;
  std::size_t contactMin = 1000000;
  std::size_t contactSamples = 0;
};

StandOutcome runStand(sfs::Feet feet, std::size_t steps)
{
  sfs::Model model = sfs::createModel(feet);
  StandOutcome out;
  for (std::size_t s = 1; s <= steps; ++s) {
    sfs::step(model);
    if (!sfs::isFinite(model)) {
      out.finiteThroughout = false;
      out.failedStep = s;
      return out;
    }
    const double gap = sfs::pelvisHeightY(model) - sfs::meanFootHeightY(model);
    out.minUprightGap = std::min(out.minUprightGap, gap);
    if (!sfs::isUpright(model)) {
      out.uprightThroughout = false;
      if (out.failedStep == 0)
        out.failedStep = s;
    }
    if (s >= kContactWindowStart) {
      const std::size_t c = sfs::footContactCount(model);
      out.contactSum += c;
      out.contactMin = std::min(out.contactMin, c);
      ++out.contactSamples;
    }
  }
  out.finalPelvisY = sfs::pelvisHeightY(model);
  out.state = sfs::stateVector(model);
  return out;
}

double contactAvg(const StandOutcome& o)
{
  return o.contactSamples ? static_cast<double>(o.contactSum) / o.contactSamples
                          : 0.0;
}

} // namespace

//==============================================================================
// Gate 1: the soft-foot biped stays finite + standing under the controller for
// kStandSteps, and two runs are bit-for-bit identical.
TEST(SoftFootSimbiconModelTest, SoftFootBipedStandsAndIsDeterministic)
{
  const StandOutcome first = runStand(sfs::Feet::Soft, kStandSteps);
  ASSERT_TRUE(first.finiteThroughout)
      << "soft biped became non-finite at step " << first.failedStep;
  ASSERT_TRUE(first.uprightThroughout)
      << "soft biped stopped being upright at step " << first.failedStep
      << " (min pelvis-above-feet gap " << first.minUprightGap << " m)";

  const StandOutcome second = runStand(sfs::Feet::Soft, kStandSteps);
  ASSERT_TRUE(second.finiteThroughout);

  // Exact comparison of the whole state -- skeleton positions and velocities
  // plus every soft point mass's positions and velocities. Reducing this to a
  // scalar would let two genuinely different states compare equal.
  ASSERT_EQ(second.state.size(), first.state.size());
  ASSERT_TRUE((second.state.array() == first.state.array()).all())
      << "soft biped run was not deterministic; largest component difference "
      << (second.state - first.state).cwiseAbs().maxCoeff();
  ASSERT_EQ(second.finalPelvisY, first.finalPelvisY);

  std::cout << std::setprecision(6)
            << "soft_foot_simbicon steps=" << kStandSteps
            << " min_upright_gap=" << first.minUprightGap
            << " final_pelvisY=" << first.finalPelvisY
            << " state_components=" << first.state.size()
            << " deterministic=true\n";
}

//==============================================================================
// Gate 1b: Reset restores the complete state, not just the skeleton's
// generalized coordinates. The soft feet carry deformation and momentum in
// independent point masses, so a reset that skipped them would resume from a
// nominal pose with squashed, still-moving feet.
TEST(SoftFootSimbiconModelTest, ResetRestoresTheFullStartingState)
{
  sfs::Model fresh = sfs::createModel(sfs::Feet::Soft);
  const Eigen::VectorXd startState = sfs::stateVector(fresh);

  sfs::Model model = sfs::createModel(sfs::Feet::Soft);
  sfs::applyPush(model, Eigen::Vector3d(0.0, 0.0, 400.0), 100);
  for (int s = 0; s < 400; ++s)
    sfs::step(model);

  // The run has to have actually disturbed the feet, or the reset below would
  // be trivially satisfied.
  const Eigen::VectorXd disturbed = sfs::stateVector(model);
  ASSERT_EQ(disturbed.size(), startState.size());
  ASSERT_GT((disturbed - startState).cwiseAbs().maxCoeff(), 1e-6);

  sfs::resetModel(model);

  const Eigen::VectorXd resetState = sfs::stateVector(model);
  EXPECT_LT((resetState - startState).cwiseAbs().maxCoeff(), 1e-12)
      << "reset did not restore the starting state";
  EXPECT_EQ(model.forceDuration, 0);
  EXPECT_EQ(model.externalForce, Eigen::Vector3d::Zero());

  std::cout << "soft_foot_simbicon reset  disturbed_by="
            << (disturbed - startState).cwiseAbs().maxCoeff()
            << "  residual_after_reset="
            << (resetState - startState).cwiseAbs().maxCoeff() << "\n";
}

//==============================================================================
// Gate 2 (paper: soft maintains more ground contact points): soft feet keep at
// least as many foot contacts as rigid feet over the settled window.
TEST(SoftFootSimbiconModelTest, SoftMaintainsAtLeastRigidFootContacts)
{
  const StandOutcome rigid = runStand(sfs::Feet::Rigid, kStandSteps);
  const StandOutcome soft = runStand(sfs::Feet::Soft, kStandSteps);
  ASSERT_TRUE(rigid.finiteThroughout);
  ASSERT_TRUE(soft.finiteThroughout);

  // Both bipeds must still be standing. A toppled rigid biped also reports few
  // foot contacts, so without this the comparison below could "pass" by
  // measuring a fall rather than the contact spreading the paper describes.
  ASSERT_TRUE(rigid.uprightThroughout) << "rigid biped fell; contact totals "
                                          "would not be comparable";
  ASSERT_TRUE(soft.uprightThroughout)
      << "soft biped fell; contact totals would "
         "not be comparable";

  const double rigidAvg = contactAvg(rigid);
  const double softAvg = contactAvg(soft);

  std::cout << std::setprecision(4) << "foot_contacts window=["
            << kContactWindowStart << "," << kStandSteps
            << "]  rigid avg=" << rigidAvg << " min=" << rigid.contactMin
            << "  soft avg=" << softAvg << " min=" << soft.contactMin << "\n";

  EXPECT_GE(soft.contactSum, rigid.contactSum)
      << "soft total foot contacts " << soft.contactSum << " < rigid total "
      << rigid.contactSum;
  EXPECT_GE(softAvg, rigidAvg);
}

//==============================================================================
// Gate 3 (paper: soft withstands larger perturbations): the largest recoverable
// lateral pelvis push for soft feet is at least that for rigid feet.
TEST(SoftFootSimbiconModelTest, SoftWithstandsAtLeastRigidPush)
{
  const double rigidPush = sfs::maxRecoverablePush(sfs::Feet::Rigid);
  const double softPush = sfs::maxRecoverablePush(sfs::Feet::Soft);

  std::cout << "max_recoverable_push  rigid=" << rigidPush
            << " N  soft=" << softPush << " N\n";

  // The sweep must have found a genuinely recoverable push (not an all-fail
  // sweep), otherwise the >= comparison would be vacuous.
  ASSERT_GT(rigidPush, 0.0)
      << "rigid biped recovered no push in the sweep -- sweep range too high";
  ASSERT_GT(softPush, 0.0);
  EXPECT_GE(softPush, rigidPush) << "soft recoverable push " << softPush
                                 << " N < rigid " << rigidPush << " N";
}
