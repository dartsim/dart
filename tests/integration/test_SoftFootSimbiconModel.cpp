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
  double checksum = 0.0;
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
  out.checksum = sfs::checksum(model);
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
  ASSERT_EQ(second.checksum, first.checksum)
      << "soft biped run was not deterministic";
  ASSERT_EQ(second.finalPelvisY, first.finalPelvisY);

  std::cout << std::setprecision(6)
            << "soft_foot_simbicon steps=" << kStandSteps
            << " min_upright_gap=" << first.minUprightGap
            << " final_pelvisY=" << first.finalPelvisY
            << " checksum=" << std::setprecision(17) << first.checksum
            << " deterministic=true\n";
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
