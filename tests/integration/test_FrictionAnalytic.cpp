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

// T0 of the friction-solver evaluation (tools/friction_eval): analytic scenes
// for the built-in backends. Default rows document the box law, Coulomb rows
// use the exact references, and the associated ablation checks its own law
// certificate because gliding changes the sliding trajectories.

#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/FbfFrictionSolver.hpp"
#include "dart/constraint/NsgsFrictionSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "friction_scenes.hpp"

#include <gtest/gtest.h>

namespace fe = friction_eval;

namespace {

enum class Backend
{
  Dantzig,
  Pgs,
  NsgsCoulomb,
  NsgsAssociated,
  NsgsBox,
  Fbf
};

class FrictionAnalytic : public ::testing::TestWithParam<Backend>
{
protected:
  fe::Metrics run(const std::string& id, const fe::Params& params)
  {
    auto scene = fe::scenes().at(id)(params, 1e-3);
    auto* solver = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(
        scene.world->getConstraintSolver());
    // gz's detector when built, else FCL (always built).
    solver->setCollisionDetector(fe::makeDetector(HAVE_ODE ? "ode" : "fcl"));
    std::shared_ptr<dart::constraint::NsgsFrictionSolver> nsgs;
    std::shared_ptr<dart::constraint::FbfFrictionSolver> fbf;
    if (GetParam() == Backend::Pgs) {
      solver->setBoxedLcpSolver(
          std::make_shared<dart::constraint::PgsBoxedLcpSolver>());
    } else if (GetParam() == Backend::Dantzig) {
      solver->setBoxedLcpSolver(
          std::make_shared<dart::constraint::DantzigBoxedLcpSolver>());
    } else if (GetParam() == Backend::Fbf) {
      dart::constraint::FbfFrictionSolver::Options options;
      options.tolerance = 1e-7;
      options.maxOuterIterations = 1000;
      fbf = std::make_shared<dart::constraint::FbfFrictionSolver>(options);
      solver->setBoxedLcpSolver(fbf);
    } else {
      using Nsgs = dart::constraint::NsgsFrictionSolver;
      Nsgs::Options options;
      options.law = GetParam() == Backend::NsgsCoulomb ? Nsgs::Law::Coulomb
                    : associated()                     ? Nsgs::Law::Associated
                                                       : Nsgs::Law::Box;
      options.maxSweeps = 1000;
      // Tight targets preserve the physical oracles; associated sliding uses
      // the public target because redundant contacts converge slowly.
      options.tolerance = associated() && id != "R1" ? 1e-5 : 1e-7;
      nsgs = std::make_shared<Nsgs>(options);
      solver->setBoxedLcpSolver(nsgs);
    }
    auto metrics = fe::run(scene);
    const auto expectStats = [&](const auto& stats) {
      EXPECT_GT(stats.numSolves, 0u) << id;
      EXPECT_EQ(stats.numFailed, 0u) << id;
      EXPECT_EQ(stats.numSolves, stats.numConverged + stats.numAcceptedAtCap)
          << id;
      EXPECT_LE(stats.maxViolation, 1e-5) << id;
    };
    if (nsgs) {
      const auto stats = nsgs->getStats();
      expectStats(stats);
      EXPECT_EQ(
          stats.numBoxContacts,
          GetParam() == Backend::NsgsBox ? stats.numContacts : 0u)
          << id;
    }
    if (fbf) {
      const auto stats = fbf->getStats();
      expectStats(stats);
      EXPECT_EQ(stats.numStepShrinks, 0u) << id;
      EXPECT_EQ(stats.numBoxContacts, 0u) << id;
    }
    // Maxima over steps skip NaN, so a non-finite state must fail here.
    for (std::size_t i = 0; i < scene.world->getNumSkeletons(); ++i) {
      const auto skeleton = scene.world->getSkeleton(i);
      EXPECT_TRUE(
          skeleton->getPositions().allFinite()
          && skeleton->getVelocities().allFinite())
          << id << ": " << skeleton->getName();
    }
    return metrics;
  }

  bool pgs() const
  {
    return GetParam() == Backend::Pgs;
  }

  bool associated() const
  {
    return GetParam() == Backend::NsgsAssociated;
  }

  bool boxLaw() const
  {
    return GetParam() == Backend::Dantzig || pgs()
           || GetParam() == Backend::NsgsBox;
  }
};

} // namespace

// Horizons are short so the suite fits the Debug CI budget (60 s).

// A1: sliding along a basis axis, where box and exact Coulomb agree.
TEST_P(FrictionAnalytic, InclineAlongTheBasis)
{
  auto m = run("A1", {{"mu", 0.3}, {"T", 0.3}});
  EXPECT_EQ(m.at("slides"), 1.0);
  if (!associated()) {
    EXPECT_NEAR(m.at("accel") / m.at("pred_exact_accel"), 1.0, 1e-3);
  }
  m = run("A1", {{"mu", 0.6}, {"T", 0.3}});
  EXPECT_EQ(m.at("slides"), 0.0);
  EXPECT_LT(m.at("creep"), 1e-5);
}

// A4: the box holds up to 1/max(|cos|, |sin|) = 1.414 mu m g at 45 degrees,
// and a push 30 degrees off the basis saturates one row only (axis snapping).
TEST_P(FrictionAnalytic, IsotropyPush)
{
  auto m = run("A4", {{"phi", 45.0}, {"k", 1.2}, {"T", 0.2}});
  EXPECT_EQ(m.at("pred_exact_slides"), 1.0);
  EXPECT_EQ(m.at("pred_box_slides"), 0.0);
  EXPECT_EQ(m.at("slides"), boxLaw() ? 0.0 : 1.0);
  m = run("A4", {{"phi", 30.0}, {"k", 1.5}, {"T", 0.2}});
  EXPECT_NEAR(
      m.at("force_ratio"), boxLaw() ? m.at("pred_box_force_ratio") : 1.0, 1e-3);
  EXPECT_NEAR(
      m.at("vel_dir_err_deg"),
      boxLaw() ? m.at("pred_box_vel_dir_err_deg") : 0.0,
      0.1);
}

// A5: each axis decelerates on its own: the stop distance scales by
// sqrt(cos^4 + sin^4) and a 30-degree launch ends near atan(tan^2) = 18.4.
// After the stop, PGS's approximate solve leaves about a micrometer of creep,
// so the stop check uses A1's 1e-5 m.
TEST_P(FrictionAnalytic, SlideToStop)
{
  if (GetParam() == Backend::Fbf)
    GTEST_SKIP()
        << "FBF runs only the A1, A4, A7 and A10 rows (T0 runtime budget)";
  for (const double phi : {30.0, 45.0}) {
    auto m = run("A5", {{"phi", phi}, {"v0", 1.0}, {"T", 0.25}});
    if (!associated()) {
      EXPECT_NEAR(
          m.at("dist_ratio"),
          boxLaw() ? m.at("pred_box_dist_ratio") : 1.0,
          1e-3)
          << phi;
      EXPECT_LT(m.at("creep"), 1e-5) << phi;
    }
    EXPECT_NEAR(
        m.at("dir_deg"),
        boxLaw() ? m.at("pred_box_dir_deg") : m.at("pred_exact_dir_deg"),
        0.1)
        << phi;
  }
}

// A7: a single contact in the plane of motion; v_roll is conserved exactly.
TEST_P(FrictionAnalytic, BackspinSphere)
{
  for (const auto& params :
       {fe::Params{{"v0", 4.0}, {"w0", 0.0}, {"T", 0.3}},
        fe::Params{{"v0", 1.0}, {"w0", -20.0}, {"T", 0.4}}}) {
    auto m = run("A7", params);
    if (associated()) {
      // Associated sliding on the circular cone glides: u_n = mu |u_t|.
      EXPECT_NEAR(m.at("initial_dilatancy"), 1.0, 1e-3);
    } else {
      EXPECT_LT(m.at("v_roll_err"), 1e-4);
      EXPECT_NEAR(m.at("roll_step"), m.at("pred_roll_step"), 1.0);
    }
  }
}

// A10: surface velocity; the box reaches the belt's signed velocity in the
// friction frame, syncing each axis on its own, in max(|cos|, |sin|) of the
// exact time.
TEST_P(FrictionAnalytic, Conveyor)
{
  for (const double beta : {0.0, 45.0}) {
    auto m = run("A10", {{"beta", beta}, {"mu", 0.6}, {"T", 0.2}});
    EXPECT_LT(m.at("v_err"), 1e-5) << beta;
    if (!associated()) {
      EXPECT_NEAR(
          m.at("sync_time"),
          boxLaw() ? m.at("pred_box_sync_time") : m.at("pred_exact_sync_time"),
          1.5e-3)
          << beta;
    }
  }
}

// A13: gz-physics' slip-compliance expectation, v = slip F within 1e-4.
TEST_P(FrictionAnalytic, SlipCompliance)
{
  if (GetParam() == Backend::Fbf)
    GTEST_SKIP()
        << "FBF runs only the A1, A4, A7 and A10 rows (T0 runtime budget)";
  for (const double dir : {0.0, 1.0}) {
    const auto m = run("A13", {{"slip", 0.05}, {"dir", dir}, {"T", 0.4}});
    EXPECT_LT(m.at("v_err"), 1e-4) << dir;
  }
}

// C1: the box tips iff mu > w/h = 0.5 and transfers (1 + mu h/w)/2 of its
// weight to the front while sliding. Dantzig's friction bounds, frozen at the
// frictionless normal impulses, keep it upright at mu = 0.6.
TEST_P(FrictionAnalytic, PainleveBox)
{
  if (GetParam() == Backend::Fbf)
    GTEST_SKIP()
        << "FBF runs only the A1, A4, A7 and A10 rows (T0 runtime budget)";
  auto m = run("C1", {{"mu", 0.4}, {"v0", 1.5}, {"T", 0.4}});
  if (!associated()) {
    EXPECT_EQ(m.at("tipped"), 0.0);
    EXPECT_NEAR(m.at("front_share"), m.at("pred_front_share"), 1e-3);
  }
  m = run("C1", {{"mu", 0.6}, {"v0", 1.5}, {"T", 0.4}});
  EXPECT_EQ(m.at("pred_tips"), 1.0);
  if (!associated()) {
    EXPECT_EQ(m.at("tipped"), GetParam() == Backend::Dantzig ? 0.0 : 1.0);
  }
}

// R1: a resting stack; PGS30 truncation lets it drift.
TEST_P(FrictionAnalytic, Stack)
{
  if (GetParam() == Backend::Fbf)
    GTEST_SKIP()
        << "FBF runs only the A1, A4, A7 and A10 rows (T0 runtime budget)";
  auto m = run("R1", {{"n", 2.0}, {"T", 0.3}});
  EXPECT_LT(m.at("max_disp"), 1e-3);
  EXPECT_LT(m.at("top_drift"), pgs() ? 1e-4 : 1e-6);
}

INSTANTIATE_TEST_SUITE_P(
    Backends,
    FrictionAnalytic,
    ::testing::Values(
        Backend::Dantzig,
        Backend::Pgs,
        Backend::NsgsCoulomb,
        Backend::NsgsAssociated,
        Backend::NsgsBox,
        Backend::Fbf),
    [](const ::testing::TestParamInfo<Backend>& info) {
      switch (info.param) {
        case Backend::Dantzig:
          return "Dantzig";
        case Backend::Pgs:
          return "Pgs";
        case Backend::NsgsCoulomb:
          return "NsgsCoulomb";
        case Backend::NsgsAssociated:
          return "NsgsAssociated";
        case Backend::NsgsBox:
          return "NsgsBox";
        case Backend::Fbf:
          return "Fbf";
      }
      return "Unknown";
    });
