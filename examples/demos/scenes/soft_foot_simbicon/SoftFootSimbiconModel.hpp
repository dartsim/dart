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

#ifndef DART_EXAMPLES_DEMOS_SCENES_SOFT_FOOT_SIMBICON_SOFTFOOTSIMBICONMODEL_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_SOFT_FOOT_SIMBICON_SOFTFOOTSIMBICONMODEL_HPP_

#include <dart/dart.hpp>

#include <memory>

#include <cstddef>

namespace dart_demos {

namespace atlas_simbicon {
class Controller;
} // namespace atlas_simbicon

namespace soft_foot_simbicon_model {

inline constexpr double kTimeStep = 0.001;

/// Duration (steps) of a scheduled pelvis push, matching the AtlasSimbicon
/// scene's own 100-step push window.
inline constexpr int kPushSteps = 100;

/// Default push magnitude (N) the interactive scene's push keys apply.
inline constexpr double kDefaultPushMagnitude = 500.0;

/// Minimum pelvis-above-feet height (m) for the biped to count as still
/// standing. Measured as pelvisY - mean(footY) rather than an absolute pelvis Y
/// because a hard push slides the whole biped sideways while it stays vertical,
/// so absolute pelvis height alone cannot tell "standing but displaced" from
/// "collapsed". A stable stance holds this near ~0.8 m; a toppled biped drops
/// it toward or below zero.
inline constexpr double kUprightGap = 0.40;

/// Which foot geometry the Atlas biped is loaded with.
enum class Feet
{
  Rigid, ///< rigid box feet (atlas_v3_no_head.sdf)
  Soft,  ///< SoftBodyNode feet (atlas_v3_no_head_soft_feet.sdf)
};

/// GUI-free state for the soft-foot SIMBICON biped: the Atlas-on-ground world,
/// its SIMBICON controller, and the handles the free-functions below drive and
/// measure. The controller math hardcodes world-Y as vertical, so the world is
/// built Y-up (see createModel).
struct Model
{
  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr atlas;
  std::shared_ptr<atlas_simbicon::Controller> controller;
  dart::dynamics::BodyNode* pelvis = nullptr;
  dart::dynamics::BodyNode* leftFoot = nullptr;
  dart::dynamics::BodyNode* rightFoot = nullptr;
  Eigen::Vector3d externalForce = Eigen::Vector3d::Zero();
  int forceDuration = 0;
  Feet feet = Feet::Rigid;
};

/// Builds the Atlas-on-ground SIMBICON world for the requested foot geometry.
Model createModel(Feet feet);

/// Schedules a pelvis push: `force` (N, world frame) applied for `steps` steps.
void applyPush(Model& model, const Eigen::Vector3d& force, int steps);

/// Applies the control the biped needs immediately before World::step(): the
/// pending pelvis push, one SIMBICON controller update, and the push-window
/// countdown. Used by the interactive scene's preStep (which steps the world
/// itself).
void prepareStep(Model& model);

/// Advances the model one step: prepareStep() followed by World::step().
void step(Model& model);

/// World-frame Y (vertical) coordinate of the pelvis.
double pelvisHeightY(const Model& model);

/// Mean world-frame Y of the two feet.
double meanFootHeightY(const Model& model);

/// Whether the biped is still standing: pelvis at least kUprightGap above the
/// feet (see kUprightGap). Robust to the biped sliding sideways under a push.
bool isUpright(const Model& model);

/// Number of contact points on the left or right foot in the last step.
std::size_t footContactCount(const Model& model);

/// Whether all skeleton and soft-point state remains finite.
bool isFinite(const Model& model);

/// The model's full simulation state, laid out as skeleton positions, skeleton
/// velocities, then each soft point mass's positions and velocities.
///
/// Returned as a vector rather than reduced to a scalar so a determinism check
/// can compare states exactly: any summary that folds the components together
/// admits collisions between genuinely different states.
Eigen::VectorXd stateVector(const Model& model);

/// Resets the biped to its starting state: skeleton configuration, soft-foot
/// point-mass state, gait phase, and any pending push.
void resetModel(Model& model);

/// Bounds of the lateral push sweep maxRecoverablePush() searches. Exported so
/// a caller can tell a measured threshold from one that saturated the ceiling.
constexpr double kPushSweepStart = 2000.0;
constexpr double kPushSweepEnd = 24000.0;
constexpr double kPushSweepStep = 2000.0;

/// Largest pelvis push magnitude (N) from which the biped recovers upright,
/// found by sweeping magnitudes in a fixed lateral direction and returning the
/// largest that leaves the biped upright after a fixed recovery window.
double maxRecoverablePush(Feet feet);

} // namespace soft_foot_simbicon_model
} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_SOFT_FOOT_SIMBICON_SOFTFOOTSIMBICONMODEL_HPP_
