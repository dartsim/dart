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

#ifndef DART_EXAMPLES_DEMOS_SCENES_ADAPTIVESOFTCONTACTMODEL_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_ADAPTIVESOFTCONTACTMODEL_HPP_

#include <dart/dart.hpp>

#include <cstddef>

namespace dart_demos {
namespace adaptive_soft_contact_model {

inline constexpr double kTimeStep = 0.001;
inline constexpr double kSoftBodyRadius = 0.36;
inline constexpr double kCompareTolerance = 0.25;
inline constexpr std::size_t kDefaultRingCount = 1;
inline constexpr std::size_t kDefaultLingerSteps = 12;

/// GUI-free state for the adaptive soft-contact simulation.
struct Model
{
  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr softSkeleton;
  dart::dynamics::SkeletonPtr pusherSkeleton;
  dart::dynamics::SoftBodyNode* softBody = nullptr;
  dart::dynamics::FreeJoint* pusherJoint = nullptr;
};

/// Deterministic scalar summaries of the model state.
struct Checksum
{
  double skeletonPositionL1 = 0.0;
  double skeletonPositionSquared = 0.0;
  double pointWorldPositionL1 = 0.0;
  double pointWorldPositionSquared = 0.0;
  std::size_t active = 0;
  std::size_t total = 0;
  std::size_t contacts = 0;
  bool finite = true;
};

/// Builds the complete programmatic adaptive soft-contact world.
Model createModel(
    bool adaptiveEnabled = true,
    std::size_t ringCount = kDefaultRingCount,
    std::size_t lingerSteps = kDefaultLingerSteps);

/// Applies adaptive-activation settings without rebuilding the world.
void configure(
    Model& model,
    bool adaptiveEnabled,
    std::size_t ringCount,
    std::size_t lingerSteps);

/// Moves the periodic pusher to the pose for the supplied simulation time.
void setPusherPose(Model& model, double time);

/// Applies the controller state required immediately before World::step().
void prepareStep(Model& model);

/// Advances one controlled step and reports whether the state remains finite.
bool step(Model& model);

/// Returns deterministic scalar summaries of the current model state.
Checksum computeChecksum(const Model& model);

/// Returns a weighted checksum of skeleton and soft-point positions.
double positionChecksum(const Model& model);

/// Returns whether all skeleton and soft-point state remains finite.
bool isFinite(const Model& model);

/// Returns the surface-pose delta used by the standalone comparison contract.
double surfacePoseDelta(const Model& lhs, const Model& rhs);

} // namespace adaptive_soft_contact_model
} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_ADAPTIVESOFTCONTACTMODEL_HPP_
