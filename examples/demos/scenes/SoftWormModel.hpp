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

#ifndef DART_EXAMPLES_DEMOS_SCENES_SOFTWORMMODEL_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_SOFTWORMMODEL_HPP_

#include <dart/dart.hpp>

#include <cstddef>

namespace dart_demos {
namespace soft_worm_model {

inline constexpr std::size_t kNumLinks = 5;
inline constexpr double kGaitAmplitude = 0.56;
inline constexpr double kGaitFrequencyHz = 1.35;

/// Builds the complete programmatic soft-worm world and returns its worm.
dart::simulation::WorldPtr createWorld(dart::dynamics::SkeletonPtr& worm);

/// Applies one traveling-wave servo command at the supplied simulation time.
void applyGait(
    const dart::dynamics::SkeletonPtr& worm, double time, bool enabled);

/// Returns the world-frame x position of the worm's root body.
double getRootX(const dart::dynamics::SkeletonPtr& worm);

/// Returns a deterministic weighted checksum of skeleton and soft-point state.
double positionChecksum(const dart::dynamics::SkeletonPtr& worm);

/// Returns whether all skeleton and soft-point state remains finite.
bool isFinite(const dart::dynamics::SkeletonPtr& worm);

} // namespace soft_worm_model
} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_SOFTWORMMODEL_HPP_
