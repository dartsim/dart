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

#pragma once

#include <Eigen/Geometry>

#include <random>

namespace dart::benchmark::collision {

inline std::mt19937 MakeDeterministicRng(unsigned int seed)
{
  return std::mt19937(seed);
}

inline Eigen::Vector3d RandomPosition(std::mt19937& rng, double range)
{
  std::uniform_real_distribution<double> dist(-range, range);
  return Eigen::Vector3d(dist(rng), dist(rng), dist(rng));
}

inline Eigen::Isometry3d RandomTransform(std::mt19937& rng, double range)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = RandomPosition(rng, range);
  return tf;
}

inline Eigen::Isometry3d RandomTransformWithRotation(
    std::mt19937& rng, double range)
{
  constexpr double kTwoPi = 6.28318530717958647692;
  std::uniform_real_distribution<double> angle_dist(0.0, kTwoPi);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = RandomPosition(rng, range);
  tf.linear() = Eigen::AngleAxisd(angle_dist(rng), Eigen::Vector3d::UnitZ())
                    .toRotationMatrix();
  return tf;
}

} // namespace dart::benchmark::collision
