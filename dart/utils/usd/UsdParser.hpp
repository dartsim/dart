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

#ifndef DART_UTILS_USD_USDPARSER_HPP_
#define DART_UTILS_USD_USDPARSER_HPP_

#include <dart/simulation/World.hpp>

#include <dart/dynamics/Skeleton.hpp>

#include <dart/common/ResourceRetriever.hpp>

namespace dart {
namespace utils {
namespace UsdParser {

/// Root joint type for the articulation root when the USD file does not
/// explicitly connect a rigid body to a parent.
enum class RootJointType
{
  /// Create a floating base (FreeJoint).
  FLOATING = 0,

  /// Create a fixed base (WeldJoint).
  FIXED = 1,
};

struct Options
{
  /// Resource retriever used to resolve the stage and its nested assets. If
  /// nullptr, a CompositeResourceRetriever with file/package/dart schemas is
  /// used.
  common::ResourceRetrieverPtr mResourceRetriever;

  /// Desired root joint type for unparented bodies.
  RootJointType mDefaultRootJointType;

  /// Whether all fixed joints should be preserved in the skeleton instead of
  /// collapsing into a single rigid body tree.
  bool mPreserveFixedJoints;

  /// Emit extra parser diagnostics to the logger.
  bool mVerbose;

  Options(
      common::ResourceRetrieverPtr resourceRetriever = nullptr,
      RootJointType defaultRootJointType = RootJointType::FLOATING,
      bool preserveFixedJoints = true,
      bool verbose = false);
};

/// Reads a USD stage and converts every articulation root into a DART skeleton
/// that is attached to the returned world.
simulation::WorldPtr readWorld(
    const common::Uri& uri, const Options& options = Options());

/// Reads the first articulation found in the USD stage and converts it into a
/// skeleton. If multiple articulations exist, the first one discovered during
/// stage traversal is returned.
dynamics::SkeletonPtr readSkeleton(
    const common::Uri& uri, const Options& options = Options());

} // namespace UsdParser
} // namespace utils
} // namespace dart

#endif // DART_UTILS_USD_USDPARSER_HPP_
