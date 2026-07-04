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

#ifndef DART_UTILS_SDF_WRITER_HPP_
#define DART_UTILS_SDF_WRITER_HPP_

#include <dart/utils/export.hpp>

#include <dart/dynamics/skeleton.hpp>

#include <dart/common/result.hpp>

#include <string>

namespace dart {
namespace utils {

namespace SdfParser {

/// Options for serializing a Skeleton to SDF.
struct DART_UTILS_API WriteOptions
{
  /// SDF version to write on the root <sdf> element.
  std::string version{"1.7"};

  /// Include ShapeNodes with VisualAspect as <visual> entries.
  bool includeVisuals{true};

  /// Include ShapeNodes with CollisionAspect as <collision> entries.
  bool includeCollisions{true};
};

/// Serialize a Skeleton to an SDF string.
///
/// The first writer slice intentionally supports a conservative DART subset:
/// BodyNode links, root FreeJoint/WeldJoint placement, revolute/prismatic/weld
/// and screw/universal child joints with passive dynamics metadata (damping,
/// Coulomb friction, spring reference, and spring stiffness), screw thread
/// pitch, topology-only ball child joints, link gravity mode, inertial
/// parameters, local joint/shape poses, and box/sphere/cylinder/mesh
/// visual/collision geometry with explicit visual material colors and absolute
/// non-file mesh URI preservation.
/// Unsupported constructs return an error instead of being silently dropped.
common::Result<std::string, common::Error> DART_UTILS_API
tryWriteSkeletonToString(
    const dynamics::Skeleton& skeleton,
    const WriteOptions& options = WriteOptions());

} // namespace SdfParser

} // namespace utils
} // namespace dart

#endif // DART_UTILS_SDF_WRITER_HPP_
