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

#ifndef DART_IO_URDF_WRITER_HPP_
#define DART_IO_URDF_WRITER_HPP_

#include <dart/dynamics/skeleton.hpp>

#include <dart/common/result.hpp>

#include <dart/io/export.hpp>

#include <string>

namespace dart::io::UrdfWriter {

/// Options for serializing a Skeleton to URDF.
struct DART_IO_API WriteOptions
{
  /// Include ShapeNodes with VisualAspect as <visual> entries.
  bool includeVisuals{true};

  /// Include ShapeNodes with CollisionAspect as <collision> entries.
  bool includeCollisions{true};
};

/// Serialize a Skeleton to a URDF string.
///
/// The first writer slice intentionally supports a conservative URDF tree:
/// one root link with identity FreeJoint/WeldJoint root placement, child
/// revolute/continuous/prismatic/fixed joints whose child link frame
/// coincides with the joint frame, continuous joint velocity/effort limit
/// metadata, standard-plane planar and floating child joints with uniform
/// scalar limit/dynamics metadata, single-DoF motor-style mimic metadata,
/// zero-offset coupler mimic metadata through SimpleTransmission entries,
/// passive joint dynamics, inertial data, local visual/collision poses, and
/// box/sphere/cylinder/absolute or package URI mesh geometry. URDF does not
/// serialize parent-joint metadata for the root link, so root joint name/type
/// are controlled by parser defaults on reparse. Explicit visual colors are
/// serialized as URDF materials.
/// Unsupported constructs return an error instead of being silently dropped.
common::Result<std::string, common::Error> DART_IO_API
tryWriteSkeletonToString(const dynamics::Skeleton& skeleton);

/// Serialize a Skeleton to a URDF string with explicit writer options.
common::Result<std::string, common::Error> DART_IO_API tryWriteSkeletonToString(
    const dynamics::Skeleton& skeleton, const WriteOptions& options);

} // namespace dart::io::UrdfWriter

#endif // DART_IO_URDF_WRITER_HPP_
