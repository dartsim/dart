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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include <dart/simulation/experimental/export.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>

#include <dart/io/export.hpp>

#include <string>
#include <vector>

namespace dart::common {
struct Uri;
} // namespace dart::common

namespace dart::dynamics {
class Skeleton;
} // namespace dart::dynamics

namespace dart::io {
struct DART_IO_API ReadOptions;
} // namespace dart::io

namespace dart::simulation::experimental {
class World;
} // namespace dart::simulation::experimental

namespace dart::simulation {
class World;
} // namespace dart::simulation

namespace dart::simulation::experimental::io {

/// Options for translating a legacy `dynamics::Skeleton` into an experimental
/// `World`.
///
/// The bridge supports tree skeletons whose joints map to the current
/// experimental multibody facade (`WeldJoint`, `RevoluteJoint`,
/// `PrismaticJoint`, `ScrewJoint`, `UniversalJoint`, `BallJoint`,
/// `PlanarJoint`, and `FreeJoint`). File-format parsers can compose this with
/// `dart::io::readSkeleton()` without making simulation-experimental depend on
/// parser-specific libraries.
struct SkeletonLoadOptions
{
  /// Prefix for synthetic fixed root links used to anchor legacy root joints.
  ///
  /// The classic Skeleton stores the root body behind a root joint to the
  /// world. The experimental multibody facade represents joints between links,
  /// so each legacy root body gets a logical anchor link.
  std::string rootAnchorPrefix = "root_anchor_";
};

/// Add a translated copy of `skeleton` to `world` and return the created
/// experimental multibody.
///
/// Supported by this bridge:
/// - body/link names, mass, inertia about COM, and local COM offsets;
/// - Weld/Revolute/Prismatic/Screw/Universal/Ball/Planar/Free joint type,
///   axes, screw pitch, fixed offsets, state, limits, actuator family, passive
///   spring/damping/friction, and commanded effort or velocity;
/// - tree roots represented through synthetic anchor links.
/// - one centered collidable Box/Sphere/Capsule/Mesh collision shape per link
///   when the legacy shape maps exactly to the experimental collision-shape
///   facade.
///
/// Ball and Free joints must use the exp-map coordinate chart. Throws
/// `InvalidArgumentException` when a Skeleton uses a joint family or coordinate
/// chart that the bridge does not yet translate.
DART_EXPERIMENTAL_API Multibody addSkeleton(
    World& world,
    const dynamics::Skeleton& skeleton,
    const SkeletonLoadOptions& options = SkeletonLoadOptions{});

/// Read a legacy Skeleton from `uri`, add a translated copy to `world`, and
/// return the created experimental multibody.
///
/// This composes `dart::io::readSkeleton()` with the parsed-Skeleton bridge
/// above using the default reader configuration. Use the `ReadOptions` overload
/// for parser-specific reader configuration. Shape offsets/multiple shapes and
/// structured diagnostics remain future model-loading work.
///
/// Throws `InvalidArgumentException` when parsing fails or when the parsed
/// Skeleton uses a joint family or coordinate chart that the bridge does not
/// yet translate.
DART_EXPERIMENTAL_API Multibody addSkeleton(
    World& world,
    const common::Uri& uri,
    const SkeletonLoadOptions& options = SkeletonLoadOptions{});

/// Read a legacy Skeleton from `uri` with explicit `dart::io::ReadOptions`, add
/// a translated copy to `world`, and return the created experimental multibody.
///
/// Use this overload when the caller must force a parser format, customize SDF
/// default root-joint handling, or pass reader-level resource/package
/// resolution options before the parsed Skeleton is translated.
DART_EXPERIMENTAL_API Multibody addSkeleton(
    World& world,
    const common::Uri& uri,
    const ::dart::io::ReadOptions& readOptions,
    const SkeletonLoadOptions& options = SkeletonLoadOptions{});

/// Add translated copies of every Skeleton in `sourceWorld` to `world` and
/// return the created experimental multibodies in source-world order.
///
/// Throws `InvalidArgumentException` before mutating `world` when any source
/// Skeleton cannot be translated by `addSkeleton`.
DART_EXPERIMENTAL_API std::vector<Multibody> addWorld(
    World& world,
    const ::dart::simulation::World& sourceWorld,
    const SkeletonLoadOptions& options = SkeletonLoadOptions{});

/// Read a legacy World from `uri`, add translated copies of every parsed
/// Skeleton to `world`, and return the created experimental multibodies.
///
/// This composes `dart::io::readWorld()` with `addWorld()` using the default
/// reader configuration. Use the `ReadOptions` overload for parser-specific
/// reader configuration. Structured diagnostics and a richer load-result object
/// remain future model-loading work.
DART_EXPERIMENTAL_API std::vector<Multibody> addWorld(
    World& world,
    const common::Uri& uri,
    const SkeletonLoadOptions& options = SkeletonLoadOptions{});

/// Read a legacy World from `uri` with explicit `dart::io::ReadOptions`, add
/// translated copies of every parsed Skeleton to `world`, and return the
/// created experimental multibodies.
DART_EXPERIMENTAL_API std::vector<Multibody> addWorld(
    World& world,
    const common::Uri& uri,
    const ::dart::io::ReadOptions& readOptions,
    const SkeletonLoadOptions& options = SkeletonLoadOptions{});

} // namespace dart::simulation::experimental::io
