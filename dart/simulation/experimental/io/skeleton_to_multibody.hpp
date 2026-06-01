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

#include <dart/simulation/experimental/export.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>

#include <dart/dynamics/fwd.hpp>

#include <string>
#include <vector>

namespace dart::simulation {
class World;
} // namespace dart::simulation

namespace dart::simulation::experimental {
class World;
} // namespace dart::simulation::experimental

namespace dart::simulation::experimental::io {

/// Options controlling how a legacy `dynamics::Skeleton` is translated into an
/// experimental `Multibody`.
struct SkeletonToMultibodyOptions
{
  /// Name for the created `Multibody`. Empty uses the skeleton's name, or an
  /// auto-generated name when the skeleton is unnamed.
  std::string name;

  /// Name for the synthetic fixed base link that represents the world frame.
  /// Each of the skeleton's root bodies is attached beneath this base link.
  std::string baseLinkName = "base";

  /// When true (the default), copy the current generalized positions and
  /// velocities of the supported joints into the created multibody. When false,
  /// the multibody is left at the zero configuration.
  bool copyState = true;

  /// When true (the default), copy per-coordinate joint properties — position,
  /// velocity, and effort limits, damping, spring stiffness and rest position,
  /// and Coulomb friction — from revolute and prismatic joints into the created
  /// multibody. Other joint types are left at their defaults.
  bool copyJointProperties = true;

  /// When true (the default), translate the first sphere, box, capsule,
  /// cylinder, or plane collision shape of each body (a shape node with a
  /// collision aspect) onto the created link, preserving the shape node's
  /// relative transform as the CollisionShape local transform. Mesh-like shape
  /// types and additional shapes per body are skipped.
  bool loadCollisionShapes = true;
};

/// Build an experimental `Multibody` from a legacy `dynamics::Skeleton`.
///
/// The translation walks the skeleton's `BodyNode` tree and creates one
/// experimental `Link` per `BodyNode` (mass, inertia about the center of mass,
/// and a center-of-mass offset), connected by the experimental `Joint` that
/// matches the legacy parent `Joint`.
///
/// Frame convention. Each experimental link frame coincides with its legacy
/// body frame, so mass, inertia, center of mass, and the joint axis map across
/// directly. The legacy parent- and child-side joint offsets become the
/// experimental joint's `transformToParent` (= A) and `transformFromParent`
/// (= C^-1), so the relative transform matches legacy exactly:
/// `A · jointMotion(q) · C^-1`. This represents offset joints, offset roots,
/// and branching parents (sibling joints at different locations) without
/// reframing. A synthetic fixed base link represents the world frame; the
/// skeleton's root bodies attach beneath it.
///
/// Supported joints: weld (fixed), revolute, prismatic, screw, universal, ball
/// (spherical), free (floating), and planar, in arbitrary trees on a fixed
/// base. Generalized positions and velocities are copied when
/// `options.copyState` is set (with the screw pitch and the free-joint
/// coordinate order converted to the experimental convention).
///
/// Limitation: the orientation-coordinate joints (ball, free, planar) require
/// an identity-rotation parent- and child-side offset (a translational offset
/// is fine), because their generalized coordinates are not yet re-expressed
/// under a rotated joint frame. The axis-based joints (revolute, prismatic,
/// screw, universal) and weld joints accept arbitrary offsets.
///
/// @param world The experimental world that will own the created multibody.
/// @param skeleton The legacy skeleton to translate.
/// @param options Translation options (name, base link name, state copy).
/// @return A handle to the created `Multibody`.
/// @throws InvalidOperationException if the skeleton uses an unsupported joint
///         type, a rotated offset on a ball/free/planar joint, or a
///         non-positive link mass. The message names the unsupported feature.
DART_EXPERIMENTAL_API Multibody buildMultibodyFromSkeleton(
    World& world,
    const dynamics::Skeleton& skeleton,
    const SkeletonToMultibodyOptions& options = {});

/// Build one experimental `Multibody` per skeleton in a legacy
/// `simulation::World`, loading a whole scene (for example, every robot parsed
/// from an SDF world by `dart::io::readWorld`) into the experimental `World`.
///
/// Each multibody is named after its source skeleton (so `options.name` is
/// ignored); the remaining options apply to every skeleton. Each skeleton is
/// translated by `buildMultibodyFromSkeleton`, so the same supported-joint set
/// and limitations apply.
///
/// @param world The experimental world that will own the created multibodies.
/// @param legacyWorld The legacy world whose skeletons are translated.
/// @param options Translation options (base link name, state copy); `name` is
///        ignored.
/// @return The created `Multibody` handles, in skeleton order.
/// @throws InvalidOperationException if any skeleton uses an unsupported
///         feature (see `buildMultibodyFromSkeleton`).
DART_EXPERIMENTAL_API std::vector<Multibody> buildMultibodiesFromWorld(
    World& world,
    const dart::simulation::World& legacyWorld,
    const SkeletonToMultibodyOptions& options = {});

} // namespace dart::simulation::experimental::io
