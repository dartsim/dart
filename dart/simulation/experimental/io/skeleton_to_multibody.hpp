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
};

/// Build an experimental `Multibody` from a legacy `dynamics::Skeleton`.
///
/// The translation walks the skeleton's `BodyNode` tree and creates one
/// experimental `Link` per `BodyNode` (mass, inertia about the center of mass,
/// and a center-of-mass offset), connected by the experimental `Joint` that
/// matches the legacy parent `Joint`.
///
/// Frame convention. The experimental dynamics anchors every joint at its
/// parent link's frame origin, whereas a legacy joint carries arbitrary
/// parent- and child-side offsets. To realize a legacy joint exactly, each
/// experimental link's frame origin is placed on that link's own outgoing
/// joint, and the inertia/center-of-mass are re-expressed in the placed frame.
/// A synthetic fixed base link represents the world frame so that a root body's
/// parent-joint offset is realized rather than dropped.
///
/// Supported joints: weld (fixed), revolute, prismatic, screw, universal, ball
/// (spherical), free (floating), and planar, forming serial chains or trees
/// whose sibling joints share a common parent-side frame, on a fixed base.
/// Generalized positions and velocities are copied when `options.copyState` is
/// set (with the screw pitch and the free-joint coordinate order converted to
/// the experimental convention).
///
/// Anchoring constraint: the axis-based joints (revolute, prismatic, screw,
/// universal) tolerate a rotated parent-side offset (the axes are rotated into
/// the placed frame); the orientation-coordinate joints (ball, free, planar)
/// require an identity parent-side offset because their generalized coordinates
/// are not yet re-expressed under a rotated parent frame.
///
/// @param world The experimental world that will own the created multibody.
/// @param skeleton The legacy skeleton to translate.
/// @param options Translation options (name, base link name, state copy).
/// @return A handle to the created `Multibody`.
/// @throws InvalidOperationException if the skeleton uses a joint type, a
///         branching offset, a non-fixed root, or a non-positive link mass that
///         this slice does not yet represent. The message names the unsupported
///         feature.
DART_EXPERIMENTAL_API Multibody buildMultibodyFromSkeleton(
    World& world,
    const dynamics::Skeleton& skeleton,
    const SkeletonToMultibodyOptions& options = {});

} // namespace dart::simulation::experimental::io
