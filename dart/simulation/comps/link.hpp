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

#include <dart/simulation/comps/component_category.hpp>
#include <dart/simulation/comps/dynamics.hpp>

#include <dart/common/stl_allocator.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <string>
#include <vector>

namespace dart::simulation::comps {

/// Link **Model** component: the static topology, geometry, and inertial
/// parameters of a body in an articulated chain, frozen at finalization. Per
/// the Model/State/Control/Contracts data contract (WP-091.20), it holds only
/// design-frozen data and the kinematic-tree topology (`parentJoint`/
/// `childJoints`), so it is the canonical "is a link" component. The per-step
/// world pose lives in `LinkState`; user-applied external forces live in
/// `LinkControl`.
///
/// **Internal Implementation Detail** - Not exposed in public API
struct LinkModel
{
  DART_SIMULATION_STATE_COMPONENT(LinkModel, "comps.LinkModel");

  using EntityVector
      = std::vector<entt::entity, dart::common::StlAllocator<entt::entity>>;

  std::string name;

  MassProperties mass;

  /// Fixed transform from the parent link frame to the joint frame.
  Eigen::Isometry3d transformFromParentToJoint = Eigen::Isometry3d::Identity();

  /// Fixed transform from the joint frame to this link frame.
  Eigen::Isometry3d transformFromParentJoint = Eigen::Isometry3d::Identity();

  entt::entity parentJoint = entt::null;
  EntityVector childJoints;

  static constexpr auto entityFields()
  {
    return std::make_tuple(&LinkModel::parentJoint, &LinkModel::childJoints);
  }
};

/// Link **State** component: the per-step world pose of the link, advanced by
/// the kinematics stage every step. Per the Model/State/Control/Contracts
/// contract (WP-091.20), this holds only mutable per-step data; design-frozen
/// parameters live in `LinkModel` and applied forces live in `LinkControl`.
///
/// **Internal Implementation Detail** - Not exposed in public API
struct LinkState
{
  DART_SIMULATION_PROPERTY_COMPONENT(LinkState, "comps.LinkState");

  Eigen::Isometry3d worldTransform = Eigen::Isometry3d::Identity();
};

/// Link **Control** component: the user-applied external force on the link. Per
/// the Model/State/Control/Contracts contract (WP-091.20), this holds only the
/// commanded/applied input (the articulated analog of the rigid `Force`
/// component); per-step kinematic state lives in `LinkState`.
///
/// **Internal Implementation Detail** - Not exposed in public API
struct LinkControl
{
  DART_SIMULATION_PROPERTY_COMPONENT(LinkControl, "comps.LinkControl");

  /// Accumulated external spatial force (wrench) expressed in the link frame,
  /// using the `[angular; linear]` = `[torque; force]` convention shared by the
  /// dynamics stages. Set by `Link::applyForce`, read by both the semi-implicit
  /// forward-dynamics and variational-integrator paths, and cleared after each
  /// world step (one-shot per step, mirroring legacy `BodyNode::addExtForce`).
  Eigen::Matrix<double, 6, 1> externalForce
      = Eigen::Matrix<double, 6, 1>::Zero();
};

} // namespace dart::simulation::comps
