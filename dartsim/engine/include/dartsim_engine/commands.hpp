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
#include <dartsim_engine/command.hpp>
#include <dartsim_engine/scene_object.hpp>

#include <memory>
#include <string>

namespace dartsim::commands {

/// Add a primitive rigid body under the world root and select it.
std::unique_ptr<Command> addRigidBody(
    ShapeType shape = ShapeType::Box,
    const Eigen::Isometry3d& transform = Eigen::Isometry3d::Identity(),
    std::string name = {});

/// Add an empty multibody under the world root and select it.
std::unique_ptr<Command> addMultiBody(std::string name = {});

/// Add a link to a multibody. `parentLink` == kNoObject creates a root link;
/// otherwise a child link connected by a `joint` is created and selected.
std::unique_ptr<Command> addLink(
    ObjectId multiBody,
    ObjectId parentLink = kNoObject,
    JointKind joint = JointKind::Revolute,
    std::string name = {});

/// Add a free (movable) frame under the world root.
std::unique_ptr<Command> addFreeFrame(
    const Eigen::Isometry3d& transform = Eigen::Isometry3d::Identity(),
    std::string name = {});

/// Add a fixed-offset frame under the world root.
std::unique_ptr<Command> addFixedFrame(
    const Eigen::Isometry3d& transform = Eigen::Isometry3d::Identity(),
    std::string name = {});

/// Remove an object (and its descendants) and update the selection.
std::unique_ptr<Command> removeObject(ObjectId id);

/// Set the (world for bodies / local for frames) transform of an object.
std::unique_ptr<Command> setTransform(
    ObjectId id, const Eigen::Isometry3d& transform);

/// Set a rigid body's mass.
std::unique_ptr<Command> setMass(ObjectId id, double mass);

/// Set a child link's single-DOF joint position.
std::unique_ptr<Command> setJointPosition(ObjectId link, double position);

/// Rename an object (no-op if the name collides with a sibling).
std::unique_ptr<Command> rename(ObjectId id, std::string name);

/// Re-parent an object in the scene tree.
std::unique_ptr<Command> reparent(ObjectId id, ObjectId newParent);

/// Set the world time step.
std::unique_ptr<Command> setTimeStep(double timeStep);

} // namespace dartsim::commands
