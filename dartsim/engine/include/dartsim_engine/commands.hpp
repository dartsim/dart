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
std::unique_ptr<Command> addRigidBody(
    const ShapeDesc& shape,
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

/// No-op compatibility overload; fixed frames require an existing parent frame.
std::unique_ptr<Command> addFixedFrame(
    const Eigen::Isometry3d& transform = Eigen::Isometry3d::Identity(),
    std::string name = {});
/// Add a fixed-offset frame under an existing frame-like object.
std::unique_ptr<Command> addFixedFrame(
    ObjectId parentFrame,
    const Eigen::Isometry3d& transform = Eigen::Isometry3d::Identity(),
    std::string name = {});

/// Remove an object (and its descendants) and update the selection.
std::unique_ptr<Command> removeObject(ObjectId id);

/// Set the (world for bodies / local for frames) transform of an object.
std::unique_ptr<Command> setTransform(
    ObjectId id, const Eigen::Isometry3d& transform);

/// Set a rigid body's mass.
std::unique_ptr<Command> setMass(ObjectId id, double mass);

/// Set a rigid body or link visual shape descriptor.
std::unique_ptr<Command> setShape(ObjectId id, ShapeDesc shape);

/// Set a child link's single-DOF joint position.
std::unique_ptr<Command> setJointPosition(ObjectId link, double position);

/// Re-parent a link within its owning multibody.
///
/// `parentLink == kNoObject` makes the link a root link of its multibody.
/// Non-root parents must be links in the same multibody, and cycles are
/// rejected.
std::unique_ptr<Command> setLinkParent(
    ObjectId link, ObjectId parentLink = kNoObject);

/// Set a child link's parent joint kind.
std::unique_ptr<Command> setJointKind(ObjectId link, JointKind kind);

/// Set a child link's single-axis joint axis.
std::unique_ptr<Command> setJointAxis(ObjectId link, Eigen::Vector3d axis);

/// Rename an object (no-op if the name collides with a sibling).
std::unique_ptr<Command> rename(ObjectId id, std::string name);

/// Set object visibility for rendering and outliner filtering.
std::unique_ptr<Command> setVisible(ObjectId id, bool visible);

/// Re-parent an object in the scene tree.
std::unique_ptr<Command> reparent(ObjectId id, ObjectId newParent);

/// Attach a frame under another frame-like object, preserving world transform.
std::unique_ptr<Command> attachFrame(ObjectId frame, ObjectId parentFrame);

/// Detach a frame to the world root, preserving world transform.
///
/// Root-level FixedFrame records are converted to FreeFrame because the
/// experimental World requires fixed frames to have a non-world parent.
std::unique_ptr<Command> detachFrame(ObjectId frame);

/// Set the world time step.
std::unique_ptr<Command> setTimeStep(double timeStep);

} // namespace dartsim::commands
