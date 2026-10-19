/*
 * Copyright (c) 2026, The DART development contributors
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

#ifndef DART_COMMON_DETAIL_MODELMUTATION_HPP_
#define DART_COMMON_DETAIL_MODELMUTATION_HPP_

#include <cstddef>

namespace dart {
namespace dynamics {
class BodyNode;
class Frame;
class Joint;
class MetaSkeleton;
class Shape;
class ShapeFrame;
class Skeleton;
} // namespace dynamics
namespace common {
class Aspect;
class Composite;
class VersionCounter;
namespace detail {

// Ownership lives outside model objects to preserve their public layouts.
// Typed overloads let the common callers skip dynamic_cast.
void checkTopologyMutation(const Composite* object);
void checkTopologyMutation(const dynamics::BodyNode* object);
void checkTopologyMutation(const dynamics::ShapeFrame* object);
void checkTopologyMutation(const dynamics::Skeleton* object);
void checkTopologyMutation(const dynamics::Joint* object);
void notifyTopologyMutation(const Composite* object);
void notifyTopologyMutation(const dynamics::BodyNode* object);
void notifyTopologyMutation(const dynamics::ShapeFrame* object);
void notifyTopologyMutation(const dynamics::Skeleton* object);
void notifyTopologyMutation(const dynamics::Joint* object);
void notifyParameterMutation(const Composite* object);
void notifyParameterMutation(const dynamics::BodyNode* object);
void notifyParameterMutation(const dynamics::ShapeFrame* object);
void notifyParameterMutation(const dynamics::Skeleton* object);
void notifyParameterMutation(const dynamics::Joint* object);
void checkFrameTopologyMutation(const dynamics::Frame* frame);
void notifyFrameTopologyMutation(const dynamics::Frame* frame);
// After a Frame's parent changed: members below it record new ancestors.
void notifyFrameParentChanged(const dynamics::Frame* frame);
// A Skeleton only weakly referenced by Worlds is being destroyed.
void notifyModelDestruction(const dynamics::Skeleton* skeleton);
std::size_t incrementAspectVersion(
    VersionCounter* object, const Aspect* aspect);
// Whether any World tracks model ownership (compiled at least once).
bool hasTrackedModels();
// Whether mutation guards can apply: a tracked World exists or one is stepping.
bool hasGuardedModels();
bool isVisualShapeVersionUpdate(const dynamics::Shape* shape);

} // namespace detail
} // namespace common

namespace collision {
class CollisionGroup;
class CollisionDetector;
namespace detail {
void notifyWorldGroupMutation(const CollisionGroup* group);
// Membership changes of a World's collision group, keyed by the group.
void notifyWorldGroupShapeFrameAdded(
    const CollisionGroup* group, const dynamics::ShapeFrame* frame);
void notifyWorldGroupShapeFrameRemoved(
    const CollisionGroup* group, const dynamics::ShapeFrame* frame);
void notifyWorldGroupSubscribed(
    const CollisionGroup* group, const dynamics::MetaSkeleton* source);
void notifyWorldGroupSubscribed(
    const CollisionGroup* group, const dynamics::BodyNode* source);
void notifyWorldGroupUnsubscribed(
    const CollisionGroup* group, const void* source);
void notifyWorldGroupSourceBodyAdded(
    const CollisionGroup* group,
    const void* source,
    const dynamics::BodyNode* body);
void notifyWorldGroupSourceBodyRemoved(
    const CollisionGroup* group, const void* source, const void* body);
void notifyWorldGroupCleared(const CollisionGroup* group);
void checkWorldDetectorTopologyMutation(const CollisionDetector* detector);
void notifyWorldDetectorTopologyMutation(const CollisionDetector* detector);
void notifyWorldDetectorParameterMutation(const CollisionDetector* detector);
} // namespace detail
} // namespace collision

namespace constraint {
class ConstraintSolver;
class ContactSurfaceHandler;
class ConstraintBase;
class BoxedLcpSolver;
namespace detail {
void checkWorldTopologyMutation(const ConstraintSolver* solver);
// The solver replaced its collision group (before it resubscribes).
void notifyWorldCollisionGroupReplaced(const ConstraintSolver* solver);
void notifyWorldSolverSkeletonAdded(
    const ConstraintSolver* solver, const dynamics::Skeleton* skeleton);
void notifyWorldSolverSkeletonRemoved(
    const ConstraintSolver* solver, const dynamics::Skeleton* skeleton);
void notifyWorldTopologyMutation(const ConstraintSolver* solver);
// The solver's contact handler chain changed: it was walked by the caller.
void notifyWorldContactHandlersChanged(const ConstraintSolver* solver);
void notifyWorldParameterMutation(const ConstraintSolver* solver);
void notifyBoxedSolverParameterMutation(const BoxedLcpSolver* solver);
void notifyConstraintParameterMutation(const ConstraintBase* constraint);
void notifyGlobalConstraintParameterMutation();
void checkContactSurfaceMutation(const ContactSurfaceHandler* handler);
// Whether checkContactSurfaceMutation() can reject anything: some World is
// compiled, or this thread is stepping one. Callers skip walking handler chains
// otherwise, so never-compiled Worlds read only what DART 6 read.
bool contactSurfaceChecksActive();
void notifyContactSurfaceMutation(const ContactSurfaceHandler* handler);
} // namespace detail
} // namespace constraint
} // namespace dart

#endif // DART_COMMON_DETAIL_MODELMUTATION_HPP_
