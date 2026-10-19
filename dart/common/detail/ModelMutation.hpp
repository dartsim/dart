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
class Shape;
} // namespace dynamics
namespace common {
class Aspect;
class Composite;
class VersionCounter;
namespace detail {

// Ownership lives outside model objects to preserve their public layouts.
void checkTopologyMutation(const Composite* object);
void checkTopologyMutation(const dynamics::BodyNode* object);
void notifyTopologyMutation(const Composite* object);
void notifyParameterMutation(const Composite* object);
void checkFrameTopologyMutation(const dynamics::Frame* frame);
void notifyFrameTopologyMutation(const dynamics::Frame* frame);
std::size_t incrementAspectVersion(
    VersionCounter* object, const Aspect* aspect);
bool isVisualShapeVersionUpdate(const dynamics::Shape* shape);

} // namespace detail
} // namespace common

namespace collision {
class CollisionGroup;
class CollisionDetector;
namespace detail {
void notifyWorldGroupMutation(const CollisionGroup* group);
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
void notifyWorldTopologyMutation(const ConstraintSolver* solver);
void notifyWorldParameterMutation(const ConstraintSolver* solver);
void notifyBoxedSolverParameterMutation(const BoxedLcpSolver* solver);
void notifyConstraintParameterMutation(const ConstraintBase* constraint);
void notifyGlobalConstraintParameterMutation();
void checkContactSurfaceMutation(const ContactSurfaceHandler* handler);
void notifyContactSurfaceMutation(const ContactSurfaceHandler* handler);
} // namespace detail
} // namespace constraint
} // namespace dart

#endif // DART_COMMON_DETAIL_MODELMUTATION_HPP_
