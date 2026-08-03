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

#include <dart/simulation/export.hpp>

namespace dart::simulation {

class FixedFrame;
class Frame;
class FreeFrame;
class DeformableBody;
class Joint;
class Link;
class LoopClosure;
class Multibody;
class RigidBody;
class World;
enum class ContactSolverMethod;
enum class ContactGradientMode;
enum class RigidBodySolver;
enum class JointType;
enum class WorldSyncStage;
enum class PhysicalParameter;
struct PhysicalParameterSelector;

// Value objects
struct Contact;
struct ContactForce;
struct StepDerivatives;
struct StepGradient;
struct DeformableDirichletBoundaryCondition;
struct DeformableMaterialProperties;
struct DeformableNeumannBoundaryCondition;
struct DeformableObstaclePolicy;
struct DeformableSurfaceTriangle;
struct DeformableTetrahedron;
struct JointConstraintProjectionPolicy;

// Options structs
struct CollisionQueryOptions;
struct FixedFrameOptions;
struct FreeFrameOptions;
struct DeformableBodyOptions;
struct DeformableEdge;
struct JointOptions;
struct JointSpec;
struct LinkOptions;
struct LoopClosureRuntimePolicy;
struct LoopClosureResidual;
struct LoopClosureSpec;
struct MultibodyOptions;
struct RigidBodyOptions;
struct DeactivationOptions;
struct WorldOptions;

} // namespace dart::simulation

namespace dart::simulation::comps {

struct MultibodyStructure;

} // namespace dart::simulation::comps

namespace dart::simulation::compute {

namespace avbd_replay {
struct RigidAvbdWarmStartReplayState;
} // namespace avbd_replay

class ComputeExecutor;
class ComputeGraph;
class ComputeNode;
class ParallelExecutor;
class SequentialExecutor;
class WorldKinematicsGraph;
class WorldStepPipeline;
class WorldStepStage;

// Built-in world-step stages
class MultibodyContactStage;
class MultibodyForwardDynamicsStage;
class MultibodyPositionStage;
class MultibodyVelocityStage;
class RigidBodyContactStage;
class RigidBodyPositionStage;
class RigidBodyVelocityStage;
class RigidIpcContactStage;
class UnifiedConstraintStage;

struct InverseDynamicsDerivatives;
struct MultibodyVariationalTreeScratchAccess;

} // namespace dart::simulation::compute

namespace dart::simulation::detail {

struct EntityAccess;
struct RigidIpcProjectedNewtonSolveScratchWorkspace;
struct WorldStorage;

/// Internal storage seam: reaches the privately-held `WorldStorage` of a
/// `World`. Declared here so `World` can befriend it and internal translation
/// units can reach it without a public accessor.
[[nodiscard]] DART_SIMULATION_API WorldStorage& storageOf(World& world);
[[nodiscard]] DART_SIMULATION_API const WorldStorage& storageOf(
    const World& world);

} // namespace dart::simulation::detail

namespace dart::simulation::io {

class SerializerRegistry;

namespace detail {
class SkeletonLoaderWorldAccess;
} // namespace detail

} // namespace dart::simulation::io
