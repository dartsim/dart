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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
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

#include <dart/simulation/detail/newton_barrier/barrier_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/line_search.hpp>
#include <dart/simulation/export.hpp>

#include <Eigen/Core>

#include <vector>

#include <cstddef>

namespace dart::simulation::detail {

using AffineVector12d = Eigen::Matrix<double, 12, 1>;
using AffineVector24d = Eigen::Matrix<double, 24, 1>;
using AffineMatrix12d = Eigen::Matrix<double, 12, 12>;
using AffineMatrix24d = Eigen::Matrix<double, 24, 24>;
using AffineVertexJacobian = Eigen::Matrix<double, 3, 12>;

enum class AffineBarrierPrimitive
{
  PointPoint,
  PointEdge,
  EdgeEdge,
  PointTriangle,
};

struct AffineBodyState
{
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Matrix3d linearMap = Eigen::Matrix3d::Identity();
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Matrix3d affineVelocity = Eigen::Matrix3d::Zero();
  double mass = 1.0;
  double density = 1.0;
  double orthogonalityStiffness = 0.0;
  bool dynamic = true;
};

struct AffineSurfaceAdapter
{
  std::vector<Eigen::Vector3d> restVertices;
  std::vector<Eigen::Vector3i> triangles;
  bool active = true;
  bool dynamic = true;
};

struct AffineBarrierOptions
{
  double squaredActivationDistance = 0.0;
  double stiffness = 1.0;
  bool projectHessianToPsd = false;
};

struct AffineFrictionOptions
{
  double coefficient = 0.0;
  double laggedNormalForce = 0.0;
  double staticFrictionDisplacement = 0.0;
  bool projectHessianToPsd = false;
};

struct AffinePrimitiveBarrierResult
{
  double value = 0.0;
  AffineVector24d gradient = AffineVector24d::Zero();
  AffineMatrix24d hessian = AffineMatrix24d::Zero();
  newton_barrier::PrimitiveBarrierResult primitive;
  AffineBarrierPrimitive primitiveType = AffineBarrierPrimitive::PointPoint;
  bool active = false;
};

struct AffinePrimitiveFrictionResult
{
  double value = 0.0;
  double work = 0.0;
  AffineVector24d gradient = AffineVector24d::Zero();
  AffineMatrix24d hessian = AffineMatrix24d::Zero();
  Eigen::Vector2d tangentialDisplacement = Eigen::Vector2d::Zero();
  double tangentialDisplacementNorm = 0.0;
  double weight = 0.0;
  AffineBarrierPrimitive primitiveType = AffineBarrierPrimitive::PointPoint;
  bool active = false;
  bool dynamicBranch = false;
};

struct AffineOrthogonalityEnergyResult
{
  double value = 0.0;
  AffineVector12d gradient = AffineVector12d::Zero();
  AffineMatrix12d hessian = AffineMatrix12d::Zero();
  bool active = false;
};

struct AffinePointTriangleMicroSolveOptions
{
  AffineBarrierOptions barrier;
  double inertialWeight = 1.0;
  double orthogonalityStiffness = 0.0;
  double gradientTolerance = 1e-8;
  double sufficientDecreaseFactor
      = newton_barrier::kDefaultSufficientDecreaseFactor;
  double backtrackingScale = newton_barrier::kDefaultBacktrackingScale;
  double maxStepNorm = 0.25;
  int maxIterations = 16;
  int maxLineSearchIterations = 16;
};

struct AffinePointTriangleMicroSolveResult
{
  AffineBodyState state;
  double initialValue = 0.0;
  double finalValue = 0.0;
  double initialGradientNorm = 0.0;
  double finalGradientNorm = 0.0;
  double initialSquaredDistance = 0.0;
  double finalSquaredDistance = 0.0;
  int iterations = 0;
  bool valid = false;
  bool converged = false;
  bool barrierActive = false;
};

struct AffinePointTriangleRuntimeStepOptions
{
  AffinePointTriangleMicroSolveOptions solve;
  double timeStep = 0.01;
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
};

struct AffinePointTriangleRuntimeStepResult
{
  AffineBodyState initialState;
  AffineBodyState inertialTarget;
  AffinePointTriangleMicroSolveResult solve;
  double displacementNorm = 0.0;
  double linearSpeed = 0.0;
  double affineVelocityNorm = 0.0;
  bool valid = false;
  bool converged = false;
};

struct AffinePointTrianglePairMicroSolveResult
{
  AffineBodyState pointState;
  AffineBodyState triangleState;
  double initialValue = 0.0;
  double finalValue = 0.0;
  double initialGradientNorm = 0.0;
  double finalGradientNorm = 0.0;
  double initialSquaredDistance = 0.0;
  double finalSquaredDistance = 0.0;
  int iterations = 0;
  bool valid = false;
  bool converged = false;
  bool barrierActive = false;
};

struct AffinePointTrianglePairRuntimeStepResult
{
  AffineBodyState initialPointState;
  AffineBodyState initialTriangleState;
  AffineBodyState pointInertialTarget;
  AffineBodyState triangleInertialTarget;
  AffinePointTrianglePairMicroSolveResult solve;
  double pointDisplacementNorm = 0.0;
  double triangleDisplacementNorm = 0.0;
  double maxLinearSpeed = 0.0;
  double maxAffineVelocityNorm = 0.0;
  bool valid = false;
  bool converged = false;
};

[[nodiscard]] DART_SIMULATION_API AffineVector12d
affineBodyStateToVector(const AffineBodyState& state);

[[nodiscard]] DART_SIMULATION_API AffineBodyState
affineBodyStateFromVector(const AffineVector12d& vector);

[[nodiscard]] DART_SIMULATION_API Eigen::Vector3d affineWorldPoint(
    const AffineBodyState& state, const Eigen::Vector3d& localPoint);

[[nodiscard]] DART_SIMULATION_API AffineVertexJacobian
affinePointJacobian(const Eigen::Vector3d& localPoint);

[[nodiscard]] DART_SIMULATION_API Eigen::Vector3d affineSurfaceVertexWorld(
    const AffineSurfaceAdapter& surface,
    const AffineBodyState& state,
    std::size_t vertex);

[[nodiscard]] DART_SIMULATION_API AffineVertexJacobian
affineSurfaceVertexJacobian(
    const AffineSurfaceAdapter& surface, std::size_t vertex);

[[nodiscard]] DART_SIMULATION_API AffinePrimitiveBarrierResult
affinePointTriangleBarrier(
    const AffineBodyState& pointBody,
    const Eigen::Vector3d& point,
    const AffineBodyState& triangleBody,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const AffineBarrierOptions& options);

[[nodiscard]] DART_SIMULATION_API AffinePrimitiveBarrierResult
affinePointEdgeBarrier(
    const AffineBodyState& pointBody,
    const Eigen::Vector3d& point,
    const AffineBodyState& edgeBody,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const AffineBarrierOptions& options);

[[nodiscard]] DART_SIMULATION_API AffinePrimitiveBarrierResult
affineEdgeEdgeBarrier(
    const AffineBodyState& edgeABody,
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const AffineBodyState& edgeBBody,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const AffineBarrierOptions& options);

[[nodiscard]] DART_SIMULATION_API AffinePrimitiveBarrierResult
affinePointPointBarrier(
    const AffineBodyState& pointABody,
    const Eigen::Vector3d& pointA,
    const AffineBodyState& pointBBody,
    const Eigen::Vector3d& pointB,
    const AffineBarrierOptions& options);

[[nodiscard]] DART_SIMULATION_API AffinePrimitiveFrictionResult
affinePointPointFrictionPotential(
    const Eigen::Vector3d& pointA,
    const AffineBodyState& laggedPointABody,
    const AffineBodyState& pointABody,
    const Eigen::Vector3d& pointB,
    const AffineBodyState& laggedPointBBody,
    const AffineBodyState& pointBBody,
    const AffineFrictionOptions& options);

[[nodiscard]] DART_SIMULATION_API AffinePrimitiveFrictionResult
affinePointEdgeFrictionPotential(
    const Eigen::Vector3d& point,
    const AffineBodyState& laggedPointBody,
    const AffineBodyState& pointBody,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const AffineBodyState& laggedEdgeBody,
    const AffineBodyState& edgeBody,
    const AffineFrictionOptions& options);

[[nodiscard]] DART_SIMULATION_API AffinePrimitiveFrictionResult
affineEdgeEdgeFrictionPotential(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const AffineBodyState& laggedEdgeABody,
    const AffineBodyState& edgeABody,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const AffineBodyState& laggedEdgeBBody,
    const AffineBodyState& edgeBBody,
    const AffineFrictionOptions& options);

[[nodiscard]] DART_SIMULATION_API AffinePrimitiveFrictionResult
affinePointTriangleFrictionPotential(
    const Eigen::Vector3d& point,
    const AffineBodyState& laggedPointBody,
    const AffineBodyState& pointBody,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const AffineBodyState& laggedTriangleBody,
    const AffineBodyState& triangleBody,
    const AffineFrictionOptions& options);

[[nodiscard]] DART_SIMULATION_API AffineOrthogonalityEnergyResult
affineOrthogonalityEnergy(
    const AffineBodyState& state,
    double stiffness,
    bool projectHessianToPsd = false);

[[nodiscard]] DART_SIMULATION_API AffinePointTriangleMicroSolveResult
affinePointTriangleMicroSolve(
    const AffineBodyState& initialPointBody,
    const AffineBodyState& inertialTarget,
    const Eigen::Vector3d& point,
    const AffineBodyState& triangleBody,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const AffinePointTriangleMicroSolveOptions& options);

[[nodiscard]] DART_SIMULATION_API AffinePointTriangleRuntimeStepResult
affinePointTriangleRuntimeStep(
    const AffineBodyState& initialPointBody,
    const Eigen::Vector3d& point,
    const AffineBodyState& triangleBody,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const AffinePointTriangleRuntimeStepOptions& options);

[[nodiscard]] DART_SIMULATION_API AffinePointTrianglePairMicroSolveResult
affinePointTrianglePairMicroSolve(
    const AffineBodyState& initialPointBody,
    const AffineBodyState& pointInertialTarget,
    const Eigen::Vector3d& point,
    const AffineBodyState& initialTriangleBody,
    const AffineBodyState& triangleInertialTarget,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const AffinePointTriangleMicroSolveOptions& options);

[[nodiscard]] DART_SIMULATION_API AffinePointTrianglePairRuntimeStepResult
affinePointTrianglePairRuntimeStep(
    const AffineBodyState& initialPointBody,
    const Eigen::Vector3d& point,
    const AffineBodyState& initialTriangleBody,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const AffinePointTriangleRuntimeStepOptions& options);

} // namespace dart::simulation::detail
