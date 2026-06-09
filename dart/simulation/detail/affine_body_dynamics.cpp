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

#include <dart/simulation/detail/affine_body_dynamics.hpp>
#include <dart/simulation/detail/newton_barrier/friction_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/psd_projection.hpp>
#include <dart/simulation/detail/newton_barrier/tangent_stencil.hpp>

#include <array>

#include <cassert>
#include <cmath>

namespace dart::simulation::detail {
namespace {

[[nodiscard]] bool hasValidBarrierOptions(const AffineBarrierOptions& options)
{
  return std::isfinite(options.squaredActivationDistance)
         && options.squaredActivationDistance > 0.0
         && std::isfinite(options.stiffness) && options.stiffness >= 0.0;
}

[[nodiscard]] bool hasValidFrictionOptions(const AffineFrictionOptions& options)
{
  const double weight = options.coefficient * options.laggedNormalForce;
  return std::isfinite(weight) && weight > 0.0
         && std::isfinite(options.staticFrictionDisplacement)
         && options.staticFrictionDisplacement > 0.0;
}

[[nodiscard]] bool hasValidState(const AffineBodyState& state)
{
  return state.translation.allFinite() && state.linearMap.allFinite();
}

[[nodiscard]] Eigen::Matrix<double, 12, 24> makePrimitiveJacobian(
    const std::array<AffineVertexJacobian, 4>& jacobians,
    const std::array<int, 4>& bodyIds,
    const AffineBodyState& bodyA,
    const AffineBodyState& bodyB)
{
  Eigen::Matrix<double, 12, 24> primitiveJacobian
      = Eigen::Matrix<double, 12, 24>::Zero();
  for (int vertex = 0; vertex < 4; ++vertex) {
    const int bodyId = bodyIds[vertex];
    const bool dynamic = bodyId == 0 ? bodyA.dynamic : bodyB.dynamic;
    if (!dynamic) {
      continue;
    }

    primitiveJacobian.block<3, 12>(3 * vertex, 12 * bodyId) = jacobians[vertex];
  }
  return primitiveJacobian;
}

[[nodiscard]] AffinePrimitiveBarrierResult chainPrimitiveBarrierToAffineBodies(
    const newton_barrier::PrimitiveBarrierResult& primitive,
    const AffineBarrierPrimitive primitiveType,
    const std::array<AffineVertexJacobian, 4>& jacobians,
    const std::array<int, 4>& bodyIds,
    const AffineBodyState& bodyA,
    const AffineBodyState& bodyB,
    const AffineBarrierOptions& options)
{
  AffinePrimitiveBarrierResult result;
  result.value = primitive.value;
  result.primitive = primitive;
  result.primitiveType = primitiveType;
  result.active = primitive.active;

  if (!primitive.active) {
    return result;
  }

  const Eigen::Matrix<double, 12, 24> primitiveJacobian
      = makePrimitiveJacobian(jacobians, bodyIds, bodyA, bodyB);
  result.gradient = primitiveJacobian.transpose() * primitive.gradient;
  result.hessian
      = primitiveJacobian.transpose() * primitive.hessian * primitiveJacobian;
  result.hessian
      = 0.5 * (result.hessian + AffineMatrix24d(result.hessian.transpose()));
  if (options.projectHessianToPsd) {
    result.hessian
        = newton_barrier::projectSymmetricMatrixToPsd<24>(result.hessian);
  }
  return result;
}

template <int Rows>
[[nodiscard]] AffinePrimitiveFrictionResult
chainPrimitiveFrictionToAffineBodies(
    const newton_barrier::FrictionPotentialResult<Rows>& potential,
    const AffineBarrierPrimitive primitiveType,
    const std::array<AffineVertexJacobian, 4>& jacobians,
    const std::array<int, 4>& bodyIds,
    const AffineBodyState& bodyA,
    const AffineBodyState& bodyB,
    const AffineFrictionOptions& options)
{
  AffinePrimitiveFrictionResult result;
  result.value = potential.value;
  result.work = potential.work;
  result.tangentialDisplacement = potential.tangentialDisplacement;
  result.tangentialDisplacementNorm = potential.tangentialDisplacementNorm;
  result.weight = potential.weight;
  result.primitiveType = primitiveType;
  result.active = potential.active;
  result.dynamicBranch = potential.dynamicBranch;

  if (!potential.active) {
    return result;
  }

  Eigen::Matrix<double, Rows, 24> jacobian
      = Eigen::Matrix<double, Rows, 24>::Zero();
  constexpr int vertexCount = Rows / 3;
  for (int vertex = 0; vertex < vertexCount; ++vertex) {
    const int bodyId = bodyIds[vertex];
    const bool dynamic = bodyId == 0 ? bodyA.dynamic : bodyB.dynamic;
    if (!dynamic) {
      continue;
    }

    jacobian.template block<3, 12>(3 * vertex, 12 * bodyId) = jacobians[vertex];
  }

  result.gradient = jacobian.transpose() * potential.gradient;
  result.hessian = jacobian.transpose() * potential.hessian * jacobian;
  result.hessian
      = 0.5 * (result.hessian + AffineMatrix24d(result.hessian.transpose()));
  if (options.projectHessianToPsd) {
    result.hessian
        = newton_barrier::projectSymmetricMatrixToPsd<24>(result.hessian);
  }
  return result;
}

[[nodiscard]] Eigen::Matrix3d affineMatrixFromVariation(
    const int row, const int col)
{
  Eigen::Matrix3d variation = Eigen::Matrix3d::Zero();
  variation(row, col) = 1.0;
  return variation;
}

} // namespace

AffineVector12d affineBodyStateToVector(const AffineBodyState& state)
{
  AffineVector12d vector;
  vector.head<3>() = state.translation;
  for (int col = 0; col < 3; ++col) {
    vector.segment<3>(3 + 3 * col) = state.linearMap.col(col);
  }
  return vector;
}

AffineBodyState affineBodyStateFromVector(const AffineVector12d& vector)
{
  AffineBodyState state;
  state.translation = vector.head<3>();
  for (int col = 0; col < 3; ++col) {
    state.linearMap.col(col) = vector.segment<3>(3 + 3 * col);
  }
  return state;
}

Eigen::Vector3d affineWorldPoint(
    const AffineBodyState& state, const Eigen::Vector3d& localPoint)
{
  return state.translation + state.linearMap * localPoint;
}

AffineVertexJacobian affinePointJacobian(const Eigen::Vector3d& localPoint)
{
  AffineVertexJacobian jacobian = AffineVertexJacobian::Zero();
  jacobian.leftCols<3>() = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  for (int col = 0; col < 3; ++col) {
    jacobian.block<3, 3>(0, 3 + 3 * col) = localPoint[col] * identity;
  }
  return jacobian;
}

Eigen::Vector3d affineSurfaceVertexWorld(
    const AffineSurfaceAdapter& surface,
    const AffineBodyState& state,
    const std::size_t vertex)
{
  assert(vertex < surface.restVertices.size());
  return affineWorldPoint(state, surface.restVertices[vertex]);
}

AffineVertexJacobian affineSurfaceVertexJacobian(
    const AffineSurfaceAdapter& surface, const std::size_t vertex)
{
  assert(vertex < surface.restVertices.size());
  return affinePointJacobian(surface.restVertices[vertex]);
}

AffinePrimitiveBarrierResult affinePointTriangleBarrier(
    const AffineBodyState& pointBody,
    const Eigen::Vector3d& point,
    const AffineBodyState& triangleBody,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const AffineBarrierOptions& options)
{
  if (!hasValidBarrierOptions(options) || !hasValidState(pointBody)
      || !hasValidState(triangleBody) || !point.allFinite()
      || !triangleA.allFinite() || !triangleB.allFinite()
      || !triangleC.allFinite()) {
    return {};
  }

  const auto primitive = newton_barrier::pointTriangleBarrier(
      affineWorldPoint(pointBody, point),
      affineWorldPoint(triangleBody, triangleA),
      affineWorldPoint(triangleBody, triangleB),
      affineWorldPoint(triangleBody, triangleC),
      options.squaredActivationDistance,
      options.stiffness);

  return chainPrimitiveBarrierToAffineBodies(
      primitive,
      AffineBarrierPrimitive::PointTriangle,
      {affinePointJacobian(point),
       affinePointJacobian(triangleA),
       affinePointJacobian(triangleB),
       affinePointJacobian(triangleC)},
      {0, 1, 1, 1},
      pointBody,
      triangleBody,
      options);
}

AffinePrimitiveBarrierResult affinePointEdgeBarrier(
    const AffineBodyState& pointBody,
    const Eigen::Vector3d& point,
    const AffineBodyState& edgeBody,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const AffineBarrierOptions& options)
{
  if (!hasValidBarrierOptions(options) || !hasValidState(pointBody)
      || !hasValidState(edgeBody) || !point.allFinite() || !edgeA.allFinite()
      || !edgeB.allFinite()) {
    return {};
  }

  const auto primitive = newton_barrier::pointEdgeBarrier(
      affineWorldPoint(pointBody, point),
      affineWorldPoint(edgeBody, edgeA),
      affineWorldPoint(edgeBody, edgeB),
      options.squaredActivationDistance,
      options.stiffness);

  return chainPrimitiveBarrierToAffineBodies(
      primitive,
      AffineBarrierPrimitive::PointEdge,
      {affinePointJacobian(point),
       affinePointJacobian(edgeA),
       affinePointJacobian(edgeB),
       AffineVertexJacobian::Zero()},
      {0, 1, 1, 0},
      pointBody,
      edgeBody,
      options);
}

AffinePrimitiveBarrierResult affineEdgeEdgeBarrier(
    const AffineBodyState& edgeABody,
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const AffineBodyState& edgeBBody,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const AffineBarrierOptions& options)
{
  if (!hasValidBarrierOptions(options) || !hasValidState(edgeABody)
      || !hasValidState(edgeBBody) || !edgeA0.allFinite() || !edgeA1.allFinite()
      || !edgeB0.allFinite() || !edgeB1.allFinite()) {
    return {};
  }

  const auto primitive = newton_barrier::edgeEdgeBarrier(
      affineWorldPoint(edgeABody, edgeA0),
      affineWorldPoint(edgeABody, edgeA1),
      affineWorldPoint(edgeBBody, edgeB0),
      affineWorldPoint(edgeBBody, edgeB1),
      options.squaredActivationDistance,
      options.stiffness);

  return chainPrimitiveBarrierToAffineBodies(
      primitive,
      AffineBarrierPrimitive::EdgeEdge,
      {affinePointJacobian(edgeA0),
       affinePointJacobian(edgeA1),
       affinePointJacobian(edgeB0),
       affinePointJacobian(edgeB1)},
      {0, 0, 1, 1},
      edgeABody,
      edgeBBody,
      options);
}

AffinePrimitiveBarrierResult affinePointPointBarrier(
    const AffineBodyState& pointABody,
    const Eigen::Vector3d& pointA,
    const AffineBodyState& pointBBody,
    const Eigen::Vector3d& pointB,
    const AffineBarrierOptions& options)
{
  if (!hasValidBarrierOptions(options) || !hasValidState(pointABody)
      || !hasValidState(pointBBody) || !pointA.allFinite()
      || !pointB.allFinite()) {
    return {};
  }

  const auto primitive = newton_barrier::pointPointBarrier(
      affineWorldPoint(pointABody, pointA),
      affineWorldPoint(pointBBody, pointB),
      options.squaredActivationDistance,
      options.stiffness);

  return chainPrimitiveBarrierToAffineBodies(
      primitive,
      AffineBarrierPrimitive::PointPoint,
      {affinePointJacobian(pointA),
       affinePointJacobian(pointB),
       AffineVertexJacobian::Zero(),
       AffineVertexJacobian::Zero()},
      {0, 1, 0, 0},
      pointABody,
      pointBBody,
      options);
}

AffinePrimitiveFrictionResult affinePointPointFrictionPotential(
    const Eigen::Vector3d& pointA,
    const AffineBodyState& laggedPointABody,
    const AffineBodyState& pointABody,
    const Eigen::Vector3d& pointB,
    const AffineBodyState& laggedPointBBody,
    const AffineBodyState& pointBBody,
    const AffineFrictionOptions& options)
{
  if (!hasValidFrictionOptions(options) || !hasValidState(laggedPointABody)
      || !hasValidState(pointABody) || !hasValidState(laggedPointBBody)
      || !hasValidState(pointBBody) || !pointA.allFinite()
      || !pointB.allFinite()) {
    return {};
  }

  const Eigen::Vector3d laggedPointAWorld
      = affineWorldPoint(laggedPointABody, pointA);
  const Eigen::Vector3d laggedPointBWorld
      = affineWorldPoint(laggedPointBBody, pointB);
  const Eigen::Vector3d pointAWorld = affineWorldPoint(pointABody, pointA);
  const Eigen::Vector3d pointBWorld = affineWorldPoint(pointBBody, pointB);

  const auto stencil = newton_barrier::pointPointTangentStencil(
      laggedPointAWorld, laggedPointBWorld);
  Eigen::Matrix<double, 6, 1> displacement;
  displacement.head<3>() = pointAWorld - laggedPointAWorld;
  displacement.tail<3>() = pointBWorld - laggedPointBWorld;

  const double weight = options.coefficient * options.laggedNormalForce;
  const auto potential = newton_barrier::projectedFrictionPotential<6>(
      stencil.projection,
      displacement,
      weight,
      options.staticFrictionDisplacement);

  return chainPrimitiveFrictionToAffineBodies<6>(
      potential,
      AffineBarrierPrimitive::PointPoint,
      {affinePointJacobian(pointA),
       affinePointJacobian(pointB),
       AffineVertexJacobian::Zero(),
       AffineVertexJacobian::Zero()},
      {0, 1, 0, 0},
      pointABody,
      pointBBody,
      options);
}

AffinePrimitiveFrictionResult affinePointEdgeFrictionPotential(
    const Eigen::Vector3d& point,
    const AffineBodyState& laggedPointBody,
    const AffineBodyState& pointBody,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const AffineBodyState& laggedEdgeBody,
    const AffineBodyState& edgeBody,
    const AffineFrictionOptions& options)
{
  if (!hasValidFrictionOptions(options) || !hasValidState(laggedPointBody)
      || !hasValidState(pointBody) || !hasValidState(laggedEdgeBody)
      || !hasValidState(edgeBody) || !point.allFinite() || !edgeA.allFinite()
      || !edgeB.allFinite()) {
    return {};
  }

  const Eigen::Vector3d laggedPointWorld
      = affineWorldPoint(laggedPointBody, point);
  const Eigen::Vector3d laggedEdgeAWorld
      = affineWorldPoint(laggedEdgeBody, edgeA);
  const Eigen::Vector3d laggedEdgeBWorld
      = affineWorldPoint(laggedEdgeBody, edgeB);
  const Eigen::Vector3d pointWorld = affineWorldPoint(pointBody, point);
  const Eigen::Vector3d edgeAWorld = affineWorldPoint(edgeBody, edgeA);
  const Eigen::Vector3d edgeBWorld = affineWorldPoint(edgeBody, edgeB);

  const auto stencil = newton_barrier::pointEdgeTangentStencil(
      laggedPointWorld, laggedEdgeAWorld, laggedEdgeBWorld);
  Eigen::Matrix<double, 9, 1> displacement;
  displacement.segment<3>(0) = pointWorld - laggedPointWorld;
  displacement.segment<3>(3) = edgeAWorld - laggedEdgeAWorld;
  displacement.segment<3>(6) = edgeBWorld - laggedEdgeBWorld;

  const double weight = options.coefficient * options.laggedNormalForce;
  const auto potential = newton_barrier::projectedFrictionPotential<9>(
      stencil.projection,
      displacement,
      weight,
      options.staticFrictionDisplacement);

  return chainPrimitiveFrictionToAffineBodies<9>(
      potential,
      AffineBarrierPrimitive::PointEdge,
      {affinePointJacobian(point),
       affinePointJacobian(edgeA),
       affinePointJacobian(edgeB),
       AffineVertexJacobian::Zero()},
      {0, 1, 1, 0},
      pointBody,
      edgeBody,
      options);
}

AffinePrimitiveFrictionResult affineEdgeEdgeFrictionPotential(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const AffineBodyState& laggedEdgeABody,
    const AffineBodyState& edgeABody,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const AffineBodyState& laggedEdgeBBody,
    const AffineBodyState& edgeBBody,
    const AffineFrictionOptions& options)
{
  if (!hasValidFrictionOptions(options) || !hasValidState(laggedEdgeABody)
      || !hasValidState(edgeABody) || !hasValidState(laggedEdgeBBody)
      || !hasValidState(edgeBBody) || !edgeA0.allFinite() || !edgeA1.allFinite()
      || !edgeB0.allFinite() || !edgeB1.allFinite()) {
    return {};
  }

  const Eigen::Vector3d laggedEdgeA0World
      = affineWorldPoint(laggedEdgeABody, edgeA0);
  const Eigen::Vector3d laggedEdgeA1World
      = affineWorldPoint(laggedEdgeABody, edgeA1);
  const Eigen::Vector3d laggedEdgeB0World
      = affineWorldPoint(laggedEdgeBBody, edgeB0);
  const Eigen::Vector3d laggedEdgeB1World
      = affineWorldPoint(laggedEdgeBBody, edgeB1);
  const Eigen::Vector3d edgeA0World = affineWorldPoint(edgeABody, edgeA0);
  const Eigen::Vector3d edgeA1World = affineWorldPoint(edgeABody, edgeA1);
  const Eigen::Vector3d edgeB0World = affineWorldPoint(edgeBBody, edgeB0);
  const Eigen::Vector3d edgeB1World = affineWorldPoint(edgeBBody, edgeB1);

  const auto stencil = newton_barrier::edgeEdgeTangentStencil(
      laggedEdgeA0World,
      laggedEdgeA1World,
      laggedEdgeB0World,
      laggedEdgeB1World);
  Eigen::Matrix<double, 12, 1> displacement;
  displacement.segment<3>(0) = edgeA0World - laggedEdgeA0World;
  displacement.segment<3>(3) = edgeA1World - laggedEdgeA1World;
  displacement.segment<3>(6) = edgeB0World - laggedEdgeB0World;
  displacement.segment<3>(9) = edgeB1World - laggedEdgeB1World;

  const double weight = options.coefficient * options.laggedNormalForce;
  const auto potential = newton_barrier::projectedFrictionPotential<12>(
      stencil.projection,
      displacement,
      weight,
      options.staticFrictionDisplacement);

  return chainPrimitiveFrictionToAffineBodies<12>(
      potential,
      AffineBarrierPrimitive::EdgeEdge,
      {affinePointJacobian(edgeA0),
       affinePointJacobian(edgeA1),
       affinePointJacobian(edgeB0),
       affinePointJacobian(edgeB1)},
      {0, 0, 1, 1},
      edgeABody,
      edgeBBody,
      options);
}

AffinePrimitiveFrictionResult affinePointTriangleFrictionPotential(
    const Eigen::Vector3d& point,
    const AffineBodyState& laggedPointBody,
    const AffineBodyState& pointBody,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const AffineBodyState& laggedTriangleBody,
    const AffineBodyState& triangleBody,
    const AffineFrictionOptions& options)
{
  if (!hasValidFrictionOptions(options) || !hasValidState(laggedPointBody)
      || !hasValidState(pointBody) || !hasValidState(laggedTriangleBody)
      || !hasValidState(triangleBody) || !point.allFinite()
      || !triangleA.allFinite() || !triangleB.allFinite()
      || !triangleC.allFinite()) {
    return {};
  }

  const Eigen::Vector3d laggedPointWorld
      = affineWorldPoint(laggedPointBody, point);
  const Eigen::Vector3d laggedTriangleAWorld
      = affineWorldPoint(laggedTriangleBody, triangleA);
  const Eigen::Vector3d laggedTriangleBWorld
      = affineWorldPoint(laggedTriangleBody, triangleB);
  const Eigen::Vector3d laggedTriangleCWorld
      = affineWorldPoint(laggedTriangleBody, triangleC);
  const Eigen::Vector3d pointWorld = affineWorldPoint(pointBody, point);
  const Eigen::Vector3d triangleAWorld
      = affineWorldPoint(triangleBody, triangleA);
  const Eigen::Vector3d triangleBWorld
      = affineWorldPoint(triangleBody, triangleB);
  const Eigen::Vector3d triangleCWorld
      = affineWorldPoint(triangleBody, triangleC);

  const auto stencil = newton_barrier::pointTriangleTangentStencil(
      laggedPointWorld,
      laggedTriangleAWorld,
      laggedTriangleBWorld,
      laggedTriangleCWorld);
  Eigen::Matrix<double, 12, 1> displacement;
  displacement.segment<3>(0) = pointWorld - laggedPointWorld;
  displacement.segment<3>(3) = triangleAWorld - laggedTriangleAWorld;
  displacement.segment<3>(6) = triangleBWorld - laggedTriangleBWorld;
  displacement.segment<3>(9) = triangleCWorld - laggedTriangleCWorld;

  const double weight = options.coefficient * options.laggedNormalForce;
  const auto potential = newton_barrier::projectedFrictionPotential<12>(
      stencil.projection,
      displacement,
      weight,
      options.staticFrictionDisplacement);

  return chainPrimitiveFrictionToAffineBodies<12>(
      potential,
      AffineBarrierPrimitive::PointTriangle,
      {affinePointJacobian(point),
       affinePointJacobian(triangleA),
       affinePointJacobian(triangleB),
       affinePointJacobian(triangleC)},
      {0, 1, 1, 1},
      pointBody,
      triangleBody,
      options);
}

AffineOrthogonalityEnergyResult affineOrthogonalityEnergy(
    const AffineBodyState& state,
    const double stiffness,
    const bool projectHessianToPsd)
{
  AffineOrthogonalityEnergyResult result;
  if (!hasValidState(state) || !std::isfinite(stiffness)
      || !(stiffness > 0.0)) {
    return result;
  }

  result.active = true;
  const Eigen::Matrix3d& a = state.linearMap;
  const Eigen::Matrix3d residual
      = a.transpose() * a - Eigen::Matrix3d::Identity();
  result.value = 0.5 * stiffness * residual.squaredNorm();

  const Eigen::Matrix3d matrixGradient = 2.0 * stiffness * a * residual;
  for (int col = 0; col < 3; ++col) {
    result.gradient.segment<3>(3 + 3 * col) = matrixGradient.col(col);
  }

  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      const Eigen::Matrix3d variation = affineMatrixFromVariation(row, col);
      const Eigen::Matrix3d residualVariation
          = variation.transpose() * a + a.transpose() * variation;
      const Eigen::Matrix3d gradientVariation
          = 2.0 * stiffness * (variation * residual + a * residualVariation);
      const int dof = 3 + 3 * col + row;
      for (int gradientCol = 0; gradientCol < 3; ++gradientCol) {
        result.hessian.block<3, 1>(3 + 3 * gradientCol, dof)
            = gradientVariation.col(gradientCol);
      }
    }
  }

  result.hessian
      = 0.5 * (result.hessian + AffineMatrix12d(result.hessian.transpose()));
  if (projectHessianToPsd) {
    result.hessian
        = newton_barrier::projectSymmetricMatrixToPsd<12>(result.hessian);
  }
  return result;
}

} // namespace dart::simulation::detail
