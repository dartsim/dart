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

#ifndef DART_DYNAMICS_DETAIL_ARTICULATEDDYNAMICSALGORITHMS_HPP_
#define DART_DYNAMICS_DETAIL_ARTICULATEDDYNAMICSALGORITHMS_HPP_

#include <dart/dynamics/body_node.hpp>

#include <algorithm>
#include <ranges>
#include <span>
#include <vector>

namespace dart {
namespace dynamics {
namespace detail {

struct RneaOptions
{
  bool mWithExternalForces = false;
  bool mWithDampingForces = false;
  bool mWithSpringForces = false;
};

struct AbaOptions
{
  bool mWithDampingForces = true;
  bool mWithSpringForces = true;
};

class ArticulatedDynamicsAccess
{
public:
  template <class BodyNodeT, class GravityT>
  static void updateTransmittedForceID(
      BodyNodeT& bodyNode, const GravityT& gravity, bool withExternalForces)
  {
    bodyNode.updateTransmittedForceID(gravity, withExternalForces);
  }

  template <class BodyNodeT>
  static void updateJointForceID(
      BodyNodeT& bodyNode,
      double timeStep,
      bool withDampingForces,
      bool withSpringForces)
  {
    bodyNode.updateJointForceID(timeStep, withDampingForces, withSpringForces);
  }

  template <class BodyNodeT, class GravityT>
  static void updateBiasForce(
      BodyNodeT& bodyNode, const GravityT& gravity, double timeStep)
  {
    bodyNode.updateBiasForce(gravity, timeStep);
  }

  template <class BodyNodeT>
  static void updateAccelerationFD(BodyNodeT& bodyNode)
  {
    bodyNode.updateAccelerationFD();
  }

  template <class BodyNodeT>
  static void updateTransmittedForceFD(BodyNodeT& bodyNode)
  {
    bodyNode.updateTransmittedForceFD();
  }

  template <class BodyNodeT>
  static void updateJointForceFD(
      BodyNodeT& bodyNode,
      double timeStep,
      bool withDampingForces,
      bool withSpringForces)
  {
    bodyNode.updateJointForceFD(timeStep, withDampingForces, withSpringForces);
  }

  static void rneaBodyNodes(
      const std::vector<BodyNode*>& bodyNodes,
      const Eigen::Vector3d& gravity,
      double timeStep)
  {
    for (auto* bodyNode : std::views::reverse(bodyNodes)) {
      bodyNode->updateTransmittedForceID(gravity, false);
      bodyNode->updateJointForceID(timeStep, false, false);
    }
  }

  static void rneaBodyNodes(
      const std::vector<BodyNode*>& bodyNodes,
      const Eigen::Vector3d& gravity,
      double timeStep,
      RneaOptions options)
  {
    const bool withExternalForces = options.mWithExternalForces;
    const bool withDampingForces = options.mWithDampingForces;
    const bool withSpringForces = options.mWithSpringForces;
    for (auto* bodyNode : std::views::reverse(bodyNodes)) {
      bodyNode->updateTransmittedForceID(gravity, withExternalForces);
      bodyNode->updateJointForceID(
          timeStep, withDampingForces, withSpringForces);
    }
  }

  static void abaBodyNodes(
      const std::vector<BodyNode*>& bodyNodes,
      const Eigen::Vector3d& gravity,
      double timeStep)
  {
    for (auto* bodyNode : std::views::reverse(bodyNodes)) {
      bodyNode->updateBiasForce(gravity, timeStep);
    }

    for (auto* bodyNode : bodyNodes) {
      bodyNode->updateAccelerationFD();
      bodyNode->updateTransmittedForceFD();
      bodyNode->updateJointForceFD(timeStep, true, true);
    }
  }

  static void abaBodyNodes(
      const std::vector<BodyNode*>& bodyNodes,
      const Eigen::Vector3d& gravity,
      double timeStep,
      AbaOptions options)
  {
    if (options.mWithDampingForces && options.mWithSpringForces) [[likely]] {
      abaBodyNodes(bodyNodes, gravity, timeStep);
      return;
    }

    const bool withDampingForces = options.mWithDampingForces;
    const bool withSpringForces = options.mWithSpringForces;
    for (auto* bodyNode : std::views::reverse(bodyNodes)) {
      bodyNode->updateBiasForce(gravity, timeStep);
    }

    for (auto* bodyNode : bodyNodes) {
      bodyNode->updateAccelerationFD();
      bodyNode->updateTransmittedForceFD();
      bodyNode->updateJointForceFD(
          timeStep, withDampingForces, withSpringForces);
    }
  }

  static void rneaBodyNodes(
      BodyNode* const* bodyNodes,
      std::size_t numBodyNodes,
      const Eigen::Vector3d& gravity,
      double timeStep)
  {
    if (numBodyNodes == 0) [[unlikely]] {
      return;
    }

    auto* const bodyNodesEnd = bodyNodes + numBodyNodes;
    for (auto* it = bodyNodesEnd; it != bodyNodes;) {
      auto& bodyNode = **--it;
      bodyNode.updateTransmittedForceID(gravity, false);
      bodyNode.updateJointForceID(timeStep, false, false);
    }
  }

  static void rneaBodyNodes(
      BodyNode* const* bodyNodes,
      std::size_t numBodyNodes,
      const Eigen::Vector3d& gravity,
      double timeStep,
      RneaOptions options)
  {
    if (numBodyNodes == 0) [[unlikely]] {
      return;
    }

    const bool withExternalForces = options.mWithExternalForces;
    const bool withDampingForces = options.mWithDampingForces;
    const bool withSpringForces = options.mWithSpringForces;
    auto* const bodyNodesEnd = bodyNodes + numBodyNodes;
    for (auto* it = bodyNodesEnd; it != bodyNodes;) {
      auto& bodyNode = **--it;
      bodyNode.updateTransmittedForceID(gravity, withExternalForces);
      bodyNode.updateJointForceID(
          timeStep, withDampingForces, withSpringForces);
    }
  }

  static void abaBodyNodes(
      BodyNode* const* bodyNodes,
      std::size_t numBodyNodes,
      const Eigen::Vector3d& gravity,
      double timeStep)
  {
    if (numBodyNodes == 0) [[unlikely]] {
      return;
    }

    auto* const bodyNodesEnd = bodyNodes + numBodyNodes;
    for (auto* it = bodyNodesEnd; it != bodyNodes;) {
      (*--it)->updateBiasForce(gravity, timeStep);
    }

    for (auto* it = bodyNodes; it != bodyNodesEnd; ++it) {
      auto& bodyNode = **it;
      bodyNode.updateAccelerationFD();
      bodyNode.updateTransmittedForceFD();
      bodyNode.updateJointForceFD(timeStep, true, true);
    }
  }

  static void abaBodyNodes(
      BodyNode* const* bodyNodes,
      std::size_t numBodyNodes,
      const Eigen::Vector3d& gravity,
      double timeStep,
      AbaOptions options)
  {
    if (options.mWithDampingForces && options.mWithSpringForces) [[likely]] {
      abaBodyNodes(bodyNodes, numBodyNodes, gravity, timeStep);
      return;
    }

    if (numBodyNodes == 0) [[unlikely]] {
      return;
    }

    const bool withDampingForces = options.mWithDampingForces;
    const bool withSpringForces = options.mWithSpringForces;
    auto* const bodyNodesEnd = bodyNodes + numBodyNodes;
    for (auto* it = bodyNodesEnd; it != bodyNodes;) {
      (*--it)->updateBiasForce(gravity, timeStep);
    }

    for (auto* it = bodyNodes; it != bodyNodesEnd; ++it) {
      auto& bodyNode = **it;
      bodyNode.updateAccelerationFD();
      bodyNode.updateTransmittedForceFD();
      bodyNode.updateJointForceFD(
          timeStep, withDampingForces, withSpringForces);
    }
  }
};

inline void rneaBodyNodes(
    const std::vector<BodyNode*>& bodyNodes,
    const Eigen::Vector3d& gravity,
    double timeStep)
{
  ArticulatedDynamicsAccess::rneaBodyNodes(bodyNodes, gravity, timeStep);
}

inline void rneaBodyNodes(
    const std::vector<BodyNode*>& bodyNodes,
    const Eigen::Vector3d& gravity,
    double timeStep,
    RneaOptions options)
{
  ArticulatedDynamicsAccess::rneaBodyNodes(
      bodyNodes, gravity, timeStep, options);
}

inline void abaBodyNodes(
    const std::vector<BodyNode*>& bodyNodes,
    const Eigen::Vector3d& gravity,
    double timeStep)
{
  ArticulatedDynamicsAccess::abaBodyNodes(bodyNodes, gravity, timeStep);
}

inline void abaBodyNodes(
    const std::vector<BodyNode*>& bodyNodes,
    const Eigen::Vector3d& gravity,
    double timeStep,
    AbaOptions options)
{
  ArticulatedDynamicsAccess::abaBodyNodes(
      bodyNodes, gravity, timeStep, options);
}

inline void rneaBodyNodes(
    BodyNode* const* bodyNodes,
    std::size_t numBodyNodes,
    const Eigen::Vector3d& gravity,
    double timeStep)
{
  ArticulatedDynamicsAccess::rneaBodyNodes(
      bodyNodes, numBodyNodes, gravity, timeStep);
}

inline void rneaBodyNodes(
    BodyNode* const* bodyNodes,
    std::size_t numBodyNodes,
    const Eigen::Vector3d& gravity,
    double timeStep,
    RneaOptions options)
{
  ArticulatedDynamicsAccess::rneaBodyNodes(
      bodyNodes, numBodyNodes, gravity, timeStep, options);
}

inline void abaBodyNodes(
    BodyNode* const* bodyNodes,
    std::size_t numBodyNodes,
    const Eigen::Vector3d& gravity,
    double timeStep)
{
  ArticulatedDynamicsAccess::abaBodyNodes(
      bodyNodes, numBodyNodes, gravity, timeStep);
}

inline void abaBodyNodes(
    BodyNode* const* bodyNodes,
    std::size_t numBodyNodes,
    const Eigen::Vector3d& gravity,
    double timeStep,
    AbaOptions options)
{
  ArticulatedDynamicsAccess::abaBodyNodes(
      bodyNodes, numBodyNodes, gravity, timeStep, options);
}

template <class BodyNodeT, class GravityT>
void rneaBodyNodes(
    BodyNodeT* const* bodyNodes,
    std::size_t numBodyNodes,
    const GravityT& gravity,
    double timeStep,
    RneaOptions options)
{
  if (numBodyNodes == 0) [[unlikely]] {
    return;
  }

  const bool withExternalForces = options.mWithExternalForces;
  const bool withDampingForces = options.mWithDampingForces;
  const bool withSpringForces = options.mWithSpringForces;
  auto* const bodyNodesEnd = bodyNodes + numBodyNodes;
  for (auto* it = bodyNodesEnd; it != bodyNodes;) {
    auto& bodyNode = **--it;
    ArticulatedDynamicsAccess::updateTransmittedForceID(
        bodyNode, gravity, withExternalForces);
    ArticulatedDynamicsAccess::updateJointForceID(
        bodyNode, timeStep, withDampingForces, withSpringForces);
  }
}

template <class BodyNodeT, class GravityT>
void abaBodyNodes(
    BodyNodeT* const* bodyNodes,
    std::size_t numBodyNodes,
    const GravityT& gravity,
    double timeStep)
{
  if (numBodyNodes == 0) [[unlikely]] {
    return;
  }

  auto* const bodyNodesEnd = bodyNodes + numBodyNodes;
  for (auto* it = bodyNodesEnd; it != bodyNodes;) {
    ArticulatedDynamicsAccess::updateBiasForce(**--it, gravity, timeStep);
  }

  for (auto* it = bodyNodes; it != bodyNodesEnd; ++it) {
    auto& bodyNode = **it;
    ArticulatedDynamicsAccess::updateAccelerationFD(bodyNode);
    ArticulatedDynamicsAccess::updateTransmittedForceFD(bodyNode);
    ArticulatedDynamicsAccess::updateJointForceFD(
        bodyNode, timeStep, true, true);
  }
}

template <class BodyNodeT, class GravityT>
void abaBodyNodes(
    BodyNodeT* const* bodyNodes,
    std::size_t numBodyNodes,
    const GravityT& gravity,
    double timeStep,
    AbaOptions options)
{
  if (options.mWithDampingForces && options.mWithSpringForces) [[likely]] {
    abaBodyNodes(bodyNodes, numBodyNodes, gravity, timeStep);
    return;
  }

  if (numBodyNodes == 0) [[unlikely]] {
    return;
  }

  const bool withDampingForces = options.mWithDampingForces;
  const bool withSpringForces = options.mWithSpringForces;
  auto* const bodyNodesEnd = bodyNodes + numBodyNodes;
  for (auto* it = bodyNodesEnd; it != bodyNodes;) {
    ArticulatedDynamicsAccess::updateBiasForce(**--it, gravity, timeStep);
  }

  for (auto* it = bodyNodes; it != bodyNodesEnd; ++it) {
    auto& bodyNode = **it;
    ArticulatedDynamicsAccess::updateAccelerationFD(bodyNode);
    ArticulatedDynamicsAccess::updateTransmittedForceFD(bodyNode);
    ArticulatedDynamicsAccess::updateJointForceFD(
        bodyNode, timeStep, withDampingForces, withSpringForces);
  }
}

/// Recursive Newton-Euler inverse dynamics over a dynamic Skeleton-like model
/// adapter.
template <class Model>
void rneaDynamic(Model& model, RneaOptions options)
{
  rneaBodyNodes(
      model.getBodyNodeData(),
      model.getNumBodyNodes(),
      model.getGravity(),
      model.getTimeStep(),
      options);
}

/// Recursive Newton-Euler inverse dynamics over a Skeleton-like model adapter.
template <class Model>
void rnea(Model& model, RneaOptions options)
{
  if (model.getNumDofs() == 0) [[unlikely]] {
    return;
  }

  rneaDynamic(model, options);
}

/// Articulated Body Algorithm forward dynamics over a Skeleton-like model
/// adapter.
template <class Model>
void aba(Model& model)
{
  abaBodyNodes(
      model.getBodyNodeData(),
      model.getNumBodyNodes(),
      model.getGravity(),
      model.getTimeStep());
}

/// Articulated Body Algorithm forward dynamics over a Skeleton-like model
/// adapter with explicit joint-force options.
template <class Model>
void aba(Model& model, AbaOptions options)
{
  abaBodyNodes(
      model.getBodyNodeData(),
      model.getNumBodyNodes(),
      model.getGravity(),
      model.getTimeStep(),
      options);
}

/// Batched RNEA over independent model adapters.
///
/// The phase-oriented traversal keeps the dependency order for each model while
/// exposing same-phase work across models. That gives future packed adapters a
/// narrow place to use cache-friendly layouts or SIMD lanes without changing
/// the public Skeleton API.
template <class Model>
void rneaBatch(std::span<Model> models, RneaOptions options)
{
  if (models.empty()) {
    return;
  }

  std::size_t maxNumBodyNodes = 0;
  const auto uniformNumBodyNodes = models.front().getNumBodyNodes();
  bool hasUniformBodyCount = true;
  bool hasOnlyDynamicModels = true;
  const bool withExternalForces = options.mWithExternalForces;
  const bool withDampingForces = options.mWithDampingForces;
  const bool withSpringForces = options.mWithSpringForces;
  for (const auto& model : models) {
    const auto numBodyNodes = model.getNumBodyNodes();
    maxNumBodyNodes = std::max(maxNumBodyNodes, numBodyNodes);
    hasUniformBodyCount &= numBodyNodes == uniformNumBodyNodes;
    hasOnlyDynamicModels &= model.getNumDofs() != 0;
  }

  if (hasUniformBodyCount && hasOnlyDynamicModels) {
    for (std::size_t i = uniformNumBodyNodes; i > 0; --i) {
      for (auto& model : models) {
        auto& bodyNode = model.getBodyNode(i - 1);
        model.updateTransmittedForceID(
            bodyNode, model.getGravity(), withExternalForces);
        model.updateJointForceID(
            bodyNode, model.getTimeStep(), withDampingForces, withSpringForces);
      }
    }
    return;
  }

  for (std::size_t i = maxNumBodyNodes; i > 0; --i) {
    for (auto& model : models) {
      if (model.getNumDofs() == 0 || i > model.getNumBodyNodes()) {
        continue;
      }

      auto& bodyNode = model.getBodyNode(i - 1);
      model.updateTransmittedForceID(
          bodyNode, model.getGravity(), withExternalForces);
      model.updateJointForceID(
          bodyNode, model.getTimeStep(), withDampingForces, withSpringForces);
    }
  }
}

/// Batched ABA over independent model adapters.
template <class Model>
void abaBatch(std::span<Model> models, AbaOptions options = {})
{
  if (models.empty()) {
    return;
  }

  std::size_t maxNumBodyNodes = 0;
  const auto uniformNumBodyNodes = models.front().getNumBodyNodes();
  bool hasUniformBodyCount = true;
  const bool withDampingForces = options.mWithDampingForces;
  const bool withSpringForces = options.mWithSpringForces;
  for (const auto& model : models) {
    const auto numBodyNodes = model.getNumBodyNodes();
    maxNumBodyNodes = std::max(maxNumBodyNodes, numBodyNodes);
    hasUniformBodyCount &= numBodyNodes == uniformNumBodyNodes;
  }

  if (hasUniformBodyCount) {
    for (std::size_t i = uniformNumBodyNodes; i > 0; --i) {
      for (auto& model : models) {
        model.updateBiasForce(
            model.getBodyNode(i - 1), model.getGravity(), model.getTimeStep());
      }
    }

    for (std::size_t i = 0; i < uniformNumBodyNodes; ++i) {
      for (auto& model : models) {
        auto& bodyNode = model.getBodyNode(i);
        model.updateAccelerationFD(bodyNode);
        model.updateTransmittedForceFD(bodyNode);
        model.updateJointForceFD(
            bodyNode, model.getTimeStep(), withDampingForces, withSpringForces);
      }
    }
    return;
  }

  for (std::size_t i = maxNumBodyNodes; i > 0; --i) {
    for (auto& model : models) {
      if (i <= model.getNumBodyNodes()) {
        model.updateBiasForce(
            model.getBodyNode(i - 1), model.getGravity(), model.getTimeStep());
      }
    }
  }

  for (std::size_t i = 0; i < maxNumBodyNodes; ++i) {
    for (auto& model : models) {
      if (i >= model.getNumBodyNodes()) {
        continue;
      }

      auto& bodyNode = model.getBodyNode(i);
      model.updateAccelerationFD(bodyNode);
      model.updateTransmittedForceFD(bodyNode);
      model.updateJointForceFD(
          bodyNode, model.getTimeStep(), withDampingForces, withSpringForces);
    }
  }
}

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_ARTICULATEDDYNAMICSALGORITHMS_HPP_
