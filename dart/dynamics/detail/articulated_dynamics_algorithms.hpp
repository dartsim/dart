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

#include <algorithm>
#include <span>

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

/// Recursive Newton-Euler inverse dynamics over a Skeleton-like model adapter.
template <class Model>
void rnea(Model& model, const RneaOptions& options)
{
  if (model.getNumDofs() == 0) {
    return;
  }

  const auto numBodyNodes = model.getNumBodyNodes();
  const auto& gravity = model.getGravity();
  const auto timeStep = model.getTimeStep();
  for (std::size_t i = numBodyNodes; i > 0; --i) {
    auto& bodyNode = model.getBodyNode(i - 1);
    model.updateTransmittedForceID(
        bodyNode, gravity, options.mWithExternalForces);
    model.updateJointForceID(
        bodyNode,
        timeStep,
        options.mWithDampingForces,
        options.mWithSpringForces);
  }
}

/// Articulated Body Algorithm forward dynamics over a Skeleton-like model
/// adapter.
template <class Model>
void aba(Model& model, const AbaOptions& options = {})
{
  const auto numBodyNodes = model.getNumBodyNodes();
  const auto& gravity = model.getGravity();
  const auto timeStep = model.getTimeStep();

  for (std::size_t i = numBodyNodes; i > 0; --i) {
    model.updateBiasForce(model.getBodyNode(i - 1), gravity, timeStep);
  }

  for (std::size_t i = 0; i < numBodyNodes; ++i) {
    auto& bodyNode = model.getBodyNode(i);
    model.updateAccelerationFD(bodyNode);
    model.updateTransmittedForceFD(bodyNode);
    model.updateJointForceFD(
        bodyNode,
        timeStep,
        options.mWithDampingForces,
        options.mWithSpringForces);
  }
}

/// Batched RNEA over independent model adapters.
///
/// The phase-oriented traversal keeps the dependency order for each model while
/// exposing same-phase work across models. That gives future packed adapters a
/// narrow place to use cache-friendly layouts or SIMD lanes without changing
/// the public Skeleton API.
template <class Model>
void rneaBatch(std::span<Model> models, const RneaOptions& options)
{
  if (models.empty()) {
    return;
  }

  std::size_t maxNumBodyNodes = 0;
  const auto uniformNumBodyNodes = models.front().getNumBodyNodes();
  bool hasUniformBodyCount = true;
  bool hasOnlyDynamicModels = true;
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
            bodyNode, model.getGravity(), options.mWithExternalForces);
        model.updateJointForceID(
            bodyNode,
            model.getTimeStep(),
            options.mWithDampingForces,
            options.mWithSpringForces);
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
          bodyNode, model.getGravity(), options.mWithExternalForces);
      model.updateJointForceID(
          bodyNode,
          model.getTimeStep(),
          options.mWithDampingForces,
          options.mWithSpringForces);
    }
  }
}

/// Batched ABA over independent model adapters.
template <class Model>
void abaBatch(std::span<Model> models, const AbaOptions& options = {})
{
  if (models.empty()) {
    return;
  }

  std::size_t maxNumBodyNodes = 0;
  const auto uniformNumBodyNodes = models.front().getNumBodyNodes();
  bool hasUniformBodyCount = true;
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
            bodyNode,
            model.getTimeStep(),
            options.mWithDampingForces,
            options.mWithSpringForces);
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
          bodyNode,
          model.getTimeStep(),
          options.mWithDampingForces,
          options.mWithSpringForces);
    }
  }
}

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_ARTICULATEDDYNAMICSALGORITHMS_HPP_
