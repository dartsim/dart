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

#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/comps/link.hpp>
#include <dart/simulation/experimental/comps/multi_body.hpp>
#include <dart/simulation/experimental/dynamics/forward_dynamics.hpp>
#include <dart/simulation/experimental/kinematics/joint_transform.hpp>
#include <dart/simulation/experimental/multi_body/joint.hpp>
#include <dart/simulation/experimental/multi_body/link.hpp>
#include <dart/simulation/experimental/multi_body/multi_body.hpp>
#include <dart/simulation/experimental/world.hpp>

namespace dart::simulation::experimental::dynamics {

ForwardDynamicsSystem::ForwardDynamicsSystem(
    const ForwardDynamicsConfig& config)
  : m_config(config)
{
}

void ForwardDynamicsSystem::initializeWorkspace(const MultiBody& multiBody)
{
  const std::size_t numLinks = multiBody.getLinkCount();

  std::vector<std::size_t> jointDOFs;
  jointDOFs.reserve(numLinks);

  auto& registry = multiBody.getWorld()->getRegistry();
  const auto& mbComp
      = registry.get<comps::MultiBodyStructure>(multiBody.getEntity());

  for (const auto& linkEntity : mbComp.links) {
    const auto& linkComp = registry.get<comps::Link>(linkEntity);
    if (linkComp.parentJoint != entt::null) {
      const auto& jointComp = registry.get<comps::Joint>(linkComp.parentJoint);
      jointDOFs.push_back(jointComp.getDOF());
    } else {
      jointDOFs.push_back(0);
    }
  }

  m_workspace.resize(numLinks, jointDOFs);
  m_linkTransforms.resize(numLinks);
  m_motionSubspaces.resize(numLinks);
}

void ForwardDynamicsSystem::computeArticulatedInertias(
    World& world,
    const MultiBody& multiBody,
    std::span<const Eigen::Isometry3d> linkTransforms)
{
  auto& registry = world.getRegistry();
  const auto& mbComp
      = registry.get<comps::MultiBodyStructure>(multiBody.getEntity());
  const std::size_t numLinks = mbComp.links.size();

  for (std::size_t i = numLinks; i > 0; --i) {
    const std::size_t idx = i - 1;
    const auto linkEntity = mbComp.links[idx];
    const auto& linkComp = registry.get<comps::Link>(linkEntity);

    auto& linkData = m_workspace.getLinkData(idx);

    linkData.articulatedInertia = makeSpatialInertia(
        linkComp.mass.mass, Eigen::Vector3d::Zero(), linkComp.mass.inertia);

    for (const auto childJointEntity : linkComp.childJoints) {
      const auto& childJoint = registry.get<comps::Joint>(childJointEntity);
      const auto childLinkEntity = childJoint.childLink;

      std::size_t childIdx = 0;
      for (std::size_t j = 0; j < numLinks; ++j) {
        if (mbComp.links[j] == childLinkEntity) {
          childIdx = j;
          break;
        }
      }

      const auto& childLinkData = m_workspace.getLinkData(childIdx);
      auto& childJointData = m_workspace.getJointData(childIdx);

      const std::size_t dof = childJoint.getDOF();
      if (dof == 0)
        continue;

      const auto S = computeMotionSubspace(childJoint);
      const auto& Ia = childLinkData.articulatedInertia;

      const Eigen::MatrixXd IaS = Ia * S;
      const Eigen::MatrixXd SIaS = S.transpose() * IaS;

      childJointData.projectedInertiaInverse = SIaS.inverse();

      const Eigen::Matrix6d Pa
          = Ia - IaS * childJointData.projectedInertiaInverse * IaS.transpose();

      const Eigen::Isometry3d T_parent_child
          = linkTransforms[idx].inverse() * linkTransforms[childIdx];

      linkData.articulatedInertia
          += transformSpatialInertia(T_parent_child.inverse(), Pa);
    }
  }
}

void ForwardDynamicsSystem::computeBiasForces(
    World& world,
    const MultiBody& multiBody,
    std::span<const Eigen::Isometry3d> linkTransforms)
{
  auto& registry = world.getRegistry();
  const auto& mbComp
      = registry.get<comps::MultiBodyStructure>(multiBody.getEntity());
  const std::size_t numLinks = mbComp.links.size();

  const Eigen::Vector6d gravityForce
      = makeSpatialForce(Eigen::Vector3d::Zero(), m_config.gravity);

  for (std::size_t i = numLinks; i > 0; --i) {
    const std::size_t idx = i - 1;
    const auto linkEntity = mbComp.links[idx];
    const auto& linkComp = registry.get<comps::Link>(linkEntity);

    auto& linkData = m_workspace.getLinkData(idx);

    const auto& Ia = linkData.articulatedInertia;
    const auto& V = linkData.spatialVelocity;

    linkData.biasForce = spatialCrossStar(V, Ia * V);

    const Eigen::Vector6d gravityInLink
        = transformSpatialForceInverse(linkTransforms[idx], gravityForce);
    linkData.biasForce -= linkComp.mass.mass * gravityInLink;

    for (const auto childJointEntity : linkComp.childJoints) {
      const auto& childJoint = registry.get<comps::Joint>(childJointEntity);
      const auto childLinkEntity = childJoint.childLink;

      std::size_t childIdx = 0;
      for (std::size_t j = 0; j < numLinks; ++j) {
        if (mbComp.links[j] == childLinkEntity) {
          childIdx = j;
          break;
        }
      }

      const auto& childLinkData = m_workspace.getLinkData(childIdx);
      auto& childJointData = m_workspace.getJointData(childIdx);

      const std::size_t dof = childJoint.getDOF();
      if (dof == 0)
        continue;

      const auto S = computeMotionSubspace(childJoint);
      const auto& Ia = childLinkData.articulatedInertia;
      const auto& pa = childLinkData.biasForce;

      childJointData.totalForce = childJoint.torque - S.transpose() * pa;

      const Eigen::MatrixXd IaS = Ia * S;
      const Eigen::VectorXd projBias
          = childJointData.projectedInertiaInverse
            * (childJointData.totalForce
               - IaS.transpose() * childLinkData.partialAcceleration);

      const Eigen::Isometry3d T_parent_child
          = linkTransforms[idx].inverse() * linkTransforms[childIdx];

      const auto pa_proj
          = pa + Ia * (S * projBias + childLinkData.partialAcceleration);
      linkData.biasForce
          += transformSpatialForceInverse(T_parent_child, pa_proj);
    }
  }
}

void ForwardDynamicsSystem::computeAccelerations(
    World& world,
    MultiBody& multiBody,
    std::span<const Eigen::Isometry3d> linkTransforms)
{
  auto& registry = world.getRegistry();
  const auto& mbComp
      = registry.get<comps::MultiBodyStructure>(multiBody.getEntity());
  const std::size_t numLinks = mbComp.links.size();

  for (std::size_t idx = 0; idx < numLinks; ++idx) {
    const auto linkEntity = mbComp.links[idx];
    auto& linkComp = registry.get<comps::Link>(linkEntity);

    auto& linkData = m_workspace.getLinkData(idx);

    if (linkComp.parentJoint == entt::null) {
      linkData.spatialAcceleration = Eigen::Vector6d::Zero();
      continue;
    }

    auto& jointComp = registry.get<comps::Joint>(linkComp.parentJoint);
    const std::size_t dof = jointComp.getDOF();

    if (dof == 0) {
      std::size_t parentIdx = 0;
      for (std::size_t j = 0; j < numLinks; ++j) {
        if (mbComp.links[j] == jointComp.parentLink) {
          parentIdx = j;
          break;
        }
      }
      const Eigen::Isometry3d T_parent_child
          = linkTransforms[parentIdx].inverse() * linkTransforms[idx];
      linkData.spatialAcceleration = transformSpatialVelocityInverse(
          T_parent_child,
          m_workspace.getLinkData(parentIdx).spatialAcceleration);
      continue;
    }

    auto& jointData = m_workspace.getJointData(idx);
    const auto& linkDataConst = m_workspace.getLinkData(idx);

    const auto S = computeMotionSubspace(jointComp);
    const auto& Ia = linkDataConst.articulatedInertia;

    std::size_t parentIdx = 0;
    for (std::size_t j = 0; j < numLinks; ++j) {
      if (mbComp.links[j] == jointComp.parentLink) {
        parentIdx = j;
        break;
      }
    }

    const Eigen::Isometry3d T_parent_child
        = linkTransforms[parentIdx].inverse() * linkTransforms[idx];
    const auto parentAccel = transformSpatialVelocityInverse(
        T_parent_child, m_workspace.getLinkData(parentIdx).spatialAcceleration);

    const Eigen::MatrixXd IaS = Ia * S;
    const Eigen::VectorXd ddq
        = jointData.projectedInertiaInverse
          * (jointData.totalForce
             - IaS.transpose()
                   * (parentAccel + linkDataConst.partialAcceleration));

    jointComp.acceleration = ddq;

    linkData.spatialAcceleration
        = parentAccel + linkDataConst.partialAcceleration + S * ddq;
  }
}

void ForwardDynamicsSystem::computeVelocities(
    World& world,
    const MultiBody& multiBody,
    std::span<const Eigen::Isometry3d> linkTransforms)
{
  auto& registry = world.getRegistry();
  const auto& mbComp
      = registry.get<comps::MultiBodyStructure>(multiBody.getEntity());
  const std::size_t numLinks = mbComp.links.size();

  for (std::size_t idx = 0; idx < numLinks; ++idx) {
    const auto linkEntity = mbComp.links[idx];
    const auto& linkComp = registry.get<comps::Link>(linkEntity);

    auto& linkData = m_workspace.getLinkData(idx);

    if (linkComp.parentJoint == entt::null) {
      linkData.spatialVelocity = Eigen::Vector6d::Zero();
      linkData.partialAcceleration = Eigen::Vector6d::Zero();
      continue;
    }

    const auto& jointComp = registry.get<comps::Joint>(linkComp.parentJoint);

    std::size_t parentIdx = 0;
    for (std::size_t j = 0; j < numLinks; ++j) {
      if (mbComp.links[j] == jointComp.parentLink) {
        parentIdx = j;
        break;
      }
    }

    const Eigen::Isometry3d T_parent_child
        = linkTransforms[parentIdx].inverse() * linkTransforms[idx];
    const auto parentVel = transformSpatialVelocityInverse(
        T_parent_child, m_workspace.getLinkData(parentIdx).spatialVelocity);

    const std::size_t dof = jointComp.getDOF();
    if (dof == 0) {
      linkData.spatialVelocity = parentVel;
      linkData.partialAcceleration = Eigen::Vector6d::Zero();
      continue;
    }

    const auto S = computeMotionSubspace(jointComp);
    const Eigen::VectorXd jointVel = S * jointComp.velocity;

    linkData.spatialVelocity = parentVel + jointVel;

    linkData.partialAcceleration
        = spatialCross(linkData.spatialVelocity, jointVel);
  }
}

void ForwardDynamicsSystem::compute(World& world, MultiBody& multiBody)
{
  initializeWorkspace(multiBody);

  auto& registry = world.getRegistry();
  const auto& mbComp
      = registry.get<comps::MultiBodyStructure>(multiBody.getEntity());

  for (std::size_t i = 0; i < mbComp.links.size(); ++i) {
    const auto& linkComp = registry.get<comps::Link>(mbComp.links[i]);
    m_linkTransforms[i] = linkComp.worldTransform;
  }

  m_workspace.reset();

  computeVelocities(world, multiBody, m_linkTransforms);
  computeArticulatedInertias(world, multiBody, m_linkTransforms);
  computeBiasForces(world, multiBody, m_linkTransforms);
  computeAccelerations(world, multiBody, m_linkTransforms);
}

void ForwardDynamicsSystem::computeAll(World& world)
{
  auto& registry = world.getRegistry();

  auto view = registry.view<comps::MultiBodyStructure>();
  for (auto entity : view) {
    MultiBody mb(entity, &world);
    compute(world, mb);
  }
}

} // namespace dart::simulation::experimental::dynamics
