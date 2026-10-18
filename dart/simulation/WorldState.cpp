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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "dart/collision/detail/CollisionHistory.hpp"
#include "dart/constraint/BallJointConstraint.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/constraint/CylindricalJointConstraint.hpp"
#include "dart/constraint/RevoluteJointConstraint.hpp"
#include "dart/constraint/WeldJointConstraint.hpp"
#include "dart/constraint/detail/ContactWarmStartCache.hpp"
#include "dart/dynamics/BallJoint.hpp"
#include "dart/dynamics/EndEffector.hpp"
#include "dart/dynamics/EulerJoint.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Marker.hpp"
#include "dart/dynamics/PlanarJoint.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/ScrewJoint.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/TranslationalJoint.hpp"
#include "dart/dynamics/TranslationalJoint2D.hpp"
#include "dart/dynamics/UniversalJoint.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/simulation/World.hpp"

#include <stdexcept>
#include <type_traits>
#include <typeinfo>

#include <cmath>

namespace dart::simulation {
namespace {

struct StateShape
{
  std::size_t dofs = 0;
  std::size_t bodies = 0;
  std::size_t softBodies = 0;
};

enum class StateJointType
{
  Ball,
  Euler,
  Free,
  Planar,
  Prismatic,
  Revolute,
  Screw,
  Translational,
  Translational2D,
  Universal,
  Weld,
};

StateJointType jointType(const dynamics::Joint& joint)
{
  if (typeid(joint) == typeid(dynamics::BallJoint))
    return StateJointType::Ball;
  if (typeid(joint) == typeid(dynamics::EulerJoint))
    return StateJointType::Euler;
  if (typeid(joint) == typeid(dynamics::FreeJoint))
    return StateJointType::Free;
  if (typeid(joint) == typeid(dynamics::PlanarJoint))
    return StateJointType::Planar;
  if (typeid(joint) == typeid(dynamics::PrismaticJoint))
    return StateJointType::Prismatic;
  if (typeid(joint) == typeid(dynamics::RevoluteJoint))
    return StateJointType::Revolute;
  if (typeid(joint) == typeid(dynamics::ScrewJoint))
    return StateJointType::Screw;
  if (typeid(joint) == typeid(dynamics::TranslationalJoint))
    return StateJointType::Translational;
  if (typeid(joint) == typeid(dynamics::TranslationalJoint2D))
    return StateJointType::Translational2D;
  if (typeid(joint) == typeid(dynamics::UniversalJoint))
    return StateJointType::Universal;
  if (typeid(joint) == typeid(dynamics::WeldJoint))
    return StateJointType::Weld;
  throw std::invalid_argument("WorldState does not support custom Joint state");
}

void require(bool condition, const char* message)
{
  if (!condition)
    throw std::invalid_argument(message);
}

enum class StateConstraintType
{
  Ball,
  Revolute,
  Cylindrical,
  Weld,
};

StateConstraintType constraintType(const constraint::ConstraintBase& manual)
{
  if (typeid(manual) == typeid(constraint::BallJointConstraint))
    return StateConstraintType::Ball;
  if (typeid(manual) == typeid(constraint::RevoluteJointConstraint))
    return StateConstraintType::Revolute;
  if (typeid(manual) == typeid(constraint::CylindricalJointConstraint))
    return StateConstraintType::Cylindrical;
  if (typeid(manual) == typeid(constraint::WeldJointConstraint))
    return StateConstraintType::Weld;
  throw std::invalid_argument(
      "WorldState does not support this manual constraint type");
}

template <typename Emit>
void inspectConstraintEndpoint(
    const World& world, const dynamics::BodyNode* endpoint, Emit emit)
{
  if (!endpoint) {
    emit(0); // World endpoint, distinct from every owned body.
    return;
  }
  // Check ownership before dereferencing the constraint's non-owning pointer.
  for (std::size_t s = 0; s < world.getNumSkeletons(); ++s) {
    const auto skeleton = world.getSkeleton(s);
    for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b)
      if (skeleton->getBodyNode(b) == endpoint) {
        emit(s + 1);
        emit(b);
        return;
      }
  }
  throw std::invalid_argument(
      "WorldState manual constraint endpoint is not owned by the World");
}

template <typename Emit>
StateShape inspectStructure(const World& world, Emit emit)
{
  require(
      typeid(world) == typeid(World),
      "WorldState does not support custom World state");
  StateShape shape;
  emit(world.getNumSkeletons());
  for (std::size_t s = 0; s < world.getNumSkeletons(); ++s) {
    const auto skeleton = world.getSkeleton(s);
    const dynamics::Skeleton& skeletonValue = *skeleton;
    require(
        typeid(skeletonValue) == typeid(dynamics::Skeleton),
        "WorldState does not support custom Skeleton state");
    emit(skeleton->getNumBodyNodes());
    emit(skeleton->getNumDofs());
    shape.dofs += skeleton->getNumDofs();
    shape.bodies += skeleton->getNumBodyNodes();
    for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b) {
      const auto* body = skeleton->getBodyNode(b);
      const bool soft = typeid(*body) == typeid(dynamics::SoftBodyNode);
      require(
          soft || typeid(*body) == typeid(dynamics::BodyNode),
          "WorldState does not support custom BodyNode state");
      emit(soft ? 1 : 0);
      const auto* parent = body->getParentBodyNode();
      emit(parent ? parent->getIndexInSkeleton() + 1 : 0);
      const auto* joint = body->getParentJoint();
      emit(static_cast<std::size_t>(jointType(*joint)));
      emit(joint->getNumDofs());
      for (std::size_t d = 0; d < joint->getNumDofs(); ++d)
        emit(joint->getIndexInSkeleton(d));
      if (!soft)
        continue;
      ++shape.softBodies;
      const auto* softBody = static_cast<const dynamics::SoftBodyNode*>(body);
      emit(softBody->getNumPointMasses());
      for (std::size_t p = 0; p < softBody->getNumPointMasses(); ++p) {
        const auto* point = softBody->getPointMass(p);
        require(
            typeid(*point) == typeid(dynamics::PointMass),
            "WorldState does not support custom PointMass state");
        emit(point->getNumConnectedPointMasses());
        for (std::size_t c = 0; c < point->getNumConnectedPointMasses(); ++c)
          emit(point->getConnectedPointMass(c)->getIndexInSoftBodyNode());
      }
      emit(softBody->getNumFaces());
      for (std::size_t f = 0; f < softBody->getNumFaces(); ++f)
        for (Eigen::Index i = 0; i < 3; ++i)
          emit(static_cast<std::size_t>(softBody->getFace(f)[i]));
    }
  }
  const auto* solver = world.getConstraintSolver();
  emit(solver->getNumConstraints());
  for (std::size_t c = 0; c < solver->getNumConstraints(); ++c) {
    const auto manual = solver->getConstraint(c);
    require(manual != nullptr, "WorldState manual constraint is null");
    emit(static_cast<std::size_t>(constraintType(*manual)));
    const auto& joint
        = static_cast<const constraint::DynamicJointConstraint&>(*manual);
    require(
        joint.getBodyNode1() != nullptr,
        "WorldState manual constraint requires a first body endpoint");
    inspectConstraintEndpoint(world, joint.getBodyNode1(), emit);
    inspectConstraintEndpoint(world, joint.getBodyNode2(), emit);
  }
  return shape;
}

void validateShapes(const WorldState& state, const StateShape& shape)
{
  const auto dofs = static_cast<Eigen::Index>(shape.dofs);
  require(
      state.positions.size() == dofs && state.velocities.size() == dofs
          && state.accelerations.size() == dofs && state.forces.size() == dofs
          && state.commands.size() == dofs
          && state.bodyExternalForces.cols()
                 == static_cast<Eigen::Index>(shape.bodies)
          && state.softBodies.size() == shape.softBodies,
      "WorldState channel shapes do not match the World");
}

void validateChannels(const WorldState& state, const StateShape& shape)
{
  validateShapes(state, shape);
  require(
      std::isfinite(state.time) && state.positions.allFinite()
          && state.velocities.allFinite() && state.accelerations.allFinite()
          && state.forces.allFinite() && state.commands.allFinite()
          && state.bodyExternalForces.allFinite(),
      "WorldState channels must be finite");
  for (const auto& soft : state.softBodies) {
    require(
        soft.points.size() == soft.externalForces.size(),
        "WorldState soft channel shapes do not match");
    for (std::size_t p = 0; p < soft.points.size(); ++p) {
      const auto& point = soft.points[p];
      require(
          point.mPositions.allFinite() && point.mVelocities.allFinite()
              && point.mAccelerations.allFinite() && point.mForces.allFinite()
              && soft.externalForces[p].allFinite(),
          "WorldState soft channels must be finite");
    }
  }
}

} // namespace

void World::validateDynamicStateSupport() const
{
  const auto validate = [](const common::Composite& composite) {
    for (const auto& entry : composite.mAspectMap) {
      const auto* aspect = entry.second.get();
      if (!aspect)
        continue;
      const auto& type = typeid(*aspect);
      if (type == typeid(dynamics::detail::BodyNodeVectorProxyAspect)
          || type == typeid(dynamics::detail::JointVectorProxyAspect)
          || type == typeid(dynamics::detail::NodeVectorProxyAspect)
          || type == typeid(dynamics::BodyNode::Aspect)
          || type == typeid(dynamics::SoftBodyNode::Aspect)
          || type == typeid(dynamics::Support)
          || type == typeid(dynamics::GenericJoint<math::R1Space>::Aspect)
          || type == typeid(dynamics::GenericJoint<math::R2Space>::Aspect)
          || type == typeid(dynamics::GenericJoint<math::R3Space>::Aspect)
          || type == typeid(dynamics::GenericJoint<math::SO3Space>::Aspect)
          || type == typeid(dynamics::GenericJoint<math::SE3Space>::Aspect))
        continue;
      require(
          !aspect->getAspectState(),
          "WorldState does not support this Aspect state");
    }
  };
  for (const auto& skeleton : mSkeletons) {
    validate(*skeleton);
    for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b) {
      const auto* body = skeleton->getBodyNode(b);
      validate(*body);
      validate(*body->getParentJoint());
      for (const auto& entry : body->mNodeMap)
        for (const auto* node : entry.second) {
          require(
              typeid(*node) == typeid(dynamics::ShapeNode)
                  || typeid(*node) == typeid(dynamics::Marker)
                  || typeid(*node) == typeid(dynamics::EndEffector),
              "WorldState does not support custom Node state");
          if (const auto* composite
              = dynamic_cast<const common::Composite*>(node))
            validate(*composite);
        }
    }
  }
}

WorldState World::captureState() const
{
  WorldState output;
  captureState(output);
  return output;
}

void World::captureState(WorldState& output) const
{
  validateDynamicStateSupport();
  if (!output.mInitialized) {
    WorldState initialized;
    const auto shape = inspectStructure(*this, [&](std::size_t token) {
      initialized.mStructure.push_back(token);
    });
    const auto dofs = static_cast<Eigen::Index>(shape.dofs);
    initialized.positions.resize(dofs);
    initialized.velocities.resize(dofs);
    initialized.accelerations.resize(dofs);
    initialized.forces.resize(dofs);
    initialized.commands.resize(dofs);
    initialized.bodyExternalForces.resize(
        6, static_cast<Eigen::Index>(shape.bodies));
    initialized.softBodies.resize(shape.softBodies);
    std::size_t softIndex = 0;
    for (const auto& skeleton : mSkeletons)
      for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b)
        if (const auto* soft = dynamic_cast<const dynamics::SoftBodyNode*>(
                skeleton->getBodyNode(b))) {
          initialized.softBodies[softIndex].points.resize(
              soft->getNumPointMasses());
          initialized.softBodies[softIndex++].externalForces.resize(
              soft->getNumPointMasses());
        }
    initialized.mInitialized = true;
    captureState(initialized);
    output = std::move(initialized);
    return;
  }

  std::size_t cursor = 0;
  const auto shape = inspectStructure(*this, [&](std::size_t token) {
    require(
        cursor < output.mStructure.size() && output.mStructure[cursor] == token,
        "WorldState structure does not match the World");
    ++cursor;
  });
  require(
      cursor == output.mStructure.size(),
      "WorldState structure does not match the World");
  // Validate destination shapes and source finiteness before changing output.
  validateShapes(output, shape);
  require(std::isfinite(mTime), "WorldState time must be finite");
  std::size_t softIndex = 0;
  for (const auto& skeleton : mSkeletons) {
    for (std::size_t d = 0; d < skeleton->getNumDofs(); ++d)
      require(
          std::isfinite(skeleton->getPosition(d))
              && std::isfinite(skeleton->getVelocity(d))
              && std::isfinite(skeleton->getAcceleration(d))
              && std::isfinite(skeleton->getForce(d))
              && std::isfinite(skeleton->getCommand(d)),
          "WorldState channels must be finite");
    for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b) {
      const auto* body = skeleton->getBodyNode(b);
      require(
          body->getExternalForceLocal().allFinite(),
          "WorldState external forces must be finite");
      if (const auto* soft
          = dynamic_cast<const dynamics::SoftBodyNode*>(body)) {
        const auto& target = output.softBodies[softIndex++];
        require(
            target.points.size() == soft->getNumPointMasses()
                && target.externalForces.size() == soft->getNumPointMasses(),
            "WorldState output soft channel shapes do not match the World");
        for (std::size_t p = 0; p < soft->getNumPointMasses(); ++p) {
          const auto* point = soft->getPointMass(p);
          const auto& state = point->getState();
          require(
              state.mPositions.allFinite() && state.mVelocities.allFinite()
                  && state.mAccelerations.allFinite()
                  && state.mForces.allFinite() && point->mFext.allFinite(),
              "WorldState soft channels must be finite");
        }
      }
    }
  }

  Eigen::Index dofIndex = 0;
  Eigen::Index bodyIndex = 0;
  softIndex = 0;
  for (const auto& skeleton : mSkeletons) {
    for (std::size_t d = 0; d < skeleton->getNumDofs(); ++d, ++dofIndex) {
      output.positions[dofIndex] = skeleton->getPosition(d);
      output.velocities[dofIndex] = skeleton->getVelocity(d);
      output.accelerations[dofIndex] = skeleton->getAcceleration(d);
      output.forces[dofIndex] = skeleton->getForce(d);
      output.commands[dofIndex] = skeleton->getCommand(d);
    }
    for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b, ++bodyIndex) {
      const auto* body = skeleton->getBodyNode(b);
      output.bodyExternalForces.col(bodyIndex) = body->getExternalForceLocal();
      if (const auto* soft
          = dynamic_cast<const dynamics::SoftBodyNode*>(body)) {
        auto& target = output.softBodies[softIndex++];
        for (std::size_t p = 0; p < soft->getNumPointMasses(); ++p) {
          const auto* point = soft->getPointMass(p);
          target.points[p] = point->getState();
          target.externalForces[p] = point->mFext;
        }
      }
    }
  }
  output.time = mTime;
}

void World::restoreState(const WorldState& state)
{
  validateDynamicStateSupport();
  require(state.mInitialized, "WorldState has not been captured");
  std::size_t cursor = 0;
  const auto shape = inspectStructure(*this, [&](std::size_t token) {
    require(
        cursor < state.mStructure.size() && state.mStructure[cursor] == token,
        "WorldState structure does not match the World");
    ++cursor;
  });
  require(
      cursor == state.mStructure.size(),
      "WorldState structure does not match the World");
  validateChannels(state, shape);
  std::size_t softIndex = 0;
  for (const auto& skeleton : mSkeletons)
    for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b)
      if (const auto* soft = dynamic_cast<const dynamics::SoftBodyNode*>(
              skeleton->getBodyNode(b)))
        require(
            state.softBodies[softIndex++].points.size()
                == soft->getNumPointMasses(),
            "WorldState soft channel shapes do not match the World");

  Eigen::Index skeletonOffset = 0;
  Eigen::Index bodyIndex = 0;
  softIndex = 0;
  for (const auto& skeleton : mSkeletons) {
    for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b, ++bodyIndex) {
      auto* body = skeleton->getBodyNode(b);
      auto* joint = body->getParentJoint();
      const auto restore = [&](auto* typed) {
        using Vector = typename std::remove_pointer_t<decltype(typed)>::Vector;
        Vector positions, velocities, accelerations, forces, commands;
        for (std::size_t d = 0; d < joint->getNumDofs(); ++d) {
          const auto i
              = skeletonOffset
                + static_cast<Eigen::Index>(joint->getIndexInSkeleton(d));
          positions[d] = state.positions[i];
          velocities[d] = state.velocities[i];
          accelerations[d] = state.accelerations[i];
          forces[d] = state.forces[i];
          commands[d] = state.commands[i];
        }
        typed->restoreDynamicState(
            positions, velocities, accelerations, forces, commands);
      };
      switch (jointType(*joint)) {
        case StateJointType::Ball:
          restore(static_cast<dynamics::GenericJoint<math::SO3Space>*>(joint));
          break;
        case StateJointType::Euler:
        case StateJointType::Planar:
        case StateJointType::Translational:
          restore(static_cast<dynamics::GenericJoint<math::R3Space>*>(joint));
          break;
        case StateJointType::Free:
          restore(static_cast<dynamics::GenericJoint<math::SE3Space>*>(joint));
          break;
        case StateJointType::Prismatic:
        case StateJointType::Revolute:
        case StateJointType::Screw:
          restore(static_cast<dynamics::GenericJoint<math::R1Space>*>(joint));
          break;
        case StateJointType::Translational2D:
        case StateJointType::Universal:
          restore(static_cast<dynamics::GenericJoint<math::R2Space>*>(joint));
          break;
        case StateJointType::Weld:
          break;
      }
      body->restoreDynamicExternalForce(
          state.bodyExternalForces.col(bodyIndex));
      if (auto* soft = dynamic_cast<dynamics::SoftBodyNode*>(body)) {
        const auto& source = state.softBodies[softIndex++];
        soft->restoreDynamicState(source.points, source.externalForces);
      }
    }
    skeletonOffset += static_cast<Eigen::Index>(skeleton->getNumDofs());
  }
  for (std::size_t c = 0; c < mConstraintSolver->getNumConstraints(); ++c) {
    auto* manual = mConstraintSolver->getConstraint(c).get();
    switch (constraintType(*manual)) {
      case StateConstraintType::Ball:
        static_cast<constraint::BallJointConstraint*>(manual)
            ->resetDynamicState();
        break;
      case StateConstraintType::Revolute:
        static_cast<constraint::RevoluteJointConstraint*>(manual)
            ->resetDynamicState();
        break;
      case StateConstraintType::Cylindrical:
        static_cast<constraint::CylindricalJointConstraint*>(manual)
            ->resetDynamicState();
        break;
      case StateConstraintType::Weld:
        static_cast<constraint::WeldJointConstraint*>(manual)
            ->resetDynamicState();
        break;
    }
  }
  // Retire references to detector-owned contacts before invalidating history.
  auto& solver = *mConstraintSolver;
  solver.mConstrainedGroups.clear();
  solver.mActiveConstraints.clear();
  solver.mContactConstraints.clear();
  solver.mReusableContactConstraints.clear();
  solver.mSoftContactConstraints.clear();
  solver.mReusableSoftContactConstraints.clear();
  solver.mJointConstraints.clear();
  solver.mMimicMotorConstraints.clear();
  solver.mCouplerConstraints.clear();
  solver.mJointCoulombFrictionConstraints.clear();
  solver.mGroupResting.clear();
  solver.mGroupAllSleepCandidates.clear();
  solver.mGroupPreserveSleepCandidates.clear();
  solver.mHadDeactivationGroups = false;
  solver.clearLastCollisionResult();
  if (auto* cache = constraint::detail::findContactWarmStartCache(&solver))
    cache->clear();
  collision::detail::CollisionHistory::clear(*solver.getCollisionGroup());
  for (const auto& skeleton : mSkeletons) {
    skeleton->clearConstraintImpulses();
    skeleton->setImpulseApplied(false);
    skeleton->clearPositionConstraintImpulses();
    skeleton->clearPositionVelocityChanges();
    skeleton->setPositionImpulseApplied(false);
    skeleton->resetUnion();
    skeleton->setIslandIndex(-1);
    DART_SUPPRESS_DEPRECATED_BEGIN
    skeleton->clearCollidingBodies();
    DART_SUPPRESS_DEPRECATED_END
  }
  // Dynamic restoration starts a fresh active trajectory, not exact history.
  wakeRestingSkeletonsForWorldChange();
  mTime = state.time;
}

} // namespace dart::simulation
