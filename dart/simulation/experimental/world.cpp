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

#include "dart/simulation/experimental/world.hpp"

#include "dart/simulation/experimental/body/rigid_body.hpp"
#include "dart/simulation/experimental/common/ecs_utils.hpp"
#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/dynamics/forward_dynamics.hpp"
#include "dart/simulation/experimental/frame/fixed_frame.hpp"
#include "dart/simulation/experimental/frame/frame.hpp"
#include "dart/simulation/experimental/frame/free_frame.hpp"
#include "dart/simulation/experimental/io/binary_io.hpp"
#include "dart/simulation/experimental/io/serializer.hpp"
#include "dart/simulation/experimental/multi_body/multi_body.hpp"
#include "dart/simulation/experimental/shape/shape_node.hpp"

#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/collision/CollisionGroup.hpp>

#include <dart/dynamics/BallJoint.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Inertia.hpp>
#include <dart/dynamics/PlanarJoint.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/ScrewJoint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/UniversalJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <dart/math/MathTypes.hpp>

#include <algorithm>
#include <format>
#include <istream>
#include <limits>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cmath>

namespace {

template <typename... Components>
std::size_t countEntities(const entt::registry& registry)
{
  std::size_t count = 0;
  auto view = registry.view<Components...>();
  for (auto entity : view) {
    (void)entity;
    ++count;
  }
  return count;
}

template <typename Component>
bool hasEntityWithName(const entt::registry& registry, std::string_view name)
{
  auto view
      = registry.view<Component, dart::simulation::experimental::comps::Name>();
  for (auto entity : view) {
    const auto& info
        = view.template get<dart::simulation::experimental::comps::Name>(
            entity);
    if (info.name == name) {
      return true;
    }
  }
  return false;
}

} // namespace

namespace dart::simulation::experimental {

struct World::ClassicAdapter
{
  explicit ClassicAdapter(World& world);

  void rebuild();
  void syncFromExperimental();
  void solveConstraints();
  void integratePositionsAndSync(
      double timeStep,
      const std::unordered_map<entt::entity, Eigen::VectorXd>&
          preConstraintVelocities);
  void updateTimeStep(double timeStep);
  void updateGravity(const Eigen::Vector3d& gravity);
  void setConstraintSolver(constraint::UniqueConstraintSolverPtr solver);

  constraint::ConstraintSolver* getConstraintSolver() const;
  const collision::CollisionResult& getLastCollisionResult() const;

private:
  World& m_world;

  std::unordered_map<entt::entity, dynamics::SkeletonPtr> m_skeletonByMultiBody;
  std::unordered_map<entt::entity, dynamics::BodyNode*> m_bodyNodeByLink;
  std::unordered_map<entt::entity, dynamics::Joint*> m_jointByEntity;
  std::unordered_map<entt::entity, comps::JointType> m_jointTypeByEntity;

  std::unordered_map<entt::entity, dynamics::SkeletonPtr> m_skeletonByRigidBody;
  std::unordered_map<entt::entity, dynamics::BodyNode*> m_bodyNodeByRigidBody;
  std::unordered_map<entt::entity, dynamics::FreeJoint*> m_freeJointByRigidBody;

  std::vector<dynamics::SkeletonPtr> m_allSkeletons;
  std::unique_ptr<constraint::ConstraintSolver> m_constraintSolver;

  void buildMultiBodySkeleton(entt::entity mbEntity);
  void buildRigidBodySkeleton(entt::entity rbEntity);
  dynamics::BodyNode* findBodyNode(entt::entity parentEntity) const;
  void attachShapeNodes();
  void syncJointLimits(const comps::Joint& jointComp, dynamics::Joint* joint);

  static Eigen::VectorXd reorderFreeToClassic(
      const Eigen::VectorXd& experimental);
  static Eigen::VectorXd reorderFreeToExperimental(
      const Eigen::VectorXd& classic);
  static bool hasFiniteLimit(
      const Eigen::VectorXd& lower, const Eigen::VectorXd& upper);
};

World::World() = default;

//==============================================================================
World::ClassicAdapter::ClassicAdapter(World& world) : m_world(world) {}

//==============================================================================
Eigen::VectorXd World::ClassicAdapter::reorderFreeToClassic(
    const Eigen::VectorXd& experimental)
{
  if (experimental.size() != 6) {
    return experimental;
  }

  Eigen::VectorXd classic(6);
  classic.head<3>() = experimental.tail<3>();
  classic.tail<3>() = experimental.head<3>();
  return classic;
}

//==============================================================================
Eigen::VectorXd World::ClassicAdapter::reorderFreeToExperimental(
    const Eigen::VectorXd& classic)
{
  if (classic.size() != 6) {
    return classic;
  }

  Eigen::VectorXd experimental(6);
  experimental.head<3>() = classic.tail<3>();
  experimental.tail<3>() = classic.head<3>();
  return experimental;
}

//==============================================================================
bool World::ClassicAdapter::hasFiniteLimit(
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper)
{
  if (lower.size() == 0 || upper.size() == 0 || lower.size() != upper.size()) {
    return false;
  }

  for (int i = 0; i < lower.size(); ++i) {
    if (std::isfinite(lower[i]) || std::isfinite(upper[i])) {
      return true;
    }
  }
  return false;
}

//==============================================================================
dynamics::BodyNode* World::ClassicAdapter::findBodyNode(
    entt::entity parentEntity) const
{
  const auto linkIt = m_bodyNodeByLink.find(parentEntity);
  if (linkIt != m_bodyNodeByLink.end()) {
    return linkIt->second;
  }

  const auto rigidIt = m_bodyNodeByRigidBody.find(parentEntity);
  if (rigidIt != m_bodyNodeByRigidBody.end()) {
    return rigidIt->second;
  }

  return nullptr;
}

//==============================================================================
void World::ClassicAdapter::syncJointLimits(
    const comps::Joint& jointComp, dynamics::Joint* joint)
{
  if (!joint) {
    return;
  }

  const std::size_t dof = joint->getNumDofs();

  if (jointComp.limits.lower.size() == static_cast<int>(dof)) {
    joint->setPositionLowerLimits(jointComp.limits.lower);
  }
  if (jointComp.limits.upper.size() == static_cast<int>(dof)) {
    joint->setPositionUpperLimits(jointComp.limits.upper);
  }

  if (jointComp.limits.velocity.size() == static_cast<int>(dof)) {
    Eigen::VectorXd lower = -jointComp.limits.velocity;
    Eigen::VectorXd upper = jointComp.limits.velocity;
    joint->setVelocityLowerLimits(lower);
    joint->setVelocityUpperLimits(upper);
  }

  if (jointComp.limits.effort.size() == static_cast<int>(dof)) {
    Eigen::VectorXd lower = -jointComp.limits.effort;
    Eigen::VectorXd upper = jointComp.limits.effort;
    joint->setForceLowerLimits(lower);
    joint->setForceUpperLimits(upper);
  }

  joint->setLimitEnforcement(
      hasFiniteLimit(jointComp.limits.lower, jointComp.limits.upper));
}

//==============================================================================
void World::ClassicAdapter::buildMultiBodySkeleton(entt::entity mbEntity)
{
  auto& registry = m_world.getRegistry();
  const auto* nameComp = registry.try_get<comps::Name>(mbEntity);
  const std::string mbName = nameComp ? nameComp->name : "multibody";

  auto skeleton = dynamics::Skeleton::create(mbName);
  skeleton->setTimeStep(m_world.getTimeStep());
  skeleton->setGravity(m_world.getGravity());

  m_skeletonByMultiBody[mbEntity] = skeleton;
  m_allSkeletons.push_back(skeleton);

  const auto& structure = registry.get<comps::MultiBodyStructure>(mbEntity);

  auto buildSubtree = [&](auto&& self,
                          entt::entity linkEntity,
                          dynamics::BodyNode* parentBody) -> void {
    if (m_bodyNodeByLink.find(linkEntity) != m_bodyNodeByLink.end()) {
      return;
    }

    const auto& linkComp = registry.get<comps::Link>(linkEntity);
    dynamics::BodyNode::Properties bodyProps;
    bodyProps.mName = linkComp.name;

    dynamics::Joint* joint = nullptr;
    dynamics::BodyNode* bodyNode = nullptr;
    comps::JointType jointType = comps::JointType::Fixed;

    if (!parentBody) {
      dynamics::WeldJoint::Properties jointProps;
      jointProps.mName = std::format("{}_root_joint", linkComp.name);
      jointProps.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
      jointProps.mT_ChildBodyToJoint
          = linkComp.transformFromParentJoint.inverse();

      auto pair = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(
          nullptr, jointProps, bodyProps);
      joint = pair.first;
      bodyNode = pair.second;
    } else {
      const auto& jointComp = registry.get<comps::Joint>(linkComp.parentJoint);
      jointType = jointComp.type;

      switch (jointComp.type) {
        case comps::JointType::Fixed: {
          dynamics::WeldJoint::Properties jointProps;
          jointProps.mName = jointComp.name;
          jointProps.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
          jointProps.mT_ChildBodyToJoint
              = linkComp.transformFromParentJoint.inverse();
          auto pair = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(
              parentBody, jointProps, bodyProps);
          joint = pair.first;
          bodyNode = pair.second;
          break;
        }
        case comps::JointType::Revolute: {
          dynamics::RevoluteJoint::Properties jointProps;
          jointProps.mName = jointComp.name;
          jointProps.mAxis = jointComp.axis;
          jointProps.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
          jointProps.mT_ChildBodyToJoint
              = linkComp.transformFromParentJoint.inverse();
          auto pair
              = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
                  parentBody, jointProps, bodyProps);
          joint = pair.first;
          bodyNode = pair.second;
          break;
        }
        case comps::JointType::Prismatic: {
          dynamics::PrismaticJoint::Properties jointProps;
          jointProps.mName = jointComp.name;
          jointProps.mAxis = jointComp.axis;
          jointProps.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
          jointProps.mT_ChildBodyToJoint
              = linkComp.transformFromParentJoint.inverse();
          auto pair
              = skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
                  parentBody, jointProps, bodyProps);
          joint = pair.first;
          bodyNode = pair.second;
          break;
        }
        case comps::JointType::Screw: {
          dynamics::ScrewJoint::Properties jointProps;
          jointProps.mName = jointComp.name;
          jointProps.mAxis = jointComp.axis;
          jointProps.mPitch = jointComp.pitch;
          jointProps.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
          jointProps.mT_ChildBodyToJoint
              = linkComp.transformFromParentJoint.inverse();
          auto pair
              = skeleton->createJointAndBodyNodePair<dynamics::ScrewJoint>(
                  parentBody, jointProps, bodyProps);
          joint = pair.first;
          bodyNode = pair.second;
          break;
        }
        case comps::JointType::Universal: {
          dynamics::UniversalJoint::Properties jointProps;
          jointProps.mName = jointComp.name;
          jointProps.mAxis[0] = jointComp.axis;
          jointProps.mAxis[1] = jointComp.axis2;
          jointProps.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
          jointProps.mT_ChildBodyToJoint
              = linkComp.transformFromParentJoint.inverse();
          auto pair
              = skeleton->createJointAndBodyNodePair<dynamics::UniversalJoint>(
                  parentBody, jointProps, bodyProps);
          joint = pair.first;
          bodyNode = pair.second;
          break;
        }
        case comps::JointType::Ball: {
          dynamics::BallJoint::Properties jointProps;
          jointProps.mName = jointComp.name;
          jointProps.mCoordinateChart
              = dynamics::BallJoint::CoordinateChart::EULER_ZYX;
          jointProps.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
          jointProps.mT_ChildBodyToJoint
              = linkComp.transformFromParentJoint.inverse();
          auto pair = skeleton->createJointAndBodyNodePair<dynamics::BallJoint>(
              parentBody, jointProps, bodyProps);
          joint = pair.first;
          bodyNode = pair.second;
          break;
        }
        case comps::JointType::Planar: {
          dynamics::PlanarJoint::Properties jointProps;
          jointProps.mName = jointComp.name;
          jointProps.mPlaneType = dynamics::PlanarJoint::PlaneType::ARBITRARY;

          const Eigen::Vector3d normal = jointComp.axis.normalized();
          const Eigen::Vector3d axis1 = jointComp.axis2.normalized();
          const Eigen::Vector3d axis2 = normal.cross(axis1).normalized();

          jointProps.mTransAxis1 = axis1;
          jointProps.mTransAxis2 = axis2;
          jointProps.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
          jointProps.mT_ChildBodyToJoint
              = linkComp.transformFromParentJoint.inverse();
          auto pair
              = skeleton->createJointAndBodyNodePair<dynamics::PlanarJoint>(
                  parentBody, jointProps, bodyProps);
          joint = pair.first;
          bodyNode = pair.second;
          break;
        }
        case comps::JointType::Free: {
          dynamics::FreeJoint::Properties jointProps;
          jointProps.mName = jointComp.name;
          jointProps.mCoordinateChart
              = dynamics::FreeJoint::CoordinateChart::EULER_ZYX;
          jointProps.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
          jointProps.mT_ChildBodyToJoint
              = linkComp.transformFromParentJoint.inverse();
          auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
              parentBody, jointProps, bodyProps);
          joint = pair.first;
          bodyNode = pair.second;
          break;
        }
        case comps::JointType::Custom:
        default:
          DART_EXPERIMENTAL_THROW_T(
              InvalidArgumentException,
              "Unsupported joint type for classic adapter");
      }
    }

    if (bodyNode) {
      m_bodyNodeByLink[linkEntity] = bodyNode;
      if (joint && linkComp.parentJoint != entt::null) {
        m_jointByEntity[linkComp.parentJoint] = joint;
        m_jointTypeByEntity[linkComp.parentJoint] = jointType;
      }

      const auto& mass = linkComp.mass;
      dynamics::Inertia inertia;
      inertia.setMass(mass.mass);
      inertia.setLocalCOM(mass.localCOM);
      inertia.setMoment(mass.inertia);
      bodyNode->setInertia(inertia);
    }

    for (auto childJointEntity : linkComp.childJoints) {
      const auto& childJoint = registry.get<comps::Joint>(childJointEntity);
      self(self, childJoint.childLink, bodyNode);
    }
  };

  for (auto linkEntity : structure.links) {
    const auto& linkComp = registry.get<comps::Link>(linkEntity);
    if (linkComp.parentJoint == entt::null) {
      buildSubtree(buildSubtree, linkEntity, nullptr);
    }
  }
}

//==============================================================================
void World::ClassicAdapter::buildRigidBodySkeleton(entt::entity rbEntity)
{
  auto& registry = m_world.getRegistry();
  const auto* nameComp = registry.try_get<comps::Name>(rbEntity);
  const std::string rbName = nameComp ? nameComp->name : "rigid_body";

  auto skeleton = dynamics::Skeleton::create(rbName);
  skeleton->setTimeStep(m_world.getTimeStep());
  skeleton->setGravity(m_world.getGravity());

  dynamics::FreeJoint::Properties jointProps;
  jointProps.mName = std::format("{}_root_joint", rbName);
  jointProps.mCoordinateChart = dynamics::FreeJoint::CoordinateChart::EULER_ZYX;

  dynamics::BodyNode::Properties bodyProps;
  bodyProps.mName = rbName;

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProps, bodyProps);

  m_skeletonByRigidBody[rbEntity] = skeleton;
  m_bodyNodeByRigidBody[rbEntity] = pair.second;
  m_freeJointByRigidBody[rbEntity] = pair.first;
  m_allSkeletons.push_back(skeleton);

  const auto& massProps = registry.get<comps::MassProperties>(rbEntity);
  dynamics::Inertia inertia;
  inertia.setMass(massProps.mass);
  inertia.setLocalCOM(massProps.localCOM);
  inertia.setMoment(massProps.inertia);
  pair.second->setInertia(inertia);
}

//==============================================================================
void World::ClassicAdapter::attachShapeNodes()
{
  auto& registry = m_world.getRegistry();
  auto view = registry.view<comps::ShapeNodeTag, comps::ShapeNode>();

  for (auto entity : view) {
    auto& shapeComp = view.get<comps::ShapeNode>(entity);
    if (shapeComp.classicShapeNode || !shapeComp.shape) {
      continue;
    }

    auto* bodyNode = findBodyNode(shapeComp.parentEntity);
    if (!bodyNode) {
      continue;
    }

    auto* shapeNode = bodyNode->createShapeNodeWith<
        dynamics::CollisionAspect,
        dynamics::DynamicsAspect>(shapeComp.shape);

    shapeNode->setRelativeTransform(shapeComp.relativeTransform);
    if (auto* collisionAspect = shapeNode->getCollisionAspect()) {
      collisionAspect->setCollidable(shapeComp.collidable);
    }
    if (auto* dynamicsAspect = shapeNode->getDynamicsAspect()) {
      dynamicsAspect->setFrictionCoeff(shapeComp.frictionCoeff);
      dynamicsAspect->setRestitutionCoeff(shapeComp.restitutionCoeff);
    }

    shapeComp.classicShapeNode = shapeNode;
  }
}

//==============================================================================
void World::ClassicAdapter::rebuild()
{
  m_skeletonByMultiBody.clear();
  m_bodyNodeByLink.clear();
  m_jointByEntity.clear();
  m_jointTypeByEntity.clear();
  m_skeletonByRigidBody.clear();
  m_bodyNodeByRigidBody.clear();
  m_freeJointByRigidBody.clear();
  m_allSkeletons.clear();

  m_constraintSolver = std::make_unique<constraint::ConstraintSolver>();
  m_constraintSolver->setTimeStep(m_world.getTimeStep());

  auto& registry = m_world.getRegistry();

  auto mbView = registry.view<comps::MultiBodyTag>();
  for (auto mbEntity : mbView) {
    buildMultiBodySkeleton(mbEntity);
  }

  auto rbView = registry.view<comps::RigidBodyTag>();
  for (auto rbEntity : rbView) {
    buildRigidBodySkeleton(rbEntity);
  }

  attachShapeNodes();

  for (const auto& skeleton : m_allSkeletons) {
    m_constraintSolver->addSkeleton(skeleton);
  }
}

//==============================================================================
void World::ClassicAdapter::syncFromExperimental()
{
  auto& registry = m_world.getRegistry();

  for (const auto& [linkEntity, bodyNode] : m_bodyNodeByLink) {
    const auto& linkComp = registry.get<comps::Link>(linkEntity);
    const auto& mass = linkComp.mass;
    dynamics::Inertia inertia;
    inertia.setMass(mass.mass);
    inertia.setLocalCOM(mass.localCOM);
    inertia.setMoment(mass.inertia);
    bodyNode->setInertia(inertia);
  }

  for (const auto& [rbEntity, bodyNode] : m_bodyNodeByRigidBody) {
    const auto& massProps = registry.get<comps::MassProperties>(rbEntity);
    dynamics::Inertia inertia;
    inertia.setMass(massProps.mass);
    inertia.setLocalCOM(massProps.localCOM);
    inertia.setMoment(massProps.inertia);
    bodyNode->setInertia(inertia);
  }

  for (const auto& [jointEntity, jointPtr] : m_jointByEntity) {
    const auto& jointComp = registry.get<comps::Joint>(jointEntity);
    Eigen::VectorXd positions = jointComp.position;
    Eigen::VectorXd velocities = jointComp.velocity;
    Eigen::VectorXd accelerations = jointComp.acceleration;
    Eigen::VectorXd forces = jointComp.torque;

    if (jointComp.type == comps::JointType::Free) {
      positions = reorderFreeToClassic(positions);
      velocities = reorderFreeToClassic(velocities);
      accelerations = reorderFreeToClassic(accelerations);
      forces = reorderFreeToClassic(forces);
    }

    jointPtr->setPositions(positions);
    jointPtr->setVelocities(velocities);
    jointPtr->setAccelerations(accelerations);
    jointPtr->setForces(forces);

    syncJointLimits(jointComp, jointPtr);
  }

  for (const auto& [rbEntity, freeJoint] : m_freeJointByRigidBody) {
    const auto& transform = registry.get<comps::Transform>(rbEntity);
    const auto& velocity = registry.get<comps::Velocity>(rbEntity);

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = transform.position;
    tf.linear() = transform.orientation.toRotationMatrix();

    freeJoint->setTransform(tf, dynamics::Frame::World());

    Eigen::Vector6d spatialVelocity;
    spatialVelocity.head<3>() = velocity.angular;
    spatialVelocity.tail<3>() = velocity.linear;
    freeJoint->setSpatialVelocity(
        spatialVelocity, dynamics::Frame::World(), dynamics::Frame::World());
  }

  for (const auto& skeleton : m_allSkeletons) {
    skeleton->computeForwardKinematics(true, true, false);
  }
}

//==============================================================================
void World::ClassicAdapter::solveConstraints()
{
  if (m_constraintSolver) {
    m_constraintSolver->solve();
  }
}

//==============================================================================
void World::ClassicAdapter::integratePositionsAndSync(
    double timeStep,
    const std::unordered_map<entt::entity, Eigen::VectorXd>&
        preConstraintVelocities)
{
  for (const auto& skeleton : m_allSkeletons) {
    if (!skeleton->isMobile()) {
      continue;
    }

    if (skeleton->isImpulseApplied()) {
      skeleton->computeImpulseForwardDynamics();
      skeleton->setImpulseApplied(false);
    }

    if (skeleton->isPositionImpulseApplied()) {
      skeleton->integratePositions(
          timeStep, skeleton->getPositionVelocityChanges());
      skeleton->setPositionImpulseApplied(false);
      skeleton->clearPositionVelocityChanges();
    } else {
      skeleton->integratePositions(timeStep);
    }
  }

  auto& registry = m_world.getRegistry();

  for (const auto& [jointEntity, jointPtr] : m_jointByEntity) {
    auto& jointComp = registry.get<comps::Joint>(jointEntity);

    Eigen::VectorXd positions = jointPtr->getPositions();
    Eigen::VectorXd velocities = jointPtr->getVelocities();
    Eigen::VectorXd accelerations = jointPtr->getAccelerations();

    if (jointComp.type == comps::JointType::Free) {
      positions = reorderFreeToExperimental(positions);
      velocities = reorderFreeToExperimental(velocities);
      accelerations = reorderFreeToExperimental(accelerations);
    }

    jointComp.position = positions;
    jointComp.velocity = velocities;

    const auto it = preConstraintVelocities.find(jointEntity);
    if (it != preConstraintVelocities.end()
        && it->second.size() == velocities.size() && timeStep > 0.0) {
      jointComp.acceleration = (velocities - it->second) / timeStep;
    } else {
      jointComp.acceleration = accelerations;
    }

    if (jointComp.childLink != entt::null
        && registry.valid(jointComp.childLink)) {
      auto& linkComp = registry.get<comps::Link>(jointComp.childLink);
      linkComp.needLocalTransformUpdate = true;
    }
  }

  for (const auto& [rbEntity, bodyNode] : m_bodyNodeByRigidBody) {
    auto& transform = registry.get<comps::Transform>(rbEntity);
    auto& velocity = registry.get<comps::Velocity>(rbEntity);

    const Eigen::Isometry3d tf = bodyNode->getWorldTransform();
    transform.position = tf.translation();
    transform.orientation = Eigen::Quaterniond(tf.linear());

    const Eigen::Vector6d spatialVelocity = bodyNode->getSpatialVelocity(
        dynamics::Frame::World(), dynamics::Frame::World());
    velocity.angular = spatialVelocity.head<3>();
    velocity.linear = spatialVelocity.tail<3>();

    if (auto* props = registry.try_get<comps::FreeFrameProperties>(rbEntity)) {
      props->localTransform = tf;
    }
    if (auto* cache = registry.try_get<comps::FrameCache>(rbEntity)) {
      cache->needTransformUpdate = true;
    }
  }
}

//==============================================================================
void World::ClassicAdapter::updateTimeStep(double timeStep)
{
  if (m_constraintSolver) {
    m_constraintSolver->setTimeStep(timeStep);
  }

  for (const auto& skeleton : m_allSkeletons) {
    skeleton->setTimeStep(timeStep);
  }
}

//==============================================================================
void World::ClassicAdapter::updateGravity(const Eigen::Vector3d& gravity)
{
  for (const auto& skeleton : m_allSkeletons) {
    skeleton->setGravity(gravity);
  }
}

//==============================================================================
void World::ClassicAdapter::setConstraintSolver(
    constraint::UniqueConstraintSolverPtr solver)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !solver, InvalidArgumentException, "Cannot set nullptr as solver");

  if (m_constraintSolver) {
    solver->setFromOtherConstraintSolver(*m_constraintSolver);
  } else {
    for (const auto& skeleton : m_allSkeletons) {
      solver->addSkeleton(skeleton);
    }
  }

  solver->setTimeStep(m_world.getTimeStep());
  m_constraintSolver = std::move(solver);
}

//==============================================================================
constraint::ConstraintSolver* World::ClassicAdapter::getConstraintSolver() const
{
  return m_constraintSolver.get();
}

//==============================================================================
const collision::CollisionResult&
World::ClassicAdapter::getLastCollisionResult() const
{
  return m_constraintSolver->getLastCollisionResult();
}

//==============================================================================
entt::registry& World::getRegistry()
{
  return m_registry;
}

//==============================================================================
const entt::registry& World::getRegistry() const
{
  return m_registry;
}

//==============================================================================
void World::clear()
{
  m_registry.clear();
  m_simulationMode = false;
  m_freeFrameCounter = 0;
  m_fixedFrameCounter = 0;
  m_multiBodyCounter = 0;
  m_rigidBodyCounter = 0;
  m_linkCounter = 0;
  m_jointCounter = 0;
  m_shapeNodeCounter = 0;
  m_classicAdapter.reset();
}

//==============================================================================
void World::ensureDesignMode() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      m_simulationMode,
      InvalidOperationException,
      "World modifications are not allowed while in simulation mode");
}

//==============================================================================
FreeFrame World::addFreeFrame()
{
  return addFreeFrame("", Frame::world());
}

//==============================================================================
FreeFrame World::addFreeFrame(std::string_view name)
{
  return addFreeFrame(name, Frame::world());
}

//==============================================================================
FreeFrame World::addFreeFrame(std::string_view name, const Frame& parent)
{
  ensureDesignMode();
  Frame parentFrame = resolveParentFrame(parent);

  std::string actualName;
  auto entity = createFrameEntity(
      name,
      parentFrame,
      Eigen::Isometry3d::Identity(),
      &m_freeFrameCounter,
      "free_frame",
      false,
      actualName);

  return FreeFrame(entity, this);
}

//==============================================================================
FixedFrame World::addFixedFrame(std::string_view name, const Frame& parent)
{
  return addFixedFrame(name, parent, Eigen::Isometry3d::Identity());
}

//==============================================================================
FixedFrame World::addFixedFrame(
    std::string_view name, const Frame& parent, const Eigen::Isometry3d& offset)
{
  ensureDesignMode();
  Frame parentFrame = resolveParentFrame(parent);

  DART_EXPERIMENTAL_THROW_T_IF(
      name.empty(),
      InvalidArgumentException,
      "FixedFrame requires a non-empty name");

  DART_EXPERIMENTAL_THROW_T_IF(
      parentFrame.isWorld(),
      InvalidArgumentException,
      "FixedFrame cannot be attached directly to the world frame");

  std::string actualName;
  auto entity = createFrameEntity(
      name,
      parentFrame,
      offset,
      &m_fixedFrameCounter,
      "fixed_frame",
      true,
      actualName);

  return FixedFrame(entity, this);
}

//==============================================================================
entt::entity World::createFrameEntity(
    std::string_view name,
    const Frame& parentFrame,
    const Eigen::Isometry3d& localTransform,
    std::size_t* autoNameCounter,
    std::string_view autoNamePrefix,
    bool isFixedFrame,
    std::string& outName)
{
  std::string actualName;
  if (name.empty()) {
    if (autoNameCounter) {
      actualName
          = std::format("{}_{:03d}", autoNamePrefix, ++(*autoNameCounter));
    } else {
      actualName = std::string(autoNamePrefix);
    }
  } else {
    actualName = std::string(name);
  }

  auto entity = m_registry.create();
  m_registry.emplace<comps::Name>(entity, actualName);
  m_registry.emplace<comps::FrameTag>(entity);

  if (isFixedFrame) {
    m_registry.emplace<comps::FixedFrameTag>(entity);
  } else {
    m_registry.emplace<comps::FreeFrameTag>(entity);
  }

  auto& state = m_registry.emplace<comps::FrameState>(entity);
  state.parentFrame = parentFrame.getEntity();

  auto& cache = m_registry.emplace<comps::FrameCache>(entity);
  cache.worldTransform = Eigen::Isometry3d::Identity();
  cache.needTransformUpdate = true;

  if (isFixedFrame) {
    auto& props = m_registry.emplace<comps::FixedFrameProperties>(entity);
    props.localTransform = localTransform;
  } else {
    auto& props = m_registry.emplace<comps::FreeFrameProperties>(entity);
    props.localTransform = localTransform;
  }

  outName = actualName;
  return entity;
}

//==============================================================================
ShapeNode World::createShapeNode(
    entt::entity parentEntity,
    const dart::dynamics::ShapePtr& shape,
    std::string_view name,
    const ShapeNodeOptions& options)
{
  ensureDesignMode();

  DART_EXPERIMENTAL_THROW_T_IF(
      !m_registry.valid(parentEntity),
      InvalidArgumentException,
      "ShapeNode parent entity is invalid");

  const bool isLink = m_registry.any_of<comps::Link>(parentEntity);
  const bool isRigidBody = m_registry.any_of<comps::RigidBodyTag>(parentEntity);
  DART_EXPERIMENTAL_THROW_T_IF(
      !isLink && !isRigidBody,
      InvalidArgumentException,
      "ShapeNode parent must be a Link or RigidBody");

  std::string actualName;
  if (name.empty()) {
    actualName = std::format("shape_node_{:03d}", ++m_shapeNodeCounter);
  } else {
    actualName = std::string(name);
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      hasEntityWithName<comps::ShapeNodeTag>(m_registry, actualName),
      InvalidArgumentException,
      "ShapeNode '{}' already exists",
      actualName);

  auto entity = m_registry.create();
  m_registry.emplace<comps::Name>(entity, actualName);
  m_registry.emplace<comps::ShapeNodeTag>(entity);
  auto& shapeComp = m_registry.emplace<comps::ShapeNode>(entity);
  shapeComp.parentEntity = parentEntity;
  shapeComp.relativeTransform = options.relativeTransform;
  shapeComp.shape = shape;
  shapeComp.collidable = options.collidable;
  shapeComp.frictionCoeff = options.frictionCoeff;
  shapeComp.restitutionCoeff = options.restitutionCoeff;

  return ShapeNode(entity, this);
}

//==============================================================================
Frame World::resolveParentFrame(const Frame& parent) const
{
  if (parent.isWorld()) {
    return Frame(entt::null, const_cast<World*>(this));
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      !parent.isValid(),
      InvalidArgumentException,
      "Parent frame is invalid or has been destroyed");

  DART_EXPERIMENTAL_THROW_T_IF(
      parent.getWorld() != this,
      InvalidArgumentException,
      "Parent frame belongs to a different world");

  return parent;
}

//==============================================================================
MultiBody World::addMultiBody(std::string_view name)
{
  ensureDesignMode();

  std::string candidateName
      = name.empty() ? std::format("multibody_{:03d}", m_multiBodyCounter + 1)
                     : std::string(name);

  if (name.empty()) {
    ++m_multiBodyCounter;
  }

  auto entity = m_registry.create();
  m_registry.emplace<comps::Name>(entity, candidateName);
  m_registry.emplace<comps::MultiBodyTag>(entity);
  m_registry.emplace<comps::MultiBodyStructure>(entity);

  return MultiBody(entity, this);
}

//==============================================================================
std::optional<MultiBody> World::getMultiBody(std::string_view name)
{
  auto view = m_registry.view<comps::MultiBodyTag, comps::Name>();
  for (auto entity : view) {
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name) {
      return MultiBody(entity, this);
    }
  }
  return std::nullopt;
}

//==============================================================================
std::size_t World::getMultiBodyCount() const
{
  return countEntities<comps::MultiBodyTag>(m_registry);
}

//==============================================================================
RigidBody World::addRigidBody(
    std::string_view name, const RigidBodyOptions& options)
{
  ensureDesignMode();

  std::string candidateName
      = name.empty() ? std::format("rigid_body_{:03d}", m_rigidBodyCounter + 1)
                     : std::string(name);

  DART_EXPERIMENTAL_THROW_T_IF(
      hasEntityWithName<comps::RigidBodyTag>(m_registry, candidateName),
      InvalidArgumentException,
      "RigidBody '{}' already exists",
      candidateName);

  Frame parent = Frame(entt::null, this);

  std::string actualName;
  Eigen::Isometry3d initialTransform = Eigen::Isometry3d::Identity();
  initialTransform.translation() = options.position;
  initialTransform.linear() = options.orientation.toRotationMatrix();

  auto entity = createFrameEntity(
      name,
      parent,
      initialTransform,
      &m_rigidBodyCounter,
      "rigid_body",
      false,
      actualName);

  m_registry.emplace<comps::RigidBodyTag>(entity);

  auto& transform = m_registry.emplace<comps::Transform>(entity);
  transform.position = options.position;
  transform.orientation = options.orientation;

  auto& velocity = m_registry.emplace<comps::Velocity>(entity);
  velocity.linear = options.linearVelocity;
  velocity.angular = options.angularVelocity;

  auto& massProps = m_registry.emplace<comps::MassProperties>(entity);
  massProps.mass = options.mass;
  massProps.inertia = options.inertia;

  m_registry.emplace<comps::Force>(entity);

  return RigidBody(entity, this);
}

//==============================================================================
bool World::hasRigidBody(std::string_view name) const
{
  return hasEntityWithName<comps::RigidBodyTag>(m_registry, name);
}

//==============================================================================
std::size_t World::getRigidBodyCount() const
{
  return countEntities<comps::RigidBodyTag>(m_registry);
}

//==============================================================================
void World::enterSimulationMode()
{
  DART_EXPERIMENTAL_THROW_T_IF(
      m_simulationMode,
      InvalidArgumentException,
      "World is already in simulation mode");

  m_simulationMode = true;

  // Initial bake so that cached transforms are up-to-date.
  updateKinematics();

  m_classicAdapter = std::make_unique<ClassicAdapter>(*this);
  m_classicAdapter->rebuild();
  m_classicAdapter->syncFromExperimental();
}

//==============================================================================
void World::updateKinematics()
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_simulationMode,
      InvalidArgumentException,
      "updateKinematics() requires simulation mode");

  auto cacheView = m_registry.view<comps::FrameTag, comps::FrameCache>();

  for (auto entity : cacheView) {
    auto& cache = cacheView.get<comps::FrameCache>(entity);
    cache.needTransformUpdate = true;
  }

  for (auto entity : cacheView) {
    Frame frame(entity, this);
    const auto& transform = frame.getTransform();
    if (m_registry.all_of<comps::Link>(entity)) {
      auto& linkComp = m_registry.get<comps::Link>(entity);
      linkComp.worldTransform = transform;
    }
  }
}

//==============================================================================
void World::step(bool clearForces)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_simulationMode,
      InvalidArgumentException,
      "step() requires simulation mode");

  if (!m_classicAdapter) {
    m_classicAdapter = std::make_unique<ClassicAdapter>(*this);
    m_classicAdapter->rebuild();
  }

  updateKinematics();

  dynamics::ForwardDynamicsConfig fdConfig;
  fdConfig.gravity = m_gravity;
  fdConfig.timeStep = m_timeStep;
  dynamics::ForwardDynamicsSystem fds(fdConfig);

  std::unordered_map<entt::entity, Eigen::VectorXd> preConstraintVelocities;

  auto mbView = m_registry.view<comps::MultiBodyStructure>();
  for (auto mbEntity : mbView) {
    MultiBody mb(mbEntity, this);

    fds.compute(*this, mb);

    const auto& mbComp = m_registry.get<comps::MultiBodyStructure>(mbEntity);
    for (auto linkEntity : mbComp.links) {
      auto& linkComp = m_registry.get<comps::Link>(linkEntity);
      if (linkComp.parentJoint == entt::null) {
        continue;
      }

      auto& jointComp = m_registry.get<comps::Joint>(linkComp.parentJoint);
      const std::size_t dof = jointComp.getDOF();

      for (std::size_t i = 0; i < dof; ++i) {
        jointComp.velocity(i) += jointComp.acceleration(i) * m_timeStep;
      }

      preConstraintVelocities[linkComp.parentJoint] = jointComp.velocity;
      linkComp.needLocalTransformUpdate = true;
    }
  }

  auto rbView = m_registry.view<
      comps::RigidBodyTag,
      comps::Transform,
      comps::Velocity,
      comps::MassProperties,
      comps::Force>();
  for (auto rbEntity : rbView) {
    auto& transform = rbView.get<comps::Transform>(rbEntity);
    auto& velocity = rbView.get<comps::Velocity>(rbEntity);
    const auto& mass = rbView.get<comps::MassProperties>(rbEntity);
    const auto& force = rbView.get<comps::Force>(rbEntity);

    if (mass.mass > 0.0) {
      const Eigen::Matrix3d R = transform.orientation.toRotationMatrix();
      const Eigen::Matrix3d inertiaWorld = R * mass.inertia * R.transpose();

      Eigen::Vector3d angularAccel = Eigen::Vector3d::Zero();
      const double det = inertiaWorld.determinant();
      if (std::abs(det) > 1e-9) {
        angularAccel
            = inertiaWorld.inverse()
              * (force.torque
                 - velocity.angular.cross(inertiaWorld * velocity.angular));
      }

      const Eigen::Vector3d linearAccel = m_gravity + force.force / mass.mass;

      velocity.linear += linearAccel * m_timeStep;
      velocity.angular += angularAccel * m_timeStep;
    }
  }

  m_classicAdapter->syncFromExperimental();
  m_classicAdapter->solveConstraints();
  m_classicAdapter->integratePositionsAndSync(
      m_timeStep, preConstraintVelocities);

  updateKinematics();

  if (clearForces) {
    auto linkView = m_registry.view<comps::Link>();
    for (auto linkEntity : linkView) {
      auto& linkComp = linkView.get<comps::Link>(linkEntity);
      linkComp.externalForce.setZero();
      linkComp.externalTorque.setZero();
    }

    auto forceView = m_registry.view<comps::RigidBodyTag, comps::Force>();
    for (auto rbEntity : forceView) {
      auto& force = forceView.get<comps::Force>(rbEntity);
      force.force.setZero();
      force.torque.setZero();
    }
  }

  m_time += m_timeStep;
  m_frame++;
}

//==============================================================================
void World::setTimeStep(double dt)
{
  m_timeStep = dt;
  if (m_classicAdapter) {
    m_classicAdapter->updateTimeStep(dt);
  }
}

//==============================================================================
void World::setGravity(const Eigen::Vector3d& gravity)
{
  m_gravity = gravity;
  if (m_classicAdapter) {
    m_classicAdapter->updateGravity(gravity);
  }
}

//==============================================================================
void World::setConstraintSolver(constraint::UniqueConstraintSolverPtr solver)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_simulationMode,
      InvalidArgumentException,
      "setConstraintSolver() requires simulation mode");

  bool rebuilt = false;
  if (!m_classicAdapter) {
    m_classicAdapter = std::make_unique<ClassicAdapter>(*this);
    m_classicAdapter->rebuild();
    rebuilt = true;
  }

  m_classicAdapter->setConstraintSolver(std::move(solver));
  if (rebuilt) {
    updateKinematics();
    m_classicAdapter->syncFromExperimental();
  }
}

//==============================================================================
constraint::ConstraintSolver* World::getConstraintSolver()
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_simulationMode,
      InvalidArgumentException,
      "getConstraintSolver() requires simulation mode");

  bool rebuilt = false;
  if (!m_classicAdapter) {
    m_classicAdapter = std::make_unique<ClassicAdapter>(*this);
    m_classicAdapter->rebuild();
    rebuilt = true;
  }

  if (rebuilt) {
    updateKinematics();
    m_classicAdapter->syncFromExperimental();
  }

  return m_classicAdapter->getConstraintSolver();
}

//==============================================================================
const constraint::ConstraintSolver* World::getConstraintSolver() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_simulationMode,
      InvalidArgumentException,
      "getConstraintSolver() requires simulation mode");
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_classicAdapter,
      InvalidOperationException,
      "Constraint solver not initialized");
  return m_classicAdapter->getConstraintSolver();
}

//==============================================================================
void World::setCollisionDetector(
    const collision::CollisionDetectorPtr& collisionDetector)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_simulationMode,
      InvalidArgumentException,
      "setCollisionDetector() requires simulation mode");
  DART_EXPERIMENTAL_THROW_T_IF(
      !collisionDetector,
      InvalidArgumentException,
      "Collision detector must be non-null");

  auto* solver = getConstraintSolver();
  DART_EXPERIMENTAL_THROW_T_IF(
      !solver, InvalidOperationException, "Constraint solver not initialized");
  solver->setCollisionDetector(collisionDetector);
}

//==============================================================================
collision::CollisionDetectorPtr World::getCollisionDetector()
{
  auto* solver = getConstraintSolver();
  DART_EXPERIMENTAL_THROW_T_IF(
      !solver, InvalidOperationException, "Constraint solver not initialized");
  return solver->getCollisionDetector();
}

//==============================================================================
collision::ConstCollisionDetectorPtr World::getCollisionDetector() const
{
  const auto* solver = getConstraintSolver();
  DART_EXPERIMENTAL_THROW_T_IF(
      !solver, InvalidOperationException, "Constraint solver not initialized");
  return solver->getCollisionDetector();
}

//==============================================================================
const collision::CollisionResult& World::detectCollisions()
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_simulationMode,
      InvalidArgumentException,
      "detectCollisions() requires simulation mode");

  if (!m_classicAdapter) {
    m_classicAdapter = std::make_unique<ClassicAdapter>(*this);
    m_classicAdapter->rebuild();
  }

  updateKinematics();
  m_classicAdapter->syncFromExperimental();

  auto* solver = m_classicAdapter->getConstraintSolver();
  DART_EXPERIMENTAL_THROW_T_IF(
      !solver, InvalidOperationException, "Constraint solver not initialized");

  auto& result = solver->getLastCollisionResult();
  result.clear();

  auto collisionGroup = solver->getCollisionGroup();
  if (collisionGroup) {
    collisionGroup->collide(solver->getCollisionOption(), &result);
  }

  return result;
}

//==============================================================================
const collision::CollisionResult& World::getLastCollisionResult() const
{
  const auto* solver = getConstraintSolver();
  DART_EXPERIMENTAL_THROW_T_IF(
      !solver, InvalidOperationException, "Constraint solver not initialized");
  return solver->getLastCollisionResult();
}

//==============================================================================
void World::saveBinary(std::ostream& output) const
{
  io::writeFormatHeader(output);

  io::EntityMap entityMap;
  io::SerializerRegistry::instance().saveAllEntities(
      output, m_registry, entityMap);

  const std::uint8_t simulationFlag = m_simulationMode ? 1 : 0;
  io::writePOD(output, simulationFlag);
  io::writePOD(output, m_freeFrameCounter);
  io::writePOD(output, m_fixedFrameCounter);
  io::writePOD(output, m_multiBodyCounter);
  io::writePOD(output, m_rigidBodyCounter);
  io::writePOD(output, m_linkCounter);
  io::writePOD(output, m_jointCounter);
}

//==============================================================================
void World::loadBinary(std::istream& input)
{
  clear();

  io::readFormatHeader(input);

  io::EntityMap entityMap;
  io::SerializerRegistry::instance().loadAllEntities(
      input, m_registry, entityMap);

  // World metadata (optional for forward-compatibility)
  if (input.peek() != std::char_traits<char>::eof()) {
    std::uint8_t simulationFlag = 0;
    io::readPOD(input, simulationFlag);
    m_simulationMode = simulationFlag != 0;

    io::readPOD(input, m_freeFrameCounter);
    io::readPOD(input, m_fixedFrameCounter);
    io::readPOD(input, m_multiBodyCounter);
    io::readPOD(input, m_rigidBodyCounter);
    io::readPOD(input, m_linkCounter);
    io::readPOD(input, m_jointCounter);
  }

  // Ensure all frame entities have cache components (not serialized)
  auto frameView = m_registry.view<comps::FrameTag>();
  for (auto entity : frameView) {
    if (!m_registry.any_of<comps::FrameCache>(entity)) {
      auto& cache = m_registry.emplace<comps::FrameCache>(entity);
      cache.worldTransform = Eigen::Isometry3d::Identity();
      cache.needTransformUpdate = true;
    } else {
      auto& cache = m_registry.get<comps::FrameCache>(entity);
      cache.needTransformUpdate = true;
    }
  }

  resetCountersFromRegistry();

  if (m_simulationMode) {
    updateKinematics();
  }
}

//==============================================================================
void World::resetCountersFromRegistry()
{
  m_freeFrameCounter = std::max(
      m_freeFrameCounter, countEntities<comps::FreeFrameTag>(m_registry));
  m_fixedFrameCounter = std::max(
      m_fixedFrameCounter, countEntities<comps::FixedFrameTag>(m_registry));
  m_multiBodyCounter = std::max(
      m_multiBodyCounter, countEntities<comps::MultiBodyTag>(m_registry));
  m_rigidBodyCounter = std::max(
      m_rigidBodyCounter, countEntities<comps::RigidBodyTag>(m_registry));
  m_linkCounter
      = std::max(m_linkCounter, countEntities<comps::Link>(m_registry));
  m_jointCounter
      = std::max(m_jointCounter, countEntities<comps::Joint>(m_registry));
  m_shapeNodeCounter = std::max(
      m_shapeNodeCounter, countEntities<comps::ShapeNodeTag>(m_registry));
}

} // namespace dart::simulation::experimental
