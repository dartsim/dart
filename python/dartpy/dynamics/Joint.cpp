/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/dart.hpp>
#include <eigen_geometry_pybind.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void Joint(pybind11::module& m)
{
  ::pybind11::class_<dart::dynamics::detail::JointProperties>(
      m, "JointProperties")
      .def(::pybind11::init<>())
      .def(::pybind11::init<const std::string&>(), ::pybind11::arg("name"))
      .def_readwrite("mName", &dart::dynamics::detail::JointProperties::mName)
      .def_readwrite(
          "mT_ParentBodyToJoint",
          &dart::dynamics::detail::JointProperties::mT_ParentBodyToJoint)
      .def_readwrite(
          "mT_ChildBodyToJoint",
          &dart::dynamics::detail::JointProperties::mT_ChildBodyToJoint)
      .def_readwrite(
          "mIsPositionLimitEnforced",
          &dart::dynamics::detail::JointProperties::mIsPositionLimitEnforced)
      .def_readwrite(
          "mActuatorType",
          &dart::dynamics::detail::JointProperties::mActuatorType)
      .def_readwrite(
          "mMimicJoint", &dart::dynamics::detail::JointProperties::mMimicJoint)
      .def_readwrite(
          "mMimicMultiplier",
          &dart::dynamics::detail::JointProperties::mMimicMultiplier)
      .def_readwrite(
          "mMimicOffset",
          &dart::dynamics::detail::JointProperties::mMimicOffset);

  ::pybind11::class_<
      dart::common::SpecializedForAspect<dart::common::EmbeddedPropertiesAspect<
          dart::dynamics::Joint,
          dart::dynamics::detail::JointProperties>>,
      dart::common::Composite,
      std::shared_ptr<dart::common::SpecializedForAspect<
          dart::common::EmbeddedPropertiesAspect<
              dart::dynamics::Joint,
              dart::dynamics::detail::JointProperties>>>>(
      m, "SpecializedForAspect_EmbeddedPropertiesAspect_Joint_JointProperties")
      .def(::pybind11::init<>());

  ::pybind11::class_<
      dart::common::RequiresAspect<dart::common::EmbeddedPropertiesAspect<
          dart::dynamics::Joint,
          dart::dynamics::detail::JointProperties>>,
      dart::common::SpecializedForAspect<dart::common::EmbeddedPropertiesAspect<
          dart::dynamics::Joint,
          dart::dynamics::detail::JointProperties>>,
      std::shared_ptr<
          dart::common::RequiresAspect<dart::common::EmbeddedPropertiesAspect<
              dart::dynamics::Joint,
              dart::dynamics::detail::JointProperties>>>>(
      m, "RequiresAspect_EmbeddedPropertiesAspect_Joint_JointProperties")
      .def(::pybind11::init<>());

  ::pybind11::class_<
      dart::common::EmbedProperties<
          dart::dynamics::Joint,
          dart::dynamics::detail::JointProperties>,
      dart::common::RequiresAspect<dart::common::EmbeddedPropertiesAspect<
          dart::dynamics::Joint,
          dart::dynamics::detail::JointProperties>>,
      std::shared_ptr<dart::common::EmbedProperties<
          dart::dynamics::Joint,
          dart::dynamics::detail::JointProperties>>>(
      m, "EmbedProperties_Joint_JointProperties");

  ::pybind11::class_<
      dart::dynamics::Joint,
      dart::common::Subject,
      dart::common::EmbedProperties<
          dart::dynamics::Joint,
          dart::dynamics::detail::JointProperties>,
      std::shared_ptr<dart::dynamics::Joint>>(m, "Joint")
      .def(
          "hasJointAspect",
          +[](const dart::dynamics::Joint* self) -> bool {
            return self->hasJointAspect();
          })
      .def(
          "setJointAspect",
          +[](dart::dynamics::Joint* self,
              const dart::common::EmbedProperties<
                  dart::dynamics::Joint,
                  dart::dynamics::detail::JointProperties>::Aspect* aspect)
              -> void { return self->setJointAspect(aspect); },
          ::pybind11::arg("aspect"))
      .def(
          "removeJointAspect",
          +[](dart::dynamics::Joint* self) -> void {
            return self->removeJointAspect();
          })
      .def(
          "releaseJointAspect",
          +[](dart::dynamics::Joint* self)
              -> std::unique_ptr<dart::common::EmbedProperties<
                  dart::dynamics::Joint,
                  dart::dynamics::detail::JointProperties>::Aspect> {
            return self->releaseJointAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::Joint* self,
              const dart::dynamics::Joint::Properties& properties) -> void {
            return self->setProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::Joint* self,
              const dart::common::EmbedProperties<
                  dart::dynamics::Joint,
                  dart::dynamics::detail::JointProperties>::AspectProperties&
                  properties) -> void {
            return self->setAspectProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "copy",
          +[](dart::dynamics::Joint* self,
              const dart::dynamics::Joint& otherJoint) -> void {
            return self->copy(otherJoint);
          },
          ::pybind11::arg("otherJoint"))
      .def(
          "copy",
          +[](dart::dynamics::Joint* self,
              const dart::dynamics::Joint* otherJoint) -> void {
            return self->copy(otherJoint);
          },
          ::pybind11::arg("otherJoint"))
      .def(
          "setName",
          +[](dart::dynamics::Joint* self, const std::string& name)
              -> const std::string& { return self->setName(name); },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("name"))
      .def(
          "setName",
          +[](dart::dynamics::Joint* self,
              const std::string& name,
              bool renameDofs) -> const std::string& {
            return self->setName(name, renameDofs);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("name"),
          ::pybind11::arg("renameDofs"))
      .def(
          "getName",
          +[](const dart::dynamics::Joint* self) -> const std::string& {
            return self->getName();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getType",
          +[](const dart::dynamics::Joint* self) -> const std::string& {
            return self->getType();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "setActuatorType",
          +[](dart::dynamics::Joint* self,
              dart::dynamics::Joint::ActuatorType actuatorType) -> void {
            return self->setActuatorType(actuatorType);
          },
          ::pybind11::arg("actuatorType"))
      .def(
          "getActuatorType",
          +[](const dart::dynamics::Joint* self)
              -> dart::dynamics::Joint::ActuatorType {
            return self->getActuatorType();
          })
      .def(
          "isKinematic",
          +[](const dart::dynamics::Joint* self) -> bool {
            return self->isKinematic();
          })
      .def(
          "isDynamic",
          +[](const dart::dynamics::Joint* self) -> bool {
            return self->isDynamic();
          })
      .def(
          "getChildBodyNode",
          +[](dart::dynamics::Joint* self) -> dart::dynamics::BodyNode* {
            return self->getChildBodyNode();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getParentBodyNode",
          +[](dart::dynamics::Joint* self) -> dart::dynamics::BodyNode* {
            return self->getParentBodyNode();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getSkeleton",
          +[](dart::dynamics::Joint* self) -> dart::dynamics::SkeletonPtr {
            return self->getSkeleton();
          })
      .def(
          "getSkeleton",
          +[](const dart::dynamics::Joint* self)
              -> std::shared_ptr<const dart::dynamics::Skeleton> {
            return self->getSkeleton();
          })
      .def(
          "setTransformFromParentBodyNode",
          +[](dart::dynamics::Joint* self, const Eigen::Isometry3d& T) -> void {
            return self->setTransformFromParentBodyNode(T);
          },
          ::pybind11::arg("T"))
      .def(
          "setTransformFromChildBodyNode",
          +[](dart::dynamics::Joint* self, const Eigen::Isometry3d& T) -> void {
            return self->setTransformFromChildBodyNode(T);
          },
          ::pybind11::arg("T"))
      .def(
          "setPositionLimitEnforced",
          +[](dart::dynamics::Joint* self,
              bool isPositionLimitEnforced) -> void {
            return self->setPositionLimitEnforced(isPositionLimitEnforced);
          },
          ::pybind11::arg("isPositionLimitEnforced"))
      .def(
          "isPositionLimitEnforced",
          +[](const dart::dynamics::Joint* self) -> bool {
            return self->isPositionLimitEnforced();
          })
      .def(
          "getIndexInSkeleton",
          +[](const dart::dynamics::Joint* self, std::size_t index)
              -> std::size_t { return self->getIndexInSkeleton(index); },
          ::pybind11::arg("index"))
      .def(
          "getIndexInTree",
          +[](const dart::dynamics::Joint* self, std::size_t index)
              -> std::size_t { return self->getIndexInTree(index); },
          ::pybind11::arg("index"))
      .def(
          "getJointIndexInSkeleton",
          +[](const dart::dynamics::Joint* self) -> std::size_t {
            return self->getJointIndexInSkeleton();
          })
      .def(
          "getJointIndexInTree",
          +[](const dart::dynamics::Joint* self) -> std::size_t {
            return self->getJointIndexInTree();
          })
      .def(
          "getTreeIndex",
          +[](const dart::dynamics::Joint* self) -> std::size_t {
            return self->getTreeIndex();
          })
      .def(
          "setDofName",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              const std::string& name) -> const std::string& {
            return self->setDofName(index, name);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"),
          ::pybind11::arg("name"))
      .def(
          "setDofName",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              const std::string& name,
              bool preserveName) -> const std::string& {
            return self->setDofName(index, name, preserveName);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"),
          ::pybind11::arg("name"),
          ::pybind11::arg("preserveName"))
      .def(
          "preserveDofName",
          +[](dart::dynamics::Joint* self, std::size_t index, bool preserve)
              -> void { return self->preserveDofName(index, preserve); },
          ::pybind11::arg("index"),
          ::pybind11::arg("preserve"))
      .def(
          "isDofNamePreserved",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> bool {
            return self->isDofNamePreserved(index);
          },
          ::pybind11::arg("index"))
      .def(
          "getDofName",
          +[](const dart::dynamics::Joint* self, std::size_t index)
              -> const std::string& { return self->getDofName(index); },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"))
      .def(
          "getNumDofs",
          +[](const dart::dynamics::Joint* self)
              -> std::size_t { return self->getNumDofs(); })
      .def(
          "setCommand",
          +[](dart::dynamics::Joint* self, std::size_t index, double command)
              -> void { return self->setCommand(index, command); },
          ::pybind11::arg("index"),
          ::pybind11::arg("command"))
      .def(
          "getCommand",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getCommand(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setCommands",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& commands)
              -> void { return self->setCommands(commands); },
          ::pybind11::arg("commands"))
      .def(
          "getCommands",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getCommands();
          })
      .def(
          "resetCommands",
          +[](dart::dynamics::Joint* self)
              -> void { return self->resetCommands(); })
      .def(
          "setPosition",
          +[](dart::dynamics::Joint* self, std::size_t index, double position)
              -> void { return self->setPosition(index, position); },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPosition",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setPositions",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& positions)
              -> void { return self->setPositions(positions); },
          ::pybind11::arg("positions"))
      .def(
          "getPositions",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getPositions();
          })
      .def(
          "setPositionLowerLimit",
          +[](dart::dynamics::Joint* self, std::size_t index, double position)
              -> void { return self->setPositionLowerLimit(index, position); },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionLowerLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getPositionLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setPositionLowerLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& lowerLimits)
              -> void { return self->setPositionLowerLimits(lowerLimits); },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getPositionLowerLimits",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getPositionLowerLimits();
          })
      .def(
          "setPositionUpperLimit",
          +[](dart::dynamics::Joint* self, std::size_t index, double position)
              -> void { return self->setPositionUpperLimit(index, position); },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionUpperLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getPositionUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setPositionUpperLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& upperLimits)
              -> void { return self->setPositionUpperLimits(upperLimits); },
          ::pybind11::arg("upperLimits"))
      .def(
          "getPositionUpperLimits",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getPositionUpperLimits();
          })
      .def(
          "isCyclic",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> bool {
            return self->isCyclic(index);
          },
          ::pybind11::arg("index"))
      .def(
          "hasPositionLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> bool {
            return self->hasPositionLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetPosition",
          +[](dart::dynamics::Joint* self, std::size_t index) -> void {
            return self->resetPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetPositions",
          +[](dart::dynamics::Joint* self)
              -> void { return self->resetPositions(); })
      .def(
          "setInitialPosition",
          +[](dart::dynamics::Joint* self, std::size_t index, double initial)
              -> void { return self->setInitialPosition(index, initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialPosition",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getInitialPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setInitialPositions",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& initial)
              -> void { return self->setInitialPositions(initial); },
          ::pybind11::arg("initial"))
      .def(
          "getInitialPositions",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getInitialPositions();
          })
      .def(
          "setVelocity",
          +[](dart::dynamics::Joint* self, std::size_t index, double velocity)
              -> void { return self->setVelocity(index, velocity); },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocity",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getVelocity(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setVelocities",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& velocities)
              -> void { return self->setVelocities(velocities); },
          ::pybind11::arg("velocities"))
      .def(
          "getVelocities",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getVelocities();
          })
      .def(
          "setVelocityLowerLimit",
          +[](dart::dynamics::Joint* self, std::size_t index, double velocity)
              -> void { return self->setVelocityLowerLimit(index, velocity); },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityLowerLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getVelocityLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setVelocityLowerLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& lowerLimits)
              -> void { return self->setVelocityLowerLimits(lowerLimits); },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getVelocityLowerLimits",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getVelocityLowerLimits();
          })
      .def(
          "setVelocityUpperLimit",
          +[](dart::dynamics::Joint* self, std::size_t index, double velocity)
              -> void { return self->setVelocityUpperLimit(index, velocity); },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityUpperLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getVelocityUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setVelocityUpperLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& upperLimits)
              -> void { return self->setVelocityUpperLimits(upperLimits); },
          ::pybind11::arg("upperLimits"))
      .def(
          "getVelocityUpperLimits",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getVelocityUpperLimits();
          })
      .def(
          "resetVelocity",
          +[](dart::dynamics::Joint* self, std::size_t index) -> void {
            return self->resetVelocity(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetVelocities",
          +[](dart::dynamics::Joint* self)
              -> void { return self->resetVelocities(); })
      .def(
          "setInitialVelocity",
          +[](dart::dynamics::Joint* self, std::size_t index, double initial)
              -> void { return self->setInitialVelocity(index, initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocity",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getInitialVelocity(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setInitialVelocities",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& initial)
              -> void { return self->setInitialVelocities(initial); },
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocities",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getInitialVelocities();
          })
      .def(
          "setAcceleration",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              double acceleration) -> void {
            return self->setAcceleration(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAcceleration",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getAcceleration(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerations",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& accelerations)
              -> void { return self->setAccelerations(accelerations); },
          ::pybind11::arg("accelerations"))
      .def(
          "getAccelerations",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getAccelerations();
          })
      .def(
          "resetAccelerations",
          +[](dart::dynamics::Joint* self)
              -> void { return self->resetAccelerations(); })
      .def(
          "setAccelerationLowerLimit",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              double acceleration) -> void {
            return self->setAccelerationLowerLimit(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationLowerLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getAccelerationLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationLowerLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& lowerLimits)
              -> void { return self->setAccelerationLowerLimits(lowerLimits); },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getAccelerationLowerLimits",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getAccelerationLowerLimits();
          })
      .def(
          "setAccelerationUpperLimit",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              double acceleration) -> void {
            return self->setAccelerationUpperLimit(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationUpperLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getAccelerationUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationUpperLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& upperLimits)
              -> void { return self->setAccelerationUpperLimits(upperLimits); },
          ::pybind11::arg("upperLimits"))
      .def(
          "getAccelerationUpperLimits",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getAccelerationUpperLimits();
          })
      .def(
          "setForce",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              double force) -> void { return self->setForce(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForce",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getForce(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setForces",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& forces)
              -> void { return self->setForces(forces); },
          ::pybind11::arg("forces"))
      .def(
          "getForces",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getForces();
          })
      .def(
          "resetForces",
          +[](dart::dynamics::Joint* self)
              -> void { return self->resetForces(); })
      .def(
          "setForceLowerLimit",
          +[](dart::dynamics::Joint* self, std::size_t index, double force)
              -> void { return self->setForceLowerLimit(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceLowerLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getForceLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setForceLowerLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& lowerLimits)
              -> void { return self->setForceLowerLimits(lowerLimits); },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getForceLowerLimits",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getForceLowerLimits();
          })
      .def(
          "setForceUpperLimit",
          +[](dart::dynamics::Joint* self, std::size_t index, double force)
              -> void { return self->setForceUpperLimit(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceUpperLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getForceUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setForceUpperLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& upperLimits)
              -> void { return self->setForceUpperLimits(upperLimits); },
          ::pybind11::arg("upperLimits"))
      .def(
          "getForceUpperLimits",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getForceUpperLimits();
          })
      .def(
          "checkSanity",
          +[](const dart::dynamics::Joint* self)
              -> bool { return self->checkSanity(); })
      .def(
          "checkSanity",
          +[](const dart::dynamics::Joint* self, bool printWarnings) -> bool {
            return self->checkSanity(printWarnings);
          },
          ::pybind11::arg("printWarnings"))
      .def(
          "setVelocityChange",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              double velocityChange) -> void {
            return self->setVelocityChange(index, velocityChange);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocityChange"))
      .def(
          "getVelocityChange",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getVelocityChange(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetVelocityChanges",
          +[](dart::dynamics::Joint* self)
              -> void { return self->resetVelocityChanges(); })
      .def(
          "setConstraintImpulse",
          +[](dart::dynamics::Joint* self, std::size_t index, double impulse)
              -> void { return self->setConstraintImpulse(index, impulse); },
          ::pybind11::arg("index"),
          ::pybind11::arg("impulse"))
      .def(
          "getConstraintImpulse",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getConstraintImpulse(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetConstraintImpulses",
          +[](dart::dynamics::Joint* self)
              -> void { return self->resetConstraintImpulses(); })
      .def(
          "integratePositions",
          +[](dart::dynamics::Joint* self, double dt) -> void {
            return self->integratePositions(dt);
          },
          ::pybind11::arg("dt"))
      .def(
          "integrateVelocities",
          +[](dart::dynamics::Joint* self, double dt) -> void {
            return self->integrateVelocities(dt);
          },
          ::pybind11::arg("dt"))
      .def(
          "getPositionDifferences",
          +[](const dart::dynamics::Joint* self,
              const Eigen::VectorXd& q2,
              const Eigen::VectorXd& q1) -> Eigen::VectorXd {
            return self->getPositionDifferences(q2, q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def(
          "setSpringStiffness",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              double k) -> void { return self->setSpringStiffness(index, k); },
          ::pybind11::arg("index"),
          ::pybind11::arg("k"))
      .def(
          "getSpringStiffness",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getSpringStiffness(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setRestPosition",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              double q0) -> void { return self->setRestPosition(index, q0); },
          ::pybind11::arg("index"),
          ::pybind11::arg("q0"))
      .def(
          "getRestPosition",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getRestPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setDampingCoefficient",
          +[](dart::dynamics::Joint* self, std::size_t index, double coeff)
              -> void { return self->setDampingCoefficient(index, coeff); },
          ::pybind11::arg("index"),
          ::pybind11::arg("coeff"))
      .def(
          "getDampingCoefficient",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getDampingCoefficient(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setCoulombFriction",
          +[](dart::dynamics::Joint* self, std::size_t index, double friction)
              -> void { return self->setCoulombFriction(index, friction); },
          ::pybind11::arg("index"),
          ::pybind11::arg("friction"))
      .def(
          "getCoulombFriction",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getCoulombFriction(index);
          },
          ::pybind11::arg("index"))
      .def(
          "computePotentialEnergy",
          +[](const dart::dynamics::Joint* self) -> double {
            return self->computePotentialEnergy();
          })
      .def(
          "getRelativeTransform",
          +[](const dart::dynamics::Joint* self) -> const Eigen::Isometry3d& {
            return self->getRelativeTransform();
          })
      .def(
          "getRelativeSpatialVelocity",
          +[](const dart::dynamics::Joint* self) -> const Eigen::Vector6d& {
            return self->getRelativeSpatialVelocity();
          })
      .def(
          "getRelativeSpatialAcceleration",
          +[](const dart::dynamics::Joint* self) -> const Eigen::Vector6d& {
            return self->getRelativeSpatialAcceleration();
          })
      .def(
          "getRelativePrimaryAcceleration",
          +[](const dart::dynamics::Joint* self) -> const Eigen::Vector6d& {
            return self->getRelativePrimaryAcceleration();
          })
      .def(
          "getRelativeJacobian",
          +[](const dart::dynamics::Joint* self) -> const dart::math::Jacobian {
            return self->getRelativeJacobian();
          })
      .def(
          "getRelativeJacobian",
          +[](const dart::dynamics::Joint* self,
              const Eigen::VectorXd& positions) -> dart::math::Jacobian {
            return self->getRelativeJacobian(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getRelativeJacobianTimeDeriv",
          +[](const dart::dynamics::Joint* self) -> const dart::math::Jacobian {
            return self->getRelativeJacobianTimeDeriv();
          })
      .def(
          "getBodyConstraintWrench",
          +[](const dart::dynamics::Joint* self) -> Eigen::Vector6d {
            return self->getBodyConstraintWrench();
          })
      .def(
          "notifyPositionUpdated",
          +[](dart::dynamics::Joint* self)
              -> void { return self->notifyPositionUpdated(); })
      .def(
          "notifyVelocityUpdated",
          +[](dart::dynamics::Joint* self)
              -> void { return self->notifyVelocityUpdated(); })
      .def(
          "notifyAccelerationUpdated",
          +[](dart::dynamics::Joint* self) -> void {
            return self->notifyAccelerationUpdated();
          })
      .def_readonly_static("FORCE", &dart::dynamics::Joint::FORCE)
      .def_readonly_static("PASSIVE", &dart::dynamics::Joint::PASSIVE)
      .def_readonly_static("SERVO", &dart::dynamics::Joint::SERVO)
      .def_readonly_static("ACCELERATION", &dart::dynamics::Joint::ACCELERATION)
      .def_readonly_static("VELOCITY", &dart::dynamics::Joint::VELOCITY)
      .def_readonly_static("LOCKED", &dart::dynamics::Joint::LOCKED)
      .def_readonly_static(
          "DefaultActuatorType", &dart::dynamics::Joint::DefaultActuatorType);

  auto attr = m.attr("Joint");

  ::pybind11::enum_<dart::dynamics::detail::ActuatorType>(attr, "ActuatorType")
      .value("FORCE", dart::dynamics::detail::ActuatorType::FORCE)
      .value("PASSIVE", dart::dynamics::detail::ActuatorType::PASSIVE)
      .value("SERVO", dart::dynamics::detail::ActuatorType::SERVO)
      .value("MIMIC", dart::dynamics::detail::ActuatorType::MIMIC)
      .value("ACCELERATION", dart::dynamics::detail::ActuatorType::ACCELERATION)
      .value("VELOCITY", dart::dynamics::detail::ActuatorType::VELOCITY)
      .value("LOCKED", dart::dynamics::detail::ActuatorType::LOCKED);
}

} // namespace python
} // namespace dart
