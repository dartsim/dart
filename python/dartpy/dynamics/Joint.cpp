// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/MimicDofProperties.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/math/MathTypes.hpp>

#include <dart/common/Aspect.hpp>
#include <dart/common/Composite.hpp>
#include <dart/common/EmbeddedAspect.hpp>
#include <dart/common/RequiresAspect.hpp>
#include <dart/common/SpecializedForAspect.hpp>
#include <dart/common/Subject.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_geometry_pybind.h>

#include <memory>
#include <string>
#include <vector>

#include <cstddef>

namespace dart {
namespace python {

void Joint(nb::module_& m)
{
  nb::enum_<dart::dynamics::detail::ActuatorType>(
      m, "ActuatorType", nb::is_arithmetic())
      .value("FORCE", dart::dynamics::detail::ActuatorType::FORCE)
      .value("PASSIVE", dart::dynamics::detail::ActuatorType::PASSIVE)
      .value("SERVO", dart::dynamics::detail::ActuatorType::SERVO)
      .value("MIMIC", dart::dynamics::detail::ActuatorType::MIMIC)
      .value("ACCELERATION", dart::dynamics::detail::ActuatorType::ACCELERATION)
      .value("VELOCITY", dart::dynamics::detail::ActuatorType::VELOCITY)
      .value("LOCKED", dart::dynamics::detail::ActuatorType::LOCKED)
      .export_values();

  m.attr("DefaultActuatorType") = dart::dynamics::detail::DefaultActuatorType;

  nb::enum_<dart::dynamics::MimicConstraintType>(
      m, "MimicConstraintType", nb::is_arithmetic())
      .value("Motor", dart::dynamics::MimicConstraintType::Motor)
      .value("Coupler", dart::dynamics::MimicConstraintType::Coupler)
      .export_values();

  dartnb::dart_class<dart::dynamics::MimicDofProperties>(
      m, "MimicDofProperties")
      .def(dartnb::init<>())
      .def_rw(
          "mReferenceJoint",
          &dart::dynamics::MimicDofProperties::mReferenceJoint,
          dartnb::setterArgument(
              &dart::dynamics::MimicDofProperties::mReferenceJoint))
      .def_rw(
          "mReferenceDofIndex",
          &dart::dynamics::MimicDofProperties::mReferenceDofIndex,
          dartnb::setterArgument(
              &dart::dynamics::MimicDofProperties::mReferenceDofIndex))
      .def_rw(
          "mMultiplier",
          &dart::dynamics::MimicDofProperties::mMultiplier,
          dartnb::setterArgument(
              &dart::dynamics::MimicDofProperties::mMultiplier))
      .def_rw(
          "mOffset",
          &dart::dynamics::MimicDofProperties::mOffset,
          dartnb::setterArgument(&dart::dynamics::MimicDofProperties::mOffset))
      .def_rw(
          "mConstraintType",
          &dart::dynamics::MimicDofProperties::mConstraintType,
          dartnb::setterArgument(
              &dart::dynamics::MimicDofProperties::mConstraintType));

  dartnb::dart_class<dart::dynamics::detail::JointProperties>(
      m, "JointProperties")
      .def(
          dartnb::init<
              const std::string&,
              const Eigen::Isometry3d&,
              const Eigen::Isometry3d&,
              bool,
              dart::dynamics::detail::ActuatorType,
              const dart::dynamics::Joint*,
              double,
              double>(),
          nb::arg("name") = "Joint",
          nb::arg("T_ParentBodyToJoint") = Eigen::Isometry3d::Identity(),
          nb::arg("T_ChildBodyToJoint") = Eigen::Isometry3d::Identity(),
          nb::arg("isPositionLimitEnforced") = false,
          nb::arg("actuatorType") = dart::dynamics::detail::DefaultActuatorType,
          nb::arg("mimicJoint").none() = nullptr,
          nb::arg("mimicMultiplier") = 1.0,
          nb::arg("mimicOffset") = 0.0)
      .def_rw(
          "mName",
          &dart::dynamics::detail::JointProperties::mName,
          dartnb::setterArgument(
              &dart::dynamics::detail::JointProperties::mName))
      .def_rw(
          "mT_ParentBodyToJoint",
          &dart::dynamics::detail::JointProperties::mT_ParentBodyToJoint,
          dartnb::setterArgument(
              &dart::dynamics::detail::JointProperties::mT_ParentBodyToJoint))
      .def_rw(
          "mT_ChildBodyToJoint",
          &dart::dynamics::detail::JointProperties::mT_ChildBodyToJoint,
          dartnb::setterArgument(
              &dart::dynamics::detail::JointProperties::mT_ChildBodyToJoint))
      .def_rw(
          "mIsPositionLimitEnforced",
          &dart::dynamics::detail::JointProperties::mIsPositionLimitEnforced,
          dartnb::setterArgument(&dart::dynamics::detail::JointProperties::
                                     mIsPositionLimitEnforced))
      .def_rw(
          "mActuatorType",
          &dart::dynamics::detail::JointProperties::mActuatorType,
          dartnb::setterArgument(
              &dart::dynamics::detail::JointProperties::mActuatorType))
      .def_rw(
          "mMimicDofProps",
          &dart::dynamics::detail::JointProperties::mMimicDofProps,
          dartnb::setterArgument(
              &dart::dynamics::detail::JointProperties::mMimicDofProps));

  dartnb::dart_class<
      dart::common::SpecializedForAspect<dart::common::EmbeddedPropertiesAspect<
          dart::dynamics::Joint,
          dart::dynamics::detail::JointProperties>>,
      dart::common::Composite>(
      m, "SpecializedForAspect_EmbeddedPropertiesAspect_Joint_JointProperties")
      .def(dartnb::init<>());

  dartnb::dart_class<
      dart::common::RequiresAspect<dart::common::EmbeddedPropertiesAspect<
          dart::dynamics::Joint,
          dart::dynamics::detail::JointProperties>>,
      dart::common::SpecializedForAspect<dart::common::EmbeddedPropertiesAspect<
          dart::dynamics::Joint,
          dart::dynamics::detail::JointProperties>>>(
      m, "RequiresAspect_EmbeddedPropertiesAspect_Joint_JointProperties")
      .def(dartnb::init<>());

  dartnb::dart_class<
      dart::common::EmbedProperties<
          dart::dynamics::Joint,
          dart::dynamics::detail::JointProperties>,
      dart::common::RequiresAspect<dart::common::EmbeddedPropertiesAspect<
          dart::dynamics::Joint,
          dart::dynamics::detail::JointProperties>>>(
      m, "EmbedProperties_Joint_JointProperties");

  dartnb::dart_class<
      dart::dynamics::Joint,
      dart::common::Subject,
      dart::common::EmbedProperties<
          dart::dynamics::Joint,
          dart::dynamics::detail::JointProperties>>(m, "Joint")
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
          nb::arg("aspect").none())
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
          nb::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::Joint* self,
              const dart::common::EmbedProperties<
                  dart::dynamics::Joint,
                  dart::dynamics::detail::JointProperties>::AspectProperties&
                  properties) -> void {
            return self->setAspectProperties(properties);
          },
          nb::arg("properties"))
      .def(
          "copy",
          +[](dart::dynamics::Joint* self,
              const dart::dynamics::Joint& otherJoint) -> void {
            return self->copy(otherJoint);
          },
          nb::arg("otherJoint"))
      .def(
          "copy",
          +[](dart::dynamics::Joint* self,
              const dart::dynamics::Joint* otherJoint) -> void {
            return self->copy(otherJoint);
          },
          nb::arg("otherJoint").none())
      .def(
          "setName",
          +[](dart::dynamics::Joint* self, const std::string& name)
              -> const std::string& { return self->setName(name); },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def(
          "setName",
          +[](dart::dynamics::Joint* self,
              const std::string& name,
              bool renameDofs) -> const std::string& {
            return self->setName(name, renameDofs);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"),
          nb::arg("renameDofs"))
      .def(
          "getName",
          +[](const dart::dynamics::Joint* self) -> const std::string& {
            return self->getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getType",
          +[](const dart::dynamics::Joint* self) -> const std::string& {
            return self->getType();
          },
          nb::rv_policy::reference_internal)
      .def(
          "setActuatorType",
          +[](dart::dynamics::Joint* self,
              dart::dynamics::Joint::ActuatorType actuatorType) -> void {
            return self->setActuatorType(actuatorType);
          },
          nb::arg("actuatorType"))
      .def(
          "getActuatorType",
          +[](const dart::dynamics::Joint* self)
              -> dart::dynamics::Joint::ActuatorType {
            return self->getActuatorType();
          })
      .def(
          "setUseCouplerConstraint",
          +[](dart::dynamics::Joint* self, bool enable) -> void {
            self->setUseCouplerConstraint(enable);
          },
          nb::arg("enable"))
      .def(
          "isUsingCouplerConstraint",
          +[](const dart::dynamics::Joint* self) -> bool {
            return self->isUsingCouplerConstraint();
          })
      .def(
          "setActuatorTypeForDof",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              dart::dynamics::Joint::ActuatorType actuatorType) -> void {
            self->setActuatorType(index, actuatorType);
          },
          nb::arg("index"),
          nb::arg("actuatorType"))
      .def(
          "setActuatorTypes",
          +[](dart::dynamics::Joint* self,
              const std::vector<dart::dynamics::Joint::ActuatorType>&
                  actuatorTypes) -> void {
            self->setActuatorTypes(actuatorTypes);
          },
          nb::arg("actuatorTypes"))
      .def(
          "getActuatorTypeForDof",
          +[](const dart::dynamics::Joint* self,
              std::size_t index) -> dart::dynamics::Joint::ActuatorType {
            return self->getActuatorType(index);
          },
          nb::arg("index"))
      .def(
          "getActuatorTypes",
          +[](const dart::dynamics::Joint* self)
              -> std::vector<dart::dynamics::Joint::ActuatorType> {
            return self->getActuatorTypes();
          })
      .def(
          "hasActuatorType",
          +[](const dart::dynamics::Joint* self,
              dart::dynamics::Joint::ActuatorType actuatorType) -> bool {
            return self->hasActuatorType(actuatorType);
          },
          nb::arg("actuatorType"))
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
          nb::rv_policy::reference_internal)
      .def(
          "getParentBodyNode",
          +[](dart::dynamics::Joint* self) -> dart::dynamics::BodyNode* {
            return self->getParentBodyNode();
          },
          nb::rv_policy::reference_internal)
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
          nb::arg("T"))
      .def(
          "setTransformFromChildBodyNode",
          +[](dart::dynamics::Joint* self, const Eigen::Isometry3d& T) -> void {
            return self->setTransformFromChildBodyNode(T);
          },
          nb::arg("T"))
      .def(
          "getTransformFromParentBodyNode",
          +[](const dart::dynamics::Joint* self) -> const Eigen::Isometry3d& {
            return self->getTransformFromParentBodyNode();
          })
      .def(
          "getTransformFromChildBodyNode",
          +[](const dart::dynamics::Joint* self) -> const Eigen::Isometry3d& {
            return self->getTransformFromChildBodyNode();
          })
      .def(
          "setLimitEnforcement",
          +[](dart::dynamics::Joint* self, bool enforce) -> void {
            return self->setLimitEnforcement(enforce);
          },
          nb::arg("enforced"))
      .def(
          "areLimitsEnforced",
          +[](const dart::dynamics::Joint* self) -> bool {
            return self->areLimitsEnforced();
          })
      .def(
          "getIndexInSkeleton",
          +[](const dart::dynamics::Joint* self, std::size_t index)
              -> std::size_t { return self->getIndexInSkeleton(index); },
          nb::arg("index"))
      .def(
          "getIndexInTree",
          +[](const dart::dynamics::Joint* self, std::size_t index)
              -> std::size_t { return self->getIndexInTree(index); },
          nb::arg("index"))
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
          nb::rv_policy::reference_internal,
          nb::arg("index"),
          nb::arg("name"))
      .def(
          "setDofName",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              const std::string& name,
              bool preserveName) -> const std::string& {
            return self->setDofName(index, name, preserveName);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"),
          nb::arg("name"),
          nb::arg("preserveName"))
      .def(
          "preserveDofName",
          +[](dart::dynamics::Joint* self, std::size_t index, bool preserve)
              -> void { return self->preserveDofName(index, preserve); },
          nb::arg("index"),
          nb::arg("preserve"))
      .def(
          "isDofNamePreserved",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> bool {
            return self->isDofNamePreserved(index);
          },
          nb::arg("index"))
      .def(
          "getDofName",
          +[](const dart::dynamics::Joint* self, std::size_t index)
              -> const std::string& { return self->getDofName(index); },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "getNumDofs",
          +[](const dart::dynamics::Joint* self) -> std::size_t {
            return self->getNumDofs();
          })
      .def(
          "setCommand",
          +[](dart::dynamics::Joint* self, std::size_t index, double command)
              -> void { return self->setCommand(index, command); },
          nb::arg("index"),
          nb::arg("command"))
      .def(
          "getCommand",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getCommand(index);
          },
          nb::arg("index"))
      .def(
          "setCommands",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& commands)
              -> void { return self->setCommands(commands); },
          nb::arg("commands"))
      .def(
          "getCommands",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getCommands();
          })
      .def(
          "resetCommands",
          +[](dart::dynamics::Joint* self) -> void {
            return self->resetCommands();
          })
      .def(
          "setPosition",
          +[](dart::dynamics::Joint* self, std::size_t index, double position)
              -> void { return self->setPosition(index, position); },
          nb::arg("index"),
          nb::arg("position"))
      .def(
          "getPosition",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getPosition(index);
          },
          nb::arg("index"))
      .def(
          "setPositions",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& positions)
              -> void { return self->setPositions(positions); },
          nb::arg("positions"))
      .def(
          "getPositions",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getPositions();
          })
      .def(
          "setPositionLowerLimit",
          +[](dart::dynamics::Joint* self, std::size_t index, double position)
              -> void { return self->setPositionLowerLimit(index, position); },
          nb::arg("index"),
          nb::arg("position"))
      .def(
          "getPositionLowerLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getPositionLowerLimit(index);
          },
          nb::arg("index"))
      .def(
          "setPositionLowerLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& lowerLimits)
              -> void { return self->setPositionLowerLimits(lowerLimits); },
          nb::arg("lowerLimits"))
      .def(
          "getPositionLowerLimits",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getPositionLowerLimits();
          })
      .def(
          "setPositionUpperLimit",
          +[](dart::dynamics::Joint* self, std::size_t index, double position)
              -> void { return self->setPositionUpperLimit(index, position); },
          nb::arg("index"),
          nb::arg("position"))
      .def(
          "getPositionUpperLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getPositionUpperLimit(index);
          },
          nb::arg("index"))
      .def(
          "setPositionUpperLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& upperLimits)
              -> void { return self->setPositionUpperLimits(upperLimits); },
          nb::arg("upperLimits"))
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
          nb::arg("index"))
      .def(
          "hasPositionLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> bool {
            return self->hasPositionLimit(index);
          },
          nb::arg("index"))
      .def(
          "resetPosition",
          +[](dart::dynamics::Joint* self, std::size_t index) -> void {
            return self->resetPosition(index);
          },
          nb::arg("index"))
      .def(
          "resetPositions",
          +[](dart::dynamics::Joint* self)
              -> void { return self->resetPositions(); })
      .def(
          "setInitialPosition",
          +[](dart::dynamics::Joint* self, std::size_t index, double initial)
              -> void { return self->setInitialPosition(index, initial); },
          nb::arg("index"),
          nb::arg("initial"))
      .def(
          "getInitialPosition",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getInitialPosition(index);
          },
          nb::arg("index"))
      .def(
          "setInitialPositions",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& initial)
              -> void { return self->setInitialPositions(initial); },
          nb::arg("initial"))
      .def(
          "getInitialPositions",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getInitialPositions();
          })
      .def(
          "setVelocity",
          +[](dart::dynamics::Joint* self, std::size_t index, double velocity)
              -> void { return self->setVelocity(index, velocity); },
          nb::arg("index"),
          nb::arg("velocity"))
      .def(
          "getVelocity",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getVelocity(index);
          },
          nb::arg("index"))
      .def(
          "setVelocities",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& velocities)
              -> void { return self->setVelocities(velocities); },
          nb::arg("velocities"))
      .def(
          "getVelocities",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getVelocities();
          })
      .def(
          "setVelocityLowerLimit",
          +[](dart::dynamics::Joint* self, std::size_t index, double velocity)
              -> void { return self->setVelocityLowerLimit(index, velocity); },
          nb::arg("index"),
          nb::arg("velocity"))
      .def(
          "getVelocityLowerLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getVelocityLowerLimit(index);
          },
          nb::arg("index"))
      .def(
          "setVelocityLowerLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& lowerLimits)
              -> void { return self->setVelocityLowerLimits(lowerLimits); },
          nb::arg("lowerLimits"))
      .def(
          "getVelocityLowerLimits",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getVelocityLowerLimits();
          })
      .def(
          "setVelocityUpperLimit",
          +[](dart::dynamics::Joint* self, std::size_t index, double velocity)
              -> void { return self->setVelocityUpperLimit(index, velocity); },
          nb::arg("index"),
          nb::arg("velocity"))
      .def(
          "getVelocityUpperLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getVelocityUpperLimit(index);
          },
          nb::arg("index"))
      .def(
          "setVelocityUpperLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& upperLimits)
              -> void { return self->setVelocityUpperLimits(upperLimits); },
          nb::arg("upperLimits"))
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
          nb::arg("index"))
      .def(
          "resetVelocities",
          +[](dart::dynamics::Joint* self)
              -> void { return self->resetVelocities(); })
      .def(
          "setInitialVelocity",
          +[](dart::dynamics::Joint* self, std::size_t index, double initial)
              -> void { return self->setInitialVelocity(index, initial); },
          nb::arg("index"),
          nb::arg("initial"))
      .def(
          "getInitialVelocity",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getInitialVelocity(index);
          },
          nb::arg("index"))
      .def(
          "setInitialVelocities",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& initial)
              -> void { return self->setInitialVelocities(initial); },
          nb::arg("initial"))
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
          nb::arg("index"),
          nb::arg("acceleration"))
      .def(
          "getAcceleration",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getAcceleration(index);
          },
          nb::arg("index"))
      .def(
          "setAccelerations",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& accelerations)
              -> void { return self->setAccelerations(accelerations); },
          nb::arg("accelerations"))
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
          nb::arg("index"),
          nb::arg("acceleration"))
      .def(
          "getAccelerationLowerLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getAccelerationLowerLimit(index);
          },
          nb::arg("index"))
      .def(
          "setAccelerationLowerLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& lowerLimits)
              -> void { return self->setAccelerationLowerLimits(lowerLimits); },
          nb::arg("lowerLimits"))
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
          nb::arg("index"),
          nb::arg("acceleration"))
      .def(
          "getAccelerationUpperLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getAccelerationUpperLimit(index);
          },
          nb::arg("index"))
      .def(
          "setAccelerationUpperLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& upperLimits)
              -> void { return self->setAccelerationUpperLimits(upperLimits); },
          nb::arg("upperLimits"))
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
          nb::arg("index"),
          nb::arg("force"))
      .def(
          "getForce",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getForce(index);
          },
          nb::arg("index"))
      .def(
          "setForces",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& forces)
              -> void { return self->setForces(forces); },
          nb::arg("forces"))
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
          nb::arg("index"),
          nb::arg("force"))
      .def(
          "getForceLowerLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getForceLowerLimit(index);
          },
          nb::arg("index"))
      .def(
          "setForceLowerLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& lowerLimits)
              -> void { return self->setForceLowerLimits(lowerLimits); },
          nb::arg("lowerLimits"))
      .def(
          "getForceLowerLimits",
          +[](const dart::dynamics::Joint* self) -> Eigen::VectorXd {
            return self->getForceLowerLimits();
          })
      .def(
          "setForceUpperLimit",
          +[](dart::dynamics::Joint* self, std::size_t index, double force)
              -> void { return self->setForceUpperLimit(index, force); },
          nb::arg("index"),
          nb::arg("force"))
      .def(
          "getForceUpperLimit",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getForceUpperLimit(index);
          },
          nb::arg("index"))
      .def(
          "setForceUpperLimits",
          +[](dart::dynamics::Joint* self, const Eigen::VectorXd& upperLimits)
              -> void { return self->setForceUpperLimits(upperLimits); },
          nb::arg("upperLimits"))
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
          nb::arg("printWarnings"))
      .def(
          "setVelocityChange",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              double velocityChange) -> void {
            return self->setVelocityChange(index, velocityChange);
          },
          nb::arg("index"),
          nb::arg("velocityChange"))
      .def(
          "getVelocityChange",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getVelocityChange(index);
          },
          nb::arg("index"))
      .def(
          "resetVelocityChanges",
          +[](dart::dynamics::Joint* self)
              -> void { return self->resetVelocityChanges(); })
      .def(
          "setConstraintImpulse",
          +[](dart::dynamics::Joint* self, std::size_t index, double impulse)
              -> void { return self->setConstraintImpulse(index, impulse); },
          nb::arg("index"),
          nb::arg("impulse"))
      .def(
          "getConstraintImpulse",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getConstraintImpulse(index);
          },
          nb::arg("index"))
      .def(
          "resetConstraintImpulses",
          +[](dart::dynamics::Joint* self)
              -> void { return self->resetConstraintImpulses(); })
      .def(
          "integratePositions",
          +[](dart::dynamics::Joint* self, double dt) -> void {
            return self->integratePositions(dt);
          },
          nb::arg("dt"))
      .def(
          "integrateVelocities",
          +[](dart::dynamics::Joint* self, double dt) -> void {
            return self->integrateVelocities(dt);
          },
          nb::arg("dt"))
      .def(
          "getPositionDifferences",
          +[](const dart::dynamics::Joint* self,
              const Eigen::VectorXd& q2,
              const Eigen::VectorXd& q1) -> Eigen::VectorXd {
            return self->getPositionDifferences(q2, q1);
          },
          nb::arg("q2"),
          nb::arg("q1"))
      .def(
          "setSpringStiffness",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              double k) -> void { return self->setSpringStiffness(index, k); },
          nb::arg("index"),
          nb::arg("k"))
      .def(
          "getSpringStiffness",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getSpringStiffness(index);
          },
          nb::arg("index"))
      .def(
          "setRestPosition",
          +[](dart::dynamics::Joint* self,
              std::size_t index,
              double q0) -> void { return self->setRestPosition(index, q0); },
          nb::arg("index"),
          nb::arg("q0"))
      .def(
          "getRestPosition",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getRestPosition(index);
          },
          nb::arg("index"))
      .def(
          "setDampingCoefficient",
          +[](dart::dynamics::Joint* self, std::size_t index, double coeff)
              -> void { return self->setDampingCoefficient(index, coeff); },
          nb::arg("index"),
          nb::arg("coeff"))
      .def(
          "getDampingCoefficient",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getDampingCoefficient(index);
          },
          nb::arg("index"))
      .def(
          "setCoulombFriction",
          +[](dart::dynamics::Joint* self, std::size_t index, double friction)
              -> void { return self->setCoulombFriction(index, friction); },
          nb::arg("index"),
          nb::arg("friction"))
      .def(
          "getCoulombFriction",
          +[](const dart::dynamics::Joint* self, std::size_t index) -> double {
            return self->getCoulombFriction(index);
          },
          nb::arg("index"))
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
          nb::arg("positions"))
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
          "getWrenchToChildBodyNode",
          &dart::dynamics::Joint::getWrenchToChildBodyNode,
          nb::arg("withRespectTo").none() = nullptr)
      .def(
          "getWrenchToParentBodyNode",
          +[](const dart::dynamics::Joint* self,
              const dart::dynamics::Frame* withRespectTo) -> Eigen::Vector6d {
            return -self->getWrenchToChildBodyNode(withRespectTo);
          },
          nb::arg("withRespectTo").none() = nullptr)
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
          });
}

} // namespace python
} // namespace dart
