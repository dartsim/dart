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
#include "Joint.hpp"

namespace dart {
namespace python {

void ZeroDofJoint(pybind11::module& m)
{
  ::pybind11::class_<dart::dynamics::ZeroDofJoint::Properties>(
      m, "ZeroDofJointProperties")
      .def(::pybind11::init<>())
      .def(
          ::pybind11::init<const dart::dynamics::Joint::Properties&>(),
          ::pybind11::arg("properties"));

  ::pybind11::class_<
      dart::dynamics::ZeroDofJoint,
      dart::dynamics::Joint,
      std::shared_ptr<dart::dynamics::ZeroDofJoint> >(m, "ZeroDofJoint")
      .def(
          "getZeroDofJointProperties",
          +[](const dart::dynamics::ZeroDofJoint* self)
              -> dart::dynamics::ZeroDofJoint::Properties {
            return self->getZeroDofJointProperties();
          })
      .def(
          "setDofName",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _arg0_,
              const std::string& _arg1_,
              bool _arg2_) -> const std::string& {
            return self->setDofName(_arg0_, _arg1_, _arg2_);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("arg0_"),
          ::pybind11::arg("arg1_"),
          ::pybind11::arg("arg2_"))
      .def(
          "preserveDofName",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _arg0_,
              bool _arg1_) { self->preserveDofName(_arg0_, _arg1_); },
          ::pybind11::arg("arg0_"),
          ::pybind11::arg("arg1_"))
      .def(
          "isDofNamePreserved",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _arg0_)
              -> bool { return self->isDofNamePreserved(_arg0_); },
          ::pybind11::arg("arg0_"))
      .def(
          "getDofName",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _arg0_)
              -> const std::string& { return self->getDofName(_arg0_); },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("arg0_"))
      .def(
          "getNumDofs",
          +[](const dart::dynamics::ZeroDofJoint* self) -> std::size_t {
            return self->getNumDofs();
          })
      .def(
          "getIndexInSkeleton",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> std::size_t { return self->getIndexInSkeleton(_index); },
          ::pybind11::arg("index"))
      .def(
          "getIndexInTree",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> std::size_t { return self->getIndexInTree(_index); },
          ::pybind11::arg("index"))
      .def(
          "setCommand",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _command) { self->setCommand(_index, _command); },
          ::pybind11::arg("index"),
          ::pybind11::arg("command"))
      .def(
          "getCommand",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getCommand(_index); },
          ::pybind11::arg("index"))
      .def(
          "setCommands",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& _commands) {
            self->setCommands(_commands);
          },
          ::pybind11::arg("commands"))
      .def(
          "getCommands",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getCommands();
          })
      .def(
          "resetCommands",
          +[](dart::dynamics::ZeroDofJoint* self) { self->resetCommands(); })
      .def(
          "setPosition",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _arg0_,
              double _arg1_) { self->setPosition(_arg0_, _arg1_); },
          ::pybind11::arg("arg0_"),
          ::pybind11::arg("arg1_"))
      .def(
          "getPosition",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getPosition(_index); },
          ::pybind11::arg("index"))
      .def(
          "setPositions",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& _positions) {
            self->setPositions(_positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getPositions",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getPositions();
          })
      .def(
          "setPositionLowerLimit",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _position) {
            self->setPositionLowerLimit(_index, _position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionLowerLimit",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getPositionLowerLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "setPositionLowerLimits",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& lowerLimits) {
            self->setPositionLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getPositionLowerLimits",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getPositionLowerLimits();
          })
      .def(
          "setPositionUpperLimit",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t index,
              double position) {
            self->setPositionUpperLimit(index, position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionUpperLimit",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t index)
              -> double { return self->getPositionUpperLimit(index); },
          ::pybind11::arg("index"))
      .def(
          "setPositionUpperLimits",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& upperLimits) {
            self->setPositionUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getPositionUpperLimits",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getPositionUpperLimits();
          })
      .def(
          "hasPositionLimit",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> bool { return self->hasPositionLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "resetPosition",
          +[](dart::dynamics::ZeroDofJoint* self, std::size_t _index) {
            self->resetPosition(_index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetPositions",
          +[](dart::dynamics::ZeroDofJoint* self) { self->resetPositions(); })
      .def(
          "setInitialPosition",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _initial) { self->setInitialPosition(_index, _initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialPosition",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getInitialPosition(_index); },
          ::pybind11::arg("index"))
      .def(
          "setInitialPositions",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& _initial) {
            self->setInitialPositions(_initial);
          },
          ::pybind11::arg("initial"))
      .def(
          "getInitialPositions",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getInitialPositions();
          })
      .def(
          "setVelocity",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _velocity) { self->setVelocity(_index, _velocity); },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocity",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getVelocity(_index); },
          ::pybind11::arg("index"))
      .def(
          "setVelocities",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& _velocities) {
            self->setVelocities(_velocities);
          },
          ::pybind11::arg("velocities"))
      .def(
          "getVelocities",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getVelocities();
          })
      .def(
          "setVelocityLowerLimit",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _velocity) {
            self->setVelocityLowerLimit(_index, _velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityLowerLimit",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getVelocityLowerLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "setVelocityLowerLimits",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& lowerLimits) {
            self->setVelocityLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getVelocityLowerLimits",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getVelocityLowerLimits();
          })
      .def(
          "setVelocityUpperLimit",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _velocity) {
            self->setVelocityUpperLimit(_index, _velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityUpperLimit",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getVelocityUpperLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "setVelocityUpperLimits",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& upperLimits) {
            self->setVelocityUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getVelocityUpperLimits",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getVelocityUpperLimits();
          })
      .def(
          "resetVelocity",
          +[](dart::dynamics::ZeroDofJoint* self, std::size_t _index) {
            self->resetVelocity(_index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetVelocities",
          +[](dart::dynamics::ZeroDofJoint* self) { self->resetVelocities(); })
      .def(
          "setInitialVelocity",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _initial) { self->setInitialVelocity(_index, _initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocity",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getInitialVelocity(_index); },
          ::pybind11::arg("index"))
      .def(
          "setInitialVelocities",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& _initial) {
            self->setInitialVelocities(_initial);
          },
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocities",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getInitialVelocities();
          })
      .def(
          "setAcceleration",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _acceleration) {
            self->setAcceleration(_index, _acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAcceleration",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getAcceleration(_index); },
          ::pybind11::arg("index"))
      .def(
          "setAccelerations",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& _accelerations) {
            self->setAccelerations(_accelerations);
          },
          ::pybind11::arg("accelerations"))
      .def(
          "getAccelerations",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getAccelerations();
          })
      .def(
          "resetAccelerations",
          +[](dart::dynamics::ZeroDofJoint* self) {
            self->resetAccelerations();
          })
      .def(
          "setAccelerationLowerLimit",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _acceleration) {
            self->setAccelerationLowerLimit(_index, _acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationLowerLimit",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getAccelerationLowerLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationLowerLimits",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& lowerLimits) {
            self->setAccelerationLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getAccelerationLowerLimits",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getAccelerationLowerLimits();
          })
      .def(
          "setAccelerationUpperLimit",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _acceleration) {
            self->setAccelerationUpperLimit(_index, _acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationUpperLimit",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getAccelerationUpperLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationUpperLimits",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& upperLimits) {
            self->setAccelerationUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getAccelerationUpperLimits",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getAccelerationUpperLimits();
          })
      .def(
          "setForce",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _force) { self->setForce(_index, _force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForce",
          +[](const dart::dynamics::ZeroDofJoint* self,
              std::size_t _index) -> double { return self->getForce(_index); },
          ::pybind11::arg("index"))
      .def(
          "setForces",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& _forces) { self->setForces(_forces); },
          ::pybind11::arg("forces"))
      .def(
          "getForces",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getForces();
          })
      .def(
          "resetForces",
          +[](dart::dynamics::ZeroDofJoint* self) { self->resetForces(); })
      .def(
          "setForceLowerLimit",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _force) { self->setForceLowerLimit(_index, _force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceLowerLimit",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getForceLowerLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "setForceLowerLimits",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& lowerLimits) {
            self->setForceLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getForceLowerLimits",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getForceLowerLimits();
          })
      .def(
          "setForceUpperLimit",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _force) { self->setForceUpperLimit(_index, _force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceUpperLimit",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getForceUpperLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "setForceUpperLimits",
          +[](dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& upperLimits) {
            self->setForceUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getForceUpperLimits",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::VectorXd {
            return self->getForceUpperLimits();
          })
      .def(
          "setVelocityChange",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _velocityChange) {
            self->setVelocityChange(_index, _velocityChange);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocityChange"))
      .def(
          "getVelocityChange",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getVelocityChange(_index); },
          ::pybind11::arg("index"))
      .def(
          "resetVelocityChanges",
          +[](dart::dynamics::ZeroDofJoint*
                  self) { self->resetVelocityChanges(); })
      .def(
          "setConstraintImpulse",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _impulse) {
            self->setConstraintImpulse(_index, _impulse);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("impulse"))
      .def(
          "getConstraintImpulse",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getConstraintImpulse(_index); },
          ::pybind11::arg("index"))
      .def(
          "resetConstraintImpulses",
          +[](dart::dynamics::ZeroDofJoint*
                  self) { self->resetConstraintImpulses(); })
      .def(
          "integratePositions",
          +[](dart::dynamics::ZeroDofJoint* self, double _dt) {
            self->integratePositions(_dt);
          },
          ::pybind11::arg("dt"))
      .def(
          "integrateVelocities",
          +[](dart::dynamics::ZeroDofJoint* self, double _dt) {
            self->integrateVelocities(_dt);
          },
          ::pybind11::arg("dt"))
      .def(
          "getPositionDifferences",
          +[](const dart::dynamics::ZeroDofJoint* self,
              const Eigen::VectorXd& _q2,
              const Eigen::VectorXd& _q1) -> Eigen::VectorXd {
            return self->getPositionDifferences(_q2, _q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def(
          "setSpringStiffness",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _k) { self->setSpringStiffness(_index, _k); },
          ::pybind11::arg("index"),
          ::pybind11::arg("k"))
      .def(
          "getSpringStiffness",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getSpringStiffness(_index); },
          ::pybind11::arg("index"))
      .def(
          "setRestPosition",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _q0) { self->setRestPosition(_index, _q0); },
          ::pybind11::arg("index"),
          ::pybind11::arg("q0"))
      .def(
          "getRestPosition",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getRestPosition(_index); },
          ::pybind11::arg("index"))
      .def(
          "setDampingCoefficient",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _d) { self->setDampingCoefficient(_index, _d); },
          ::pybind11::arg("index"),
          ::pybind11::arg("d"))
      .def(
          "getDampingCoefficient",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getDampingCoefficient(_index); },
          ::pybind11::arg("index"))
      .def(
          "setCoulombFriction",
          +[](dart::dynamics::ZeroDofJoint* self,
              std::size_t _index,
              double _friction) {
            self->setCoulombFriction(_index, _friction);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("friction"))
      .def(
          "getCoulombFriction",
          +[](const dart::dynamics::ZeroDofJoint* self, std::size_t _index)
              -> double { return self->getCoulombFriction(_index); },
          ::pybind11::arg("index"))
      .def(
          "computePotentialEnergy",
          +[](const dart::dynamics::ZeroDofJoint* self) -> double {
            return self->computePotentialEnergy();
          })
      .def(
          "getBodyConstraintWrench",
          +[](const dart::dynamics::ZeroDofJoint* self) -> Eigen::Vector6d {
            return self->getBodyConstraintWrench();
          });
}

} // namespace python
} // namespace dart
