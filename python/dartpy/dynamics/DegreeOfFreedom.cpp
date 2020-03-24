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
#include <pybind11/pybind11.h>
#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

namespace py = pybind11;

namespace dart {
namespace python {

void DegreeOfFreedom(py::module& m)
{
  ::py::class_<
      dart::dynamics::DegreeOfFreedom,
      dart::common::Subject,
      std::shared_ptr<dart::dynamics::DegreeOfFreedom>>(m, "DegreeOfFreedom")
      .def(
          "setName",
          +[](dart::dynamics::DegreeOfFreedom* self, const std::string& _name)
              -> const std::string& { return self->setName(_name); },
          ::py::return_value_policy::reference_internal,
          ::py::arg("name"))
      .def(
          "setName",
          +[](dart::dynamics::DegreeOfFreedom* self,
              const std::string& _name,
              bool _preserveName) -> const std::string& {
            return self->setName(_name, _preserveName);
          },
          ::py::return_value_policy::reference_internal,
          ::py::arg("name"),
          ::py::arg("preserveName"))
      .def(
          "getName",
          +[](const dart::dynamics::DegreeOfFreedom* self)
              -> const std::string& { return self->getName(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "preserveName",
          +[](dart::dynamics::DegreeOfFreedom* self, bool _preserve) {
            self->preserveName(_preserve);
          },
          ::py::arg("preserve"))
      .def(
          "isNamePreserved",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> bool {
            return self->isNamePreserved();
          })
      .def(
          "getIndexInSkeleton",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> std::size_t {
            return self->getIndexInSkeleton();
          })
      .def(
          "getIndexInTree",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> std::size_t {
            return self->getIndexInTree();
          })
      .def(
          "getIndexInJoint",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> std::size_t {
            return self->getIndexInJoint();
          })
      .def(
          "getTreeIndex",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> std::size_t {
            return self->getTreeIndex();
          })
      .def(
          "setCommand",
          +[](dart::dynamics::DegreeOfFreedom* self, double _command) {
            self->setCommand(_command);
          },
          ::py::arg("command"))
      .def(
          "getCommand",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getCommand();
          })
      .def(
          "resetCommand",
          +[](dart::dynamics::DegreeOfFreedom* self) { self->resetCommand(); })
      .def(
          "setPosition",
          +[](dart::dynamics::DegreeOfFreedom* self, double _position) {
            self->setPosition(_position);
          },
          ::py::arg("position"))
      .def(
          "getPosition",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getPosition();
          })
      .def(
          "setPositionLimits",
          +[](dart::dynamics::DegreeOfFreedom* self,
              double _lowerLimit,
              double _upperLimit) {
            self->setPositionLimits(_lowerLimit, _upperLimit);
          },
          ::py::arg("lowerLimit"),
          ::py::arg("upperLimit"))
      .def(
          "setPositionLimits",
          +[](dart::dynamics::DegreeOfFreedom* self,
              const std::pair<double, double>& _limits) {
            self->setPositionLimits(_limits);
          },
          ::py::arg("limits"))
      .def(
          "getPositionLimits",
          +[](const dart::dynamics::DegreeOfFreedom* self)
              -> std::pair<double, double> {
            return self->getPositionLimits();
          })
      .def(
          "setPositionLowerLimit",
          +[](dart::dynamics::DegreeOfFreedom* self, double _limit) {
            self->setPositionLowerLimit(_limit);
          },
          ::py::arg("limit"))
      .def(
          "getPositionLowerLimit",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getPositionLowerLimit();
          })
      .def(
          "setPositionUpperLimit",
          +[](dart::dynamics::DegreeOfFreedom* self, double _limit) {
            self->setPositionUpperLimit(_limit);
          },
          ::py::arg("limit"))
      .def(
          "getPositionUpperLimit",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getPositionUpperLimit();
          })
      .def(
          "isCyclic",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> bool {
            return self->isCyclic();
          })
      .def(
          "hasPositionLimit",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> bool {
            return self->hasPositionLimit();
          })
      .def(
          "resetPosition",
          +[](dart::dynamics::DegreeOfFreedom* self) { self->resetPosition(); })
      .def(
          "setInitialPosition",
          +[](dart::dynamics::DegreeOfFreedom* self, double _initial) {
            self->setInitialPosition(_initial);
          },
          ::py::arg("initial"))
      .def(
          "getInitialPosition",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getInitialPosition();
          })
      .def(
          "setVelocity",
          +[](dart::dynamics::DegreeOfFreedom* self, double _velocity) {
            self->setVelocity(_velocity);
          },
          ::py::arg("velocity"))
      .def(
          "getVelocity",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getVelocity();
          })
      .def(
          "setVelocityLimits",
          +[](dart::dynamics::DegreeOfFreedom* self,
              double _lowerLimit,
              double _upperLimit) {
            self->setVelocityLimits(_lowerLimit, _upperLimit);
          },
          ::py::arg("lowerLimit"),
          ::py::arg("upperLimit"))
      .def(
          "setVelocityLimits",
          +[](dart::dynamics::DegreeOfFreedom* self,
              const std::pair<double, double>& _limits) {
            self->setVelocityLimits(_limits);
          },
          ::py::arg("limits"))
      .def(
          "getVelocityLimits",
          +[](const dart::dynamics::DegreeOfFreedom* self)
              -> std::pair<double, double> {
            return self->getVelocityLimits();
          })
      .def(
          "setVelocityLowerLimit",
          +[](dart::dynamics::DegreeOfFreedom* self, double _limit) {
            self->setVelocityLowerLimit(_limit);
          },
          ::py::arg("limit"))
      .def(
          "getVelocityLowerLimit",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getVelocityLowerLimit();
          })
      .def(
          "setVelocityUpperLimit",
          +[](dart::dynamics::DegreeOfFreedom* self, double _limit) {
            self->setVelocityUpperLimit(_limit);
          },
          ::py::arg("limit"))
      .def(
          "getVelocityUpperLimit",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getVelocityUpperLimit();
          })
      .def(
          "resetVelocity",
          +[](dart::dynamics::DegreeOfFreedom* self) { self->resetVelocity(); })
      .def(
          "setInitialVelocity",
          +[](dart::dynamics::DegreeOfFreedom* self, double _initial) {
            self->setInitialVelocity(_initial);
          },
          ::py::arg("initial"))
      .def(
          "getInitialVelocity",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getInitialVelocity();
          })
      .def(
          "setAcceleration",
          +[](dart::dynamics::DegreeOfFreedom* self, double _acceleration) {
            self->setAcceleration(_acceleration);
          },
          ::py::arg("acceleration"))
      .def(
          "getAcceleration",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getAcceleration();
          })
      .def(
          "resetAcceleration",
          +[](dart::dynamics::DegreeOfFreedom* self) {
            self->resetAcceleration();
          })
      .def(
          "setAccelerationLimits",
          +[](dart::dynamics::DegreeOfFreedom* self,
              double _lowerLimit,
              double _upperLimit) {
            self->setAccelerationLimits(_lowerLimit, _upperLimit);
          },
          ::py::arg("lowerLimit"),
          ::py::arg("upperLimit"))
      .def(
          "setAccelerationLimits",
          +[](dart::dynamics::DegreeOfFreedom* self,
              const std::pair<double, double>& _limits) {
            self->setAccelerationLimits(_limits);
          },
          ::py::arg("limits"))
      .def(
          "getAccelerationLimits",
          +[](const dart::dynamics::DegreeOfFreedom* self)
              -> std::pair<double, double> {
            return self->getAccelerationLimits();
          })
      .def(
          "setAccelerationLowerLimit",
          +[](dart::dynamics::DegreeOfFreedom* self, double _limit) {
            self->setAccelerationLowerLimit(_limit);
          },
          ::py::arg("limit"))
      .def(
          "getAccelerationLowerLimit",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getAccelerationLowerLimit();
          })
      .def(
          "setAccelerationUpperLimit",
          +[](dart::dynamics::DegreeOfFreedom* self, double _limit) {
            self->setAccelerationUpperLimit(_limit);
          },
          ::py::arg("limit"))
      .def(
          "getAccelerationUpperLimit",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getAccelerationUpperLimit();
          })
      .def(
          "setForce",
          +[](dart::dynamics::DegreeOfFreedom* self, double _force) {
            self->setForce(_force);
          },
          ::py::arg("force"))
      .def(
          "getForce",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getForce();
          })
      .def(
          "resetForce",
          +[](dart::dynamics::DegreeOfFreedom* self) { self->resetForce(); })
      .def(
          "setForceLimits",
          +[](dart::dynamics::DegreeOfFreedom* self,
              double _lowerLimit,
              double _upperLimit) {
            self->setForceLimits(_lowerLimit, _upperLimit);
          },
          ::py::arg("lowerLimit"),
          ::py::arg("upperLimit"))
      .def(
          "setForceLimits",
          +[](dart::dynamics::DegreeOfFreedom* self,
              const std::pair<double, double>& _limits) {
            self->setForceLimits(_limits);
          },
          ::py::arg("limits"))
      .def(
          "getForceLimits",
          +[](const dart::dynamics::DegreeOfFreedom* self)
              -> std::pair<double, double> { return self->getForceLimits(); })
      .def(
          "setForceLowerLimit",
          +[](dart::dynamics::DegreeOfFreedom* self, double _limit) {
            self->setForceLowerLimit(_limit);
          },
          ::py::arg("limit"))
      .def(
          "getForceLowerLimit",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getForceLowerLimit();
          })
      .def(
          "setForceUpperLimit",
          +[](dart::dynamics::DegreeOfFreedom* self, double _limit) {
            self->setForceUpperLimit(_limit);
          },
          ::py::arg("limit"))
      .def(
          "getForceUpperLimit",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getForceUpperLimit();
          })
      .def(
          "setVelocityChange",
          +[](dart::dynamics::DegreeOfFreedom* self, double _velocityChange) {
            self->setVelocityChange(_velocityChange);
          },
          ::py::arg("velocityChange"))
      .def(
          "getVelocityChange",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getVelocityChange();
          })
      .def(
          "resetVelocityChange",
          +[](dart::dynamics::DegreeOfFreedom* self) {
            self->resetVelocityChange();
          })
      .def(
          "setConstraintImpulse",
          +[](dart::dynamics::DegreeOfFreedom* self, double _impulse) {
            self->setConstraintImpulse(_impulse);
          },
          ::py::arg("impulse"))
      .def(
          "getConstraintImpulse",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getConstraintImpulse();
          })
      .def(
          "resetConstraintImpulse",
          +[](dart::dynamics::DegreeOfFreedom* self) {
            self->resetConstraintImpulse();
          })
      .def(
          "setSpringStiffness",
          +[](dart::dynamics::DegreeOfFreedom* self, double _k) {
            self->setSpringStiffness(_k);
          },
          ::py::arg("k"))
      .def(
          "getSpringStiffness",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getSpringStiffness();
          })
      .def(
          "setRestPosition",
          +[](dart::dynamics::DegreeOfFreedom* self, double _q0) {
            self->setRestPosition(_q0);
          },
          ::py::arg("q0"))
      .def(
          "getRestPosition",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getRestPosition();
          })
      .def(
          "setDampingCoefficient",
          +[](dart::dynamics::DegreeOfFreedom* self, double _coeff) {
            self->setDampingCoefficient(_coeff);
          },
          ::py::arg("coeff"))
      .def(
          "getDampingCoefficient",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getDampingCoefficient();
          })
      .def(
          "setCoulombFriction",
          +[](dart::dynamics::DegreeOfFreedom* self, double _friction) {
            self->setCoulombFriction(_friction);
          },
          ::py::arg("friction"))
      .def(
          "getCoulombFriction",
          +[](const dart::dynamics::DegreeOfFreedom* self) -> double {
            return self->getCoulombFriction();
          })
      .def(
          "getSkeleton",
          +[](dart::dynamics::DegreeOfFreedom* self)
              -> dart::dynamics::SkeletonPtr { return self->getSkeleton(); })
      .def(
          "getSkeleton",
          +[](const dart::dynamics::DegreeOfFreedom* self)
              -> dart::dynamics::ConstSkeletonPtr {
            return self->getSkeleton();
          });
}

} // namespace python
} // namespace dart
