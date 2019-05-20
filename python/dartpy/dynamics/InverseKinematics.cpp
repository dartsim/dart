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

namespace dart {
namespace python {

void InverseKinematics(pybind11::module& m)
{
  ::pybind11::class_<
      dart::dynamics::InverseKinematics::ErrorMethod,
      dart::common::Subject,
      std::shared_ptr<dart::dynamics::InverseKinematics::ErrorMethod>>(
      m, "InverseKinematicsErrorMethod")
      .def(
          "clone",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self,
              dart::dynamics::InverseKinematics* _newIK)
              -> std::unique_ptr<
                  dart::dynamics::InverseKinematics::ErrorMethod> {
            return self->clone(_newIK);
          },
          ::pybind11::arg("newIK"))
      .def(
          "computeError",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> Eigen::Vector6d { return self->computeError(); })
      .def(
          "computeDesiredTransform",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Isometry3d& _currentTf,
              const Eigen::Vector6d& _error) -> Eigen::Isometry3d {
            return self->computeDesiredTransform(_currentTf, _error);
          },
          ::pybind11::arg("currentTf"),
          ::pybind11::arg("error"))
      .def(
          "getMethodName",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> const std::string& { return self->getMethodName(); },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "setBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->setBounds();
          })
      .def(
          "setBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector6d& _lower) { self->setBounds(_lower); },
          ::pybind11::arg("lower"))
      .def(
          "setBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector6d& _lower,
              const Eigen::Vector6d& _upper) {
            self->setBounds(_lower, _upper);
          },
          ::pybind11::arg("lower"),
          ::pybind11::arg("upper"))
      .def(
          "setBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const std::pair<Eigen::Vector6d, Eigen::Vector6d>& _bounds) {
            self->setBounds(_bounds);
          },
          ::pybind11::arg("bounds"))
      .def(
          "getBounds",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> const std::pair<Eigen::Vector6d, Eigen::Vector6d>& {
            return self->getBounds();
          })
      .def(
          "setAngularBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->setAngularBounds();
          })
      .def(
          "setAngularBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _lower) {
            self->setAngularBounds(_lower);
          },
          ::pybind11::arg("lower"))
      .def(
          "setAngularBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _lower,
              const Eigen::Vector3d& _upper) {
            self->setAngularBounds(_lower, _upper);
          },
          ::pybind11::arg("lower"),
          ::pybind11::arg("upper"))
      .def(
          "setAngularBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds) {
            self->setAngularBounds(_bounds);
          },
          ::pybind11::arg("bounds"))
      .def(
          "getAngularBounds",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> std::pair<Eigen::Vector3d, Eigen::Vector3d> {
            return self->getAngularBounds();
          })
      .def(
          "setLinearBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->setLinearBounds();
          })
      .def(
          "setLinearBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _lower) { self->setLinearBounds(_lower); },
          ::pybind11::arg("lower"))
      .def(
          "setLinearBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _lower,
              const Eigen::Vector3d& _upper) {
            self->setLinearBounds(_lower, _upper);
          },
          ::pybind11::arg("lower"),
          ::pybind11::arg("upper"))
      .def(
          "setLinearBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds) {
            self->setLinearBounds(_bounds);
          },
          ::pybind11::arg("bounds"))
      .def(
          "getLinearBounds",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> std::pair<Eigen::Vector3d, Eigen::Vector3d> {
            return self->getLinearBounds();
          })
      .def(
          "setErrorLengthClamp",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->setErrorLengthClamp();
          })
      .def(
          "setErrorLengthClamp",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              double _clampSize) { self->setErrorLengthClamp(_clampSize); },
          ::pybind11::arg("clampSize"))
      .def(
          "getErrorLengthClamp",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> double { return self->getErrorLengthClamp(); })
      .def(
          "setErrorWeights",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector6d& _weights) {
            self->setErrorWeights(_weights);
          },
          ::pybind11::arg("weights"))
      .def(
          "setAngularErrorWeights",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->setAngularErrorWeights();
          })
      .def(
          "setAngularErrorWeights",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _weights) {
            self->setAngularErrorWeights(_weights);
          },
          ::pybind11::arg("weights"))
      .def(
          "getAngularErrorWeights",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> Eigen::Vector3d { return self->getAngularErrorWeights(); })
      .def(
          "setLinearErrorWeights",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->setLinearErrorWeights();
          })
      .def(
          "setLinearErrorWeights",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _weights) {
            self->setLinearErrorWeights(_weights);
          },
          ::pybind11::arg("weights"))
      .def(
          "getLinearErrorWeights",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> Eigen::Vector3d { return self->getLinearErrorWeights(); })
      .def(
          "getErrorMethodProperties",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> dart::dynamics::InverseKinematics::ErrorMethod::Properties {
            return self->getErrorMethodProperties();
          })
      .def(
          "clearCache",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->clearCache();
          });

  ::pybind11::class_<
      dart::dynamics::InverseKinematics,
      dart::common::Subject,
      std::shared_ptr<dart::dynamics::InverseKinematics>>(
      m, "InverseKinematics")
      .def(
          "clone",
          +[](const dart::dynamics::InverseKinematics* self,
              dart::dynamics::JacobianNode* _newNode)
              -> dart::dynamics::InverseKinematicsPtr {
            return self->clone(_newNode);
          },
          ::pybind11::arg("newNode"))
      .def(
          "setActive",
          +[](dart::dynamics::InverseKinematics* self) { self->setActive(); })
      .def(
          "setActive",
          +[](dart::dynamics::InverseKinematics* self, bool _active) {
            self->setActive(_active);
          },
          ::pybind11::arg("active"))
      .def(
          "setInactive",
          +[](dart::dynamics::InverseKinematics* self) { self->setInactive(); })
      .def(
          "isActive",
          +[](const dart::dynamics::InverseKinematics* self) -> bool {
            return self->isActive();
          })
      .def(
          "setHierarchyLevel",
          +[](dart::dynamics::InverseKinematics* self, std::size_t _level) {
            self->setHierarchyLevel(_level);
          },
          ::pybind11::arg("level"))
      .def(
          "getHierarchyLevel",
          +[](const dart::dynamics::InverseKinematics* self) -> std::size_t {
            return self->getHierarchyLevel();
          })
      .def(
          "useChain",
          +[](dart::dynamics::InverseKinematics* self) { self->useChain(); })
      .def(
          "useWholeBody",
          +[](dart::dynamics::InverseKinematics* self) {
            self->useWholeBody();
          })
      .def(
          "setDofs",
          +[](dart::dynamics::InverseKinematics* self,
              const std::vector<std::size_t>& _dofs) { self->setDofs(_dofs); },
          ::pybind11::arg("dofs"))
      .def(
          "setObjective",
          +[](dart::dynamics::InverseKinematics* self,
              const std::shared_ptr<dart::optimizer::Function>& _objective) {
            self->setObjective(_objective);
          },
          ::pybind11::arg("objective"))
      .def(
          "getObjective",
          +[](const dart::dynamics::InverseKinematics* self)
              -> std::shared_ptr<const dart::optimizer::Function> {
            return self->getObjective();
          })
      .def(
          "setNullSpaceObjective",
          +[](dart::dynamics::InverseKinematics* self,
              const std::shared_ptr<dart::optimizer::Function>& _nsObjective) {
            self->setNullSpaceObjective(_nsObjective);
          },
          ::pybind11::arg("nsObjective"))
      .def(
          "getNullSpaceObjective",
          +[](const dart::dynamics::InverseKinematics* self)
              -> std::shared_ptr<const dart::optimizer::Function> {
            return self->getNullSpaceObjective();
          })
      .def(
          "hasNullSpaceObjective",
          +[](const dart::dynamics::InverseKinematics* self) -> bool {
            return self->hasNullSpaceObjective();
          })
      .def(
          "getErrorMethod",
          +[](dart::dynamics::InverseKinematics* self)
              -> dart::dynamics::InverseKinematics::ErrorMethod& {
            return self->getErrorMethod();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getProblem",
          +[](const dart::dynamics::InverseKinematics* self)
              -> std::shared_ptr<const dart::optimizer::Problem> {
            return self->getProblem();
          })
      .def(
          "resetProblem",
          +[](dart::dynamics::InverseKinematics* self) {
            self->resetProblem();
          })
      .def(
          "resetProblem",
          +[](dart::dynamics::InverseKinematics* self, bool _clearSeeds) {
            self->resetProblem(_clearSeeds);
          },
          ::pybind11::arg("clearSeeds"))
      .def(
          "setSolver",
          +[](dart::dynamics::InverseKinematics* self,
              const std::shared_ptr<dart::optimizer::Solver>& _newSolver) {
            self->setSolver(_newSolver);
          },
          ::pybind11::arg("newSolver"))
      .def(
          "getSolver",
          +[](dart::dynamics::InverseKinematics* self)
              -> std::shared_ptr<dart::optimizer::Solver> {
            return self->getSolver();
          })
      .def(
          "setOffset",
          +[](dart::dynamics::InverseKinematics* self) { self->setOffset(); })
      .def(
          "setOffset",
          +[](dart::dynamics::InverseKinematics* self,
              const Eigen::Vector3d& _offset) { self->setOffset(_offset); },
          ::pybind11::arg("offset"))
      .def(
          "getOffset",
          +[](const dart::dynamics::InverseKinematics* self)
              -> const Eigen::Vector3d& { return self->getOffset(); },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "hasOffset",
          +[](const dart::dynamics::InverseKinematics* self) -> bool {
            return self->hasOffset();
          })
      .def(
          "setTarget",
          +[](dart::dynamics::InverseKinematics* self,
              std::shared_ptr<dart::dynamics::SimpleFrame> _newTarget) {
            self->setTarget(_newTarget);
          },
          ::pybind11::arg("newTarget"))
      .def(
          "getTarget",
          +[](dart::dynamics::InverseKinematics* self)
              -> std::shared_ptr<dart::dynamics::SimpleFrame> {
            return self->getTarget();
          })
      .def(
          "getTarget",
          +[](const dart::dynamics::InverseKinematics* self)
              -> std::shared_ptr<const dart::dynamics::SimpleFrame> {
            return self->getTarget();
          })
      .def(
          "getPositions",
          +[](const dart::dynamics::InverseKinematics* self)
              -> Eigen::VectorXd { return self->getPositions(); })
      .def(
          "setPositions",
          +[](dart::dynamics::InverseKinematics* self,
              const Eigen::VectorXd& _q) { self->setPositions(_q); },
          ::pybind11::arg("q"))
      .def(
          "clearCaches",
          +[](dart::dynamics::InverseKinematics* self) { self->clearCaches(); })
      .def_static(
          "create",
          +[](dart::dynamics::JacobianNode* _node)
              -> dart::dynamics::InverseKinematicsPtr {
            return dart::dynamics::InverseKinematics::create(_node);
          },
          ::pybind11::arg("node"));
}

} // namespace python
} // namespace dart
