/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

#include <dart/dart.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace dart {
namespace python {

void InverseKinematics(py::module& m)
{
  ::py::class_<dart::dynamics::InverseKinematics::GradientMethod::Properties>(
      m, "InverseKinematicsGradientMethodProperties")
      .def(
          ::py::init<double, const Eigen::VectorXd&>(),
          ::py::arg("clamp") = dart::dynamics::DefaultIKGradientComponentClamp,
          ::py::arg("weights") = Eigen::VectorXd())
      .def_readwrite(
          "mComponentWiseClamp",
          &dart::dynamics::InverseKinematics::GradientMethod::Properties::
              mComponentWiseClamp)
      .def_readwrite(
          "mComponentWeights",
          &dart::dynamics::InverseKinematics::GradientMethod::Properties::
              mComponentWeights);

  ::py::class_<
      dart::dynamics::InverseKinematics::GradientMethod,
      dart::common::Subject,
      std::shared_ptr<dart::dynamics::InverseKinematics::GradientMethod>>(
      m, "InverseKinematicsGradientMethod")
      .def(
          "clone",
          +[](const dart::dynamics::InverseKinematics::GradientMethod* self,
              dart::dynamics::InverseKinematics* _newIK)
              -> std::unique_ptr<
                  dart::dynamics::InverseKinematics::GradientMethod> {
            return self->clone(_newIK);
          },
          ::py::arg("newIK"))
      .def(
          "computeGradient",
          +[](dart::dynamics::InverseKinematics::GradientMethod* self,
              const Eigen::Vector6d& _error,
              Eigen::VectorXd& _grad) { self->computeGradient(_error, _grad); },
          ::py::arg("error"),
          ::py::arg("grad"))
      .def(
          "evalGradient",
          +[](dart::dynamics::InverseKinematics::GradientMethod* self,
              const Eigen::VectorXd& _q,
              Eigen::VectorXd& _grad) {
            Eigen::Map<Eigen::VectorXd> grad_map(_grad.data(), _grad.size());
            self->evalGradient(_q, grad_map);
          },
          ::py::arg("q"),
          ::py::arg("grad"))
      .def(
          "getMethodName",
          +[](const dart::dynamics::InverseKinematics::GradientMethod* self)
              -> const std::string& { return self->getMethodName(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "clampGradient",
          +[](const dart::dynamics::InverseKinematics::GradientMethod* self,
              Eigen::VectorXd& _grad) { self->clampGradient(_grad); },
          ::py::arg("grad"))
      .def(
          "setComponentWiseClamp",
          +[](dart::dynamics::InverseKinematics::GradientMethod* self) {
            self->setComponentWiseClamp();
          })
      .def(
          "setComponentWiseClamp",
          +[](dart::dynamics::InverseKinematics::GradientMethod* self,
              double _clamp) { self->setComponentWiseClamp(_clamp); },
          ::py::arg("clamp"))
      .def(
          "getComponentWiseClamp",
          +[](const dart::dynamics::InverseKinematics::GradientMethod* self)
              -> double { return self->getComponentWiseClamp(); })
      .def(
          "applyWeights",
          +[](const dart::dynamics::InverseKinematics::GradientMethod* self,
              Eigen::VectorXd& _grad) { self->applyWeights(_grad); },
          ::py::arg("grad"))
      .def(
          "setComponentWeights",
          +[](dart::dynamics::InverseKinematics::GradientMethod* self,
              const Eigen::VectorXd& _weights) {
            self->setComponentWeights(_weights);
          },
          ::py::arg("weights"))
      .def(
          "getComponentWeights",
          +[](const dart::dynamics::InverseKinematics::GradientMethod* self)
              -> const Eigen::VectorXd& { return self->getComponentWeights(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "convertJacobianMethodOutputToGradient",
          +[](dart::dynamics::InverseKinematics::GradientMethod* self,
              Eigen::VectorXd& grad,
              const std::vector<std::size_t>& dofs) {
            self->convertJacobianMethodOutputToGradient(grad, dofs);
          },
          ::py::arg("grad"),
          ::py::arg("dofs"))
      .def(
          "getGradientMethodProperties",
          +[](const dart::dynamics::InverseKinematics::GradientMethod* self)
              -> dart::dynamics::InverseKinematics::GradientMethod::Properties {
            return self->getGradientMethodProperties();
          })
      .def(
          "clearCache",
          +[](dart::dynamics::InverseKinematics::GradientMethod* self) {
            self->clearCache();
          })
      .def(
          "getIK",
          +[](dart::dynamics::InverseKinematics::GradientMethod* self)
              -> dart::dynamics::InverseKinematics* { return self->getIK(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "getIK",
          +[](const dart::dynamics::InverseKinematics::GradientMethod* self)
              -> const dart::dynamics::InverseKinematics* {
            return self->getIK();
          },
          ::py::return_value_policy::reference_internal);

  ::py::class_<
      dart::dynamics::InverseKinematics::JacobianDLS::UniqueProperties>(
      m, "InverseKinematicsJacobianDLSUniqueProperties")
      .def(
          ::py::init<double>(),
          ::py::arg("damping") = dart::dynamics::DefaultIKDLSCoefficient)
      .def_readwrite(
          "mDamping",
          &dart::dynamics::InverseKinematics::JacobianDLS::UniqueProperties::
              mDamping);

  ::py::class_<
      dart::dynamics::InverseKinematics::JacobianDLS::Properties,
      dart::dynamics::InverseKinematics::GradientMethod::Properties,
      dart::dynamics::InverseKinematics::JacobianDLS::UniqueProperties>(
      m, "InverseKinematicsJacobianDLSProperties")
      .def(
          ::py::init<
              const dart::dynamics::InverseKinematics::GradientMethod::
                  Properties&,
              const dart::dynamics::InverseKinematics::JacobianDLS::
                  UniqueProperties&>(),
          ::py::arg("gradientProperties")
          = dart::dynamics::InverseKinematics::GradientMethod::Properties(),
          ::py::arg("dlsProperties")
          = dart::dynamics::InverseKinematics::JacobianDLS::UniqueProperties());

  ::py::class_<
      dart::dynamics::InverseKinematics::JacobianDLS,
      dart::dynamics::InverseKinematics::GradientMethod,
      std::shared_ptr<dart::dynamics::InverseKinematics::JacobianDLS>>(
      m, "InverseKinematicsJacobianDLS")
      .def(
          ::py::init<
              dart::dynamics::InverseKinematics*,
              dart::dynamics::InverseKinematics::JacobianDLS::Properties>(),
          ::py::arg("ik"),
          ::py::arg("properties")
          = dart::dynamics::InverseKinematics::JacobianDLS::Properties())
      .def(
          "setDampingCoefficient",
          +[](dart::dynamics::InverseKinematics::JacobianDLS* self) {
            self->setDampingCoefficient();
          })
      .def(
          "setDampingCoefficient",
          +[](dart::dynamics::InverseKinematics::JacobianDLS* self,
              double _damping) { self->setDampingCoefficient(_damping); },
          ::py::arg("damping"))
      .def(
          "getDampingCoefficient",
          +[](const dart::dynamics::InverseKinematics::JacobianDLS* self)
              -> double { return self->getDampingCoefficient(); })
      .def(
          "getJacobianDLSProperties",
          +[](const dart::dynamics::InverseKinematics::JacobianDLS* self)
              -> dart::dynamics::InverseKinematics::JacobianDLS::Properties {
            return self->getJacobianDLSProperties();
          });

  ::py::class_<
      dart::dynamics::InverseKinematics::JacobianTranspose,
      dart::dynamics::InverseKinematics::GradientMethod,
      std::shared_ptr<dart::dynamics::InverseKinematics::JacobianTranspose>>(
      m, "InverseKinematicsJacobianTranspose")
      .def(
          ::py::init<
              dart::dynamics::InverseKinematics*,
              dart::dynamics::InverseKinematics::GradientMethod::Properties>(),
          ::py::arg("ik"),
          ::py::arg("properties")
          = dart::dynamics::InverseKinematics::GradientMethod::Properties());

  ::py::class_<dart::dynamics::InverseKinematics::ErrorMethod::Properties>(
      m, "InverseKinematicsErrorMethodProperties")
      .def(
          ::py::init<
              const dart::dynamics::InverseKinematics::ErrorMethod::Bounds&,
              double,
              const Eigen::Vector6d&>(),
          ::py::arg("bounds")
          = dart::dynamics::InverseKinematics::ErrorMethod::Bounds(
              Eigen::Vector6d::Constant(-dart::dynamics::DefaultIKTolerance),
              Eigen::Vector6d::Constant(dart::dynamics::DefaultIKTolerance)),
          ::py::arg("errorClamp") = dart::dynamics::DefaultIKErrorClamp,
          ::py::arg("errorWeights") = Eigen::compose(
              Eigen::Vector3d::Constant(dart::dynamics::DefaultIKAngularWeight),
              Eigen::Vector3d::Constant(dart::dynamics::DefaultIKLinearWeight)))
      .def_readwrite(
          "mBounds",
          &dart::dynamics::InverseKinematics::ErrorMethod::Properties::mBounds)
      .def_readwrite(
          "mErrorLengthClamp",
          &dart::dynamics::InverseKinematics::ErrorMethod::Properties::
              mErrorLengthClamp)
      .def_readwrite(
          "mErrorWeights",
          &dart::dynamics::InverseKinematics::ErrorMethod::Properties::
              mErrorWeights);

  ::py::class_<
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
          ::py::arg("newIK"))
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
          ::py::arg("currentTf"),
          ::py::arg("error"))
      .def(
          "getMethodName",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> const std::string& { return self->getMethodName(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "setBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->setBounds();
          })
      .def(
          "setBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector6d& _lower) { self->setBounds(_lower); },
          ::py::arg("lower"))
      .def(
          "setBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector6d& _lower,
              const Eigen::Vector6d& _upper) {
            self->setBounds(_lower, _upper);
          },
          ::py::arg("lower"),
          ::py::arg("upper"))
      .def(
          "setBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const std::pair<Eigen::Vector6d, Eigen::Vector6d>& _bounds) {
            self->setBounds(_bounds);
          },
          ::py::arg("bounds"))
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
          ::py::arg("lower"))
      .def(
          "setAngularBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _lower,
              const Eigen::Vector3d& _upper) {
            self->setAngularBounds(_lower, _upper);
          },
          ::py::arg("lower"),
          ::py::arg("upper"))
      .def(
          "setAngularBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds) {
            self->setAngularBounds(_bounds);
          },
          ::py::arg("bounds"))
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
          ::py::arg("lower"))
      .def(
          "setLinearBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _lower,
              const Eigen::Vector3d& _upper) {
            self->setLinearBounds(_lower, _upper);
          },
          ::py::arg("lower"),
          ::py::arg("upper"))
      .def(
          "setLinearBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds) {
            self->setLinearBounds(_bounds);
          },
          ::py::arg("bounds"))
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
          ::py::arg("clampSize"))
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
          ::py::arg("weights"))
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
          ::py::arg("weights"))
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
          ::py::arg("weights"))
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

  ::py::class_<
      dart::dynamics::InverseKinematics::TaskSpaceRegion::UniqueProperties>(
      m, "InverseKinematicsTaskSpaceRegionUniqueProperties")
      .def(
          ::py::init<bool, dart::dynamics::SimpleFramePtr>(),
          ::py::arg("computeErrorFromCenter") = true,
          ::py::arg("referenceFrame") = nullptr)
      .def_readwrite(
          "mComputeErrorFromCenter",
          &dart::dynamics::InverseKinematics::TaskSpaceRegion::
              UniqueProperties::mComputeErrorFromCenter)
      .def_readwrite(
          "mReferenceFrame",
          &dart::dynamics::InverseKinematics::TaskSpaceRegion::
              UniqueProperties::mReferenceFrame);

  ::py::class_<
      dart::dynamics::InverseKinematics::TaskSpaceRegion::Properties,
      dart::dynamics::InverseKinematics::ErrorMethod::Properties,
      dart::dynamics::InverseKinematics::TaskSpaceRegion::UniqueProperties>(
      m, "InverseKinematicsTaskSpaceRegionProperties")
      .def(
          ::py::init<
              const dart::dynamics::InverseKinematics::ErrorMethod::Properties&,
              const dart::dynamics::InverseKinematics::TaskSpaceRegion::
                  UniqueProperties&>(),
          ::py::arg("errorProperties")
          = dart::dynamics::InverseKinematics::ErrorMethod::Properties(),
          ::py::arg("taskSpaceProperties") = dart::dynamics::InverseKinematics::
              TaskSpaceRegion::UniqueProperties());

  ::py::class_<
      dart::dynamics::InverseKinematics::TaskSpaceRegion,
      dart::dynamics::InverseKinematics::ErrorMethod,
      std::shared_ptr<dart::dynamics::InverseKinematics::TaskSpaceRegion>>(
      m, "InverseKinematicsTaskSpaceRegion")
      .def(
          ::py::init<
              dart::dynamics::InverseKinematics*,
              dart::dynamics::InverseKinematics::TaskSpaceRegion::Properties>(),
          ::py::arg("ik"),
          ::py::arg("properties")
          = dart::dynamics::InverseKinematics::TaskSpaceRegion::Properties())
      .def(
          "setComputeFromCenter",
          &dart::dynamics::InverseKinematics::TaskSpaceRegion::
              setComputeFromCenter,
          ::py::arg("computeFromCenter"),
          "Set whether this TaskSpaceRegion should compute its error vector "
          "from the center of the region.")
      .def(
          "isComputingFromCenter",
          &dart::dynamics::InverseKinematics::TaskSpaceRegion::
              isComputingFromCenter,
          "Get whether this TaskSpaceRegion is set to compute its error vector "
          "from the center of the region.")
      .def(
          "setReferenceFrame",
          &dart::dynamics::InverseKinematics::TaskSpaceRegion::
              setReferenceFrame,
          ::py::arg("referenceFrame"),
          "Set the reference frame that the task space region is expressed. "
          "Pass None to use the parent frame of the target frame instead.")
      .def(
          "getReferenceFrame",
          &dart::dynamics::InverseKinematics::TaskSpaceRegion::
              getReferenceFrame,
          "Get the reference frame that the task space region is expressed.")
      .def(
          "getTaskSpaceRegionProperties",
          &dart::dynamics::InverseKinematics::TaskSpaceRegion::
              getTaskSpaceRegionProperties,
          "Get the Properties of this TaskSpaceRegion.");

  ::py::class_<
      dart::dynamics::InverseKinematics,
      dart::common::Subject,
      std::shared_ptr<dart::dynamics::InverseKinematics>>(
      m, "InverseKinematics")
      .def(
          ::py::init(
              +[](dart::dynamics::JacobianNode* node)
                  -> dart::dynamics::InverseKinematicsPtr {
                return dart::dynamics::InverseKinematics::create(node);
              }),
          ::py::arg("node"))
      .def(
          "findSolution",
          +[](dart::dynamics::InverseKinematics* self,
              Eigen::VectorXd& positions) -> bool {
            return self->findSolution(positions);
          },
          py::arg("positions"))
      .def(
          "solveAndApply",
          +[](dart::dynamics::InverseKinematics* self) -> bool {
            return self->solveAndApply();
          })
      .def(
          "solveAndApply",
          +[](dart::dynamics::InverseKinematics* self,
              bool allowIncompleteResult) -> bool {
            return self->solveAndApply(allowIncompleteResult);
          },
          py::arg("allowIncompleteResult"))
      .def(
          "solveAndApply",
          +[](dart::dynamics::InverseKinematics* self,
              Eigen::VectorXd& positions,
              bool allowIncompleteResult) -> bool {
            return self->solveAndApply(positions, allowIncompleteResult);
          },
          py::arg("positions"),
          py::arg("allowIncompleteResult"))
      .def(
          "clone",
          +[](const dart::dynamics::InverseKinematics* self,
              dart::dynamics::JacobianNode* _newNode)
              -> dart::dynamics::InverseKinematicsPtr {
            return self->clone(_newNode);
          },
          ::py::arg("newNode"))
      .def(
          "setActive",
          +[](dart::dynamics::InverseKinematics* self) { self->setActive(); })
      .def(
          "setActive",
          +[](dart::dynamics::InverseKinematics* self, bool _active) {
            self->setActive(_active);
          },
          ::py::arg("active"))
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
          ::py::arg("level"))
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
          ::py::arg("dofs"))
      .def(
          "setObjective",
          +[](dart::dynamics::InverseKinematics* self,
              const std::shared_ptr<dart::optimizer::Function>& _objective) {
            self->setObjective(_objective);
          },
          ::py::arg("objective"))
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
          ::py::arg("nsObjective"))
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
          ::py::return_value_policy::reference_internal)
      .def(
          "getGradientMethod",
          +[](dart::dynamics::InverseKinematics* self)
              -> dart::dynamics::InverseKinematics::GradientMethod& {
            return self->getGradientMethod();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "getGradientMethod",
          +[](const dart::dynamics::InverseKinematics* self)
              -> const dart::dynamics::InverseKinematics::GradientMethod& {
            return self->getGradientMethod();
          },
          ::py::return_value_policy::reference_internal)
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
          ::py::arg("clearSeeds"))
      .def(
          "setSolver",
          +[](dart::dynamics::InverseKinematics* self,
              const std::shared_ptr<dart::optimizer::Solver>& _newSolver) {
            self->setSolver(_newSolver);
          },
          ::py::arg("newSolver"))
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
          ::py::arg("offset"))
      .def(
          "getOffset",
          +[](const dart::dynamics::InverseKinematics* self)
              -> const Eigen::Vector3d& { return self->getOffset(); },
          ::py::return_value_policy::reference_internal)
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
          ::py::arg("newTarget"))
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
          ::py::arg("q"))
      .def(
          "clearCaches", +[](dart::dynamics::InverseKinematics* self) {
            self->clearCaches();
          });

  // HierarchicalIK class (parent of WholeBodyIK) - MUST be defined first
  ::py::class_<
      dart::dynamics::HierarchicalIK,
      dart::common::Subject,
      std::shared_ptr<dart::dynamics::HierarchicalIK>>(m, "HierarchicalIK")
      .def(
          "solveAndApply",
          +[](dart::dynamics::HierarchicalIK* self) -> bool {
            return self->solveAndApply();
          })
      .def(
          "solveAndApply",
          +[](dart::dynamics::HierarchicalIK* self, bool allowIncompleteResult)
              -> bool { return self->solveAndApply(allowIncompleteResult); },
          ::py::arg("allowIncompleteResult") = true)
      .def(
          "solveAndApply",
          +[](dart::dynamics::HierarchicalIK* self,
              Eigen::VectorXd& positions,
              bool allowIncompleteResult) -> bool {
            return self->solveAndApply(positions, allowIncompleteResult);
          },
          ::py::arg("positions"),
          ::py::arg("allowIncompleteResult") = true)
      .def(
          "findSolution",
          +[](dart::dynamics::HierarchicalIK* self, Eigen::VectorXd& positions)
              -> bool { return self->findSolution(positions); },
          ::py::arg("positions"))
      .def(
          "getSkeleton",
          +[](dart::dynamics::HierarchicalIK* self)
              -> dart::dynamics::SkeletonPtr { return self->getSkeleton(); })
      .def(
          "getSkeleton",
          +[](const dart::dynamics::HierarchicalIK* self)
              -> dart::dynamics::ConstSkeletonPtr {
            return self->getSkeleton();
          })
      .def(
          "getPositions",
          +[](const dart::dynamics::HierarchicalIK* self) -> Eigen::VectorXd {
            return self->getPositions();
          })
      .def(
          "setPositions",
          +[](dart::dynamics::HierarchicalIK* self, const Eigen::VectorXd& q) {
            self->setPositions(q);
          },
          ::py::arg("q"))
      .def(
          "refreshIKHierarchy",
          +[](dart::dynamics::HierarchicalIK* self) {
            self->refreshIKHierarchy();
          })
      .def(
          "clearCaches",
          +[](dart::dynamics::HierarchicalIK* self) { self->clearCaches(); });

  // WholeBodyIK class (inherits from HierarchicalIK)
  ::py::class_<
      dart::dynamics::WholeBodyIK,
      dart::dynamics::HierarchicalIK,
      std::shared_ptr<dart::dynamics::WholeBodyIK>>(m, "WholeBodyIK")
      .def_static(
          "create",
          +[](dart::dynamics::SkeletonPtr skel)
              -> std::shared_ptr<dart::dynamics::WholeBodyIK> {
            return dart::dynamics::WholeBodyIK::create(skel);
          },
          ::py::arg("skel"))
      .def(
          "clone",
          +[](const dart::dynamics::WholeBodyIK* self,
              dart::dynamics::SkeletonPtr newSkel)
              -> std::shared_ptr<dart::dynamics::HierarchicalIK> {
            return self->clone(newSkel);
          },
          ::py::arg("newSkel"))
      .def(
          "cloneWholeBodyIK",
          +[](const dart::dynamics::WholeBodyIK* self,
              dart::dynamics::SkeletonPtr newSkel)
              -> std::shared_ptr<dart::dynamics::WholeBodyIK> {
            return self->cloneWholeBodyIK(newSkel);
          },
          ::py::arg("newSkel"));
}

} // namespace python
} // namespace dart
