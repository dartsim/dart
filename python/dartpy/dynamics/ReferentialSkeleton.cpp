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

void ReferentialSkeleton(pybind11::module& m)
{
  ::pybind11::class_<
      dart::dynamics::ReferentialSkeleton,
      dart::dynamics::MetaSkeleton,
      std::shared_ptr<dart::dynamics::ReferentialSkeleton>>(
      m, "ReferentialSkeleton")
      .def(
          "getLockableReference",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> std::unique_ptr<dart::common::LockableReference> {
            return self->getLockableReference();
          })
      .def(
          "setName",
          +[](dart::dynamics::ReferentialSkeleton* self,
              const std::string& _name) -> const std::string& {
            return self->setName(_name);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("name"))
      .def(
          "getName",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> const std::string& { return self->getName(); },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getNumSkeletons",
          +[](const dart::dynamics::ReferentialSkeleton* self) -> std::size_t {
            return self->getNumSkeletons();
          })
      .def(
          "hasSkeleton",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Skeleton* skel) -> bool {
            return self->hasSkeleton(skel);
          },
          ::pybind11::arg("skel"))
      .def(
          "getNumBodyNodes",
          +[](const dart::dynamics::ReferentialSkeleton* self) -> std::size_t {
            return self->getNumBodyNodes();
          })
      .def(
          "getBodyNodes",
          +[](dart::dynamics::ReferentialSkeleton* self,
              const std::string& name)
              -> std::vector<dart::dynamics::BodyNode*> {
            return self->getBodyNodes(name);
          },
          ::pybind11::arg("name"))
      .def(
          "getBodyNodes",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const std::string& name)
              -> std::vector<const dart::dynamics::BodyNode*> {
            return self->getBodyNodes(name);
          },
          ::pybind11::arg("name"))
      .def(
          "hasBodyNode",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::BodyNode* bodyNode) -> bool {
            return self->hasBodyNode(bodyNode);
          },
          ::pybind11::arg("bodyNode"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::BodyNode* _bn) -> std::size_t {
            return self->getIndexOf(_bn);
          },
          ::pybind11::arg("bn"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::BodyNode* _bn,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_bn, _warning);
          },
          ::pybind11::arg("bn"),
          ::pybind11::arg("warning"))
      .def(
          "getNumJoints",
          +[](const dart::dynamics::ReferentialSkeleton* self) -> std::size_t {
            return self->getNumJoints();
          })
      .def(
          "getJoints",
          +[](dart::dynamics::ReferentialSkeleton* self)
              -> std::vector<dart::dynamics::Joint*> {
            return self->getJoints();
          })
      .def(
          "getJoints",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> std::vector<const dart::dynamics::Joint*> {
            return self->getJoints();
          })
      .def(
          "getJoints",
          +[](dart::dynamics::ReferentialSkeleton* self,
              const std::string& name) -> std::vector<dart::dynamics::Joint*> {
            return self->getJoints(name);
          },
          ::pybind11::arg("name"))
      .def(
          "getJoints",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const std::string& name)
              -> std::vector<const dart::dynamics::Joint*> {
            return self->getJoints(name);
          },
          ::pybind11::arg("name"))
      .def(
          "hasJoint",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Joint* joint) -> bool {
            return self->hasJoint(joint);
          },
          ::pybind11::arg("joint"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Joint* _joint) -> std::size_t {
            return self->getIndexOf(_joint);
          },
          ::pybind11::arg("joint"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Joint* _joint,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_joint, _warning);
          },
          ::pybind11::arg("joint"),
          ::pybind11::arg("warning"))
      .def(
          "getNumDofs",
          +[](const dart::dynamics::ReferentialSkeleton* self) -> std::size_t {
            return self->getNumDofs();
          })
      .def(
          "getDofs",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> std::vector<const dart::dynamics::DegreeOfFreedom*> {
            return self->getDofs();
          })
      .def(
          "getIndexOf",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::DegreeOfFreedom* _dof) -> std::size_t {
            return self->getIndexOf(_dof);
          },
          ::pybind11::arg("dof"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::DegreeOfFreedom* _dof,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_dof, _warning);
          },
          ::pybind11::arg("dof"),
          ::pybind11::arg("warning"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian { return self->getJacobian(_node); },
          ::pybind11::arg("node"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobian(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_node, _localOffset, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getWorldJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian { return self->getWorldJacobian(_node); },
          ::pybind11::arg("node"))
      .def(
          "getWorldJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getWorldJacobian(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(
                _node, _localOffset, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(
                _node, _localOffset, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(
                _node, _localOffset, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(
                _node, _localOffset, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getAngularJacobianDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getAngularJacobianDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getMass",
          +[](const dart::dynamics::ReferentialSkeleton* self) -> double {
            return self->getMass();
          })
      .def(
          "clearExternalForces",
          +[](dart::dynamics::ReferentialSkeleton* self) {
            self->clearExternalForces();
          })
      .def(
          "clearInternalForces",
          +[](dart::dynamics::ReferentialSkeleton* self) {
            self->clearInternalForces();
          })
      .def(
          "computeKineticEnergy",
          +[](const dart::dynamics::ReferentialSkeleton* self) -> double {
            return self->computeKineticEnergy();
          })
      .def(
          "computePotentialEnergy",
          +[](const dart::dynamics::ReferentialSkeleton* self) -> double {
            return self->computePotentialEnergy();
          })
      //      .def(
      //          "clearCollidingBodies",
      //          +[](dart::dynamics::ReferentialSkeleton* self) {
      //            self->clearCollidingBodies();
      //          })
      .def(
          "getCOM",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> Eigen::Vector3d { return self->getCOM(); })
      .def(
          "getCOM",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _withRespectTo) -> Eigen::Vector3d {
            return self->getCOM(_withRespectTo);
          },
          ::pybind11::arg("withRespectTo"))
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> Eigen::Vector6d { return self->getCOMSpatialVelocity(); })
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector6d {
            return self->getCOMSpatialVelocity(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getCOMSpatialVelocity(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> Eigen::Vector3d { return self->getCOMLinearVelocity(); })
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getCOMLinearVelocity(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getCOMLinearVelocity(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> Eigen::Vector6d { return self->getCOMSpatialAcceleration(); })
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector6d {
            return self->getCOMSpatialAcceleration(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getCOMSpatialAcceleration(
                _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> Eigen::Vector3d { return self->getCOMLinearAcceleration(); })
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration(
                _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> dart::math::Jacobian { return self->getCOMJacobian(); })
      .def(
          "getCOMJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getCOMJacobian(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMLinearJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> dart::math::LinearJacobian {
            return self->getCOMLinearJacobian();
          })
      .def(
          "getCOMLinearJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getCOMLinearJacobian(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMJacobianSpatialDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> dart::math::Jacobian {
            return self->getCOMJacobianSpatialDeriv();
          })
      .def(
          "getCOMJacobianSpatialDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getCOMJacobianSpatialDeriv(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMLinearJacobianDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> dart::math::LinearJacobian {
            return self->getCOMLinearJacobianDeriv();
          })
      .def(
          "getCOMLinearJacobianDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getCOMLinearJacobianDeriv(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"));
}

} // namespace python
} // namespace dart
