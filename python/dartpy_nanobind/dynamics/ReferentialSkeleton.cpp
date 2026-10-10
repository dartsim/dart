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

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/JacobianNode.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/MetaSkeleton.hpp>
#include <dart/dynamics/ReferentialSkeleton.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/math/MathTypes.hpp>

#include <dart/common/LockableReference.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

#include <cstddef>

namespace dart {
namespace python {

void ReferentialSkeleton(nb::module_& m)
{
  dartnb::dart_class<
      dart::dynamics::ReferentialSkeleton,
      dart::dynamics::MetaSkeleton>(m, "ReferentialSkeleton")
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
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def(
          "getName",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> const std::string& { return self->getName(); },
          nb::rv_policy::reference_internal)
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
          nb::arg("skel").none())
      .def(
          "getNumBodyNodes",
          +[](const dart::dynamics::ReferentialSkeleton* self) -> std::size_t {
            return self->getNumBodyNodes();
          })
      .def(
          "getBodyNodes",
          +[](dart::dynamics::ReferentialSkeleton* self)
              -> const std::vector<dart::dynamics::BodyNode*>& {
            return self->getBodyNodes();
          })
      .def(
          "getBodyNodes",
          +[](dart::dynamics::ReferentialSkeleton* self,
              const std::string& name)
              -> std::vector<dart::dynamics::BodyNode*> {
            return self->getBodyNodes(name);
          },
          nb::arg("name"))
      .def(
          "getBodyNodes",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const std::string& name)
              -> std::vector<const dart::dynamics::BodyNode*> {
            return self->getBodyNodes(name);
          },
          nb::arg("name"))
      .def(
          "hasBodyNode",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::BodyNode* bodyNode) -> bool {
            return self->hasBodyNode(bodyNode);
          },
          nb::arg("bodyNode").none())
      .def(
          "getIndexOf",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::BodyNode* _bn) -> std::size_t {
            return self->getIndexOf(_bn);
          },
          nb::arg("bn").none())
      .def(
          "getIndexOf",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::BodyNode* _bn,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_bn, _warning);
          },
          nb::arg("bn").none(),
          nb::arg("warning"))
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
          },
          nb::rv_policy::reference_internal)
      .def(
          "getJoints",
          +[](const dart::dynamics::ReferentialSkeleton* self)
              -> std::vector<const dart::dynamics::Joint*> {
            return self->getJoints();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getJoints",
          +[](dart::dynamics::ReferentialSkeleton* self,
              const std::string& name) -> std::vector<dart::dynamics::Joint*> {
            return self->getJoints(name);
          },
          nb::arg("name"),
          nb::rv_policy::reference_internal)
      .def(
          "getJoints",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const std::string& name)
              -> std::vector<const dart::dynamics::Joint*> {
            return self->getJoints(name);
          },
          nb::arg("name"),
          nb::rv_policy::reference_internal)
      .def(
          "hasJoint",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Joint* joint) -> bool {
            return self->hasJoint(joint);
          },
          nb::arg("joint").none())
      .def(
          "getIndexOf",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Joint* _joint) -> std::size_t {
            return self->getIndexOf(_joint);
          },
          nb::arg("joint").none())
      .def(
          "getIndexOf",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Joint* _joint,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_joint, _warning);
          },
          nb::arg("joint").none(),
          nb::arg("warning"))
      .def(
          "getNumDofs",
          +[](const dart::dynamics::ReferentialSkeleton* self) -> std::size_t {
            return self->getNumDofs();
          })
      .def(
          "getDofs",
          +[](dart::dynamics::ReferentialSkeleton* self)
              -> std::vector<dart::dynamics::DegreeOfFreedom*> {
            return self->getDofs();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getIndexOf",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::DegreeOfFreedom* _dof) -> std::size_t {
            return self->getIndexOf(_dof);
          },
          nb::arg("dof").none())
      .def(
          "getIndexOf",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::DegreeOfFreedom* _dof,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_dof, _warning);
          },
          nb::arg("dof").none(),
          nb::arg("warning"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian { return self->getJacobian(_node); },
          nb::arg("node").none())
      .def(
          "getJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobian(_node, _localOffset);
          },
          nb::arg("node").none(),
          nb::arg("localOffset"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_node, _localOffset, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("localOffset"),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getWorldJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian { return self->getWorldJacobian(_node); },
          nb::arg("node").none())
      .def(
          "getWorldJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getWorldJacobian(_node, _localOffset);
          },
          nb::arg("node").none(),
          nb::arg("localOffset"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node);
          },
          nb::arg("node").none())
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _localOffset);
          },
          nb::arg("node").none(),
          nb::arg("localOffset"))
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
          nb::arg("node").none(),
          nb::arg("localOffset"),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_node);
          },
          nb::arg("node").none())
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node);
          },
          nb::arg("node").none())
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node, _localOffset);
          },
          nb::arg("node").none(),
          nb::arg("localOffset"))
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
          nb::arg("node").none(),
          nb::arg("localOffset"),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node);
          },
          nb::arg("node").none())
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node, _localOffset);
          },
          nb::arg("node").none(),
          nb::arg("localOffset"))
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
          nb::arg("node").none(),
          nb::arg("localOffset"),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node);
          },
          nb::arg("node").none())
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node, _localOffset);
          },
          nb::arg("node").none(),
          nb::arg("localOffset"))
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
          nb::arg("node").none(),
          nb::arg("localOffset"),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getAngularJacobianDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv(_node);
          },
          nb::arg("node").none())
      .def(
          "getAngularJacobianDeriv",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("withRespectTo").none())
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
          nb::arg("relativeTo").none())
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getCOMSpatialVelocity(_relativeTo, _inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("relativeTo").none())
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getCOMLinearVelocity(_relativeTo, _inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("relativeTo").none())
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getCOMSpatialAcceleration(
                _relativeTo, _inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("relativeTo").none())
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::ReferentialSkeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration(
                _relativeTo, _inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("inCoordinatesOf").none());
}

} // namespace python
} // namespace dart
