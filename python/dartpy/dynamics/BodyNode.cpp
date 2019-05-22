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
#include <pybind11/stl.h>
#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

PYBIND11_DECLARE_HOLDER_TYPE(T, dart::dynamics::TemplateBodyNodePtr<T>, true);

namespace dart {
namespace python {

void BodyNode(pybind11::module& m)
{
  ::pybind11::class_<dart::dynamics::detail::BodyNodeAspectProperties>(
      m, "BodyNodeAspectProperties")
      .def(::pybind11::init<>())
      .def(::pybind11::init<const std::string&>(), ::pybind11::arg("name"))
      .def(
          ::pybind11::
              init<const std::string&, const dart::dynamics::Inertia&>(),
          ::pybind11::arg("name"),
          ::pybind11::arg("inertia"))
      .def(
          ::pybind11::
              init<const std::string&, const dart::dynamics::Inertia&, bool>(),
          ::pybind11::arg("name"),
          ::pybind11::arg("inertia"),
          ::pybind11::arg("isCollidable"))
      .def(
          ::pybind11::init<
              const std::string&,
              const dart::dynamics::Inertia&,
              bool,
              double>(),
          ::pybind11::arg("name"),
          ::pybind11::arg("inertia"),
          ::pybind11::arg("isCollidable"),
          ::pybind11::arg("frictionCoeff"))
      .def(
          ::pybind11::init<
              const std::string&,
              const dart::dynamics::Inertia&,
              bool,
              double,
              double>(),
          ::pybind11::arg("name"),
          ::pybind11::arg("inertia"),
          ::pybind11::arg("isCollidable"),
          ::pybind11::arg("frictionCoeff"),
          ::pybind11::arg("restitutionCoeff"))
      .def(
          ::pybind11::init<
              const std::string&,
              const dart::dynamics::Inertia&,
              bool,
              double,
              double,
              bool>(),
          ::pybind11::arg("name"),
          ::pybind11::arg("inertia"),
          ::pybind11::arg("isCollidable"),
          ::pybind11::arg("frictionCoeff"),
          ::pybind11::arg("restitutionCoeff"),
          ::pybind11::arg("gravityMode"))
      .def_readwrite(
          "mName", &dart::dynamics::detail::BodyNodeAspectProperties::mName)
      .def_readwrite(
          "mInertia",
          &dart::dynamics::detail::BodyNodeAspectProperties::mInertia)
      .def_readwrite(
          "mIsCollidable",
          &dart::dynamics::detail::BodyNodeAspectProperties::mIsCollidable)
      .def_readwrite(
          "mFrictionCoeff",
          &dart::dynamics::detail::BodyNodeAspectProperties::mFrictionCoeff)
      .def_readwrite(
          "mRestitutionCoeff",
          &dart::dynamics::detail::BodyNodeAspectProperties::mRestitutionCoeff)
      .def_readwrite(
          "mGravityMode",
          &dart::dynamics::detail::BodyNodeAspectProperties::mGravityMode);

  ::pybind11::class_<dart::dynamics::BodyNode::Properties>(
      m, "BodyNodeProperties")
      .def(::pybind11::init<>())
      .def(
          ::pybind11::init<
              const dart::dynamics::detail::BodyNodeAspectProperties&>(),
          ::pybind11::arg("aspectProperties"));

  ::pybind11::class_<
      dart::dynamics::TemplatedJacobianNode<dart::dynamics::BodyNode>,
      dart::dynamics::JacobianNode,
      std::shared_ptr<
          dart::dynamics::TemplatedJacobianNode<dart::dynamics::BodyNode>>>(
      m, "TemplatedJacobianBodyNode")
      .def(
          "getJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset) -> dart::math::Jacobian {
            return self->getJacobian(_offset);
          },
          ::pybind11::arg("offset"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_offset, _inCoordinatesOf);
          },
          ::pybind11::arg("offset"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getWorldJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset) -> dart::math::Jacobian {
            return self->getWorldJacobian(_offset);
          },
          ::pybind11::arg("offset"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
               dart::dynamics::BodyNode>* self) -> dart::math::LinearJacobian {
            return self->getLinearJacobian();
          })
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset) -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_offset);
          },
          ::pybind11::arg("offset"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_offset, _inCoordinatesOf);
          },
          ::pybind11::arg("offset"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
               dart::dynamics::BodyNode>* self) -> dart::math::AngularJacobian {
            return self->getAngularJacobian();
          })
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset) -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_offset);
          },
          ::pybind11::arg("offset"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_offset, _inCoordinatesOf);
          },
          ::pybind11::arg("offset"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset) -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_offset);
          },
          ::pybind11::arg("offset"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_offset, _inCoordinatesOf);
          },
          ::pybind11::arg("offset"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
               dart::dynamics::BodyNode>* self) -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv();
          })
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset) -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_offset);
          },
          ::pybind11::arg("offset"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_offset, _inCoordinatesOf);
          },
          ::pybind11::arg("offset"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getAngularJacobianDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
               dart::dynamics::BodyNode>* self) -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv();
          })
      .def(
          "getAngularJacobianDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"));

  ::pybind11::class_<
      dart::dynamics::BodyNode,
      dart::dynamics::TemplatedJacobianNode<dart::dynamics::BodyNode>,
      dart::dynamics::Frame,
      dart::dynamics::BodyNodePtr>(m, "BodyNode")
      .def(
          "setAllNodeStates",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode::AllNodeStates& states) {
            self->setAllNodeStates(states);
          },
          ::pybind11::arg("states"))
      .def(
          "getAllNodeStates",
          +[](const dart::dynamics::BodyNode* self)
              -> dart::dynamics::BodyNode::AllNodeStates {
            return self->getAllNodeStates();
          })
      .def(
          "setAllNodeProperties",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode::AllNodeProperties& properties) {
            self->setAllNodeProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "getAllNodeProperties",
          +[](const dart::dynamics::BodyNode* self)
              -> dart::dynamics::BodyNode::AllNodeProperties {
            return self->getAllNodeProperties();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode::CompositeProperties&
                  _properties) { self->setProperties(_properties); },
          ::pybind11::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode::AspectProperties& _properties) {
            self->setProperties(_properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "setAspectState",
          +[](dart::dynamics::BodyNode* self,
              const dart::common::EmbedStateAndPropertiesOnTopOf<
                  dart::dynamics::BodyNode,
                  dart::dynamics::detail::BodyNodeState,
                  dart::dynamics::detail::BodyNodeAspectProperties,
                  dart::common::RequiresAspect<
                      dart::common::ProxyStateAndPropertiesAspect<
                          dart::dynamics::BodyNode,
                          dart::common::ProxyCloneable<
                              dart::common::Aspect::State,
                              dart::dynamics::BodyNode,
                              dart::common::CloneableMap<std::map<
                                  std::type_index,
                                  std::unique_ptr<
                                      dart::common::CloneableVector<
                                          std::unique_ptr<
                                              dart::dynamics::Node::State,
                                              std::default_delete<
                                                  dart::dynamics::Node::
                                                      State>>>,
                                      std::default_delete<
                                          dart::common::CloneableVector<
                                              std::unique_ptr<
                                                  dart::dynamics::Node::State,
                                                  std::default_delete<
                                                      dart::dynamics::Node::
                                                          State>>>>>,
                                  std::less<std::type_index>,
                                  std::allocator<std::pair<
                                      const std::type_index,
                                      std::unique_ptr<
                                          dart::common::CloneableVector<
                                              std::unique_ptr<
                                                  dart::dynamics::Node::State,
                                                  std::default_delete<
                                                      dart::dynamics::Node::
                                                          State>>>,
                                          std::default_delete<
                                              dart::common::CloneableVector<
                                                  std::unique_ptr<
                                                      dart::dynamics::Node::
                                                          State,
                                                      std::default_delete<
                                                          dart::dynamics::Node::
                                                              State>>>>>>>>>,
                              &dart::dynamics::detail::setAllNodeStates,
                              &dart::dynamics::detail::getAllNodeStates>,
                          dart::common::ProxyCloneable<
                              dart::common::Aspect::Properties,
                              dart::dynamics::BodyNode,
                              dart::common::CloneableMap<std::map<
                                  std::type_index,
                                  std::unique_ptr<
                                      dart::common::CloneableVector<
                                          std::unique_ptr<
                                              dart::dynamics::Node::Properties,
                                              std::default_delete<
                                                  dart::dynamics::Node::
                                                      Properties>>>,
                                      std::default_delete<
                                          dart::common::CloneableVector<
                                              std::unique_ptr<
                                                  dart::dynamics::Node::
                                                      Properties,
                                                  std::default_delete<
                                                      dart::dynamics::Node::
                                                          Properties>>>>>,
                                  std::less<std::type_index>,
                                  std::allocator<std::pair<
                                      const std::type_index,
                                      std::unique_ptr<
                                          dart::common::CloneableVector<
                                              std::unique_ptr<
                                                  dart::dynamics::Node::
                                                      Properties,
                                                  std::default_delete<
                                                      dart::dynamics::Node::
                                                          Properties>>>,
                                          std::default_delete<
                                              dart::common::CloneableVector<
                                                  std::unique_ptr<
                                                      dart::dynamics::Node::
                                                          Properties,
                                                      std::default_delete<
                                                          dart::dynamics::Node::
                                                              Properties>>>>>>>>>,
                              &dart::dynamics::detail::setAllNodeProperties,
                              &dart::dynamics::detail::
                                  getAllNodeProperties>>>>::AspectState&
                  state) { self->setAspectState(state); },
          ::pybind11::arg("state"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode::AspectProperties& properties) {
            self->setAspectProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "getBodyNodeProperties",
          +[](const dart::dynamics::BodyNode* self)
              -> dart::dynamics::BodyNode::Properties {
            return self->getBodyNodeProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode& otherBodyNode) {
            self->copy(otherBodyNode);
          },
          ::pybind11::arg("otherBodyNode"))
      .def(
          "copy",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode* otherBodyNode) {
            self->copy(otherBodyNode);
          },
          ::pybind11::arg("otherBodyNode"))
      .def(
          "duplicateNodes",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode* otherBodyNode) {
            self->duplicateNodes(otherBodyNode);
          },
          ::pybind11::arg("otherBodyNode"))
      .def(
          "matchNodes",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode* otherBodyNode) {
            self->matchNodes(otherBodyNode);
          },
          ::pybind11::arg("otherBodyNode"))
      .def(
          "setName",
          +[](dart::dynamics::BodyNode* self, const std::string& _name)
              -> const std::string& { return self->setName(_name); },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("name"))
      .def(
          "getName",
          +[](const dart::dynamics::BodyNode* self) -> const std::string& {
            return self->getName();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "setGravityMode",
          +[](dart::dynamics::BodyNode* self, bool _gravityMode) {
            self->setGravityMode(_gravityMode);
          },
          ::pybind11::arg("gravityMode"))
      .def(
          "getGravityMode",
          +[](const dart::dynamics::BodyNode* self)
              -> bool { return self->getGravityMode(); })
      .def(
          "isCollidable",
          +[](const dart::dynamics::BodyNode* self)
              -> bool { return self->isCollidable(); })
      .def(
          "setCollidable",
          +[](dart::dynamics::BodyNode* self, bool _isCollidable) {
            self->setCollidable(_isCollidable);
          },
          ::pybind11::arg("isCollidable"))
      .def(
          "setMass",
          +[](dart::dynamics::BodyNode* self,
              double mass) { self->setMass(mass); },
          ::pybind11::arg("mass"))
      .def(
          "getMass",
          +[](const dart::dynamics::BodyNode* self)
              -> double { return self->getMass(); })
      .def(
          "setMomentOfInertia",
          +[](dart::dynamics::BodyNode* self,
              double _Ixx,
              double _Iyy,
              double _Izz) { self->setMomentOfInertia(_Ixx, _Iyy, _Izz); },
          ::pybind11::arg("Ixx"),
          ::pybind11::arg("Iyy"),
          ::pybind11::arg("Izz"))
      .def(
          "setMomentOfInertia",
          +[](dart::dynamics::BodyNode* self,
              double _Ixx,
              double _Iyy,
              double _Izz,
              double _Ixy) {
            self->setMomentOfInertia(_Ixx, _Iyy, _Izz, _Ixy);
          },
          ::pybind11::arg("Ixx"),
          ::pybind11::arg("Iyy"),
          ::pybind11::arg("Izz"),
          ::pybind11::arg("Ixy"))
      .def(
          "setMomentOfInertia",
          +[](dart::dynamics::BodyNode* self,
              double _Ixx,
              double _Iyy,
              double _Izz,
              double _Ixy,
              double _Ixz) {
            self->setMomentOfInertia(_Ixx, _Iyy, _Izz, _Ixy, _Ixz);
          },
          ::pybind11::arg("Ixx"),
          ::pybind11::arg("Iyy"),
          ::pybind11::arg("Izz"),
          ::pybind11::arg("Ixy"),
          ::pybind11::arg("Ixz"))
      .def(
          "setMomentOfInertia",
          +[](dart::dynamics::BodyNode* self,
              double _Ixx,
              double _Iyy,
              double _Izz,
              double _Ixy,
              double _Ixz,
              double _Iyz) {
            self->setMomentOfInertia(_Ixx, _Iyy, _Izz, _Ixy, _Ixz, _Iyz);
          },
          ::pybind11::arg("Ixx"),
          ::pybind11::arg("Iyy"),
          ::pybind11::arg("Izz"),
          ::pybind11::arg("Ixy"),
          ::pybind11::arg("Ixz"),
          ::pybind11::arg("Iyz"))
      .def(
          "getMomentOfInertia",
          +[](const dart::dynamics::BodyNode* self,
              double& _Ixx,
              double& _Iyy,
              double& _Izz,
              double& _Ixy,
              double& _Ixz,
              double& _Iyz) {
            self->getMomentOfInertia(_Ixx, _Iyy, _Izz, _Ixy, _Ixz, _Iyz);
          },
          ::pybind11::arg("Ixx"),
          ::pybind11::arg("Iyy"),
          ::pybind11::arg("Izz"),
          ::pybind11::arg("Ixy"),
          ::pybind11::arg("Ixz"),
          ::pybind11::arg("Iyz"))
      .def(
          "setInertia",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::Inertia& inertia) {
            self->setInertia(inertia);
          },
          ::pybind11::arg("inertia"))
      .def(
          "setLocalCOM",
          +[](dart::dynamics::BodyNode* self, const Eigen::Vector3d& _com) {
            self->setLocalCOM(_com);
          },
          ::pybind11::arg("com"))
      .def(
          "getLocalCOM",
          +[](const dart::dynamics::BodyNode* self) -> const Eigen::Vector3d& {
            return self->getLocalCOM();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getCOM",
          +[](const dart::dynamics::BodyNode* self) -> Eigen::Vector3d {
            return self->getCOM();
          })
      .def(
          "getCOM",
          +[](const dart::dynamics::BodyNode* self,
              const dart::dynamics::Frame* _withRespectTo) -> Eigen::Vector3d {
            return self->getCOM(_withRespectTo);
          },
          ::pybind11::arg("withRespectTo"))
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::BodyNode* self) -> Eigen::Vector3d {
            return self->getCOMLinearVelocity();
          })
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::BodyNode* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getCOMLinearVelocity(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::BodyNode* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getCOMLinearVelocity(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::BodyNode* self) -> Eigen::Vector6d {
            return self->getCOMSpatialVelocity();
          })
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::BodyNode* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getCOMSpatialVelocity(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::BodyNode* self) -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration();
          })
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::BodyNode* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::BodyNode* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration(
                _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::BodyNode* self) -> Eigen::Vector6d {
            return self->getCOMSpatialAcceleration();
          })
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::BodyNode* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getCOMSpatialAcceleration(
                _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "setFrictionCoeff",
          +[](dart::dynamics::BodyNode* self, double _coeff) {
            self->setFrictionCoeff(_coeff);
          },
          ::pybind11::arg("coeff"))
      .def(
          "getFrictionCoeff",
          +[](const dart::dynamics::BodyNode* self) -> double {
            return self->getFrictionCoeff();
          })
      .def(
          "setRestitutionCoeff",
          +[](dart::dynamics::BodyNode* self, double _coeff) {
            self->setRestitutionCoeff(_coeff);
          },
          ::pybind11::arg("coeff"))
      .def(
          "getRestitutionCoeff",
          +[](const dart::dynamics::BodyNode* self) -> double {
            return self->getRestitutionCoeff();
          })
      .def(
          "getIndexInSkeleton",
          +[](const dart::dynamics::BodyNode* self) -> std::size_t {
            return self->getIndexInSkeleton();
          })
      .def(
          "getIndexInTree",
          +[](const dart::dynamics::BodyNode* self) -> std::size_t {
            return self->getIndexInTree();
          })
      .def(
          "getTreeIndex",
          +[](const dart::dynamics::BodyNode* self) -> std::size_t {
            return self->getTreeIndex();
          })
      .def(
          "remove",
          +[](dart::dynamics::BodyNode* self) -> dart::dynamics::SkeletonPtr {
            return self->remove();
          })
      .def(
          "remove",
          +[](dart::dynamics::BodyNode* self, const std::string& _name)
              -> dart::dynamics::SkeletonPtr { return self->remove(_name); },
          ::pybind11::arg("name"))
      .def(
          "moveTo",
          +[](dart::dynamics::BodyNode* self,
              dart::dynamics::BodyNode* _newParent) -> bool {
            return self->moveTo(_newParent);
          },
          ::pybind11::arg("newParent"))
      .def(
          "moveTo",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::SkeletonPtr& _newSkeleton,
              dart::dynamics::BodyNode* _newParent) -> bool {
            return self->moveTo(_newSkeleton, _newParent);
          },
          ::pybind11::arg("newSkeleton"),
          ::pybind11::arg("newParent"))
      .def(
          "split",
          +[](dart::dynamics::BodyNode* self,
              const std::string& _skeletonName) -> dart::dynamics::SkeletonPtr {
            return self->split(_skeletonName);
          },
          ::pybind11::arg("skeletonName"))
      .def(
          "copyTo",
          +[](dart::dynamics::BodyNode* self,
              dart::dynamics::BodyNode* _newParent)
              -> std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> {
            return self->copyTo(_newParent);
          },
          ::pybind11::arg("newParent"))
      .def(
          "copyTo",
          +[](dart::dynamics::BodyNode* self,
              dart::dynamics::BodyNode* _newParent,
              bool _recursive)
              -> std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> {
            return self->copyTo(_newParent, _recursive);
          },
          ::pybind11::arg("newParent"),
          ::pybind11::arg("recursive"))
      .def(
          "copyTo",
          +[](const dart::dynamics::BodyNode* self,
              const dart::dynamics::SkeletonPtr& _newSkeleton,
              dart::dynamics::BodyNode* _newParent)
              -> std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> {
            return self->copyTo(_newSkeleton, _newParent);
          },
          ::pybind11::arg("newSkeleton"),
          ::pybind11::arg("newParent"))
      .def(
          "copyTo",
          +[](const dart::dynamics::BodyNode* self,
              const dart::dynamics::SkeletonPtr& _newSkeleton,
              dart::dynamics::BodyNode* _newParent,
              bool _recursive)
              -> std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> {
            return self->copyTo(_newSkeleton, _newParent, _recursive);
          },
          ::pybind11::arg("newSkeleton"),
          ::pybind11::arg("newParent"),
          ::pybind11::arg("recursive"))
      .def(
          "copyAs",
          +[](const dart::dynamics::BodyNode* self,
              const std::string& _skeletonName) -> dart::dynamics::SkeletonPtr {
            return self->copyAs(_skeletonName);
          },
          ::pybind11::arg("skeletonName"))
      .def(
          "copyAs",
          +[](const dart::dynamics::BodyNode* self,
              const std::string& _skeletonName,
              bool _recursive) -> dart::dynamics::SkeletonPtr {
            return self->copyAs(_skeletonName, _recursive);
          },
          ::pybind11::arg("skeletonName"),
          ::pybind11::arg("recursive"))
      .def(
          "getSkeleton",
          +[](dart::dynamics::BodyNode* self) -> dart::dynamics::SkeletonPtr {
            return self->getSkeleton();
          })
      .def(
          "getSkeleton",
          +[](const dart::dynamics::BodyNode* self)
              -> dart::dynamics::ConstSkeletonPtr {
            return self->getSkeleton();
          })
      .def(
          "getParentJoint",
          +[](dart::dynamics::BodyNode* self) -> dart::dynamics::Joint* {
            return self->getParentJoint();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getParentBodyNode",
          +[](dart::dynamics::BodyNode* self) -> dart::dynamics::BodyNode* {
            return self->getParentBodyNode();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getNumChildBodyNodes",
          +[](const dart::dynamics::BodyNode* self) -> std::size_t {
            return self->getNumChildBodyNodes();
          })
      .def(
          "getNumChildJoints",
          +[](const dart::dynamics::BodyNode* self) -> std::size_t {
            return self->getNumChildJoints();
          })
      .def(
          "getNumShapeNodes",
          +[](const dart::dynamics::BodyNode* self) -> std::size_t {
            return self->getNumShapeNodes();
          })
      .def(
          "getShapeNode",
          +[](dart::dynamics::BodyNode* self,
              std::size_t index) -> dart::dynamics::ShapeNode* {
            return self->getShapeNode(index);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"))
      .def(
          "createShapeNode",
          +[](dart::dynamics::BodyNode* self,
              dart::dynamics::ShapePtr shape) -> dart::dynamics::ShapeNode* {
            return self->createShapeNode(shape);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("shape"))
      .def(
          "createShapeNode",
          +[](dart::dynamics::BodyNode* self,
              dart::dynamics::ShapePtr shape,
              const std::string& name) -> dart::dynamics::ShapeNode* {
            return self->createShapeNode(shape, name);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("shape"),
          ::pybind11::arg("name"))
      .def(
          "getShapeNodes",
          +[](dart::dynamics::BodyNode* self)
              -> const std::vector<dart::dynamics::ShapeNode*> {
            return self->getShapeNodes();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "removeAllShapeNodes",
          +[](dart::dynamics::BodyNode* self) { self->removeAllShapeNodes(); })
      .def(
          "getNumEndEffectors",
          +[](const dart::dynamics::BodyNode* self) -> std::size_t {
            return self->getNumEndEffectors();
          })
      .def(
          "getNumMarkers",
          +[](const dart::dynamics::BodyNode* self) -> std::size_t {
            return self->getNumMarkers();
          })
      .def(
          "dependsOn",
          +[](const dart::dynamics::BodyNode* self, std::size_t _genCoordIndex)
              -> bool { return self->dependsOn(_genCoordIndex); },
          ::pybind11::arg("genCoordIndex"))
      .def(
          "getNumDependentGenCoords",
          +[](const dart::dynamics::BodyNode* self) -> std::size_t {
            return self->getNumDependentGenCoords();
          })
      .def(
          "getDependentGenCoordIndex",
          +[](const dart::dynamics::BodyNode* self,
              std::size_t _arrayIndex) -> std::size_t {
            return self->getDependentGenCoordIndex(_arrayIndex);
          },
          ::pybind11::arg("arrayIndex"))
      .def(
          "getNumDependentDofs",
          +[](const dart::dynamics::BodyNode* self) -> std::size_t {
            return self->getNumDependentDofs();
          })
      .def(
          "getChainDofs",
          +[](const dart::dynamics::BodyNode* self)
              -> const std::vector<const dart::dynamics::DegreeOfFreedom*> {
            return self->getChainDofs();
          })
      .def(
          "addExtForce",
          +[](dart::dynamics::BodyNode* self, const Eigen::Vector3d& _force) {
            self->addExtForce(_force);
          },
          ::pybind11::arg("force"))
      .def(
          "addExtForce",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _force,
              const Eigen::Vector3d& _offset) {
            self->addExtForce(_force, _offset);
          },
          ::pybind11::arg("force"),
          ::pybind11::arg("offset"))
      .def(
          "addExtForce",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _force,
              const Eigen::Vector3d& _offset,
              bool _isForceLocal) {
            self->addExtForce(_force, _offset, _isForceLocal);
          },
          ::pybind11::arg("force"),
          ::pybind11::arg("offset"),
          ::pybind11::arg("isForceLocal"))
      .def(
          "addExtForce",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _force,
              const Eigen::Vector3d& _offset,
              bool _isForceLocal,
              bool _isOffsetLocal) {
            self->addExtForce(_force, _offset, _isForceLocal, _isOffsetLocal);
          },
          ::pybind11::arg("force"),
          ::pybind11::arg("offset"),
          ::pybind11::arg("isForceLocal"),
          ::pybind11::arg("isOffsetLocal"))
      .def(
          "setExtForce",
          +[](dart::dynamics::BodyNode* self, const Eigen::Vector3d& _force) {
            self->setExtForce(_force);
          },
          ::pybind11::arg("force"))
      .def(
          "setExtForce",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _force,
              const Eigen::Vector3d& _offset) {
            self->setExtForce(_force, _offset);
          },
          ::pybind11::arg("force"),
          ::pybind11::arg("offset"))
      .def(
          "setExtForce",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _force,
              const Eigen::Vector3d& _offset,
              bool _isForceLocal) {
            self->setExtForce(_force, _offset, _isForceLocal);
          },
          ::pybind11::arg("force"),
          ::pybind11::arg("offset"),
          ::pybind11::arg("isForceLocal"))
      .def(
          "setExtForce",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _force,
              const Eigen::Vector3d& _offset,
              bool _isForceLocal,
              bool _isOffsetLocal) {
            self->setExtForce(_force, _offset, _isForceLocal, _isOffsetLocal);
          },
          ::pybind11::arg("force"),
          ::pybind11::arg("offset"),
          ::pybind11::arg("isForceLocal"),
          ::pybind11::arg("isOffsetLocal"))
      .def(
          "addExtTorque",
          +[](dart::dynamics::BodyNode* self, const Eigen::Vector3d& _torque) {
            self->addExtTorque(_torque);
          },
          ::pybind11::arg("torque"))
      .def(
          "addExtTorque",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _torque,
              bool _isLocal) { self->addExtTorque(_torque, _isLocal); },
          ::pybind11::arg("torque"),
          ::pybind11::arg("isLocal"))
      .def(
          "setExtTorque",
          +[](dart::dynamics::BodyNode* self, const Eigen::Vector3d& _torque) {
            self->setExtTorque(_torque);
          },
          ::pybind11::arg("torque"))
      .def(
          "setExtTorque",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _torque,
              bool _isLocal) { self->setExtTorque(_torque, _isLocal); },
          ::pybind11::arg("torque"),
          ::pybind11::arg("isLocal"))
      .def(
          "clearExternalForces",
          +[](dart::dynamics::BodyNode* self) { self->clearExternalForces(); })
      .def(
          "clearInternalForces",
          +[](dart::dynamics::BodyNode* self) { self->clearInternalForces(); })
      .def(
          "getExternalForceGlobal",
          +[](const dart::dynamics::BodyNode* self) -> Eigen::Vector6d {
            return self->getExternalForceGlobal();
          })
      .def(
          "isReactive",
          +[](const dart::dynamics::BodyNode* self)
              -> bool { return self->isReactive(); })
      .def(
          "setConstraintImpulse",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector6d& _constImp) {
            self->setConstraintImpulse(_constImp);
          },
          ::pybind11::arg("constImp"))
      .def(
          "addConstraintImpulse",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector6d& _constImp) {
            self->addConstraintImpulse(_constImp);
          },
          ::pybind11::arg("constImp"))
      .def(
          "addConstraintImpulse",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _constImp,
              const Eigen::Vector3d& _offset) {
            self->addConstraintImpulse(_constImp, _offset);
          },
          ::pybind11::arg("constImp"),
          ::pybind11::arg("offset"))
      .def(
          "addConstraintImpulse",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _constImp,
              const Eigen::Vector3d& _offset,
              bool _isImpulseLocal) {
            self->addConstraintImpulse(_constImp, _offset, _isImpulseLocal);
          },
          ::pybind11::arg("constImp"),
          ::pybind11::arg("offset"),
          ::pybind11::arg("isImpulseLocal"))
      .def(
          "addConstraintImpulse",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _constImp,
              const Eigen::Vector3d& _offset,
              bool _isImpulseLocal,
              bool _isOffsetLocal) {
            self->addConstraintImpulse(
                _constImp, _offset, _isImpulseLocal, _isOffsetLocal);
          },
          ::pybind11::arg("constImp"),
          ::pybind11::arg("offset"),
          ::pybind11::arg("isImpulseLocal"),
          ::pybind11::arg("isOffsetLocal"))
      .def(
          "clearConstraintImpulse",
          +[](dart::dynamics::BodyNode*
                  self) { self->clearConstraintImpulse(); })
      .def(
          "computeLagrangian",
          +[](const dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& gravity) -> double {
            return self->computeLagrangian(gravity);
          },
          ::pybind11::arg("gravity"))
      .def(
          "computeKineticEnergy",
          +[](const dart::dynamics::BodyNode* self) -> double {
            return self->computeKineticEnergy();
          })
      .def(
          "computePotentialEnergy",
          +[](const dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& gravity) -> double {
            return self->computePotentialEnergy(gravity);
          },
          ::pybind11::arg("gravity"))
      .def(
          "getLinearMomentum",
          +[](const dart::dynamics::BodyNode* self) -> Eigen::Vector3d {
            return self->getLinearMomentum();
          })
      .def(
          "getAngularMomentum",
          +[](dart::dynamics::BodyNode* self) -> Eigen::Vector3d {
            return self->getAngularMomentum();
          })
      .def(
          "getAngularMomentum",
          +[](dart::dynamics::BodyNode* self, const Eigen::Vector3d& _pivot)
              -> Eigen::Vector3d { return self->getAngularMomentum(_pivot); },
          ::pybind11::arg("pivot"))
      .def(
          "dirtyTransform",
          +[](dart::dynamics::BodyNode* self) { self->dirtyTransform(); })
      .def(
          "dirtyVelocity",
          +[](dart::dynamics::BodyNode* self) { self->dirtyVelocity(); })
      .def(
          "dirtyAcceleration",
          +[](dart::dynamics::BodyNode* self) { self->dirtyAcceleration(); })
      .def(
          "dirtyArticulatedInertia",
          +[](dart::dynamics::BodyNode*
                  self) { self->dirtyArticulatedInertia(); })
      .def(
          "dirtyExternalForces",
          +[](dart::dynamics::BodyNode* self) { self->dirtyExternalForces(); })
      .def("dirtyCoriolisForces", +[](dart::dynamics::BodyNode* self) {
        self->dirtyCoriolisForces();
      });
}

} // namespace python
} // namespace dart
