// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <nanobind/stl/map.h>
#include <nanobind/stl/pair.h>
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
#include "pointers.hpp"

#include <dart/dynamics/BallJoint.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/EulerJoint.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Inertia.hpp>
#include <dart/dynamics/JacobianNode.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Node.hpp>
#include <dart/dynamics/PlanarJoint.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/ScrewJoint.hpp>
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/TemplatedJacobianNode.hpp>
#include <dart/dynamics/TranslationalJoint.hpp>
#include <dart/dynamics/TranslationalJoint2D.hpp>
#include <dart/dynamics/UniversalJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <dart/math/MathTypes.hpp>

#include <dart/common/Aspect.hpp>
#include <dart/common/Cloneable.hpp>
#include <dart/common/EmbeddedAspect.hpp>
#include <dart/common/Macros.hpp>
#include <dart/common/ProxyAspect.hpp>
#include <dart/common/RequiresAspect.hpp>

#include <Eigen/Core>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <typeindex>
#include <utility>
#include <vector>

#include <cstddef>

#define DARTPY_DEFINE_CREATE_CHILD_JOINT_AND_BODY_NODE_PAIR(joint_type)        \
  .def(                                                                        \
      "create" #joint_type "AndBodyNodePair",                                  \
      +[](dart::dynamics::BodyNode* self)                                      \
          -> std::                                                             \
              pair<dart::dynamics::joint_type*, dart::dynamics::BodyNode*> {   \
                return self->createChildJointAndBodyNodePair<                  \
                    dart::dynamics::joint_type,                                \
                    dart::dynamics::BodyNode>();                               \
              },                                                               \
      nb::rv_policy::reference_internal)                                       \
      .def(                                                                    \
          "create" #joint_type "AndBodyNodePair",                              \
          +[](dart::dynamics::BodyNode* self,                                  \
              const dart::dynamics::joint_type::Properties& jointProperties)   \
              -> std::pair<                                                    \
                  dart::dynamics::joint_type*,                                 \
                  dart::dynamics::BodyNode*> {                                 \
            return self->createChildJointAndBodyNodePair<                      \
                dart::dynamics::joint_type,                                    \
                dart::dynamics::BodyNode>(jointProperties);                    \
          },                                                                   \
          nb::rv_policy::reference_internal,                                   \
          nb::arg("jointProperties"))                                          \
      .def(                                                                    \
          "create" #joint_type "AndBodyNodePair",                              \
          +[](dart::dynamics::BodyNode* self,                                  \
              const dart::dynamics::joint_type::Properties& jointProperties,   \
              const dart::dynamics::BodyNode::Properties& bodyProperties)      \
              -> std::pair<                                                    \
                  dart::dynamics::joint_type*,                                 \
                  dart::dynamics::BodyNode*> {                                 \
            return self->createChildJointAndBodyNodePair<                      \
                dart::dynamics::joint_type,                                    \
                dart::dynamics::BodyNode>(jointProperties, bodyProperties);    \
          },                                                                   \
          nb::rv_policy::reference_internal,                                   \
          nb::arg("jointProperties"),                                          \
          nb::arg("bodyProperties"))

namespace dart {
namespace python {

void BodyNode(nb::module_& m)
{
  dartnb::dart_class<dart::dynamics::detail::BodyNodeAspectProperties>(
      m, "BodyNodeAspectProperties")
      .def(dartnb::init<>())
      .def(dartnb::init<const std::string&>(), nb::arg("name"))
      .def(
          dartnb::init<const std::string&, const dart::dynamics::Inertia&>(),
          nb::arg("name"),
          nb::arg("inertia"))
      .def(
          dartnb::
              init<const std::string&, const dart::dynamics::Inertia&, bool>(),
          nb::arg("name"),
          nb::arg("inertia"),
          nb::arg("isCollidable"))
      .def(
          dartnb::init<
              const std::string&,
              const dart::dynamics::Inertia&,
              bool,
              bool>(),
          nb::arg("name"),
          nb::arg("inertia"),
          nb::arg("isCollidable"),
          nb::arg("gravityMode"))
      .def_rw(
          "mName",
          &dart::dynamics::detail::BodyNodeAspectProperties::mName,
          dartnb::setterArgument(
              &dart::dynamics::detail::BodyNodeAspectProperties::mName))
      .def_rw(
          "mInertia",
          &dart::dynamics::detail::BodyNodeAspectProperties::mInertia,
          dartnb::setterArgument(
              &dart::dynamics::detail::BodyNodeAspectProperties::mInertia))
      .def_rw(
          "mIsCollidable",
          &dart::dynamics::detail::BodyNodeAspectProperties::mIsCollidable,
          dartnb::setterArgument(
              &dart::dynamics::detail::BodyNodeAspectProperties::mIsCollidable))
      .def_rw(
          "mGravityMode",
          &dart::dynamics::detail::BodyNodeAspectProperties::mGravityMode,
          dartnb::setterArgument(
              &dart::dynamics::detail::BodyNodeAspectProperties::mGravityMode));

  dartnb::dart_class<dart::dynamics::BodyNode::Properties>(
      m, "BodyNodeProperties")
      .def(dartnb::init<>())
      .def(
          dartnb::init<
              const dart::dynamics::detail::BodyNodeAspectProperties&>(),
          nb::arg("aspectProperties"));

  dartnb::dart_class<
      dart::dynamics::TemplatedJacobianNode<dart::dynamics::BodyNode>,
      dart::dynamics::JacobianNode>(m, "TemplatedJacobianBodyNode")
      .def(
          "getJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_inCoordinatesOf);
          },
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset) -> dart::math::Jacobian {
            return self->getJacobian(_offset);
          },
          nb::arg("offset"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_offset, _inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getWorldJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset) -> dart::math::Jacobian {
            return self->getWorldJacobian(_offset);
          },
          nb::arg("offset"))
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
          nb::arg("inCoordinatesOf").none())
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset) -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_offset);
          },
          nb::arg("offset"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_offset, _inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_inCoordinatesOf);
          },
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset) -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_offset);
          },
          nb::arg("offset"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_offset, _inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_inCoordinatesOf);
          },
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset) -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_offset);
          },
          nb::arg("offset"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_offset, _inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("inCoordinatesOf").none())
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset) -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_offset);
          },
          nb::arg("offset"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::TemplatedJacobianNode<
                  dart::dynamics::BodyNode>* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_offset, _inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("inCoordinatesOf").none());

  dartnb::dart_class<
      dart::dynamics::BodyNode,
      dart::dynamics::TemplatedJacobianNode<dart::dynamics::BodyNode>,
      dart::dynamics::Frame>(m, "BodyNode")
      .def(
          "setAllNodeStates",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode::AllNodeStates& states) {
            self->setAllNodeStates(states);
          },
          nb::arg("states"))
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
          nb::arg("properties"))
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
          nb::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode::AspectProperties& _properties) {
            self->setProperties(_properties);
          },
          nb::arg("properties"))
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
          nb::arg("state"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode::AspectProperties& properties) {
            self->setAspectProperties(properties);
          },
          nb::arg("properties"))
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
          nb::arg("otherBodyNode"))
      .def(
          "copy",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode* otherBodyNode) {
            self->copy(otherBodyNode);
          },
          nb::arg("otherBodyNode").none())
      .def(
          "duplicateNodes",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode* otherBodyNode) {
            self->duplicateNodes(otherBodyNode);
          },
          nb::arg("otherBodyNode").none())
      .def(
          "matchNodes",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::BodyNode* otherBodyNode) {
            self->matchNodes(otherBodyNode);
          },
          nb::arg("otherBodyNode").none())
      .def(
          "setName",
          +[](dart::dynamics::BodyNode* self, const std::string& _name)
              -> const std::string& { return self->setName(_name); },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def(
          "getName",
          +[](const dart::dynamics::BodyNode* self) -> const std::string& {
            return self->getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "setGravityMode",
          +[](dart::dynamics::BodyNode* self, bool _gravityMode) {
            self->setGravityMode(_gravityMode);
          },
          nb::arg("gravityMode"))
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
          nb::arg("isCollidable"))
      .def(
          "setMass",
          +[](dart::dynamics::BodyNode* self,
              double mass) { self->setMass(mass); },
          nb::arg("mass"))
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
          nb::arg("Ixx"),
          nb::arg("Iyy"),
          nb::arg("Izz"))
      .def(
          "setMomentOfInertia",
          +[](dart::dynamics::BodyNode* self,
              double _Ixx,
              double _Iyy,
              double _Izz,
              double _Ixy) {
            self->setMomentOfInertia(_Ixx, _Iyy, _Izz, _Ixy);
          },
          nb::arg("Ixx"),
          nb::arg("Iyy"),
          nb::arg("Izz"),
          nb::arg("Ixy"))
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
          nb::arg("Ixx"),
          nb::arg("Iyy"),
          nb::arg("Izz"),
          nb::arg("Ixy"),
          nb::arg("Ixz"))
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
          nb::arg("Ixx"),
          nb::arg("Iyy"),
          nb::arg("Izz"),
          nb::arg("Ixy"),
          nb::arg("Ixz"),
          nb::arg("Iyz"))
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
          nb::arg("Ixx"),
          nb::arg("Iyy"),
          nb::arg("Izz"),
          nb::arg("Ixy"),
          nb::arg("Ixz"),
          nb::arg("Iyz"))
      .def(
          "setInertia",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::Inertia& inertia) {
            self->setInertia(inertia);
          },
          nb::arg("inertia"))
      .def(
          "getInertia",
          +[](const dart::dynamics::BodyNode* self)
              -> const dart::dynamics::Inertia& { return self->getInertia(); },
          nb::rv_policy::reference_internal)
      .def(
          "setLocalCOM",
          +[](dart::dynamics::BodyNode* self, const Eigen::Vector3d& _com) {
            self->setLocalCOM(_com);
          },
          nb::arg("com"))
      .def(
          "getLocalCOM",
          +[](const dart::dynamics::BodyNode* self) -> const Eigen::Vector3d& {
            return self->getLocalCOM();
          },
          nb::rv_policy::reference_internal)
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
          nb::arg("withRespectTo").none())
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
          nb::arg("relativeTo").none())
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::BodyNode* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getCOMLinearVelocity(_relativeTo, _inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("relativeTo").none())
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::BodyNode* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration(
                _relativeTo, _inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("name"))
      .def(
          "moveTo",
          +[](dart::dynamics::BodyNode* self,
              dart::dynamics::BodyNode* _newParent) -> bool {
            return self->moveTo(_newParent);
          },
          nb::arg("newParent").none())
      .def(
          "moveTo",
          +[](dart::dynamics::BodyNode* self,
              const dart::dynamics::SkeletonPtr& _newSkeleton,
              dart::dynamics::BodyNode* _newParent) -> bool {
            return self->moveTo(_newSkeleton, _newParent);
          },
          nb::arg("newSkeleton").none(),
          nb::arg("newParent").none())
      .def(
          "split",
          +[](dart::dynamics::BodyNode* self,
              const std::string& _skeletonName) -> dart::dynamics::SkeletonPtr {
            return self->split(_skeletonName);
          },
          nb::arg("skeletonName"))
      .def(
          "copyTo",
          +[](dart::dynamics::BodyNode* self,
              dart::dynamics::BodyNode* _newParent) -> nb::object {
            return nb::cast(self->copyTo(_newParent), nb::rv_policy::reference);
          },
          nb::arg("newParent").none())
      .def(
          "copyTo",
          +[](dart::dynamics::BodyNode* self,
              dart::dynamics::BodyNode* _newParent,
              bool _recursive) -> nb::object {
            return nb::cast(
                self->copyTo(_newParent, _recursive), nb::rv_policy::reference);
          },
          nb::arg("newParent").none(),
          nb::arg("recursive"))
      .def(
          "copyTo",
          +[](const dart::dynamics::BodyNode* self,
              const dart::dynamics::SkeletonPtr& _newSkeleton,
              dart::dynamics::BodyNode* _newParent) -> nb::object {
            return nb::cast(
                self->copyTo(_newSkeleton, _newParent),
                nb::rv_policy::reference);
          },
          nb::arg("newSkeleton").none(),
          nb::arg("newParent").none())
      .def(
          "copyTo",
          +[](const dart::dynamics::BodyNode* self,
              const dart::dynamics::SkeletonPtr& _newSkeleton,
              dart::dynamics::BodyNode* _newParent,
              bool _recursive) -> nb::object {
            return nb::cast(
                self->copyTo(_newSkeleton, _newParent, _recursive),
                nb::rv_policy::reference);
          },
          nb::arg("newSkeleton").none(),
          nb::arg("newParent").none(),
          nb::arg("recursive"))
      .def(
          "copyAs",
          +[](const dart::dynamics::BodyNode* self,
              const std::string& _skeletonName) -> dart::dynamics::SkeletonPtr {
            return self->copyAs(_skeletonName);
          },
          nb::arg("skeletonName"))
      .def(
          "copyAs",
          +[](const dart::dynamics::BodyNode* self,
              const std::string& _skeletonName,
              bool _recursive) -> dart::dynamics::SkeletonPtr {
            return self->copyAs(_skeletonName, _recursive);
          },
          nb::arg("skeletonName"),
          nb::arg("recursive"))
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
          nb::rv_policy::reference_internal)
      .def(
          "getParentBodyNode",
          +[](dart::dynamics::BodyNode* self) -> dart::dynamics::BodyNode* {
            return self->getParentBodyNode();
          },
          nb::rv_policy::reference_internal)
      // clang-format off
      DARTPY_DEFINE_CREATE_CHILD_JOINT_AND_BODY_NODE_PAIR(WeldJoint)
      DARTPY_DEFINE_CREATE_CHILD_JOINT_AND_BODY_NODE_PAIR(RevoluteJoint)
      DARTPY_DEFINE_CREATE_CHILD_JOINT_AND_BODY_NODE_PAIR(PrismaticJoint)
      DARTPY_DEFINE_CREATE_CHILD_JOINT_AND_BODY_NODE_PAIR(ScrewJoint)
      DARTPY_DEFINE_CREATE_CHILD_JOINT_AND_BODY_NODE_PAIR(UniversalJoint)
      DARTPY_DEFINE_CREATE_CHILD_JOINT_AND_BODY_NODE_PAIR(TranslationalJoint2D)
      DARTPY_DEFINE_CREATE_CHILD_JOINT_AND_BODY_NODE_PAIR(PlanarJoint)
      DARTPY_DEFINE_CREATE_CHILD_JOINT_AND_BODY_NODE_PAIR(EulerJoint)
      DARTPY_DEFINE_CREATE_CHILD_JOINT_AND_BODY_NODE_PAIR(BallJoint)
      DARTPY_DEFINE_CREATE_CHILD_JOINT_AND_BODY_NODE_PAIR(TranslationalJoint)
      DARTPY_DEFINE_CREATE_CHILD_JOINT_AND_BODY_NODE_PAIR(FreeJoint)
      // clang-format on
      .def(
          "getNumChildBodyNodes",
          +[](const dart::dynamics::BodyNode* self) -> std::size_t {
            return self->getNumChildBodyNodes();
          })
      .def(
          "getChildBodyNode",
          +[](dart::dynamics::BodyNode* self,
              std::size_t index) -> dart::dynamics::BodyNode* {
            return self->getChildBodyNode(index);
          },
          nb::arg("index"),
          nb::rv_policy::reference_internal)
      .def(
          "getNumChildJoints",
          +[](const dart::dynamics::BodyNode* self) -> std::size_t {
            return self->getNumChildJoints();
          })
      .def(
          "getChildJoint",
          +[](dart::dynamics::BodyNode* self, std::size_t index)
              -> dart::dynamics::Joint* { return self->getChildJoint(index); },
          nb::arg("index"),
          nb::rv_policy::reference_internal)
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
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "createShapeNode",
          +[](dart::dynamics::BodyNode* self,
              dart::dynamics::ShapePtr shape) -> dart::dynamics::ShapeNode* {
            return self->createShapeNode(shape);
          },
          nb::rv_policy::reference_internal,
          nb::arg("shape").none())
      .def(
          "createShapeNode",
          +[](dart::dynamics::BodyNode* self,
              dart::dynamics::ShapePtr shape,
              const std::string& name) -> dart::dynamics::ShapeNode* {
            return self->createShapeNode(shape, name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("shape").none(),
          nb::arg("name"))
      .def(
          "getShapeNodes",
          +[](dart::dynamics::BodyNode* self)
              -> const std::vector<dart::dynamics::ShapeNode*> {
            DART_SUPPRESS_DEPRECATED_BEGIN
            return self->getShapeNodes();
            DART_SUPPRESS_DEPRECATED_END
          },
          nb::rv_policy::reference_internal)
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
          nb::arg("genCoordIndex"))
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
          nb::arg("arrayIndex"))
      .def(
          "getNumDependentDofs",
          +[](const dart::dynamics::BodyNode* self) -> std::size_t {
            return self->getNumDependentDofs();
          })
      .def(
          "getChainDofs",
          +[](dart::dynamics::BodyNode* self)
              -> std::vector<dart::dynamics::DegreeOfFreedom*> {
            std::vector<dart::dynamics::DegreeOfFreedom*> dofs;
            for (const auto* dof : self->getChainDofs())
              dofs.push_back(const_cast<dart::dynamics::DegreeOfFreedom*>(dof));
            return dofs;
          },
          nb::rv_policy::reference_internal)
      .def(
          "addExtForce",
          +[](dart::dynamics::BodyNode* self, const Eigen::Vector3d& _force) {
            self->addExtForce(_force);
          },
          nb::arg("force"))
      .def(
          "addExtForce",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _force,
              const Eigen::Vector3d& _offset) {
            self->addExtForce(_force, _offset);
          },
          nb::arg("force"),
          nb::arg("offset"))
      .def(
          "addExtForce",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _force,
              const Eigen::Vector3d& _offset,
              bool _isForceLocal) {
            self->addExtForce(_force, _offset, _isForceLocal);
          },
          nb::arg("force"),
          nb::arg("offset"),
          nb::arg("isForceLocal"))
      .def(
          "addExtForce",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _force,
              const Eigen::Vector3d& _offset,
              bool _isForceLocal,
              bool _isOffsetLocal) {
            self->addExtForce(_force, _offset, _isForceLocal, _isOffsetLocal);
          },
          nb::arg("force"),
          nb::arg("offset"),
          nb::arg("isForceLocal"),
          nb::arg("isOffsetLocal"))
      .def(
          "setExtForce",
          +[](dart::dynamics::BodyNode* self, const Eigen::Vector3d& _force) {
            self->setExtForce(_force);
          },
          nb::arg("force"))
      .def(
          "setExtForce",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _force,
              const Eigen::Vector3d& _offset) {
            self->setExtForce(_force, _offset);
          },
          nb::arg("force"),
          nb::arg("offset"))
      .def(
          "setExtForce",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _force,
              const Eigen::Vector3d& _offset,
              bool _isForceLocal) {
            self->setExtForce(_force, _offset, _isForceLocal);
          },
          nb::arg("force"),
          nb::arg("offset"),
          nb::arg("isForceLocal"))
      .def(
          "setExtForce",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _force,
              const Eigen::Vector3d& _offset,
              bool _isForceLocal,
              bool _isOffsetLocal) {
            self->setExtForce(_force, _offset, _isForceLocal, _isOffsetLocal);
          },
          nb::arg("force"),
          nb::arg("offset"),
          nb::arg("isForceLocal"),
          nb::arg("isOffsetLocal"))
      .def(
          "addExtTorque",
          +[](dart::dynamics::BodyNode* self, const Eigen::Vector3d& _torque) {
            self->addExtTorque(_torque);
          },
          nb::arg("torque"))
      .def(
          "addExtTorque",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _torque,
              bool _isLocal) { self->addExtTorque(_torque, _isLocal); },
          nb::arg("torque"),
          nb::arg("isLocal"))
      .def(
          "setExtTorque",
          +[](dart::dynamics::BodyNode* self, const Eigen::Vector3d& _torque) {
            self->setExtTorque(_torque);
          },
          nb::arg("torque"))
      .def(
          "setExtTorque",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _torque,
              bool _isLocal) { self->setExtTorque(_torque, _isLocal); },
          nb::arg("torque"),
          nb::arg("isLocal"))
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
          "getBodyForce",
          &dart::dynamics::BodyNode::getBodyForce,
          "Get spatial body force transmitted from the parent joint. \n\nThe "
          "spatial body force is transmitted to this BodyNode from the parent "
          "body through the connecting joint. It is expressed in this "
          "BodyNode's frame.")
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
          nb::arg("constImp"))
      .def(
          "addConstraintImpulse",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector6d& _constImp) {
            self->addConstraintImpulse(_constImp);
          },
          nb::arg("constImp"))
      .def(
          "addConstraintImpulse",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _constImp,
              const Eigen::Vector3d& _offset) {
            self->addConstraintImpulse(_constImp, _offset);
          },
          nb::arg("constImp"),
          nb::arg("offset"))
      .def(
          "addConstraintImpulse",
          +[](dart::dynamics::BodyNode* self,
              const Eigen::Vector3d& _constImp,
              const Eigen::Vector3d& _offset,
              bool _isImpulseLocal) {
            self->addConstraintImpulse(_constImp, _offset, _isImpulseLocal);
          },
          nb::arg("constImp"),
          nb::arg("offset"),
          nb::arg("isImpulseLocal"))
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
          nb::arg("constImp"),
          nb::arg("offset"),
          nb::arg("isImpulseLocal"),
          nb::arg("isOffsetLocal"))
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
          nb::arg("gravity"))
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
          nb::arg("gravity"))
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
          nb::arg("pivot"))
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
      .def(
          "dirtyCoriolisForces",
          +[](dart::dynamics::BodyNode* self) { self->dirtyCoriolisForces(); })
      .def(
          "setColor",
          nb::overload_cast<const Eigen::Vector3d&>(
              &dynamics::BodyNode::setColor),
          nb::arg("color"))
      .def(
          "setColor",
          nb::overload_cast<const Eigen::Vector4d&>(
              &dynamics::BodyNode::setColor),
          nb::arg("color"))
      .def("setAlpha", &dynamics::BodyNode::setAlpha, nb::arg("alpha"));
}

} // namespace python
} // namespace dart

#undef DARTPY_DEFINE_CREATE_CHILD_JOINT_AND_BODY_NODE_PAIR
