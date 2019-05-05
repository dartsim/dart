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

void Skeleton(pybind11::module& m)
{
  ::pybind11::class_<
      dart::dynamics::Skeleton,
      dart::dynamics::MetaSkeleton,
      std::shared_ptr<dart::dynamics::Skeleton>>(m, "Skeleton")
      .def(
          "getPtr",
          +[](dart::dynamics::Skeleton* self) -> dart::dynamics::SkeletonPtr {
            return self->getPtr();
          })
      .def(
          "getPtr",
          +[](const dart::dynamics::Skeleton* self)
              -> dart::dynamics::ConstSkeletonPtr { return self->getPtr(); })
      .def(
          "getSkeleton",
          +[](dart::dynamics::Skeleton* self) -> dart::dynamics::SkeletonPtr {
            return self->getSkeleton();
          })
      .def(
          "getSkeleton",
          +[](const dart::dynamics::Skeleton* self)
              -> dart::dynamics::ConstSkeletonPtr {
            return self->getSkeleton();
          })
      .def(
          "getLockableReference",
          +[](const dart::dynamics::Skeleton* self)
              -> std::unique_ptr<dart::common::LockableReference> {
            return self->getLockableReference();
          })
      .def(
          "clone",
          +[](const dart::dynamics::Skeleton* self)
              -> dart::dynamics::SkeletonPtr { return self->cloneSkeleton(); })
      .def(
          "clone",
          +[](const dart::dynamics::Skeleton* self,
              const std::string& cloneName) -> dart::dynamics::SkeletonPtr {
            return self->cloneSkeleton(cloneName);
          },
          ::pybind11::arg("cloneName"))
      .def(
          "setConfiguration",
          +[](dart::dynamics::Skeleton* self,
              const dart::dynamics::Skeleton::Configuration& configuration)
              -> void { return self->setConfiguration(configuration); },
          ::pybind11::arg("configuration"))
      .def(
          "getConfiguration",
          +[](const dart::dynamics::Skeleton* self)
              -> dart::dynamics::Skeleton::Configuration {
            return self->getConfiguration();
          })
      .def(
          "getConfiguration",
          +[](const dart::dynamics::Skeleton* self,
              int flags) -> dart::dynamics::Skeleton::Configuration {
            return self->getConfiguration(flags);
          },
          ::pybind11::arg("flags"))
      .def(
          "getConfiguration",
          +[](const dart::dynamics::Skeleton* self,
              const std::vector<std::size_t>& indices)
              -> dart::dynamics::Skeleton::Configuration {
            return self->getConfiguration(indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "getConfiguration",
          +[](const dart::dynamics::Skeleton* self,
              const std::vector<std::size_t>& indices,
              int flags) -> dart::dynamics::Skeleton::Configuration {
            return self->getConfiguration(indices, flags);
          },
          ::pybind11::arg("indices"),
          ::pybind11::arg("flags"))
      .def(
          "setState",
          +[](dart::dynamics::Skeleton* self,
              const dart::dynamics::Skeleton::State& state) -> void {
            return self->setState(state);
          },
          ::pybind11::arg("state"))
      .def(
          "getState",
          +[](const dart::dynamics::Skeleton* self)
              -> dart::dynamics::Skeleton::State { return self->getState(); })
      .def(
          "setProperties",
          +[](dart::dynamics::Skeleton* self,
              const dart::dynamics::Skeleton::Properties& properties) -> void {
            return self->setProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "getProperties",
          +[](const dart::dynamics::Skeleton* self)
              -> dart::dynamics::Skeleton::Properties {
            return self->getProperties();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::Skeleton* self,
              const dart::dynamics::Skeleton::AspectProperties& properties)
              -> void { return self->setProperties(properties); },
          ::pybind11::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::Skeleton* self,
              const dart::dynamics::Skeleton::AspectProperties& properties)
              -> void { return self->setAspectProperties(properties); },
          ::pybind11::arg("properties"))
      .def(
          "setName",
          +[](dart::dynamics::Skeleton* self, const std::string& _name)
              -> const std::string& { return self->setName(_name); },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("name"))
      .def(
          "getName",
          +[](const dart::dynamics::Skeleton* self) -> const std::string& {
            return self->getName();
          },
          ::pybind11::return_value_policy::reference_internal)
      //      .def("enableSelfCollision", +[](dart::dynamics::Skeleton *self) ->
      //      void { return self->enableSelfCollision(); })
      //      .def("enableSelfCollision", +[](dart::dynamics::Skeleton *self,
      //      bool enableAdjacentBodyCheck) -> void { return
      //      self->enableSelfCollision(enableAdjacentBodyCheck); },
      //      ::pybind11::arg("enableAdjacentBodyCheck"))
      //      .def("disableSelfCollision", +[](dart::dynamics::Skeleton *self)
      //      -> void { return self->disableSelfCollision(); })
      .def(
          "setSelfCollisionCheck",
          +[](dart::dynamics::Skeleton* self, bool enable) -> void {
            return self->setSelfCollisionCheck(enable);
          },
          ::pybind11::arg("enable"))
      .def(
          "getSelfCollisionCheck",
          +[](const dart::dynamics::Skeleton* self) -> bool {
            return self->getSelfCollisionCheck();
          })
      .def(
          "enableSelfCollisionCheck",
          +[](dart::dynamics::Skeleton* self) -> void {
            return self->enableSelfCollisionCheck();
          })
      .def(
          "disableSelfCollisionCheck",
          +[](dart::dynamics::Skeleton* self) -> void {
            return self->disableSelfCollisionCheck();
          })
      .def(
          "isEnabledSelfCollisionCheck",
          +[](const dart::dynamics::Skeleton* self) -> bool {
            return self->isEnabledSelfCollisionCheck();
          })
      .def(
          "setAdjacentBodyCheck",
          +[](dart::dynamics::Skeleton* self, bool enable) -> void {
            return self->setAdjacentBodyCheck(enable);
          },
          ::pybind11::arg("enable"))
      .def(
          "getAdjacentBodyCheck",
          +[](const dart::dynamics::Skeleton* self) -> bool {
            return self->getAdjacentBodyCheck();
          })
      .def(
          "enableAdjacentBodyCheck",
          +[](dart::dynamics::Skeleton* self) -> void {
            return self->enableAdjacentBodyCheck();
          })
      .def(
          "disableAdjacentBodyCheck",
          +[](dart::dynamics::Skeleton* self) -> void {
            return self->disableAdjacentBodyCheck();
          })
      .def(
          "isEnabledAdjacentBodyCheck",
          +[](const dart::dynamics::Skeleton* self) -> bool {
            return self->isEnabledAdjacentBodyCheck();
          })
      .def(
          "setMobile",
          +[](dart::dynamics::Skeleton* self, bool _isMobile) -> void {
            return self->setMobile(_isMobile);
          },
          ::pybind11::arg("isMobile"))
      .def(
          "isMobile",
          +[](const dart::dynamics::Skeleton* self) -> bool {
            return self->isMobile();
          })
      .def(
          "setTimeStep",
          +[](dart::dynamics::Skeleton* self, double _timeStep) -> void {
            return self->setTimeStep(_timeStep);
          },
          ::pybind11::arg("timeStep"))
      .def(
          "getTimeStep",
          +[](const dart::dynamics::Skeleton* self) -> double {
            return self->getTimeStep();
          })
      .def(
          "setGravity",
          +[](dart::dynamics::Skeleton* self, const Eigen::Vector3d& _gravity)
              -> void { return self->setGravity(_gravity); },
          ::pybind11::arg("gravity"))
      .def(
          "getGravity",
          +[](const dart::dynamics::Skeleton* self) -> const Eigen::Vector3d& {
            return self->getGravity();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getNumBodyNodes",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumBodyNodes();
          })
      .def(
          "getNumRigidBodyNodes",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumRigidBodyNodes();
          })
      .def(
          "getNumSoftBodyNodes",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumSoftBodyNodes();
          })
      .def(
          "getNumTrees",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumTrees();
          })
      .def(
          "getRootBodyNode",
          +[](dart::dynamics::Skeleton* self) -> dart::dynamics::BodyNode* {
            return self->getRootBodyNode();
          },
          pybind11::return_value_policy::reference_internal)
      .def(
          "getRootBodyNode",
          +[](dart::dynamics::Skeleton* self,
              std::size_t index) -> dart::dynamics::BodyNode* {
            return self->getRootBodyNode(index);
          },
          ::pybind11::arg("treeIndex"),
          pybind11::return_value_policy::reference_internal)
      .def(
          "getRootJoint",
          +[](dart::dynamics::Skeleton* self) -> dart::dynamics::Joint* {
            return self->getRootJoint();
          },
          pybind11::return_value_policy::reference_internal)
      .def(
          "getRootJoint",
          +[](dart::dynamics::Skeleton* self, std::size_t index)
              -> dart::dynamics::Joint* { return self->getRootJoint(index); },
          ::pybind11::arg("treeIndex"),
          pybind11::return_value_policy::reference_internal)
      .def(
          "getBodyNodes",
          +[](dart::dynamics::Skeleton* self, const std::string& name)
              -> std::vector<dart::dynamics::BodyNode*> {
            return self->getBodyNodes(name);
          },
          ::pybind11::arg("name"))
      .def(
          "getBodyNodes",
          +[](const dart::dynamics::Skeleton* self, const std::string& name)
              -> std::vector<const dart::dynamics::BodyNode*> {
            return self->getBodyNodes(name);
          },
          ::pybind11::arg("name"))
      .def(
          "hasBodyNode",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::BodyNode* bodyNode) -> bool {
            return self->hasBodyNode(bodyNode);
          },
          ::pybind11::arg("bodyNode"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::BodyNode* _bn) -> std::size_t {
            return self->getIndexOf(_bn);
          },
          ::pybind11::arg("bn"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::BodyNode* _bn,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_bn, _warning);
          },
          ::pybind11::arg("bn"),
          ::pybind11::arg("warning"))
      .def(
          "getTreeBodyNodes",
          +[](const dart::dynamics::Skeleton* self, std::size_t _treeIdx)
              -> std::vector<const dart::dynamics::BodyNode*> {
            return self->getTreeBodyNodes(_treeIdx);
          },
          ::pybind11::arg("treeIdx"))
      .def(
          "getNumJoints",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumJoints();
          })
      .def(
          "getJoint",
          +[](dart::dynamics::Skeleton* self, std::size_t _idx)
              -> dart::dynamics::Joint* { return self->getJoint(_idx); },
          ::pybind11::arg("idx"))
      .def(
          "getJoint",
          +[](dart::dynamics::Skeleton* self, const std::string& name)
              -> dart::dynamics::Joint* { return self->getJoint(name); },
          ::pybind11::arg("name"))
      .def(
          "getJoints",
          +[](dart::dynamics::Skeleton* self)
              -> std::vector<dart::dynamics::Joint*> {
            return self->getJoints();
          })
      .def(
          "getJoints",
          +[](const dart::dynamics::Skeleton* self)
              -> std::vector<const dart::dynamics::Joint*> {
            return self->getJoints();
          })
      .def(
          "getJoints",
          +[](dart::dynamics::Skeleton* self,
              const std::string& name) -> std::vector<dart::dynamics::Joint*> {
            return self->getJoints(name);
          },
          ::pybind11::arg("name"))
      .def(
          "getJoints",
          +[](const dart::dynamics::Skeleton* self, const std::string& name)
              -> std::vector<const dart::dynamics::Joint*> {
            return self->getJoints(name);
          },
          ::pybind11::arg("name"))
      .def(
          "hasJoint",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Joint* joint) -> bool {
            return self->hasJoint(joint);
          },
          ::pybind11::arg("joint"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Joint* _joint) -> std::size_t {
            return self->getIndexOf(_joint);
          },
          ::pybind11::arg("joint"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Joint* _joint,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_joint, _warning);
          },
          ::pybind11::arg("joint"),
          ::pybind11::arg("warning"))
      .def(
          "getNumDofs",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumDofs();
          })
      .def(
          "getDofs",
          +[](const dart::dynamics::Skeleton* self)
              -> std::vector<const dart::dynamics::DegreeOfFreedom*> {
            return self->getDofs();
          })
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::DegreeOfFreedom* _dof) -> std::size_t {
            return self->getIndexOf(_dof);
          },
          ::pybind11::arg("dof"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::DegreeOfFreedom* _dof,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_dof, _warning);
          },
          ::pybind11::arg("dof"),
          ::pybind11::arg("warning"))
      .def(
          "checkIndexingConsistency",
          +[](const dart::dynamics::Skeleton* self) -> bool {
            return self->checkIndexingConsistency();
          })
      .def(
          "getIK",
          +[](const dart::dynamics::Skeleton* self)
              -> std::shared_ptr<const dart::dynamics::WholeBodyIK> {
            return self->getIK();
          })
      .def(
          "clearIK",
          +[](dart::dynamics::Skeleton* self)
              -> void { return self->clearIK(); })
      .def(
          "getNumMarkers",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumMarkers();
          })
      .def(
          "getNumMarkers",
          +[](const dart::dynamics::Skeleton* self, std::size_t treeIndex)
              -> std::size_t { return self->getNumMarkers(treeIndex); },
          ::pybind11::arg("treeIndex"))
      .def(
          "getNumShapeNodes",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumShapeNodes();
          })
      .def(
          "getNumShapeNodes",
          +[](const dart::dynamics::Skeleton* self, std::size_t treeIndex)
              -> std::size_t { return self->getNumShapeNodes(treeIndex); },
          ::pybind11::arg("treeIndex"))
      .def(
          "getNumEndEffectors",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumEndEffectors();
          })
      .def(
          "getNumEndEffectors",
          +[](const dart::dynamics::Skeleton* self, std::size_t treeIndex)
              -> std::size_t { return self->getNumEndEffectors(treeIndex); },
          ::pybind11::arg("treeIndex"))
      .def(
          "integratePositions",
          +[](dart::dynamics::Skeleton* self, double _dt) -> void {
            return self->integratePositions(_dt);
          },
          ::pybind11::arg("dt"))
      .def(
          "integrateVelocities",
          +[](dart::dynamics::Skeleton* self, double _dt) -> void {
            return self->integrateVelocities(_dt);
          },
          ::pybind11::arg("dt"))
      .def(
          "getPositionDifferences",
          +[](const dart::dynamics::Skeleton* self,
              const Eigen::VectorXd& _q2,
              const Eigen::VectorXd& _q1) -> Eigen::VectorXd {
            return self->getPositionDifferences(_q2, _q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def(
          "getVelocityDifferences",
          +[](const dart::dynamics::Skeleton* self,
              const Eigen::VectorXd& _dq2,
              const Eigen::VectorXd& _dq1) -> Eigen::VectorXd {
            return self->getVelocityDifferences(_dq2, _dq1);
          },
          ::pybind11::arg("dq2"),
          ::pybind11::arg("dq1"))
      .def(
          "getSupportVersion",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getSupportVersion();
          })
      .def(
          "getSupportVersion",
          +[](const dart::dynamics::Skeleton* self, std::size_t _treeIdx)
              -> std::size_t { return self->getSupportVersion(_treeIdx); },
          ::pybind11::arg("treeIdx"))
      .def(
          "computeForwardKinematics",
          +[](dart::dynamics::Skeleton* self) -> void {
            return self->computeForwardKinematics();
          })
      .def(
          "computeForwardKinematics",
          +[](dart::dynamics::Skeleton* self, bool _updateTransforms) -> void {
            return self->computeForwardKinematics(_updateTransforms);
          },
          ::pybind11::arg("updateTransforms"))
      .def(
          "computeForwardKinematics",
          +[](dart::dynamics::Skeleton* self,
              bool _updateTransforms,
              bool _updateVels) -> void {
            return self->computeForwardKinematics(
                _updateTransforms, _updateVels);
          },
          ::pybind11::arg("updateTransforms"),
          ::pybind11::arg("updateVels"))
      .def(
          "computeForwardKinematics",
          +[](dart::dynamics::Skeleton* self,
              bool _updateTransforms,
              bool _updateVels,
              bool _updateAccs) -> void {
            return self->computeForwardKinematics(
                _updateTransforms, _updateVels, _updateAccs);
          },
          ::pybind11::arg("updateTransforms"),
          ::pybind11::arg("updateVels"),
          ::pybind11::arg("updateAccs"))
      .def(
          "computeForwardDynamics",
          +[](dart::dynamics::Skeleton* self) -> void {
            return self->computeForwardDynamics();
          })
      .def(
          "computeInverseDynamics",
          +[](dart::dynamics::Skeleton* self) -> void {
            return self->computeInverseDynamics();
          })
      .def(
          "computeInverseDynamics",
          +[](dart::dynamics::Skeleton* self,
              bool _withExternalForces) -> void {
            return self->computeInverseDynamics(_withExternalForces);
          },
          ::pybind11::arg("withExternalForces"))
      .def(
          "computeInverseDynamics",
          +[](dart::dynamics::Skeleton* self,
              bool _withExternalForces,
              bool _withDampingForces) -> void {
            return self->computeInverseDynamics(
                _withExternalForces, _withDampingForces);
          },
          ::pybind11::arg("withExternalForces"),
          ::pybind11::arg("withDampingForces"))
      .def(
          "computeInverseDynamics",
          +[](dart::dynamics::Skeleton* self,
              bool _withExternalForces,
              bool _withDampingForces,
              bool _withSpringForces) -> void {
            return self->computeInverseDynamics(
                _withExternalForces, _withDampingForces, _withSpringForces);
          },
          ::pybind11::arg("withExternalForces"),
          ::pybind11::arg("withDampingForces"),
          ::pybind11::arg("withSpringForces"))
      .def(
          "clearConstraintImpulses",
          +[](dart::dynamics::Skeleton* self) -> void {
            return self->clearConstraintImpulses();
          })
      .def(
          "updateBiasImpulse",
          +[](dart::dynamics::Skeleton* self,
              dart::dynamics::BodyNode* _bodyNode) -> void {
            return self->updateBiasImpulse(_bodyNode);
          },
          ::pybind11::arg("bodyNode"))
      .def(
          "updateBiasImpulse",
          +[](dart::dynamics::Skeleton* self,
              dart::dynamics::BodyNode* _bodyNode,
              const Eigen::Vector6d& _imp) -> void {
            return self->updateBiasImpulse(_bodyNode, _imp);
          },
          ::pybind11::arg("bodyNode"),
          ::pybind11::arg("imp"))
      .def(
          "updateBiasImpulse",
          +[](dart::dynamics::Skeleton* self,
              dart::dynamics::BodyNode* _bodyNode1,
              const Eigen::Vector6d& _imp1,
              dart::dynamics::BodyNode* _bodyNode2,
              const Eigen::Vector6d& _imp2) -> void {
            return self->updateBiasImpulse(
                _bodyNode1, _imp1, _bodyNode2, _imp2);
          },
          ::pybind11::arg("bodyNode1"),
          ::pybind11::arg("imp1"),
          ::pybind11::arg("bodyNode2"),
          ::pybind11::arg("imp2"))
      .def(
          "updateBiasImpulse",
          +[](dart::dynamics::Skeleton* self,
              dart::dynamics::SoftBodyNode* _softBodyNode,
              dart::dynamics::PointMass* _pointMass,
              const Eigen::Vector3d& _imp) -> void {
            return self->updateBiasImpulse(_softBodyNode, _pointMass, _imp);
          },
          ::pybind11::arg("softBodyNode"),
          ::pybind11::arg("pointMass"),
          ::pybind11::arg("imp"))
      .def(
          "updateVelocityChange",
          +[](dart::dynamics::Skeleton* self)
              -> void { return self->updateVelocityChange(); })
      .def(
          "setImpulseApplied",
          +[](dart::dynamics::Skeleton* self, bool _val) -> void {
            return self->setImpulseApplied(_val);
          },
          ::pybind11::arg("val"))
      .def(
          "isImpulseApplied",
          +[](const dart::dynamics::Skeleton* self) -> bool {
            return self->isImpulseApplied();
          })
      .def(
          "computeImpulseForwardDynamics",
          +[](dart::dynamics::Skeleton* self) -> void {
            return self->computeImpulseForwardDynamics();
          })
      .def(
          "getJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian { return self->getJacobian(_node); },
          ::pybind11::arg("node"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobian(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::Skeleton* self,
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
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian { return self->getWorldJacobian(_node); },
          ::pybind11::arg("node"))
      .def(
          "getWorldJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getWorldJacobian(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::Skeleton* self,
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
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::Skeleton* self,
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
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::Skeleton* self,
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
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
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
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getAngularJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getMass",
          +[](const dart::dynamics::Skeleton* self)
              -> double { return self->getMass(); })
      .def(
          "clearExternalForces",
          +[](dart::dynamics::Skeleton* self)
              -> void { return self->clearExternalForces(); })
      .def(
          "clearInternalForces",
          +[](dart::dynamics::Skeleton* self)
              -> void { return self->clearInternalForces(); })
      //      .def("notifyArticulatedInertiaUpdate",
      //      +[](dart::dynamics::Skeleton *self, std::size_t _treeIdx) -> void
      //      { return self->notifyArticulatedInertiaUpdate(_treeIdx); },
      //      ::pybind11::arg("treeIdx"))
      .def(
          "dirtyArticulatedInertia",
          +[](dart::dynamics::Skeleton* self, std::size_t _treeIdx) -> void {
            return self->dirtyArticulatedInertia(_treeIdx);
          },
          ::pybind11::arg("treeIdx"))
      //      .def("notifySupportUpdate", +[](dart::dynamics::Skeleton *self,
      //      std::size_t _treeIdx) -> void { return
      //      self->notifySupportUpdate(_treeIdx); },
      //      ::pybind11::arg("treeIdx"))
      .def(
          "dirtySupportPolygon",
          +[](dart::dynamics::Skeleton* self, std::size_t _treeIdx) -> void {
            return self->dirtySupportPolygon(_treeIdx);
          },
          ::pybind11::arg("treeIdx"))
      .def(
          "computeKineticEnergy",
          +[](const dart::dynamics::Skeleton* self) -> double {
            return self->computeKineticEnergy();
          })
      .def(
          "computePotentialEnergy",
          +[](const dart::dynamics::Skeleton* self) -> double {
            return self->computePotentialEnergy();
          })
      //      .def("clearCollidingBodies", +[](dart::dynamics::Skeleton *self)
      //      -> void { return self->clearCollidingBodies(); })
      .def(
          "getCOM",
          +[](const dart::dynamics::Skeleton* self) -> Eigen::Vector3d {
            return self->getCOM();
          })
      .def(
          "getCOM",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _withRespectTo) -> Eigen::Vector3d {
            return self->getCOM(_withRespectTo);
          },
          ::pybind11::arg("withRespectTo"))
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::Skeleton* self) -> Eigen::Vector6d {
            return self->getCOMSpatialVelocity();
          })
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector6d {
            return self->getCOMSpatialVelocity(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getCOMSpatialVelocity(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::Skeleton* self) -> Eigen::Vector3d {
            return self->getCOMLinearVelocity();
          })
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getCOMLinearVelocity(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getCOMLinearVelocity(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::Skeleton* self) -> Eigen::Vector6d {
            return self->getCOMSpatialAcceleration();
          })
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector6d {
            return self->getCOMSpatialAcceleration(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::Skeleton* self,
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
          +[](const dart::dynamics::Skeleton* self) -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration();
          })
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::Skeleton* self,
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
          +[](const dart::dynamics::Skeleton* self) -> dart::math::Jacobian {
            return self->getCOMJacobian();
          })
      .def(
          "getCOMJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getCOMJacobian(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMLinearJacobian",
          +[](const dart::dynamics::Skeleton* self)
              -> dart::math::LinearJacobian {
            return self->getCOMLinearJacobian();
          })
      .def(
          "getCOMLinearJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getCOMLinearJacobian(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMJacobianSpatialDeriv",
          +[](const dart::dynamics::Skeleton* self) -> dart::math::Jacobian {
            return self->getCOMJacobianSpatialDeriv();
          })
      .def(
          "getCOMJacobianSpatialDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getCOMJacobianSpatialDeriv(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMLinearJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self)
              -> dart::math::LinearJacobian {
            return self->getCOMLinearJacobianDeriv();
          })
      .def(
          "getCOMLinearJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getCOMLinearJacobianDeriv(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "resetUnion",
          +[](dart::dynamics::Skeleton* self)
              -> void { return self->resetUnion(); })
      .def_static(
          "create",
          +[]() -> dart::dynamics::
                    SkeletonPtr { return dart::dynamics::Skeleton::create(); })
      .def_static(
          "create",
          +[](const std::string& _name) -> dart::dynamics::SkeletonPtr {
            return dart::dynamics::Skeleton::create(_name);
          },
          ::pybind11::arg("name"))
      .def_static(
          "create",
          +[](const dart::dynamics::Skeleton::AspectPropertiesData& properties)
              -> dart::dynamics::SkeletonPtr {
            return dart::dynamics::Skeleton::create(properties);
          },
          ::pybind11::arg("properties"))
      .def_readwrite(
          "mUnionRootSkeleton", &dart::dynamics::Skeleton::mUnionRootSkeleton)
      .def_readwrite("mUnionSize", &dart::dynamics::Skeleton::mUnionSize)
      .def_readwrite("mUnionIndex", &dart::dynamics::Skeleton::mUnionIndex);
}

} // namespace python
} // namespace dart
