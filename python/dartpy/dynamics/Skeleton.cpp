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

namespace py = pybind11;

#define DARTPY_DEFINE_CREATE_JOINT_AND_BODY_NODE_PAIR(joint_type)              \
  .def(                                                                        \
      "create" #joint_type "AndBodyNodePair",                                  \
      +[](dart::dynamics::Skeleton* self)                                      \
          -> std::                                                             \
              pair<dart::dynamics::joint_type*, dart::dynamics::BodyNode*> {   \
                return self->createJointAndBodyNodePair<                       \
                    dart::dynamics::joint_type,                                \
                    dart::dynamics::BodyNode>();                               \
              },                                                               \
      ::py::return_value_policy::reference_internal)                           \
      .def(                                                                    \
          "create" #joint_type "AndBodyNodePair",                              \
          +[](dart::dynamics::Skeleton* self,                                  \
              dart::dynamics::BodyNode* parent)                                \
              -> std::pair<                                                    \
                  dart::dynamics::joint_type*,                                 \
                  dart::dynamics::BodyNode*> {                                 \
            return self->createJointAndBodyNodePair<                           \
                dart::dynamics::joint_type,                                    \
                dart::dynamics::BodyNode>(parent);                             \
          },                                                                   \
          ::py::return_value_policy::reference_internal,                       \
          ::py::arg("parent"))                                                 \
      .def(                                                                    \
          "create" #joint_type "AndBodyNodePair",                              \
          +[](dart::dynamics::Skeleton* self,                                  \
              dart::dynamics::BodyNode* parent,                                \
              const dart::dynamics::joint_type::Properties& jointProperties)   \
              -> std::pair<                                                    \
                  dart::dynamics::joint_type*,                                 \
                  dart::dynamics::BodyNode*> {                                 \
            return self->createJointAndBodyNodePair<                           \
                dart::dynamics::joint_type,                                    \
                dart::dynamics::BodyNode>(parent, jointProperties);            \
          },                                                                   \
          ::py::return_value_policy::reference_internal,                       \
          ::py::arg("parent"),                                                 \
          ::py::arg("jointProperties"))                                        \
      .def(                                                                    \
          "create" #joint_type "AndBodyNodePair",                              \
          +[](dart::dynamics::Skeleton* self,                                  \
              dart::dynamics::BodyNode* parent,                                \
              const dart::dynamics::joint_type::Properties& jointProperties,   \
              const dart::dynamics::BodyNode::Properties& bodyProperties)      \
              -> std::pair<                                                    \
                  dart::dynamics::joint_type*,                                 \
                  dart::dynamics::BodyNode*> {                                 \
            return self->createJointAndBodyNodePair<                           \
                dart::dynamics::joint_type,                                    \
                dart::dynamics::BodyNode>(                                     \
                parent, jointProperties, bodyProperties);                      \
          },                                                                   \
          ::py::return_value_policy::reference_internal,                       \
          ::py::arg("parent").none(true),                                      \
          ::py::arg("jointProperties"),                                        \
          ::py::arg("bodyProperties"))

namespace dart {
namespace python {

void Skeleton(py::module& m)
{
  ::py::class_<
      dart::dynamics::Skeleton,
      dart::dynamics::MetaSkeleton,
      std::shared_ptr<dart::dynamics::Skeleton>>(m, "Skeleton")
      .def(::py::init(+[]() -> dart::dynamics::SkeletonPtr {
        return dart::dynamics::Skeleton::create();
      }))
      .def(
          ::py::init(
              +[](const std::string& _name) -> dart::dynamics::SkeletonPtr {
                return dart::dynamics::Skeleton::create(_name);
              }),
          ::py::arg("name"))
      .def(
          ::py::init(
              +[](const dart::dynamics::Skeleton::AspectPropertiesData&
                      properties) -> dart::dynamics::SkeletonPtr {
                return dart::dynamics::Skeleton::create(properties);
              }),
          ::py::arg("properties"))
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
          ::py::arg("cloneName"))
      .def(
          "setConfiguration",
          +[](dart::dynamics::Skeleton* self,
              const dart::dynamics::Skeleton::Configuration& configuration)
              -> void { return self->setConfiguration(configuration); },
          ::py::arg("configuration"))
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
          ::py::arg("flags"))
      .def(
          "getConfiguration",
          +[](const dart::dynamics::Skeleton* self,
              const std::vector<std::size_t>& indices)
              -> dart::dynamics::Skeleton::Configuration {
            return self->getConfiguration(indices);
          },
          ::py::arg("indices"))
      .def(
          "getConfiguration",
          +[](const dart::dynamics::Skeleton* self,
              const std::vector<std::size_t>& indices,
              int flags) -> dart::dynamics::Skeleton::Configuration {
            return self->getConfiguration(indices, flags);
          },
          ::py::arg("indices"),
          ::py::arg("flags"))
      .def(
          "setState",
          +[](dart::dynamics::Skeleton* self,
              const dart::dynamics::Skeleton::State& state) -> void {
            return self->setState(state);
          },
          ::py::arg("state"))
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
          ::py::arg("properties"))
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
          ::py::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::Skeleton* self,
              const dart::dynamics::Skeleton::AspectProperties& properties)
              -> void { return self->setAspectProperties(properties); },
          ::py::arg("properties"))
      .def(
          "setName",
          +[](dart::dynamics::Skeleton* self, const std::string& _name)
              -> const std::string& { return self->setName(_name); },
          ::py::return_value_policy::reference_internal,
          ::py::arg("name"))
      .def(
          "getName",
          +[](const dart::dynamics::Skeleton* self) -> const std::string& {
            return self->getName();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "setSelfCollisionCheck",
          +[](dart::dynamics::Skeleton* self, bool enable) -> void {
            return self->setSelfCollisionCheck(enable);
          },
          ::py::arg("enable"))
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
          ::py::arg("enable"))
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
          ::py::arg("isMobile"))
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
          ::py::arg("timeStep"))
      .def(
          "getTimeStep",
          +[](const dart::dynamics::Skeleton* self) -> double {
            return self->getTimeStep();
          })
      .def(
          "setGravity",
          +[](dart::dynamics::Skeleton* self, const Eigen::Vector3d& _gravity)
              -> void { return self->setGravity(_gravity); },
          ::py::arg("gravity"))
      .def(
          "getGravity",
          +[](const dart::dynamics::Skeleton* self) -> const Eigen::Vector3d& {
            return self->getGravity();
          },
          ::py::return_value_policy::reference_internal)
      // clang-format off
      DARTPY_DEFINE_CREATE_JOINT_AND_BODY_NODE_PAIR(WeldJoint)
      DARTPY_DEFINE_CREATE_JOINT_AND_BODY_NODE_PAIR(RevoluteJoint)
      DARTPY_DEFINE_CREATE_JOINT_AND_BODY_NODE_PAIR(PrismaticJoint)
      DARTPY_DEFINE_CREATE_JOINT_AND_BODY_NODE_PAIR(ScrewJoint)
      DARTPY_DEFINE_CREATE_JOINT_AND_BODY_NODE_PAIR(UniversalJoint)
      DARTPY_DEFINE_CREATE_JOINT_AND_BODY_NODE_PAIR(TranslationalJoint2D)
      DARTPY_DEFINE_CREATE_JOINT_AND_BODY_NODE_PAIR(PlanarJoint)
      DARTPY_DEFINE_CREATE_JOINT_AND_BODY_NODE_PAIR(EulerJoint)
      DARTPY_DEFINE_CREATE_JOINT_AND_BODY_NODE_PAIR(BallJoint)
      DARTPY_DEFINE_CREATE_JOINT_AND_BODY_NODE_PAIR(TranslationalJoint)
      DARTPY_DEFINE_CREATE_JOINT_AND_BODY_NODE_PAIR(FreeJoint)
      // clang-format on
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
          py::return_value_policy::reference)
      .def(
          "getRootBodyNode",
          +[](dart::dynamics::Skeleton* self,
              std::size_t index) -> dart::dynamics::BodyNode* {
            return self->getRootBodyNode(index);
          },
          ::py::arg("treeIndex"),
          py::return_value_policy::reference)
      .def(
          "getRootJoint",
          +[](dart::dynamics::Skeleton* self) -> dart::dynamics::Joint* {
            return self->getRootJoint();
          },
          py::return_value_policy::reference_internal)
      .def(
          "getRootJoint",
          +[](dart::dynamics::Skeleton* self, std::size_t index)
              -> dart::dynamics::Joint* { return self->getRootJoint(index); },
          ::py::arg("treeIndex"),
          py::return_value_policy::reference_internal)
      .def(
          "getBodyNodes",
          +[](dart::dynamics::Skeleton* self)
              -> const std::vector<dart::dynamics::BodyNode*>& {
            return self->getBodyNodes();
          })
      .def(
          "getBodyNodes",
          +[](dart::dynamics::Skeleton* self, const std::string& name)
              -> std::vector<dart::dynamics::BodyNode*> {
            return self->getBodyNodes(name);
          },
          ::py::arg("name"))
      .def(
          "getBodyNodes",
          +[](const dart::dynamics::Skeleton* self, const std::string& name)
              -> std::vector<const dart::dynamics::BodyNode*> {
            return self->getBodyNodes(name);
          },
          ::py::arg("name"))
      .def(
          "hasBodyNode",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::BodyNode* bodyNode) -> bool {
            return self->hasBodyNode(bodyNode);
          },
          ::py::arg("bodyNode"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::BodyNode* _bn) -> std::size_t {
            return self->getIndexOf(_bn);
          },
          ::py::arg("bn"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::BodyNode* _bn,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_bn, _warning);
          },
          ::py::arg("bn"),
          ::py::arg("warning"))
      .def(
          "getTreeBodyNodes",
          +[](const dart::dynamics::Skeleton* self, std::size_t _treeIdx)
              -> std::vector<const dart::dynamics::BodyNode*> {
            return self->getTreeBodyNodes(_treeIdx);
          },
          ::py::arg("treeIdx"))
      .def(
          "getNumJoints",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumJoints();
          })
      .def(
          "getJoint",
          +[](dart::dynamics::Skeleton* self, std::size_t _idx)
              -> dart::dynamics::Joint* { return self->getJoint(_idx); },
          ::py::arg("idx"),
          py::return_value_policy::reference_internal)
      .def(
          "getJoint",
          +[](dart::dynamics::Skeleton* self, const std::string& name)
              -> dart::dynamics::Joint* { return self->getJoint(name); },
          ::py::arg("name"),
          py::return_value_policy::reference_internal)
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
          ::py::arg("name"))
      .def(
          "getJoints",
          +[](const dart::dynamics::Skeleton* self, const std::string& name)
              -> std::vector<const dart::dynamics::Joint*> {
            return self->getJoints(name);
          },
          ::py::arg("name"))
      .def(
          "hasJoint",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Joint* joint) -> bool {
            return self->hasJoint(joint);
          },
          ::py::arg("joint"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Joint* _joint) -> std::size_t {
            return self->getIndexOf(_joint);
          },
          ::py::arg("joint"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Joint* _joint,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_joint, _warning);
          },
          ::py::arg("joint"),
          ::py::arg("warning"))
      .def(
          "getNumDofs",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumDofs();
          })
      .def(
          "getDof",
          +[](dart::dynamics::Skeleton* self,
              std::size_t index) -> dart::dynamics::DegreeOfFreedom* {
            return self->getDof(index);
          },
          ::py::return_value_policy::reference_internal,
          ::py::arg("index"))
      .def(
          "getDof",
          +[](dart::dynamics::Skeleton* self,
              const std::string& name) -> dart::dynamics::DegreeOfFreedom* {
            return self->getDof(name);
          },
          ::py::return_value_policy::reference_internal,
          ::py::arg("name"))
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
          ::py::arg("dof"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::DegreeOfFreedom* _dof,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_dof, _warning);
          },
          ::py::arg("dof"),
          ::py::arg("warning"))
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
          ::py::arg("treeIndex"))
      .def(
          "getNumShapeNodes",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumShapeNodes();
          })
      .def(
          "getNumShapeNodes",
          +[](const dart::dynamics::Skeleton* self, std::size_t treeIndex)
              -> std::size_t { return self->getNumShapeNodes(treeIndex); },
          ::py::arg("treeIndex"))
      .def(
          "getShapeNode",
          +[](dart::dynamics::Skeleton* self,
              std::size_t index) -> dart::dynamics::ShapeNode* {
            return self->getShapeNode(index);
          },
          ::py::return_value_policy::reference_internal,
          ::py::arg("index"))
      .def(
          "getShapeNode",
          +[](dart::dynamics::Skeleton* self,
              const std::string& name) -> dart::dynamics::ShapeNode* {
            return self->getShapeNode(name);
          },
          ::py::return_value_policy::reference_internal,
          ::py::arg("name"))
      .def(
          "getNumEndEffectors",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumEndEffectors();
          })
      .def(
          "getNumEndEffectors",
          +[](const dart::dynamics::Skeleton* self, std::size_t treeIndex)
              -> std::size_t { return self->getNumEndEffectors(treeIndex); },
          ::py::arg("treeIndex"))
      .def(
          "integratePositions",
          +[](dart::dynamics::Skeleton* self, double _dt) -> void {
            return self->integratePositions(_dt);
          },
          ::py::arg("dt"))
      .def(
          "integrateVelocities",
          +[](dart::dynamics::Skeleton* self, double _dt) -> void {
            return self->integrateVelocities(_dt);
          },
          ::py::arg("dt"))
      .def(
          "getPositionDifferences",
          +[](const dart::dynamics::Skeleton* self,
              const Eigen::VectorXd& _q2,
              const Eigen::VectorXd& _q1) -> Eigen::VectorXd {
            return self->getPositionDifferences(_q2, _q1);
          },
          ::py::arg("q2"),
          ::py::arg("q1"))
      .def(
          "getVelocityDifferences",
          +[](const dart::dynamics::Skeleton* self,
              const Eigen::VectorXd& _dq2,
              const Eigen::VectorXd& _dq1) -> Eigen::VectorXd {
            return self->getVelocityDifferences(_dq2, _dq1);
          },
          ::py::arg("dq2"),
          ::py::arg("dq1"))
      .def(
          "getSupportVersion",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getSupportVersion();
          })
      .def(
          "getSupportVersion",
          +[](const dart::dynamics::Skeleton* self, std::size_t _treeIdx)
              -> std::size_t { return self->getSupportVersion(_treeIdx); },
          ::py::arg("treeIdx"))
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
          ::py::arg("updateTransforms"))
      .def(
          "computeForwardKinematics",
          +[](dart::dynamics::Skeleton* self,
              bool _updateTransforms,
              bool _updateVels) -> void {
            return self->computeForwardKinematics(
                _updateTransforms, _updateVels);
          },
          ::py::arg("updateTransforms"),
          ::py::arg("updateVels"))
      .def(
          "computeForwardKinematics",
          +[](dart::dynamics::Skeleton* self,
              bool _updateTransforms,
              bool _updateVels,
              bool _updateAccs) -> void {
            return self->computeForwardKinematics(
                _updateTransforms, _updateVels, _updateAccs);
          },
          ::py::arg("updateTransforms"),
          ::py::arg("updateVels"),
          ::py::arg("updateAccs"))
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
          ::py::arg("withExternalForces"))
      .def(
          "computeInverseDynamics",
          +[](dart::dynamics::Skeleton* self,
              bool _withExternalForces,
              bool _withDampingForces) -> void {
            return self->computeInverseDynamics(
                _withExternalForces, _withDampingForces);
          },
          ::py::arg("withExternalForces"),
          ::py::arg("withDampingForces"))
      .def(
          "computeInverseDynamics",
          +[](dart::dynamics::Skeleton* self,
              bool _withExternalForces,
              bool _withDampingForces,
              bool _withSpringForces) -> void {
            return self->computeInverseDynamics(
                _withExternalForces, _withDampingForces, _withSpringForces);
          },
          ::py::arg("withExternalForces"),
          ::py::arg("withDampingForces"),
          ::py::arg("withSpringForces"))
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
          ::py::arg("bodyNode"))
      .def(
          "updateBiasImpulse",
          +[](dart::dynamics::Skeleton* self,
              dart::dynamics::BodyNode* _bodyNode,
              const Eigen::Vector6d& _imp) -> void {
            return self->updateBiasImpulse(_bodyNode, _imp);
          },
          ::py::arg("bodyNode"),
          ::py::arg("imp"))
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
          ::py::arg("bodyNode1"),
          ::py::arg("imp1"),
          ::py::arg("bodyNode2"),
          ::py::arg("imp2"))
      .def(
          "updateBiasImpulse",
          +[](dart::dynamics::Skeleton* self,
              dart::dynamics::SoftBodyNode* _softBodyNode,
              dart::dynamics::PointMass* _pointMass,
              const Eigen::Vector3d& _imp) -> void {
            return self->updateBiasImpulse(_softBodyNode, _pointMass, _imp);
          },
          ::py::arg("softBodyNode"),
          ::py::arg("pointMass"),
          ::py::arg("imp"))
      .def(
          "updateVelocityChange",
          +[](dart::dynamics::Skeleton* self)
              -> void { return self->updateVelocityChange(); })
      .def(
          "setImpulseApplied",
          +[](dart::dynamics::Skeleton* self, bool _val) -> void {
            return self->setImpulseApplied(_val);
          },
          ::py::arg("val"))
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
          ::py::arg("node"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_node, _inCoordinatesOf);
          },
          ::py::arg("node"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobian(_node, _localOffset);
          },
          ::py::arg("node"),
          ::py::arg("localOffset"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_node, _localOffset, _inCoordinatesOf);
          },
          ::py::arg("node"),
          ::py::arg("localOffset"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getWorldJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian { return self->getWorldJacobian(_node); },
          ::py::arg("node"))
      .def(
          "getWorldJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getWorldJacobian(_node, _localOffset);
          },
          ::py::arg("node"),
          ::py::arg("localOffset"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node);
          },
          ::py::arg("node"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _inCoordinatesOf);
          },
          ::py::arg("node"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _localOffset);
          },
          ::py::arg("node"),
          ::py::arg("localOffset"))
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
          ::py::arg("node"),
          ::py::arg("localOffset"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_node);
          },
          ::py::arg("node"))
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_node, _inCoordinatesOf);
          },
          ::py::arg("node"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node);
          },
          ::py::arg("node"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node, _inCoordinatesOf);
          },
          ::py::arg("node"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node, _localOffset);
          },
          ::py::arg("node"),
          ::py::arg("localOffset"))
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
          ::py::arg("node"),
          ::py::arg("localOffset"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node);
          },
          ::py::arg("node"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node, _inCoordinatesOf);
          },
          ::py::arg("node"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node, _localOffset);
          },
          ::py::arg("node"),
          ::py::arg("localOffset"))
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
          ::py::arg("node"),
          ::py::arg("localOffset"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node);
          },
          ::py::arg("node"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node, _inCoordinatesOf);
          },
          ::py::arg("node"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node, _localOffset);
          },
          ::py::arg("node"),
          ::py::arg("localOffset"))
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
          ::py::arg("node"),
          ::py::arg("localOffset"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getAngularJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv(_node);
          },
          ::py::arg("node"))
      .def(
          "getAngularJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv(_node, _inCoordinatesOf);
          },
          ::py::arg("node"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getMass",
          +[](const dart::dynamics::Skeleton* self)
              -> double { return self->getMass(); })
      .def(
          "getMassMatrix",
          +[](const dart::dynamics::Skeleton* self,
              std::size_t treeIndex) -> const Eigen::MatrixXd& {
            return self->getMassMatrix(treeIndex);
          })
      // TODO(JS): Redefining get[~]() that are already defined in MetaSkeleton.
      // We need this because the methods with same name (but different
      // arguments) are hidden. Update (or remove) once following issue is
      // resolved: https://github.com/pybind/pybind11/issues/974
      .def(
          "getMassMatrix",
          +[](const dart::dynamics::Skeleton* self) -> const Eigen::MatrixXd& {
            return self->getMassMatrix();
          })
      .def(
          "getAugMassMatrix",
          +[](const dart::dynamics::Skeleton* self,
              std::size_t treeIndex) -> const Eigen::MatrixXd& {
            return self->getAugMassMatrix(treeIndex);
          })
      .def(
          "getAugMassMatrix",
          +[](const dart::dynamics::Skeleton* self) -> const Eigen::MatrixXd& {
            return self->getAugMassMatrix();
          })
      .def(
          "getInvMassMatrix",
          +[](const dart::dynamics::Skeleton* self,
              std::size_t treeIndex) -> const Eigen::MatrixXd& {
            return self->getInvMassMatrix(treeIndex);
          })
      .def(
          "getInvMassMatrix",
          +[](const dart::dynamics::Skeleton* self) -> const Eigen::MatrixXd& {
            return self->getInvMassMatrix();
          })
      .def(
          "getCoriolisForces",
          +[](dart::dynamics::Skeleton* self,
              std::size_t treeIndex) -> const Eigen::VectorXd& {
            return self->getCoriolisForces(treeIndex);
          })
      .def(
          "getCoriolisForces",
          +[](dart::dynamics::Skeleton* self) -> const Eigen::VectorXd& {
            return self->getCoriolisForces();
          })
      .def(
          "getGravityForces",
          +[](dart::dynamics::Skeleton* self,
              std::size_t treeIndex) -> const Eigen::VectorXd& {
            return self->getGravityForces(treeIndex);
          })
      .def(
          "getGravityForces",
          +[](dart::dynamics::Skeleton* self) -> const Eigen::VectorXd& {
            return self->getGravityForces();
          })
      .def(
          "getCoriolisAndGravityForces",
          +[](dart::dynamics::Skeleton* self,
              std::size_t treeIndex) -> const Eigen::VectorXd& {
            return self->getCoriolisAndGravityForces(treeIndex);
          })
      .def(
          "getCoriolisAndGravityForces",
          +[](dart::dynamics::Skeleton* self) -> const Eigen::VectorXd& {
            return self->getCoriolisAndGravityForces();
          })
      .def(
          "getExternalForces",
          +[](dart::dynamics::Skeleton* self,
              std::size_t treeIndex) -> const Eigen::VectorXd& {
            return self->getExternalForces(treeIndex);
          })
      .def(
          "getExternalForces",
          +[](dart::dynamics::Skeleton* self) -> const Eigen::VectorXd& {
            return self->getExternalForces();
          })
      .def(
          "getConstraintForces",
          +[](dart::dynamics::Skeleton* self,
              std::size_t treeIndex) -> const Eigen::VectorXd& {
            return self->getConstraintForces(treeIndex);
          })
      .def(
          "getConstraintForces",
          +[](dart::dynamics::Skeleton* self) -> const Eigen::VectorXd& {
            return self->getConstraintForces();
          })
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
      //      ::py::arg("treeIdx"))
      .def(
          "dirtyArticulatedInertia",
          +[](dart::dynamics::Skeleton* self, std::size_t _treeIdx) -> void {
            return self->dirtyArticulatedInertia(_treeIdx);
          },
          ::py::arg("treeIdx"))
      //      .def("notifySupportUpdate", +[](dart::dynamics::Skeleton *self,
      //      std::size_t _treeIdx) -> void { return
      //      self->notifySupportUpdate(_treeIdx); },
      //      ::py::arg("treeIdx"))
      .def(
          "dirtySupportPolygon",
          +[](dart::dynamics::Skeleton* self, std::size_t _treeIdx) -> void {
            return self->dirtySupportPolygon(_treeIdx);
          },
          ::py::arg("treeIdx"))
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
          ::py::arg("withRespectTo"))
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
          ::py::arg("relativeTo"))
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getCOMSpatialVelocity(_relativeTo, _inCoordinatesOf);
          },
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("relativeTo"))
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getCOMLinearVelocity(_relativeTo, _inCoordinatesOf);
          },
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("relativeTo"))
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getCOMSpatialAcceleration(
                _relativeTo, _inCoordinatesOf);
          },
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("relativeTo"))
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration(
                _relativeTo, _inCoordinatesOf);
          },
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("inCoordinatesOf"))
      .def(
          "resetUnion",
          +[](dart::dynamics::Skeleton* self)
              -> void { return self->resetUnion(); })
      .def_readwrite(
          "mUnionRootSkeleton", &dart::dynamics::Skeleton::mUnionRootSkeleton)
      .def_readwrite("mUnionSize", &dart::dynamics::Skeleton::mUnionSize)
      .def_readwrite("mUnionIndex", &dart::dynamics::Skeleton::mUnionIndex);
}

} // namespace python
} // namespace dart
