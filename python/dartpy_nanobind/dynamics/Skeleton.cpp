// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include <dart/dynamics/BallJoint.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/EulerJoint.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/HierarchicalIK.hpp>
#include <dart/dynamics/JacobianNode.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/MetaSkeleton.hpp>
#include <dart/dynamics/PlanarJoint.hpp>
#include <dart/dynamics/PointMass.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/ScrewJoint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>
#include <dart/dynamics/TranslationalJoint.hpp>
#include <dart/dynamics/TranslationalJoint2D.hpp>
#include <dart/dynamics/UniversalJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <dart/math/MathTypes.hpp>

#include <dart/common/LockableReference.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cstddef>

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
      nb::rv_policy::reference_internal)                                       \
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
          nb::rv_policy::reference_internal,                                   \
          nb::arg("parent").none())                                            \
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
          nb::rv_policy::reference_internal,                                   \
          nb::arg("parent").none(),                                            \
          nb::arg("jointProperties"))                                          \
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
          nb::rv_policy::reference_internal,                                   \
          nb::arg("parent").none(),                                            \
          nb::arg("jointProperties"),                                          \
          nb::arg("bodyProperties"))

namespace dart {
namespace python {

void Skeleton(nb::module_& m)
{
  dartnb::dart_class<dart::dynamics::Skeleton, dart::dynamics::MetaSkeleton>(
      m, "Skeleton")
      .def(dartnb::factory(+[]() -> dart::dynamics::SkeletonPtr {
        return dart::dynamics::Skeleton::create();
      }))
      .def(
          dartnb::factory(
              +[](const std::string& _name) -> dart::dynamics::SkeletonPtr {
                return dart::dynamics::Skeleton::create(_name);
              }),
          nb::arg("name"))
      .def(
          dartnb::factory(
              +[](const dart::dynamics::Skeleton::AspectPropertiesData&
                      properties) -> dart::dynamics::SkeletonPtr {
                return dart::dynamics::Skeleton::create(properties);
              }),
          nb::arg("properties"))
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
          nb::arg("cloneName"))
      .def(
          "setConfiguration",
          +[](dart::dynamics::Skeleton* self,
              const dart::dynamics::Skeleton::Configuration& configuration)
              -> void { return self->setConfiguration(configuration); },
          nb::arg("configuration"))
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
          nb::arg("flags"))
      .def(
          "getConfiguration",
          +[](const dart::dynamics::Skeleton* self,
              const std::vector<std::size_t>& indices)
              -> dart::dynamics::Skeleton::Configuration {
            return self->getConfiguration(indices);
          },
          nb::arg("indices"))
      .def(
          "getConfiguration",
          +[](const dart::dynamics::Skeleton* self,
              const std::vector<std::size_t>& indices,
              int flags) -> dart::dynamics::Skeleton::Configuration {
            return self->getConfiguration(indices, flags);
          },
          nb::arg("indices"),
          nb::arg("flags"))
      .def(
          "setState",
          +[](dart::dynamics::Skeleton* self,
              const dart::dynamics::Skeleton::State& state) -> void {
            return self->setState(state);
          },
          nb::arg("state"))
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
          nb::arg("properties"))
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
          nb::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::Skeleton* self,
              const dart::dynamics::Skeleton::AspectProperties& properties)
              -> void { return self->setAspectProperties(properties); },
          nb::arg("properties"))
      .def(
          "setName",
          +[](dart::dynamics::Skeleton* self, const std::string& _name)
              -> const std::string& { return self->setName(_name); },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def(
          "getName",
          +[](const dart::dynamics::Skeleton* self) -> const std::string& {
            return self->getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "setSelfCollisionCheck",
          +[](dart::dynamics::Skeleton* self, bool enable) -> void {
            return self->setSelfCollisionCheck(enable);
          },
          nb::arg("enable"))
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
          nb::arg("enable"))
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
          nb::arg("isMobile"))
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
          nb::arg("timeStep"))
      .def(
          "getTimeStep",
          +[](const dart::dynamics::Skeleton* self) -> double {
            return self->getTimeStep();
          })
      .def(
          "setGravity",
          +[](dart::dynamics::Skeleton* self, const Eigen::Vector3d& _gravity)
              -> void { return self->setGravity(_gravity); },
          nb::arg("gravity"))
      .def(
          "getGravity",
          +[](const dart::dynamics::Skeleton* self) -> const Eigen::Vector3d& {
            return self->getGravity();
          },
          nb::rv_policy::reference_internal)
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
      .def(
          "createFreeJointAndSoftBodyNodePair",
          [](dart::dynamics::Skeleton* self,
             dart::dynamics::BodyNode* parent,
             const dart::dynamics::FreeJoint::Properties& jointProperties,
             const dart::dynamics::SoftBodyNode::Properties& bodyProperties) {
            return self->createJointAndBodyNodePair<
                dart::dynamics::FreeJoint,
                dart::dynamics::SoftBodyNode>(
                parent, jointProperties, bodyProperties);
          },
          nb::rv_policy::reference_internal,
          nb::arg("parent").none(),
          nb::arg("jointProperties"),
          nb::arg("bodyProperties"))
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
          nb::rv_policy::reference)
      .def(
          "getRootBodyNode",
          +[](dart::dynamics::Skeleton* self,
              std::size_t index) -> dart::dynamics::BodyNode* {
            return self->getRootBodyNode(index);
          },
          nb::arg("treeIndex"),
          nb::rv_policy::reference)
      .def(
          "getRootJoint",
          +[](dart::dynamics::Skeleton* self) -> dart::dynamics::Joint* {
            return self->getRootJoint();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getRootJoint",
          +[](dart::dynamics::Skeleton* self, std::size_t index)
              -> dart::dynamics::Joint* { return self->getRootJoint(index); },
          nb::arg("treeIndex"),
          nb::rv_policy::reference_internal)
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
          nb::arg("name"))
      .def(
          "getBodyNodes",
          +[](const dart::dynamics::Skeleton* self, const std::string& name)
              -> std::vector<const dart::dynamics::BodyNode*> {
            return self->getBodyNodes(name);
          },
          nb::arg("name"))
      .def(
          "hasBodyNode",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::BodyNode* bodyNode) -> bool {
            return self->hasBodyNode(bodyNode);
          },
          nb::arg("bodyNode").none())
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::BodyNode* _bn) -> std::size_t {
            return self->getIndexOf(_bn);
          },
          nb::arg("bn").none())
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::BodyNode* _bn,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_bn, _warning);
          },
          nb::arg("bn").none(),
          nb::arg("warning"))
      .def(
          "getTreeBodyNodes",
          +[](const dart::dynamics::Skeleton* self, std::size_t _treeIdx)
              -> std::vector<const dart::dynamics::BodyNode*> {
            return self->getTreeBodyNodes(_treeIdx);
          },
          nb::arg("treeIdx"))
      .def(
          "getNumJoints",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumJoints();
          })
      .def(
          "getJoint",
          +[](dart::dynamics::Skeleton* self, std::size_t _idx)
              -> dart::dynamics::Joint* { return self->getJoint(_idx); },
          nb::arg("idx"),
          nb::rv_policy::reference_internal)
      .def(
          "getJoint",
          +[](dart::dynamics::Skeleton* self, const std::string& name)
              -> dart::dynamics::Joint* { return self->getJoint(name); },
          nb::arg("name"),
          nb::rv_policy::reference_internal)
      .def(
          "getJoints",
          +[](dart::dynamics::Skeleton* self)
              -> std::vector<dart::dynamics::Joint*> {
            return self->getJoints();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getJoints",
          +[](const dart::dynamics::Skeleton* self)
              -> std::vector<const dart::dynamics::Joint*> {
            return self->getJoints();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getJoints",
          +[](dart::dynamics::Skeleton* self,
              const std::string& name) -> std::vector<dart::dynamics::Joint*> {
            return self->getJoints(name);
          },
          nb::arg("name"),
          nb::rv_policy::reference_internal)
      .def(
          "getJoints",
          +[](const dart::dynamics::Skeleton* self, const std::string& name)
              -> std::vector<const dart::dynamics::Joint*> {
            return self->getJoints(name);
          },
          nb::arg("name"),
          nb::rv_policy::reference_internal)
      .def(
          "hasJoint",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Joint* joint) -> bool {
            return self->hasJoint(joint);
          },
          nb::arg("joint").none())
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Joint* _joint) -> std::size_t {
            return self->getIndexOf(_joint);
          },
          nb::arg("joint").none())
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Joint* _joint,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_joint, _warning);
          },
          nb::arg("joint").none(),
          nb::arg("warning"))
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
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "getDof",
          +[](dart::dynamics::Skeleton* self,
              const std::string& name) -> dart::dynamics::DegreeOfFreedom* {
            return self->getDof(name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def(
          "getDofs",
          +[](dart::dynamics::Skeleton* self)
              -> std::vector<dart::dynamics::DegreeOfFreedom*> {
            return self->getDofs();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::DegreeOfFreedom* _dof) -> std::size_t {
            return self->getIndexOf(_dof);
          },
          nb::arg("dof").none())
      .def(
          "getIndexOf",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::DegreeOfFreedom* _dof,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_dof, _warning);
          },
          nb::arg("dof").none(),
          nb::arg("warning"))
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
          nb::arg("treeIndex"))
      .def(
          "getNumShapeNodes",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumShapeNodes();
          })
      .def(
          "getNumShapeNodes",
          +[](const dart::dynamics::Skeleton* self, std::size_t treeIndex)
              -> std::size_t { return self->getNumShapeNodes(treeIndex); },
          nb::arg("treeIndex"))
      .def(
          "getShapeNode",
          +[](dart::dynamics::Skeleton* self,
              std::size_t index) -> dart::dynamics::ShapeNode* {
            return self->getShapeNode(index);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "getShapeNode",
          +[](dart::dynamics::Skeleton* self,
              const std::string& name) -> dart::dynamics::ShapeNode* {
            return self->getShapeNode(name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def(
          "getNumEndEffectors",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getNumEndEffectors();
          })
      .def(
          "getNumEndEffectors",
          +[](const dart::dynamics::Skeleton* self, std::size_t treeIndex)
              -> std::size_t { return self->getNumEndEffectors(treeIndex); },
          nb::arg("treeIndex"))
      .def(
          "integratePositions",
          +[](dart::dynamics::Skeleton* self, double _dt) -> void {
            return self->integratePositions(_dt);
          },
          nb::arg("dt"))
      .def(
          "integrateVelocities",
          +[](dart::dynamics::Skeleton* self, double _dt) -> void {
            return self->integrateVelocities(_dt);
          },
          nb::arg("dt"))
      .def(
          "getPositionDifferences",
          +[](const dart::dynamics::Skeleton* self,
              const Eigen::VectorXd& _q2,
              const Eigen::VectorXd& _q1) -> Eigen::VectorXd {
            return self->getPositionDifferences(_q2, _q1);
          },
          nb::arg("q2"),
          nb::arg("q1"))
      .def(
          "getVelocityDifferences",
          +[](const dart::dynamics::Skeleton* self,
              const Eigen::VectorXd& _dq2,
              const Eigen::VectorXd& _dq1) -> Eigen::VectorXd {
            return self->getVelocityDifferences(_dq2, _dq1);
          },
          nb::arg("dq2"),
          nb::arg("dq1"))
      .def(
          "getSupportVersion",
          +[](const dart::dynamics::Skeleton* self) -> std::size_t {
            return self->getSupportVersion();
          })
      .def(
          "getSupportVersion",
          +[](const dart::dynamics::Skeleton* self, std::size_t _treeIdx)
              -> std::size_t { return self->getSupportVersion(_treeIdx); },
          nb::arg("treeIdx"))
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
          nb::arg("updateTransforms"))
      .def(
          "computeForwardKinematics",
          +[](dart::dynamics::Skeleton* self,
              bool _updateTransforms,
              bool _updateVels) -> void {
            return self->computeForwardKinematics(
                _updateTransforms, _updateVels);
          },
          nb::arg("updateTransforms"),
          nb::arg("updateVels"))
      .def(
          "computeForwardKinematics",
          +[](dart::dynamics::Skeleton* self,
              bool _updateTransforms,
              bool _updateVels,
              bool _updateAccs) -> void {
            return self->computeForwardKinematics(
                _updateTransforms, _updateVels, _updateAccs);
          },
          nb::arg("updateTransforms"),
          nb::arg("updateVels"),
          nb::arg("updateAccs"))
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
          nb::arg("withExternalForces"))
      .def(
          "computeInverseDynamics",
          +[](dart::dynamics::Skeleton* self,
              bool _withExternalForces,
              bool _withDampingForces) -> void {
            return self->computeInverseDynamics(
                _withExternalForces, _withDampingForces);
          },
          nb::arg("withExternalForces"),
          nb::arg("withDampingForces"))
      .def(
          "computeInverseDynamics",
          +[](dart::dynamics::Skeleton* self,
              bool _withExternalForces,
              bool _withDampingForces,
              bool _withSpringForces) -> void {
            return self->computeInverseDynamics(
                _withExternalForces, _withDampingForces, _withSpringForces);
          },
          nb::arg("withExternalForces"),
          nb::arg("withDampingForces"),
          nb::arg("withSpringForces"))
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
          nb::arg("bodyNode").none())
      .def(
          "updateBiasImpulse",
          +[](dart::dynamics::Skeleton* self,
              dart::dynamics::BodyNode* _bodyNode,
              const Eigen::Vector6d& _imp) -> void {
            return self->updateBiasImpulse(_bodyNode, _imp);
          },
          nb::arg("bodyNode").none(),
          nb::arg("imp"))
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
          nb::arg("bodyNode1").none(),
          nb::arg("imp1"),
          nb::arg("bodyNode2").none(),
          nb::arg("imp2"))
      .def(
          "updateBiasImpulse",
          +[](dart::dynamics::Skeleton* self,
              dart::dynamics::SoftBodyNode* _softBodyNode,
              dart::dynamics::PointMass* _pointMass,
              const Eigen::Vector3d& _imp) -> void {
            return self->updateBiasImpulse(_softBodyNode, _pointMass, _imp);
          },
          nb::arg("softBodyNode").none(),
          nb::arg("pointMass").none(),
          nb::arg("imp"))
      .def(
          "updateVelocityChange",
          +[](dart::dynamics::Skeleton* self)
              -> void { return self->updateVelocityChange(); })
      .def(
          "setImpulseApplied",
          +[](dart::dynamics::Skeleton* self, bool _val) -> void {
            return self->setImpulseApplied(_val);
          },
          nb::arg("val"))
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
          nb::arg("node").none())
      .def(
          "getJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobian(_node, _localOffset);
          },
          nb::arg("node").none(),
          nb::arg("localOffset"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::Skeleton* self,
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
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian { return self->getWorldJacobian(_node); },
          nb::arg("node").none())
      .def(
          "getWorldJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getWorldJacobian(_node, _localOffset);
          },
          nb::arg("node").none(),
          nb::arg("localOffset"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node);
          },
          nb::arg("node").none())
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _localOffset);
          },
          nb::arg("node").none(),
          nb::arg("localOffset"))
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
          nb::arg("node").none(),
          nb::arg("localOffset"),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_node);
          },
          nb::arg("node").none())
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node);
          },
          nb::arg("node").none())
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node, _localOffset);
          },
          nb::arg("node").none(),
          nb::arg("localOffset"))
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
          nb::arg("node").none(),
          nb::arg("localOffset"),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node);
          },
          nb::arg("node").none())
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node, _localOffset);
          },
          nb::arg("node").none(),
          nb::arg("localOffset"))
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
          nb::arg("node").none(),
          nb::arg("localOffset"),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node);
          },
          nb::arg("node").none())
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node, _localOffset);
          },
          nb::arg("node").none(),
          nb::arg("localOffset"))
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
          nb::arg("node").none(),
          nb::arg("localOffset"),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getAngularJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv(_node);
          },
          nb::arg("node").none())
      .def(
          "getAngularJacobianDeriv",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv(_node, _inCoordinatesOf);
          },
          nb::arg("node").none(),
          nb::arg("inCoordinatesOf").none())
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
      // Preserve MetaSkeleton overloads hidden by the tree-index overloads.
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
      //      nb::arg("treeIdx"))
      .def(
          "dirtyArticulatedInertia",
          +[](dart::dynamics::Skeleton* self, std::size_t _treeIdx) -> void {
            return self->dirtyArticulatedInertia(_treeIdx);
          },
          nb::arg("treeIdx"))
      //      .def("notifySupportUpdate", +[](dart::dynamics::Skeleton *self,
      //      std::size_t _treeIdx) -> void { return
      //      self->notifySupportUpdate(_treeIdx); },
      //      nb::arg("treeIdx"))
      .def(
          "dirtySupportPolygon",
          +[](dart::dynamics::Skeleton* self, std::size_t _treeIdx) -> void {
            return self->dirtySupportPolygon(_treeIdx);
          },
          nb::arg("treeIdx"))
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
          nb::arg("withRespectTo").none())
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
          nb::arg("relativeTo").none())
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getCOMSpatialVelocity(_relativeTo, _inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("relativeTo").none())
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getCOMLinearVelocity(_relativeTo, _inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("relativeTo").none())
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::Skeleton* self,
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
          +[](const dart::dynamics::Skeleton* self) -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration();
          })
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::Skeleton* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration(_relativeTo);
          },
          nb::arg("relativeTo").none())
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::Skeleton* self,
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
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("inCoordinatesOf").none())
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
          nb::arg("inCoordinatesOf").none())
      .def(
          "resetUnion",
          +[](dart::dynamics::Skeleton* self)
              -> void { return self->resetUnion(); })
      .def_rw(
          "mUnionRootSkeleton",
          &dart::dynamics::Skeleton::mUnionRootSkeleton,
          dartnb::setterArgument(&dart::dynamics::Skeleton::mUnionRootSkeleton))
      .def_rw(
          "mUnionSize",
          &dart::dynamics::Skeleton::mUnionSize,
          dartnb::setterArgument(&dart::dynamics::Skeleton::mUnionSize))
      .def_rw(
          "mUnionIndex",
          &dart::dynamics::Skeleton::mUnionIndex,
          dartnb::setterArgument(&dart::dynamics::Skeleton::mUnionIndex));
}

} // namespace python
} // namespace dart

#undef DARTPY_DEFINE_CREATE_JOINT_AND_BODY_NODE_PAIR
