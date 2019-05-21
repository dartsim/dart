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

namespace dart {
namespace python {

void MetaSkeleton(pybind11::module& m)
{
  ::pybind11::class_<
      dart::dynamics::MetaSkeleton,
      std::shared_ptr<dart::dynamics::MetaSkeleton>>(m, "MetaSkeleton")
      .def(
          "cloneMetaSkeleton",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::string& cloneName) -> dart::dynamics::MetaSkeletonPtr {
            return self->cloneMetaSkeleton(cloneName);
          },
          ::pybind11::arg("cloneName"))
      .def(
          "cloneMetaSkeleton",
          +[](const dart::dynamics::MetaSkeleton* self)
              -> dart::dynamics::MetaSkeletonPtr {
            return self->cloneMetaSkeleton();
          })
      .def(
          "getLockableReference",
          +[](const dart::dynamics::MetaSkeleton* self)
              -> std::unique_ptr<dart::common::LockableReference> {
            return self->getLockableReference();
          })
      .def(
          "setName",
          +[](dart::dynamics::MetaSkeleton* self, const std::string& _name)
              -> const std::string& { return self->setName(_name); },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("name"))
      .def(
          "getName",
          +[](const dart::dynamics::MetaSkeleton* self) -> const std::string& {
            return self->getName();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getNumBodyNodes",
          +[](const dart::dynamics::MetaSkeleton* self) -> std::size_t {
            return self->getNumBodyNodes();
          })
      .def(
          "getBodyNode",
          +[](dart::dynamics::MetaSkeleton* self, std::size_t index)
              -> dart::dynamics::BodyNode* { return self->getBodyNode(index); },
          ::pybind11::arg("index"),
          pybind11::return_value_policy::reference)
      .def(
          "getBodyNode",
          +[](dart::dynamics::MetaSkeleton* self, const std::string& name)
              -> dart::dynamics::BodyNode* { return self->getBodyNode(name); },
          ::pybind11::arg("treeIndex"),
          pybind11::return_value_policy::reference)
      .def(
          "getBodyNodes",
          +[](dart::dynamics::MetaSkeleton* self, const std::string& name)
              -> std::vector<dart::dynamics::BodyNode*> {
            return self->getBodyNodes(name);
          },
          ::pybind11::arg("name"))
      .def(
          "getBodyNodes",
          +[](const dart::dynamics::MetaSkeleton* self, const std::string& name)
              -> std::vector<const dart::dynamics::BodyNode*> {
            return self->getBodyNodes(name);
          },
          ::pybind11::arg("name"))
      .def(
          "hasBodyNode",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::BodyNode* bodyNode) -> bool {
            return self->hasBodyNode(bodyNode);
          },
          ::pybind11::arg("bodyNode"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::BodyNode* _bn) -> std::size_t {
            return self->getIndexOf(_bn);
          },
          ::pybind11::arg("bn"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::BodyNode* _bn,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_bn, _warning);
          },
          ::pybind11::arg("bn"),
          ::pybind11::arg("warning"))
      .def(
          "getNumJoints",
          +[](const dart::dynamics::MetaSkeleton* self) -> std::size_t {
            return self->getNumJoints();
          })
      .def(
          "getJoint",
          +[](dart::dynamics::MetaSkeleton* self, std::size_t index)
              -> dart::dynamics::Joint* { return self->getJoint(index); },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"))
      .def(
          "getJoint",
          +[](dart::dynamics::MetaSkeleton* self, const std::string& name)
              -> dart::dynamics::Joint* { return self->getJoint(name); },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("name"))
      .def(
          "getJoints",
          +[](dart::dynamics::MetaSkeleton* self)
              -> std::vector<dart::dynamics::Joint*> {
            return self->getJoints();
          })
      .def(
          "getJoints",
          +[](const dart::dynamics::MetaSkeleton* self)
              -> std::vector<const dart::dynamics::Joint*> {
            return self->getJoints();
          })
      .def(
          "getJoints",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::string& name) -> std::vector<dart::dynamics::Joint*> {
            return self->getJoints(name);
          },
          ::pybind11::arg("name"))
      .def(
          "getJoints",
          +[](const dart::dynamics::MetaSkeleton* self, const std::string& name)
              -> std::vector<const dart::dynamics::Joint*> {
            return self->getJoints(name);
          },
          ::pybind11::arg("name"))
      .def(
          "hasJoint",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Joint* joint) -> bool {
            return self->hasJoint(joint);
          },
          ::pybind11::arg("joint"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Joint* _joint) -> std::size_t {
            return self->getIndexOf(_joint);
          },
          ::pybind11::arg("joint"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Joint* _joint,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_joint, _warning);
          },
          ::pybind11::arg("joint"),
          ::pybind11::arg("warning"))
      .def(
          "getNumDofs",
          +[](const dart::dynamics::MetaSkeleton* self) -> std::size_t {
            return self->getNumDofs();
          })
      .def(
          "getDof",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t index) -> dart::dynamics::DegreeOfFreedom* {
            return self->getDof(index);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"))
      .def(
          "getDofs",
          +[](dart::dynamics::MetaSkeleton* self)
              -> std::vector<dart::dynamics::DegreeOfFreedom*> {
            return self->getDofs();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getIndexOf",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::DegreeOfFreedom* _dof) -> std::size_t {
            return self->getIndexOf(_dof);
          },
          ::pybind11::arg("dof"))
      .def(
          "getIndexOf",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::DegreeOfFreedom* _dof,
              bool _warning) -> std::size_t {
            return self->getIndexOf(_dof, _warning);
          },
          ::pybind11::arg("dof"),
          ::pybind11::arg("warning"))
      .def(
          "setCommand",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t _index,
              double _command) { self->setCommand(_index, _command); },
          ::pybind11::arg("index"),
          ::pybind11::arg("command"))
      .def(
          "getCommand",
          +[](const dart::dynamics::MetaSkeleton* self, std::size_t _index)
              -> double { return self->getCommand(_index); },
          ::pybind11::arg("index"))
      .def(
          "setCommands",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& _commands) {
            self->setCommands(_commands);
          },
          ::pybind11::arg("commands"))
      .def(
          "setCommands",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& _indices,
              const Eigen::VectorXd& _commands) {
            self->setCommands(_indices, _commands);
          },
          ::pybind11::arg("indices"),
          ::pybind11::arg("commands"))
      .def(
          "getCommands",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getCommands();
          })
      .def(
          "getCommands",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& _indices) -> Eigen::VectorXd {
            return self->getCommands(_indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "resetCommands",
          +[](dart::dynamics::MetaSkeleton* self) { self->resetCommands(); })
      .def(
          "setPosition",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t index,
              double _position) { self->setPosition(index, _position); },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPosition",
          +[](const dart::dynamics::MetaSkeleton* self, std::size_t _index)
              -> double { return self->getPosition(_index); },
          ::pybind11::arg("index"))
      .def(
          "setPositions",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& _positions) {
            self->setPositions(_positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "setPositions",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& _indices,
              const Eigen::VectorXd& _positions) {
            self->setPositions(_indices, _positions);
          },
          ::pybind11::arg("indices"),
          ::pybind11::arg("positions"))
      .def(
          "getPositions",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getPositions();
          })
      .def(
          "getPositions",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& _indices) -> Eigen::VectorXd {
            return self->getPositions(_indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "resetPositions",
          +[](dart::dynamics::MetaSkeleton* self) { self->resetPositions(); })
      .def(
          "setPositionLowerLimit",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t _index,
              double _position) {
            self->setPositionLowerLimit(_index, _position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "setPositionLowerLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& positions) {
            self->setPositionLowerLimits(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "setPositionLowerLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices,
              const Eigen::VectorXd& positions) {
            self->setPositionLowerLimits(indices, positions);
          },
          ::pybind11::arg("indices"),
          ::pybind11::arg("positions"))
      .def(
          "getPositionLowerLimit",
          +[](const dart::dynamics::MetaSkeleton* self, std::size_t _index)
              -> double { return self->getPositionLowerLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "getPositionLowerLimits",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getPositionLowerLimits();
          })
      .def(
          "getPositionLowerLimits",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices) -> Eigen::VectorXd {
            return self->getPositionLowerLimits(indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "setPositionUpperLimit",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t _index,
              double _position) {
            self->setPositionUpperLimit(_index, _position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "setPositionUpperLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& positions) {
            self->setPositionUpperLimits(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "setPositionUpperLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices,
              const Eigen::VectorXd& positions) {
            self->setPositionUpperLimits(indices, positions);
          },
          ::pybind11::arg("indices"),
          ::pybind11::arg("positions"))
      .def(
          "getPositionUpperLimit",
          +[](const dart::dynamics::MetaSkeleton* self, std::size_t _index)
              -> double { return self->getPositionUpperLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "getPositionUpperLimits",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getPositionUpperLimits();
          })
      .def(
          "getPositionUpperLimits",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices) -> Eigen::VectorXd {
            return self->getPositionUpperLimits(indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "setVelocity",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t _index,
              double _velocity) { self->setVelocity(_index, _velocity); },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocity",
          +[](const dart::dynamics::MetaSkeleton* self, std::size_t _index)
              -> double { return self->getVelocity(_index); },
          ::pybind11::arg("index"))
      .def(
          "setVelocities",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& _velocities) {
            self->setVelocities(_velocities);
          },
          ::pybind11::arg("velocities"))
      .def(
          "setVelocities",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& _indices,
              const Eigen::VectorXd& _velocities) {
            self->setVelocities(_indices, _velocities);
          },
          ::pybind11::arg("indices"),
          ::pybind11::arg("velocities"))
      .def(
          "getVelocities",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getVelocities();
          })
      .def(
          "getVelocities",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& _indices) -> Eigen::VectorXd {
            return self->getVelocities(_indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "resetVelocities",
          +[](dart::dynamics::MetaSkeleton* self) { self->resetVelocities(); })
      .def(
          "setVelocityLowerLimit",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t _index,
              double _velocity) {
            self->setVelocityLowerLimit(_index, _velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "setVelocityLowerLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& velocities) {
            self->setVelocityLowerLimits(velocities);
          },
          ::pybind11::arg("velocities"))
      .def(
          "setVelocityLowerLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices,
              const Eigen::VectorXd& velocities) {
            self->setVelocityLowerLimits(indices, velocities);
          },
          ::pybind11::arg("indices"),
          ::pybind11::arg("velocities"))
      .def(
          "getVelocityLowerLimit",
          +[](dart::dynamics::MetaSkeleton* self, std::size_t _index)
              -> double { return self->getVelocityLowerLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "getVelocityLowerLimits",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getVelocityLowerLimits();
          })
      .def(
          "getVelocityLowerLimits",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices) -> Eigen::VectorXd {
            return self->getVelocityLowerLimits(indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "setVelocityUpperLimit",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t _index,
              double _velocity) {
            self->setVelocityUpperLimit(_index, _velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "setVelocityUpperLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& velocities) {
            self->setVelocityUpperLimits(velocities);
          },
          ::pybind11::arg("velocities"))
      .def(
          "setVelocityUpperLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices,
              const Eigen::VectorXd& velocities) {
            self->setVelocityUpperLimits(indices, velocities);
          },
          ::pybind11::arg("indices"),
          ::pybind11::arg("velocities"))
      .def(
          "getVelocityUpperLimit",
          +[](dart::dynamics::MetaSkeleton* self, std::size_t _index)
              -> double { return self->getVelocityUpperLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "getVelocityUpperLimits",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getVelocityUpperLimits();
          })
      .def(
          "getVelocityUpperLimits",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices) -> Eigen::VectorXd {
            return self->getVelocityUpperLimits(indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "setAcceleration",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t _index,
              double _acceleration) {
            self->setAcceleration(_index, _acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAcceleration",
          +[](const dart::dynamics::MetaSkeleton* self, std::size_t _index)
              -> double { return self->getAcceleration(_index); },
          ::pybind11::arg("index"))
      .def(
          "setAccelerations",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& _accelerations) {
            self->setAccelerations(_accelerations);
          },
          ::pybind11::arg("accelerations"))
      .def(
          "setAccelerations",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& _indices,
              const Eigen::VectorXd& _accelerations) {
            self->setAccelerations(_indices, _accelerations);
          },
          ::pybind11::arg("indices"),
          ::pybind11::arg("accelerations"))
      .def(
          "getAccelerations",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getAccelerations();
          })
      .def(
          "getAccelerations",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& _indices) -> Eigen::VectorXd {
            return self->getAccelerations(_indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "resetAccelerations",
          +[](dart::dynamics::MetaSkeleton*
                  self) { self->resetAccelerations(); })
      .def(
          "setAccelerationLowerLimit",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t _index,
              double _acceleration) {
            self->setAccelerationLowerLimit(_index, _acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "setAccelerationLowerLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& accelerations) {
            self->setAccelerationLowerLimits(accelerations);
          },
          ::pybind11::arg("accelerations"))
      .def(
          "setAccelerationLowerLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices,
              const Eigen::VectorXd& accelerations) {
            self->setAccelerationLowerLimits(indices, accelerations);
          },
          ::pybind11::arg("indices"),
          ::pybind11::arg("accelerations"))
      .def(
          "getAccelerationLowerLimit",
          +[](const dart::dynamics::MetaSkeleton* self, std::size_t _index)
              -> double { return self->getAccelerationLowerLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "getAccelerationLowerLimits",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getAccelerationLowerLimits();
          })
      .def(
          "getAccelerationLowerLimits",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices) -> Eigen::VectorXd {
            return self->getAccelerationLowerLimits(indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "setAccelerationUpperLimit",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t _index,
              double _acceleration) {
            self->setAccelerationUpperLimit(_index, _acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "setAccelerationUpperLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& accelerations) {
            self->setAccelerationUpperLimits(accelerations);
          },
          ::pybind11::arg("accelerations"))
      .def(
          "setAccelerationUpperLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices,
              const Eigen::VectorXd& accelerations) {
            self->setAccelerationUpperLimits(indices, accelerations);
          },
          ::pybind11::arg("indices"),
          ::pybind11::arg("accelerations"))
      .def(
          "getAccelerationUpperLimit",
          +[](const dart::dynamics::MetaSkeleton* self, std::size_t _index)
              -> double { return self->getAccelerationUpperLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "getAccelerationUpperLimits",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getAccelerationUpperLimits();
          })
      .def(
          "getAccelerationUpperLimits",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices) -> Eigen::VectorXd {
            return self->getAccelerationUpperLimits(indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "setForce",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t _index,
              double _force) { self->setForce(_index, _force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForce",
          +[](const dart::dynamics::MetaSkeleton* self,
              std::size_t _index) -> double { return self->getForce(_index); },
          ::pybind11::arg("index"))
      .def(
          "setForces",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& _forces) { self->setForces(_forces); },
          ::pybind11::arg("forces"))
      .def(
          "setForces",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& _index,
              const Eigen::VectorXd& _forces) {
            self->setForces(_index, _forces);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("forces"))
      .def(
          "getForces",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getForces();
          })
      .def(
          "getForces",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& _indices) -> Eigen::VectorXd {
            return self->getForces(_indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "resetGeneralizedForces",
          +[](dart::dynamics::MetaSkeleton*
                  self) { self->resetGeneralizedForces(); })
      .def(
          "setForceLowerLimit",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t _index,
              double _force) { self->setForceLowerLimit(_index, _force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "setForceLowerLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& forces) {
            self->setForceLowerLimits(forces);
          },
          ::pybind11::arg("forces"))
      .def(
          "setForceLowerLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices,
              const Eigen::VectorXd& forces) {
            self->setForceLowerLimits(indices, forces);
          },
          ::pybind11::arg("indices"),
          ::pybind11::arg("forces"))
      .def(
          "getForceLowerLimit",
          +[](const dart::dynamics::MetaSkeleton* self, std::size_t _index)
              -> double { return self->getForceLowerLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "getForceLowerLimits",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getForceLowerLimits();
          })
      .def(
          "getForceLowerLimits",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices) -> Eigen::VectorXd {
            return self->getForceLowerLimits(indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "setForceUpperLimit",
          +[](dart::dynamics::MetaSkeleton* self,
              std::size_t _index,
              double _force) { self->setForceUpperLimit(_index, _force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "setForceUpperLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& forces) {
            self->setForceUpperLimits(forces);
          },
          ::pybind11::arg("forces"))
      .def(
          "setForceUpperLimits",
          +[](dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices,
              const Eigen::VectorXd& forces) {
            self->setForceUpperLimits(indices, forces);
          },
          ::pybind11::arg("indices"),
          ::pybind11::arg("forces"))
      .def(
          "getForceUpperLimit",
          +[](const dart::dynamics::MetaSkeleton* self, std::size_t _index)
              -> double { return self->getForceUpperLimit(_index); },
          ::pybind11::arg("index"))
      .def(
          "getForceUpperLimits",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getForceUpperLimits();
          })
      .def(
          "getForceUpperLimits",
          +[](const dart::dynamics::MetaSkeleton* self,
              const std::vector<std::size_t>& indices) -> Eigen::VectorXd {
            return self->getForceUpperLimits(indices);
          },
          ::pybind11::arg("indices"))
      .def(
          "getVelocityChanges",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getVelocityChanges();
          })
      .def(
          "setJointConstraintImpulses",
          +[](dart::dynamics::MetaSkeleton* self,
              const Eigen::VectorXd& _impulses) {
            self->setJointConstraintImpulses(_impulses);
          },
          ::pybind11::arg("impulses"))
      .def(
          "getJointConstraintImpulses",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::VectorXd {
            return self->getJointConstraintImpulses();
          })
      .def(
          "getJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian { return self->getJacobian(_node); },
          ::pybind11::arg("node"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::JacobianNode* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(_node, _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobian(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
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
          "getJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset,
              const dart::dynamics::JacobianNode* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobian(
                _node, _localOffset, _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getWorldJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian { return self->getWorldJacobian(_node); },
          ::pybind11::arg("node"))
      .def(
          "getWorldJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getWorldJacobian(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
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
          "getLinearJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::JacobianNode* _relativeTo)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _relativeTo);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("relativeTo"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::JacobianNode* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(
                _node, _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset,
              const dart::dynamics::JacobianNode* _relativeTo)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(_node, _localOffset, _relativeTo);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"),
          ::pybind11::arg("relativeTo"))
      .def(
          "getLinearJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset,
              const dart::dynamics::JacobianNode* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobian(
                _node, _localOffset, _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::JacobianNode* _relativeTo)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(_node, _relativeTo);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("relativeTo"))
      .def(
          "getAngularJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::JacobianNode* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobian(
                _node, _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
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
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::JacobianNode* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(
                _node, _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianSpatialDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset,
              const dart::dynamics::JacobianNode* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianSpatialDeriv(
                _node, _localOffset, _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset) -> dart::math::Jacobian {
            return self->getJacobianClassicDeriv(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getJacobianClassicDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
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
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const Eigen::Vector3d& _localOffset)
              -> dart::math::LinearJacobian {
            return self->getLinearJacobianDeriv(_node, _localOffset);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("localOffset"))
      .def(
          "getLinearJacobianDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
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
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "getAngularJacobianDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::JacobianNode* _node,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::AngularJacobian {
            return self->getAngularJacobianDeriv(_node, _inCoordinatesOf);
          },
          ::pybind11::arg("node"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getMass",
          +[](const dart::dynamics::MetaSkeleton* self)
              -> double { return self->getMass(); })
      .def(
          "getMassMatrix",
          +[](const dart::dynamics::MetaSkeleton* self)
              -> const Eigen::MatrixXd& { return self->getMassMatrix(); })
      .def(
          "getAugMassMatrix",
          +[](const dart::dynamics::MetaSkeleton* self)
              -> const Eigen::MatrixXd& { return self->getAugMassMatrix(); })
      .def(
          "getInvMassMatrix",
          +[](const dart::dynamics::MetaSkeleton* self)
              -> const Eigen::MatrixXd& { return self->getInvMassMatrix(); })
      .def(
          "getCoriolisForces",
          +[](dart::dynamics::MetaSkeleton* self) -> const Eigen::VectorXd& {
            return self->getCoriolisForces();
          })
      .def(
          "getGravityForces",
          +[](dart::dynamics::MetaSkeleton* self) -> const Eigen::VectorXd& {
            return self->getCoriolisForces();
          })
      .def(
          "getCoriolisAndGravityForces",
          +[](dart::dynamics::MetaSkeleton* self) -> const Eigen::VectorXd& {
            return self->getCoriolisAndGravityForces();
          })
      .def(
          "getExternalForces",
          +[](dart::dynamics::MetaSkeleton* self) -> const Eigen::VectorXd& {
            return self->getCoriolisAndGravityForces();
          })
      .def(
          "getConstraintForces",
          +[](dart::dynamics::MetaSkeleton* self) -> const Eigen::VectorXd& {
            return self->getConstraintForces();
          })
      .def(
          "clearExternalForces",
          +[](dart::dynamics::MetaSkeleton*
                  self) { self->clearExternalForces(); })
      .def(
          "clearInternalForces",
          +[](dart::dynamics::MetaSkeleton*
                  self) { self->clearInternalForces(); })
      .def(
          "computeLagrangian",
          +[](const dart::dynamics::MetaSkeleton* self) -> double {
            return self->computeLagrangian();
          })
      .def(
          "computeKineticEnergy",
          +[](const dart::dynamics::MetaSkeleton* self) -> double {
            return self->computeKineticEnergy();
          })
      .def(
          "computePotentialEnergy",
          +[](const dart::dynamics::MetaSkeleton* self) -> double {
            return self->computePotentialEnergy();
          })
      .def(
          "getCOM",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::Vector3d {
            return self->getCOM();
          })
      .def(
          "getCOM",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Frame* _withRespectTo) -> Eigen::Vector3d {
            return self->getCOM(_withRespectTo);
          },
          ::pybind11::arg("withRespectTo"))
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::Vector6d {
            return self->getCOMSpatialVelocity();
          })
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector6d {
            return self->getCOMSpatialVelocity(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMSpatialVelocity",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getCOMSpatialVelocity(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::Vector3d {
            return self->getCOMLinearVelocity();
          })
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getCOMLinearVelocity(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMLinearVelocity",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getCOMLinearVelocity(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::Vector6d {
            return self->getCOMSpatialAcceleration();
          })
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector6d {
            return self->getCOMSpatialAcceleration(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMSpatialAcceleration",
          +[](const dart::dynamics::MetaSkeleton* self,
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
          +[](const dart::dynamics::MetaSkeleton* self) -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration();
          })
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getCOMLinearAcceleration(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getCOMLinearAcceleration",
          +[](const dart::dynamics::MetaSkeleton* self,
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
          +[](const dart::dynamics::MetaSkeleton* self)
              -> dart::math::Jacobian { return self->getCOMJacobian(); })
      .def(
          "getCOMJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getCOMJacobian(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMLinearJacobian",
          +[](const dart::dynamics::MetaSkeleton* self)
              -> dart::math::LinearJacobian {
            return self->getCOMLinearJacobian();
          })
      .def(
          "getCOMLinearJacobian",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getCOMLinearJacobian(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMJacobianSpatialDeriv",
          +[](const dart::dynamics::MetaSkeleton* self)
              -> dart::math::Jacobian {
            return self->getCOMJacobianSpatialDeriv();
          })
      .def(
          "getCOMJacobianSpatialDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::Jacobian {
            return self->getCOMJacobianSpatialDeriv(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getCOMLinearJacobianDeriv",
          +[](const dart::dynamics::MetaSkeleton* self)
              -> dart::math::LinearJacobian {
            return self->getCOMLinearJacobianDeriv();
          })
      .def(
          "getCOMLinearJacobianDeriv",
          +[](const dart::dynamics::MetaSkeleton* self,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> dart::math::LinearJacobian {
            return self->getCOMLinearJacobianDeriv(_inCoordinatesOf);
          },
          ::pybind11::arg("inCoordinatesOf"));
}

} // namespace python
} // namespace dart
