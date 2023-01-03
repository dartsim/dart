/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include "Joint.hpp"

#include <dart/dart.hpp>

#include <eigen_geometry_pybind.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void PlanarJoint(py::module& m)
{
  ::py::class_<dart::dynamics::PlanarJoint::UniqueProperties>(
      m, "PlanarJointUniqueProperties")
      .def(::py::init<>());

  ::py::class_<
      dart::dynamics::PlanarJoint::Properties,
      dart::dynamics::GenericJoint<math::R3Space>::Properties,
      dart::dynamics::PlanarJoint::UniqueProperties>(m, "PlanarJointProperties")
      .def(::py::init<>())
      .def(
          ::py::init<const dart::dynamics::GenericJoint<
              dart::math::R3Space>::Properties&>(),
          ::py::arg("genericJointProperties"))
      .def(
          ::py::init<
              const dart::dynamics::GenericJoint<
                  dart::math::R3Space>::Properties&,
              const dart::dynamics::PlanarJoint::UniqueProperties&>(),
          ::py::arg("genericJointProperties"),
          ::py::arg("uniqueProperties"))
      .def_readwrite(
          "mPlaneType",
          &dart::dynamics::detail::PlanarJointUniqueProperties::mPlaneType)
      .def_readwrite(
          "mTransAxis1",
          &dart::dynamics::detail::PlanarJointUniqueProperties::mTransAxis1)
      .def_readwrite(
          "mTransAxis2",
          &dart::dynamics::detail::PlanarJointUniqueProperties::mTransAxis2)
      .def_readwrite(
          "mRotAxis",
          &dart::dynamics::detail::PlanarJointUniqueProperties::mRotAxis);

  DARTPY_DEFINE_JOINT_COMMON_BASE(PlanarJoint, R3Space)

  ::py::class_<
      dart::dynamics::PlanarJoint,
      dart::common::EmbedPropertiesOnTopOf<
          dart::dynamics::PlanarJoint,
          dart::dynamics::detail::PlanarJointUniqueProperties,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>>,
      std::shared_ptr<dart::dynamics::PlanarJoint>>(m, "PlanarJoint")
      .def(
          "hasPlanarJointAspect",
          +[](const dart::dynamics::PlanarJoint* self) -> bool {
            return self->hasPlanarJointAspect();
          })
      .def(
          "setPlanarJointAspect",
          +[](dart::dynamics::PlanarJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::PlanarJoint,
                  dart::dynamics::detail::PlanarJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<3>>>::Aspect* aspect) {
            self->setPlanarJointAspect(aspect);
          },
          ::py::arg("aspect"))
      .def(
          "removePlanarJointAspect",
          +[](dart::dynamics::PlanarJoint* self) {
            self->removePlanarJointAspect();
          })
      .def(
          "releasePlanarJointAspect",
          +[](dart::dynamics::PlanarJoint* self)
              -> std::unique_ptr<dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::PlanarJoint,
                  dart::dynamics::detail::PlanarJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<3>>>::Aspect> {
            return self->releasePlanarJointAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::PlanarJoint* self,
              const dart::dynamics::PlanarJoint::Properties& _properties) {
            self->setProperties(_properties);
          },
          ::py::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::PlanarJoint* self,
              const dart::dynamics::PlanarJoint::UniqueProperties&
                  _properties) { self->setProperties(_properties); },
          ::py::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::PlanarJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::PlanarJoint,
                  dart::dynamics::detail::PlanarJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<3>>>::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          ::py::arg("properties"))
      .def(
          "getPlanarJointProperties",
          +[](const dart::dynamics::PlanarJoint* self)
              -> dart::dynamics::PlanarJoint::Properties {
            return self->getPlanarJointProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::PlanarJoint* self,
              const dart::dynamics::PlanarJoint* _otherJoint) {
            self->copy(_otherJoint);
          },
          ::py::arg("otherJoint"))
      .def(
          "getType",
          +[](const dart::dynamics::PlanarJoint* self) -> const std::string& {
            return self->getType();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::PlanarJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          ::py::arg("index"))
      .def(
          "setXYPlane",
          +[](dart::dynamics::PlanarJoint* self) { self->setXYPlane(); })
      .def(
          "setXYPlane",
          +[](dart::dynamics::PlanarJoint* self, bool _renameDofs) {
            self->setXYPlane(_renameDofs);
          },
          ::py::arg("renameDofs"))
      .def(
          "setYZPlane",
          +[](dart::dynamics::PlanarJoint* self) { self->setYZPlane(); })
      .def(
          "setYZPlane",
          +[](dart::dynamics::PlanarJoint* self, bool _renameDofs) {
            self->setYZPlane(_renameDofs);
          },
          ::py::arg("renameDofs"))
      .def(
          "setZXPlane",
          +[](dart::dynamics::PlanarJoint* self) { self->setZXPlane(); })
      .def(
          "setZXPlane",
          +[](dart::dynamics::PlanarJoint* self, bool _renameDofs) {
            self->setZXPlane(_renameDofs);
          },
          ::py::arg("renameDofs"))
      .def(
          "setArbitraryPlane",
          +[](dart::dynamics::PlanarJoint* self,
              const Eigen::Vector3d& _transAxis1,
              const Eigen::Vector3d& _transAxis2) {
            self->setArbitraryPlane(_transAxis1, _transAxis2);
          },
          ::py::arg("transAxis1"),
          ::py::arg("transAxis2"))
      .def(
          "setArbitraryPlane",
          +[](dart::dynamics::PlanarJoint* self,
              const Eigen::Vector3d& _transAxis1,
              const Eigen::Vector3d& _transAxis2,
              bool _renameDofs) {
            self->setArbitraryPlane(_transAxis1, _transAxis2, _renameDofs);
          },
          ::py::arg("transAxis1"),
          ::py::arg("transAxis2"),
          ::py::arg("renameDofs"))
      .def(
          "getPlaneType",
          +[](const dart::dynamics::PlanarJoint* self)
              -> dart::dynamics::PlanarJoint::PlaneType {
            return self->getPlaneType();
          })
      .def(
          "getRotationalAxis",
          +[](const dart::dynamics::PlanarJoint* self)
              -> const Eigen::Vector3d& { return self->getRotationalAxis(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "getTranslationalAxis1",
          +[](const dart::dynamics::PlanarJoint* self)
              -> const Eigen::Vector3d& {
            return self->getTranslationalAxis1();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "getTranslationalAxis2",
          +[](const dart::dynamics::PlanarJoint* self)
              -> const Eigen::Vector3d& {
            return self->getTranslationalAxis2();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::PlanarJoint* self,
              const Eigen::Vector3d& _positions)
              -> Eigen::Matrix<double, 6, 3> {
            return self->getRelativeJacobianStatic(_positions);
          },
          ::py::arg("positions"))
      .def_static(
          "getStaticType",
          +[]() -> const std::
                    string& {
                      return dart::dynamics::PlanarJoint::getStaticType();
                    },
          ::py::return_value_policy::reference_internal);
}

} // namespace python
} // namespace dart
