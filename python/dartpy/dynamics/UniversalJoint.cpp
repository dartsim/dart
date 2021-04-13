/*
 * Copyright (c) 2011-2021, The DART development contributors
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
#include <eigen_geometry_pybind.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include "Joint.hpp"

namespace py = pybind11;

namespace dart {
namespace python {

void UniversalJoint(py::module& m)
{
  ::py::class_<dart::dynamics::UniversalJoint::UniqueProperties>(
      m, "UniversalJointUniqueProperties")
      .def(::py::init<>());

  ::py::class_<
      dart::dynamics::UniversalJoint::Properties,
      dart::dynamics::GenericJoint<math::R2Space>::Properties,
      dart::dynamics::UniversalJoint::UniqueProperties>(
      m, "UniversalJointProperties")
      .def(::py::init<>())
      .def(
          ::py::init<const dart::dynamics::GenericJoint<
              dart::math::R2Space>::Properties&>(),
          ::py::arg("genericJointProperties"))
      .def(
          ::py::init<
              const dart::dynamics::GenericJoint<
                  dart::math::R2Space>::Properties&,
              const dart::dynamics::UniversalJoint::UniqueProperties&>(),
          ::py::arg("genericJointProperties"),
          ::py::arg("uniqueProperties"))
      .def_readwrite(
          "mAxis",
          &dart::dynamics::detail::UniversalJointUniqueProperties::mAxis);

  DARTPY_DEFINE_JOINT_COMMON_BASE(UniversalJoint, R2Space)

  ::py::class_<
      dart::dynamics::UniversalJoint,
      dart::common::EmbedPropertiesOnTopOf<
          dart::dynamics::UniversalJoint,
          dart::dynamics::detail::UniversalJointUniqueProperties,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2> > >,
      std::shared_ptr<dart::dynamics::UniversalJoint> >(m, "UniversalJoint")
      .def(
          "hasUniversalJointAspect",
          +[](const dart::dynamics::UniversalJoint* self) -> bool {
            return self->hasUniversalJointAspect();
          })
      .def(
          "setUniversalJointAspect",
          +[](dart::dynamics::UniversalJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::UniversalJoint,
                  dart::dynamics::detail::UniversalJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<2> > >::Aspect* aspect) {
            self->setUniversalJointAspect(aspect);
          },
          ::py::arg("aspect"))
      .def(
          "removeUniversalJointAspect",
          +[](dart::dynamics::UniversalJoint* self) {
            self->removeUniversalJointAspect();
          })
      .def(
          "releaseUniversalJointAspect",
          +[](dart::dynamics::UniversalJoint* self)
              -> std::unique_ptr<dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::UniversalJoint,
                  dart::dynamics::detail::UniversalJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<2> > >::Aspect> {
            return self->releaseUniversalJointAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::UniversalJoint* self,
              const dart::dynamics::UniversalJoint::Properties& _properties) {
            self->setProperties(_properties);
          },
          ::py::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::UniversalJoint* self,
              const dart::dynamics::UniversalJoint::UniqueProperties&
                  _properties) { self->setProperties(_properties); },
          ::py::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::UniversalJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::UniversalJoint,
                  dart::dynamics::detail::UniversalJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<2> > >::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          ::py::arg("properties"))
      .def(
          "getUniversalJointProperties",
          +[](const dart::dynamics::UniversalJoint* self)
              -> dart::dynamics::UniversalJoint::Properties {
            return self->getUniversalJointProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::UniversalJoint* self,
              const dart::dynamics::UniversalJoint* _otherJoint) {
            self->copy(_otherJoint);
          },
          ::py::arg("otherJoint"))
      .def(
          "getType",
          +[](const dart::dynamics::UniversalJoint* self)
              -> const std::string& { return self->getType(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::UniversalJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          ::py::arg("index"))
      .def(
          "setAxis1",
          +[](dart::dynamics::UniversalJoint* self,
              const Eigen::Vector3d& _axis) { self->setAxis1(_axis); },
          ::py::arg("axis"))
      .def(
          "setAxis2",
          +[](dart::dynamics::UniversalJoint* self,
              const Eigen::Vector3d& _axis) { self->setAxis2(_axis); },
          ::py::arg("axis"))
      .def(
          "getAxis1",
          +[](const dart::dynamics::UniversalJoint* self)
              -> const Eigen::Vector3d& { return self->getAxis1(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "getAxis2",
          +[](const dart::dynamics::UniversalJoint* self)
              -> const Eigen::Vector3d& { return self->getAxis2(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::UniversalJoint* self,
              const Eigen::Vector2d& _positions)
              -> Eigen::Matrix<double, 6, 2> {
            return self->getRelativeJacobianStatic(_positions);
          },
          ::py::arg("positions"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::UniversalJoint::getStaticType();
          },
          ::py::return_value_policy::reference_internal);
}

} // namespace python
} // namespace dart
