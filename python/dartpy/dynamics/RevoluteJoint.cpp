// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <nanobind/stl/unique_ptr.h>

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

#include "Joint.hpp"

#include <dart/dynamics/GenericJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>

#include <dart/math/ConfigurationSpace.hpp>

#include <dart/common/Aspect.hpp>
#include <dart/common/EmbeddedAspect.hpp>

#include <Eigen/Core>
#include <eigen_geometry_pybind.h>

#include <memory>
#include <string>

#include <cstddef>

namespace dart {
namespace python {

void RevoluteJoint(nb::module_& m)
{
  dartnb::dart_class<dart::dynamics::RevoluteJoint::UniqueProperties>(
      m, "RevoluteJointUniqueProperties")
      .def(dartnb::init<>())
      .def(dartnb::init<const Eigen::Vector3d&>(), nb::arg("axis"));

  dartnb::dart_class<
      dart::dynamics::RevoluteJoint::Properties,
      dart::dynamics::GenericJoint<math::R1Space>::Properties,
      dart::dynamics::RevoluteJoint::UniqueProperties>(
      m, "RevoluteJointProperties")
      .def(dartnb::init<>())
      .def(
          dartnb::init<const dart::dynamics::GenericJoint<
              dart::math::R1Space>::Properties&>(),
          nb::arg("genericJointProperties"))
      .def(
          dartnb::init<
              const dart::dynamics::GenericJoint<
                  dart::math::R1Space>::Properties&,
              const dart::dynamics::RevoluteJoint::UniqueProperties&>(),
          nb::arg("genericJointProperties"),
          nb::arg("uniqueProperties"))
      .def_rw(
          "mAxis",
          &dart::dynamics::detail::RevoluteJointUniqueProperties::mAxis,
          dartnb::setterArgument(
              &dart::dynamics::detail::RevoluteJointUniqueProperties::mAxis));

  DARTPY_DEFINE_JOINT_COMMON_BASE(RevoluteJoint, R1Space)

  dartnb::dart_class<
      dart::dynamics::RevoluteJoint,
      dart::dynamics::detail::RevoluteJointBase>(m, "RevoluteJoint")
      .def(
          "hasRevoluteJointAspect",
          +[](const dart::dynamics::RevoluteJoint* self) -> bool {
            return self->hasRevoluteJointAspect();
          })
      .def(
          "setRevoluteJointAspect",
          +[](dart::dynamics::RevoluteJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::RevoluteJoint,
                  dart::dynamics::detail::RevoluteJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<1>>>::Aspect* aspect) {
            self->setRevoluteJointAspect(aspect);
          },
          nb::arg("aspect").none())
      .def(
          "removeRevoluteJointAspect",
          +[](dart::dynamics::RevoluteJoint* self) {
            self->removeRevoluteJointAspect();
          })
      .def(
          "releaseRevoluteJointAspect",
          +[](dart::dynamics::RevoluteJoint* self)
              -> std::unique_ptr<dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::RevoluteJoint,
                  dart::dynamics::detail::RevoluteJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<1>>>::Aspect> {
            return self->releaseRevoluteJointAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::RevoluteJoint* self,
              const dart::dynamics::RevoluteJoint::Properties& _properties) {
            self->setProperties(_properties);
          },
          nb::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::RevoluteJoint* self,
              const dart::dynamics::RevoluteJoint::UniqueProperties&
                  _properties) { self->setProperties(_properties); },
          nb::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::RevoluteJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::RevoluteJoint,
                  dart::dynamics::detail::RevoluteJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<1>>>::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          nb::arg("properties"))
      .def(
          "getRevoluteJointProperties",
          +[](const dart::dynamics::RevoluteJoint* self)
              -> dart::dynamics::RevoluteJoint::Properties {
            return self->getRevoluteJointProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::RevoluteJoint* self,
              const dart::dynamics::RevoluteJoint* _otherJoint) {
            self->copy(_otherJoint);
          },
          nb::arg("otherJoint").none())
      .def(
          "getType",
          +[](const dart::dynamics::RevoluteJoint* self) -> const std::string& {
            return self->getType();
          },
          nb::rv_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::RevoluteJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          nb::arg("index"))
      .def(
          "setAxis",
          +[](dart::dynamics::RevoluteJoint* self,
              const Eigen::Vector3d& _axis) { self->setAxis(_axis); },
          nb::arg("axis"))
      .def(
          "getAxis",
          +[](const dart::dynamics::RevoluteJoint* self)
              -> const Eigen::Vector3d& { return self->getAxis(); },
          nb::rv_policy::reference_internal)
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::RevoluteJoint* self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::Vector& positions)
              -> dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::JacobianMatrix {
            return self->getRelativeJacobianStatic(positions);
          },
          nb::arg("positions"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::RevoluteJoint::getStaticType();
          },
          nb::rv_policy::reference_internal);
}

} // namespace python
} // namespace dart
