// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/array.hpp"

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
#include <dart/dynamics/UniversalJoint.hpp>

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

void UniversalJoint(nb::module_& m)
{
  dartnb::dart_class<dart::dynamics::UniversalJoint::UniqueProperties>(
      m, "UniversalJointUniqueProperties")
      .def(dartnb::init<>());

  dartnb::dart_class<
      dart::dynamics::UniversalJoint::Properties,
      dart::dynamics::GenericJoint<math::R2Space>::Properties,
      dart::dynamics::UniversalJoint::UniqueProperties>(
      m, "UniversalJointProperties")
      .def(dartnb::init<>())
      .def(
          dartnb::init<const dart::dynamics::GenericJoint<
              dart::math::R2Space>::Properties&>(),
          nb::arg("genericJointProperties"))
      .def(
          dartnb::init<
              const dart::dynamics::GenericJoint<
                  dart::math::R2Space>::Properties&,
              const dart::dynamics::UniversalJoint::UniqueProperties&>(),
          nb::arg("genericJointProperties"),
          nb::arg("uniqueProperties"))
      .def_rw(
          "mAxis",
          &dart::dynamics::detail::UniversalJointUniqueProperties::mAxis,
          dartnb::setterArgument(
              &dart::dynamics::detail::UniversalJointUniqueProperties::mAxis));

  DARTPY_DEFINE_JOINT_COMMON_BASE(UniversalJoint, R2Space)

  dartnb::dart_class<
      dart::dynamics::UniversalJoint,
      dart::common::EmbedPropertiesOnTopOf<
          dart::dynamics::UniversalJoint,
          dart::dynamics::detail::UniversalJointUniqueProperties,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>>>(
      m, "UniversalJoint")
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
                      dart::math::RealVectorSpace<2>>>::Aspect* aspect) {
            self->setUniversalJointAspect(aspect);
          },
          nb::arg("aspect").none())
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
                      dart::math::RealVectorSpace<2>>>::Aspect> {
            return self->releaseUniversalJointAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::UniversalJoint* self,
              const dart::dynamics::UniversalJoint::Properties& _properties) {
            self->setProperties(_properties);
          },
          nb::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::UniversalJoint* self,
              const dart::dynamics::UniversalJoint::UniqueProperties&
                  _properties) { self->setProperties(_properties); },
          nb::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::UniversalJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::UniversalJoint,
                  dart::dynamics::detail::UniversalJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<2>>>::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          nb::arg("properties"))
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
          nb::arg("otherJoint").none())
      .def(
          "getType",
          +[](const dart::dynamics::UniversalJoint* self)
              -> const std::string& { return self->getType(); },
          nb::rv_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::UniversalJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          nb::arg("index"))
      .def(
          "setAxis1",
          +[](dart::dynamics::UniversalJoint* self,
              const Eigen::Vector3d& _axis) { self->setAxis1(_axis); },
          nb::arg("axis"))
      .def(
          "setAxis2",
          +[](dart::dynamics::UniversalJoint* self,
              const Eigen::Vector3d& _axis) { self->setAxis2(_axis); },
          nb::arg("axis"))
      .def(
          "getAxis1",
          +[](const dart::dynamics::UniversalJoint* self)
              -> const Eigen::Vector3d& { return self->getAxis1(); },
          nb::rv_policy::reference_internal)
      .def(
          "getAxis2",
          +[](const dart::dynamics::UniversalJoint* self)
              -> const Eigen::Vector3d& { return self->getAxis2(); },
          nb::rv_policy::reference_internal)
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::UniversalJoint* self,
              const Eigen::Vector2d& _positions)
              -> Eigen::Matrix<double, 6, 2> {
            return self->getRelativeJacobianStatic(_positions);
          },
          nb::arg("positions"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::UniversalJoint::getStaticType();
          },
          nb::rv_policy::reference_internal);
}

} // namespace python
} // namespace dart
