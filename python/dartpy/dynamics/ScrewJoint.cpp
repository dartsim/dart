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
#include <dart/dynamics/ScrewJoint.hpp>

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

void ScrewJoint(nb::module_& m)
{
  dartnb::dart_class<dart::dynamics::ScrewJoint::UniqueProperties>(
      m, "ScrewJointUniqueProperties")
      .def(dartnb::init<>())
      .def(dartnb::init<const Eigen::Vector3d&>(), nb::arg("axis"))
      .def(
          dartnb::init<const Eigen::Vector3d&, double>(),
          nb::arg("axis"),
          nb::arg("pitch"));

  dartnb::dart_class<
      dart::dynamics::ScrewJoint::Properties,
      dart::dynamics::GenericJoint<math::R1Space>::Properties,
      dart::dynamics::ScrewJoint::UniqueProperties>(m, "ScrewJointProperties")
      .def(dartnb::init<>())
      .def(
          dartnb::init<const dart::dynamics::GenericJoint<
              dart::math::R1Space>::Properties&>(),
          nb::arg("genericJointProperties"))
      .def(
          dartnb::init<
              const dart::dynamics::GenericJoint<
                  dart::math::R1Space>::Properties&,
              const dart::dynamics::ScrewJoint::UniqueProperties&>(),
          nb::arg("genericJointProperties"),
          nb::arg("revoluteProperties"))
      .def_rw(
          "mAxis",
          &dart::dynamics::detail::ScrewJointUniqueProperties::mAxis,
          dartnb::setterArgument(
              &dart::dynamics::detail::ScrewJointUniqueProperties::mAxis))
      .def_rw(
          "mPitch",
          &dart::dynamics::detail::ScrewJointUniqueProperties::mPitch,
          dartnb::setterArgument(
              &dart::dynamics::detail::ScrewJointUniqueProperties::mPitch));

  DARTPY_DEFINE_JOINT_COMMON_BASE(ScrewJoint, R1Space)

  dartnb::dart_class<
      dart::dynamics::ScrewJoint,
      dart::common::EmbedPropertiesOnTopOf<
          dart::dynamics::ScrewJoint,
          dart::dynamics::detail::ScrewJointUniqueProperties,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>>>(
      m, "ScrewJoint")
      .def(
          "hasScrewJointAspect",
          +[](const dart::dynamics::ScrewJoint* self) -> bool {
            return self->hasScrewJointAspect();
          })
      .def(
          "setScrewJointAspect",
          +[](dart::dynamics::ScrewJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::ScrewJoint,
                  dart::dynamics::detail::ScrewJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<1>>>::Aspect* aspect) {
            self->setScrewJointAspect(aspect);
          },
          nb::arg("aspect").none())
      .def(
          "removeScrewJointAspect",
          +[](dart::dynamics::ScrewJoint* self) {
            self->removeScrewJointAspect();
          })
      .def(
          "releaseScrewJointAspect",
          +[](dart::dynamics::ScrewJoint* self)
              -> std::unique_ptr<dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::ScrewJoint,
                  dart::dynamics::detail::ScrewJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<1>>>::Aspect> {
            return self->releaseScrewJointAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::ScrewJoint* self,
              const dart::dynamics::ScrewJoint::Properties& _properties) {
            self->setProperties(_properties);
          },
          nb::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::ScrewJoint* self,
              const dart::dynamics::ScrewJoint::UniqueProperties& _properties) {
            self->setProperties(_properties);
          },
          nb::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::ScrewJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::ScrewJoint,
                  dart::dynamics::detail::ScrewJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<1>>>::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          nb::arg("properties"))
      .def(
          "getScrewJointProperties",
          +[](const dart::dynamics::ScrewJoint* self)
              -> dart::dynamics::ScrewJoint::Properties {
            return self->getScrewJointProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::ScrewJoint* self,
              const dart::dynamics::ScrewJoint* _otherJoint) {
            self->copy(_otherJoint);
          },
          nb::arg("otherJoint").none())
      .def(
          "getType",
          +[](const dart::dynamics::ScrewJoint* self) -> const std::string& {
            return self->getType();
          },
          nb::rv_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::ScrewJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          nb::arg("index"))
      .def(
          "setAxis",
          +[](dart::dynamics::ScrewJoint* self, const Eigen::Vector3d& _axis) {
            self->setAxis(_axis);
          },
          nb::arg("axis"))
      .def(
          "getAxis",
          +[](const dart::dynamics::ScrewJoint* self)
              -> const Eigen::Vector3d& { return self->getAxis(); },
          nb::rv_policy::reference_internal)
      .def(
          "setPitch",
          +[](dart::dynamics::ScrewJoint* self, double _pitch) {
            self->setPitch(_pitch);
          },
          nb::arg("pitch"))
      .def(
          "getPitch",
          +[](const dart::dynamics::ScrewJoint* self) -> double {
            return self->getPitch();
          })
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::ScrewJoint* self,
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
            return dart::dynamics::ScrewJoint::getStaticType();
          },
          nb::rv_policy::reference_internal);
}

} // namespace python
} // namespace dart
