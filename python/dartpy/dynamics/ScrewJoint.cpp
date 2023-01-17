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

void ScrewJoint(py::module& m)
{
  ::py::class_<dart::dynamics::ScrewJoint::UniqueProperties>(
      m, "ScrewJointUniqueProperties")
      .def(::py::init<>())
      .def(::py::init<const math::Vector3d&>(), ::py::arg("axis"))
      .def(
          ::py::init<const math::Vector3d&, double>(),
          ::py::arg("axis"),
          ::py::arg("pitch"));

  ::py::class_<
      dart::dynamics::ScrewJoint::Properties,
      dart::dynamics::GenericJoint<math::R1Space>::Properties,
      dart::dynamics::ScrewJoint::UniqueProperties>(m, "ScrewJointProperties")
      .def(::py::init<>())
      .def(
          ::py::init<const dart::dynamics::GenericJoint<
              dart::math::R1Space>::Properties&>(),
          ::py::arg("genericJointProperties"))
      .def(
          ::py::init<
              const dart::dynamics::GenericJoint<
                  dart::math::R1Space>::Properties&,
              const dart::dynamics::ScrewJoint::UniqueProperties&>(),
          ::py::arg("genericJointProperties"),
          ::py::arg("revoluteProperties"))
      .def_readwrite(
          "mAxis", &dart::dynamics::detail::ScrewJointUniqueProperties::mAxis)
      .def_readwrite(
          "mPitch",
          &dart::dynamics::detail::ScrewJointUniqueProperties::mPitch);

  DARTPY_DEFINE_JOINT_COMMON_BASE(ScrewJoint, R1Space)

  ::py::class_<
      dart::dynamics::ScrewJoint,
      dart::common::EmbedPropertiesOnTopOf<
          dart::dynamics::ScrewJoint,
          dart::dynamics::detail::ScrewJointUniqueProperties,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>>,
      std::shared_ptr<dart::dynamics::ScrewJoint>>(m, "ScrewJoint")
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
          ::py::arg("aspect"))
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
          ::py::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::ScrewJoint* self,
              const dart::dynamics::ScrewJoint::UniqueProperties& _properties) {
            self->setProperties(_properties);
          },
          ::py::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::ScrewJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::ScrewJoint,
                  dart::dynamics::detail::ScrewJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<1>>>::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          ::py::arg("properties"))
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
          ::py::arg("otherJoint"))
      .def(
          "getType",
          +[](const dart::dynamics::ScrewJoint* self) -> const std::string& {
            return self->getType();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::ScrewJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          ::py::arg("index"))
      .def(
          "setAxis",
          +[](dart::dynamics::ScrewJoint* self, const math::Vector3d& _axis) {
            self->setAxis(_axis);
          },
          ::py::arg("axis"))
      .def(
          "getAxis",
          +[](const dart::dynamics::ScrewJoint* self) -> const math::Vector3d& {
            return self->getAxis();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "setPitch",
          +[](dart::dynamics::ScrewJoint* self, double _pitch) {
            self->setPitch(_pitch);
          },
          ::py::arg("pitch"))
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
          ::py::arg("positions"))
      .def_static(
          "getStaticType",
          +[]() -> const std::
                    string& {
                      return dart::dynamics::ScrewJoint::getStaticType();
                    },
          ::py::return_value_policy::reference_internal);
}

} // namespace python
} // namespace dart
