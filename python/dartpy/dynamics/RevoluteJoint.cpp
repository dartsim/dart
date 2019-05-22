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
#include <eigen_geometry_pybind.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void RevoluteJoint(pybind11::module& m)
{
  ::pybind11::class_<dart::dynamics::RevoluteJoint::UniqueProperties>(
      m, "RevoluteJointUniqueProperties")
      .def(::pybind11::init<>())
      .def(::pybind11::init<const Eigen::Vector3d&>(), ::pybind11::arg("axis"));

  ::pybind11::class_<
      dart::dynamics::RevoluteJoint::Properties,
      dart::dynamics::GenericJoint<math::R1Space>::Properties,
      dart::dynamics::RevoluteJoint::UniqueProperties>(
      m, "RevoluteJointProperties")
      .def(::pybind11::init<>())
      .def(
          ::pybind11::init<const dart::dynamics::GenericJoint<
              dart::math::R1Space>::Properties&>(),
          ::pybind11::arg("genericJointProperties"))
      .def(
          ::pybind11::init<
              const dart::dynamics::GenericJoint<
                  dart::math::R1Space>::Properties&,
              const dart::dynamics::RevoluteJoint::UniqueProperties&>(),
          ::pybind11::arg("genericJointProperties"),
          ::pybind11::arg("revoluteProperties"))
      .def_readwrite(
          "mAxis",
          &dart::dynamics::detail::RevoluteJointUniqueProperties::mAxis);

  ::pybind11::class_<
      dart::common::SpecializedForAspect<dart::common::EmbeddedPropertiesAspect<
          dart::dynamics::RevoluteJoint,
          dart::dynamics::detail::RevoluteJointUniqueProperties>>,
      dart::common::Composite,
      std::shared_ptr<dart::common::SpecializedForAspect<
          dart::common::EmbeddedPropertiesAspect<
              dart::dynamics::RevoluteJoint,
              dart::dynamics::detail::RevoluteJointUniqueProperties>>>>(
      m,
      "SpecializedForAspect_EmbeddedPropertiesAspect_RevoluteJoint_"
      "RevoluteJointUniqueProperties")
      .def(::pybind11::init<>());

  ::pybind11::class_<
      dart::common::RequiresAspect<dart::common::EmbeddedPropertiesAspect<
          dart::dynamics::RevoluteJoint,
          dart::dynamics::detail::RevoluteJointUniqueProperties>>,
      dart::common::SpecializedForAspect<dart::common::EmbeddedPropertiesAspect<
          dart::dynamics::RevoluteJoint,
          dart::dynamics::detail::RevoluteJointUniqueProperties>>,
      std::shared_ptr<
          dart::common::RequiresAspect<dart::common::EmbeddedPropertiesAspect<
              dart::dynamics::RevoluteJoint,
              dart::dynamics::detail::RevoluteJointUniqueProperties>>>>(
      m,
      "RequiresAspect_EmbeddedPropertiesAspect_RevoluteJoint_"
      "RevoluteJointUniqueProperties")
      .def(::pybind11::init<>());

  ::pybind11::class_<
      dart::common::EmbedProperties<
          dart::dynamics::RevoluteJoint,
          dart::dynamics::detail::RevoluteJointUniqueProperties>,
      dart::common::RequiresAspect<dart::common::EmbeddedPropertiesAspect<
          dart::dynamics::RevoluteJoint,
          dart::dynamics::detail::RevoluteJointUniqueProperties>>,
      std::shared_ptr<dart::common::EmbedProperties<
          dart::dynamics::RevoluteJoint,
          dart::dynamics::detail::RevoluteJointUniqueProperties>>>(
      m, "EmbedProperties_RevoluteJoint_RevoluteJointUniqueProperties");

  ::pybind11::class_<
      dart::common::CompositeJoiner<
          dart::common::EmbedProperties<
              dart::dynamics::RevoluteJoint,
              dart::dynamics::detail::RevoluteJointUniqueProperties>,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>>,
      dart::common::EmbedProperties<
          dart::dynamics::RevoluteJoint,
          dart::dynamics::detail::RevoluteJointUniqueProperties>,
      dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>,
      std::shared_ptr<dart::common::CompositeJoiner<
          dart::common::EmbedProperties<
              dart::dynamics::RevoluteJoint,
              dart::dynamics::detail::RevoluteJointUniqueProperties>,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>>>>(
      m,
      "CompositeJoiner_EmbedProperties_RevoluteJoint_"
      "RevoluteJointUniqueProperties_GenericJoint_R1");

  ::pybind11::class_<
      dart::common::EmbedPropertiesOnTopOf<
          dart::dynamics::RevoluteJoint,
          dart::dynamics::detail::RevoluteJointUniqueProperties,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>>,
      dart::common::CompositeJoiner<
          dart::common::EmbedProperties<
              dart::dynamics::RevoluteJoint,
              dart::dynamics::detail::RevoluteJointUniqueProperties>,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>>,
      std::shared_ptr<dart::common::EmbedPropertiesOnTopOf<
          dart::dynamics::RevoluteJoint,
          dart::dynamics::detail::RevoluteJointUniqueProperties,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>>>>(
      m,
      "EmbedPropertiesOnTopOf_RevoluteJoint_RevoluteJointUniqueProperties_"
      "GenericJoint_R1");

  ::pybind11::class_<
      dart::dynamics::RevoluteJoint,
      dart::dynamics::detail::RevoluteJointBase,
      std::shared_ptr<dart::dynamics::RevoluteJoint>>(m, "RevoluteJoint")
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
          ::pybind11::arg("aspect"))
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
          ::pybind11::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::RevoluteJoint* self,
              const dart::dynamics::RevoluteJoint::UniqueProperties&
                  _properties) { self->setProperties(_properties); },
          ::pybind11::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::RevoluteJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::RevoluteJoint,
                  dart::dynamics::detail::RevoluteJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<1>>>::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          ::pybind11::arg("properties"))
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
          ::pybind11::arg("otherJoint"))
      .def(
          "getType",
          +[](const dart::dynamics::RevoluteJoint* self) -> const std::string& {
            return self->getType();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::RevoluteJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          ::pybind11::arg("index"))
      .def(
          "setAxis",
          +[](dart::dynamics::RevoluteJoint* self,
              const Eigen::Vector3d& _axis) { self->setAxis(_axis); },
          ::pybind11::arg("axis"))
      .def(
          "getAxis",
          +[](const dart::dynamics::RevoluteJoint* self)
              -> const Eigen::Vector3d& { return self->getAxis(); },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::RevoluteJoint* self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::Vector& positions)
              -> dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::JacobianMatrix {
            return self->getRelativeJacobianStatic(positions);
          },
          ::pybind11::arg("positions"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::RevoluteJoint::getStaticType();
          },
          ::pybind11::return_value_policy::reference_internal);
}

} // namespace python
} // namespace dart
