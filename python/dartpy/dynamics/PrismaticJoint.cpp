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
#include "Joint.hpp"

namespace dart {
namespace python {

void PrismaticJoint(pybind11::module& m)
{
  ::pybind11::class_<dart::dynamics::PrismaticJoint::UniqueProperties>(
      m, "PrismaticJointUniqueProperties")
      .def(::pybind11::init<>())
      .def(::pybind11::init<const Eigen::Vector3d&>(), ::pybind11::arg("axis"));

  ::pybind11::class_<
      dart::dynamics::PrismaticJoint::Properties,
      dart::dynamics::GenericJoint<math::R1Space>::Properties,
      dart::dynamics::PrismaticJoint::UniqueProperties>(
      m, "PrismaticJointProperties")
      .def(::pybind11::init<>())
      .def(
          ::pybind11::init<const dart::dynamics::GenericJoint<
              dart::math::R1Space>::Properties&>(),
          ::pybind11::arg("genericJointProperties"))
      .def(
          ::pybind11::init<
              const dart::dynamics::GenericJoint<
                  dart::math::R1Space>::Properties&,
              const dart::dynamics::PrismaticJoint::UniqueProperties&>(),
          ::pybind11::arg("genericJointProperties"),
          ::pybind11::arg("revoluteProperties"))
      .def_readwrite(
          "mAxis",
          &dart::dynamics::detail::PrismaticJointUniqueProperties::mAxis);

  DARTPY_DEFINE_JOINT_COMMON_BASE(PrismaticJoint, R1Space)

  ::pybind11::class_<
      dart::dynamics::PrismaticJoint,
      dart::common::EmbedPropertiesOnTopOf<
          dart::dynamics::PrismaticJoint,
          dart::dynamics::detail::PrismaticJointUniqueProperties,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>>,
      std::shared_ptr<dart::dynamics::PrismaticJoint>>(m, "PrismaticJoint")
      .def(
          "hasPrismaticJointAspect",
          +[](const dart::dynamics::PrismaticJoint* self) -> bool {
            return self->hasPrismaticJointAspect();
          })
      .def(
          "setPrismaticJointAspect",
          +[](dart::dynamics::PrismaticJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::PrismaticJoint,
                  dart::dynamics::detail::PrismaticJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<1>>>::Aspect* aspect) {
            self->setPrismaticJointAspect(aspect);
          },
          ::pybind11::arg("aspect"))
      .def(
          "removePrismaticJointAspect",
          +[](dart::dynamics::PrismaticJoint* self) {
            self->removePrismaticJointAspect();
          })
      .def(
          "releasePrismaticJointAspect",
          +[](dart::dynamics::PrismaticJoint* self)
              -> std::unique_ptr<dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::PrismaticJoint,
                  dart::dynamics::detail::PrismaticJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<1>>>::Aspect> {
            return self->releasePrismaticJointAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::PrismaticJoint* self,
              const dart::dynamics::PrismaticJoint::Properties& _properties) {
            self->setProperties(_properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::PrismaticJoint* self,
              const dart::dynamics::PrismaticJoint::UniqueProperties&
                  _properties) { self->setProperties(_properties); },
          ::pybind11::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::PrismaticJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::PrismaticJoint,
                  dart::dynamics::detail::PrismaticJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<1>>>::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          ::pybind11::arg("properties"))
      .def(
          "getPrismaticJointProperties",
          +[](const dart::dynamics::PrismaticJoint* self)
              -> dart::dynamics::PrismaticJoint::Properties {
            return self->getPrismaticJointProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::PrismaticJoint* self,
              const dart::dynamics::PrismaticJoint* _otherJoint) {
            self->copy(_otherJoint);
          },
          ::pybind11::arg("otherJoint"))
      .def(
          "getType",
          +[](const dart::dynamics::PrismaticJoint* self)
              -> const std::string& { return self->getType(); },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::PrismaticJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          ::pybind11::arg("index"))
      .def(
          "setAxis",
          +[](dart::dynamics::PrismaticJoint* self,
              const Eigen::Vector3d& _axis) { self->setAxis(_axis); },
          ::pybind11::arg("axis"))
      .def(
          "getAxis",
          +[](const dart::dynamics::PrismaticJoint* self)
              -> const Eigen::Vector3d& { return self->getAxis(); },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::PrismaticJoint* self,
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
            return dart::dynamics::PrismaticJoint::getStaticType();
          },
          ::pybind11::return_value_policy::reference_internal);
}

} // namespace python
} // namespace dart
