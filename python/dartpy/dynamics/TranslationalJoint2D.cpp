/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include <dart/dart.hpp>
#include <eigen_geometry_pybind.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include "Joint.hpp"

namespace py = pybind11;

namespace dart {
namespace python {

void TranslationalJoint2D(py::module& m)
{
  ::py::class_<dart::dynamics::TranslationalJoint2D::UniqueProperties>(
      m, "TranslationalJoint2DUniqueProperties")
      .def(::py::init<>());

  ::py::class_<
      dart::dynamics::TranslationalJoint2D::Properties,
      dart::dynamics::GenericJoint<math::R2Space>::Properties,
      dart::dynamics::TranslationalJoint2D::UniqueProperties>(
      m, "TranslationalJoint2DProperties")
      .def(::py::init<>())
      .def(
          ::py::init<const dart::dynamics::GenericJoint<
              dart::math::R2Space>::Properties&>(),
          ::py::arg("genericJointProperties"))
      .def(
          ::py::init<
              const dart::dynamics::GenericJoint<
                  dart::math::R2Space>::Properties&,
              const dart::dynamics::TranslationalJoint2D::UniqueProperties&>(),
          ::py::arg("genericJointProperties"),
          ::py::arg("uniqueProperties"));

  DARTPY_DEFINE_JOINT_COMMON_BASE(TranslationalJoint2D, R2Space)

  ::py::class_<
      dart::dynamics::TranslationalJoint2D,
      dart::common::EmbedPropertiesOnTopOf<
          dart::dynamics::TranslationalJoint2D,
          dart::dynamics::detail::TranslationalJoint2DUniqueProperties,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2> > >,
      std::shared_ptr<dart::dynamics::TranslationalJoint2D> >(
      m, "TranslationalJoint2D")
      .def(
          "hasTranslationalJoint2DAspect",
          +[](const dart::dynamics::TranslationalJoint2D* self) -> bool {
            return self->hasTranslationalJoint2DAspect();
          })
      .def(
          "setTranslationalJoint2DAspect",
          +[](dart::dynamics::TranslationalJoint2D* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::TranslationalJoint2D,
                  dart::dynamics::detail::TranslationalJoint2DUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<2> > >::Aspect* aspect) {
            self->setTranslationalJoint2DAspect(aspect);
          },
          ::py::arg("aspect"))
      .def(
          "removeTranslationalJoint2DAspect",
          +[](dart::dynamics::TranslationalJoint2D* self) {
            self->removeTranslationalJoint2DAspect();
          })
      .def(
          "releaseTranslationalJoint2DAspect",
          +[](dart::dynamics::TranslationalJoint2D* self)
              -> std::unique_ptr<dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::TranslationalJoint2D,
                  dart::dynamics::detail::TranslationalJoint2DUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<2> > >::Aspect> {
            return self->releaseTranslationalJoint2DAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::TranslationalJoint2D* self,
              const dart::dynamics::TranslationalJoint2D::Properties&
                  properties) { self->setProperties(properties); },
          ::py::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::TranslationalJoint2D* self,
              const dart::dynamics::TranslationalJoint2D::UniqueProperties&
                  properties) { self->setProperties(properties); },
          ::py::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::TranslationalJoint2D* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::TranslationalJoint2D,
                  dart::dynamics::detail::TranslationalJoint2DUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<2> > >::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          ::py::arg("properties"))
      .def(
          "getTranslationalJoint2DProperties",
          +[](const dart::dynamics::TranslationalJoint2D* self)
              -> dart::dynamics::TranslationalJoint2D::Properties {
            return self->getTranslationalJoint2DProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::TranslationalJoint2D* self,
              const dart::dynamics::TranslationalJoint2D* otherJoint) {
            self->copy(otherJoint);
          },
          ::py::arg("otherJoint"))
      .def(
          "getType",
          +[](const dart::dynamics::TranslationalJoint2D* self)
              -> const std::string& { return self->getType(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::TranslationalJoint2D* self,
              std::size_t index) -> bool { return self->isCyclic(index); },
          ::py::arg("index"))
      .def(
          "setXYPlane",
          +[](dart::dynamics::TranslationalJoint2D* self) {
            self->setXYPlane();
          })
      .def(
          "setXYPlane",
          +[](dart::dynamics::TranslationalJoint2D* self, bool renameDofs) {
            self->setXYPlane(renameDofs);
          },
          ::py::arg("renameDofs"))
      .def(
          "setYZPlane",
          +[](dart::dynamics::TranslationalJoint2D* self) {
            self->setYZPlane();
          })
      .def(
          "setYZPlane",
          +[](dart::dynamics::TranslationalJoint2D* self, bool renameDofs) {
            self->setYZPlane(renameDofs);
          },
          ::py::arg("renameDofs"))
      .def(
          "setZXPlane",
          +[](dart::dynamics::TranslationalJoint2D* self) {
            self->setZXPlane();
          })
      .def(
          "setZXPlane",
          +[](dart::dynamics::TranslationalJoint2D* self, bool renameDofs) {
            self->setZXPlane(renameDofs);
          },
          ::py::arg("renameDofs"))
      .def(
          "setArbitraryPlane",
          +[](dart::dynamics::TranslationalJoint2D* self,
              const Eigen::Vector3d& transAxis1,
              const Eigen::Vector3d& transAxis2) {
            self->setArbitraryPlane(transAxis1, transAxis2);
          },
          ::py::arg("transAxis1"),
          ::py::arg("transAxis2"))
      .def(
          "setArbitraryPlane",
          +[](dart::dynamics::TranslationalJoint2D* self,
              const Eigen::Vector3d& transAxis1,
              const Eigen::Vector3d& transAxis2,
              bool renameDofs) {
            self->setArbitraryPlane(transAxis1, transAxis2, renameDofs);
          },
          ::py::arg("transAxis1"),
          ::py::arg("transAxis2"),
          ::py::arg("renameDofs"))
      .def(
          "getPlaneType",
          +[](const dart::dynamics::TranslationalJoint2D* self)
              -> dart::dynamics::TranslationalJoint2D::PlaneType {
            return self->getPlaneType();
          })
      .def(
          "getTranslationalAxis1",
          +[](const dart::dynamics::TranslationalJoint2D* self)
              -> Eigen::Vector3d { return self->getTranslationalAxis1(); })
      .def(
          "getTranslationalAxis2",
          +[](const dart::dynamics::TranslationalJoint2D* self)
              -> Eigen::Vector3d { return self->getTranslationalAxis2(); })
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::TranslationalJoint2D* self,
              const Eigen::Vector2d& positions) -> Eigen::Matrix<double, 6, 2> {
            return self->getRelativeJacobianStatic(positions);
          },
          ::py::arg("positions"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::TranslationalJoint2D::getStaticType();
          },
          ::py::return_value_policy::reference_internal);
}

} // namespace python
} // namespace dart
