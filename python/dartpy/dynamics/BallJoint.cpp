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

#include <dart/dart.hpp>

#include <eigen_geometry_pybind.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void BallJoint(py::module& m)
{
  ::py::class_<
      dart::dynamics::BallJoint::Properties,
      dart::dynamics::detail::GenericJointProperties<dart::math::SO3Space>>(
      m, "BallJointProperties")
      .def(::py::init<>())
      .def(
          ::py::init<const dart::dynamics::GenericJoint<
              dart::math::SO3Space>::Properties&>(),
          ::py::arg("properties"));

  ::py::class_<
      dart::dynamics::BallJoint,
      dart::dynamics::GenericJoint<dart::math::SO3Space>,
      std::shared_ptr<dart::dynamics::BallJoint>>(m, "BallJoint")
      .def(
          "getType",
          +[](const dart::dynamics::BallJoint* self) -> const std::string& {
            return self->getType();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::BallJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          ::py::arg("index"))
      .def(
          "getBallJointProperties",
          +[](const dart::dynamics::BallJoint* self)
              -> dart::dynamics::BallJoint::Properties {
            return self->getBallJointProperties();
          })
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::BallJoint* self,
              const math::Vector3d& _positions) -> math::Matrix<double, 6, 3> {
            return self->getRelativeJacobianStatic(_positions);
          },
          ::py::arg("positions"))
      .def(
          "getPositionDifferencesStatic",
          +[](const dart::dynamics::BallJoint* self,
              const math::Vector3d& _q2,
              const math::Vector3d& _q1) -> math::Vector3d {
            return self->getPositionDifferencesStatic(_q2, _q1);
          },
          ::py::arg("q2"),
          ::py::arg("q1"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::BallJoint::getStaticType();
          },
          ::py::return_value_policy::reference_internal)
      .def_static(
          "convertToPositions",
          +[](const math::Matrix3d& _tf) -> math::Vector3d {
            return dart::dynamics::BallJoint::convertToPositions(_tf);
          },
          ::py::arg("tf"))
      .def_static(
          "convertToTransform",
          +[](const math::Vector3d& _positions) -> math::Isometry3d {
            return dart::dynamics::BallJoint::convertToTransform(_positions);
          },
          ::py::arg("positions"))
      .def_static(
          "convertToRotation",
          +[](const math::Vector3d& _positions) -> math::Matrix3d {
            return dart::dynamics::BallJoint::convertToRotation(_positions);
          },
          ::py::arg("positions"));
}

} // namespace python
} // namespace dart
