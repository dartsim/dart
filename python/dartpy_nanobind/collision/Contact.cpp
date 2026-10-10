// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/eigen.hpp"

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

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/Contact.hpp>

#include <Eigen/Core>

namespace dart {
namespace python {

void Contact(nb::module_& m)
{
  dartnb::dart_class<dart::collision::Contact>(m, "Contact")
      .def(dartnb::init<>())
      .def_static(
          "getNormalEpsilon",
          +[]() -> double {
            return dart::collision::Contact::getNormalEpsilon();
          })
      .def_static(
          "getNormalEpsilonSquared",
          +[]() -> double {
            return dart::collision::Contact::getNormalEpsilonSquared();
          })
      .def_static(
          "isZeroNormal",
          +[](const Eigen::Vector3d& normal) -> bool {
            return dart::collision::Contact::isZeroNormal(normal);
          },
          nb::arg("normal"))
      .def_static(
          "isNonZeroNormal",
          +[](const Eigen::Vector3d& normal) -> bool {
            return dart::collision::Contact::isNonZeroNormal(normal);
          },
          nb::arg("normal"))
      .def_rw(
          "point",
          &dart::collision::Contact::point,
          dartnb::setterArgument(&dart::collision::Contact::point))
      .def_rw(
          "normal",
          &dart::collision::Contact::normal,
          dartnb::setterArgument(&dart::collision::Contact::normal))
      .def_rw(
          "force",
          &dart::collision::Contact::force,
          dartnb::setterArgument(&dart::collision::Contact::force))
      .def_rw(
          "collisionObject1",
          &dart::collision::Contact::collisionObject1,
          dartnb::setterArgument(&dart::collision::Contact::collisionObject1))
      .def_rw(
          "collisionObject2",
          &dart::collision::Contact::collisionObject2,
          dartnb::setterArgument(&dart::collision::Contact::collisionObject2))
      .def_rw(
          "penetrationDepth",
          &dart::collision::Contact::penetrationDepth,
          dartnb::setterArgument(&dart::collision::Contact::penetrationDepth))
      .def_rw(
          "triID1",
          &dart::collision::Contact::triID1,
          dartnb::setterArgument(&dart::collision::Contact::triID1))
      .def_rw(
          "triID2",
          &dart::collision::Contact::triID2,
          dartnb::setterArgument(&dart::collision::Contact::triID2))
      .def_rw(
          "userData",
          &dart::collision::Contact::userData,
          dartnb::setterArgument(&dart::collision::Contact::userData));
}

} // namespace python
} // namespace dart
