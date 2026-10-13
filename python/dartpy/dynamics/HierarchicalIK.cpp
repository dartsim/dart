// clang-format off
#include "detail/dart_nb.hpp"
#include "detail/hierarchical_ik.hpp"
// clang-format on

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

#include <dart/dynamics/Skeleton.hpp>

#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <dart/optimizer/Solver.hpp>

namespace dart {
namespace python {

void HierarchicalIK(nb::module_& m)
{
  using IK = dynamics::HierarchicalIK;
  dartnb::dart_class<IK, common::Subject>(m, "HierarchicalIK")
      .def(
          "solveAndApply",
          nb::overload_cast<bool>(&IK::solveAndApply),
          nb::arg("allowIncompleteResult") = true)
      .def("setObjective", &IK::setObjective, nb::arg("objective").none())
      .def("getObjective", nb::overload_cast<>(&IK::getObjective))
      .def(
          "setNullSpaceObjective",
          &IK::setNullSpaceObjective,
          nb::arg("nsObjective").none())
      .def(
          "getNullSpaceObjective",
          nb::overload_cast<>(&IK::getNullSpaceObjective))
      .def("getProblem", nb::overload_cast<>(&IK::getProblem))
      .def("setSolver", &IK::setSolver, nb::arg("solver").none())
      .def("getSolver", nb::overload_cast<>(&IK::getSolver))
      .def("getSkeleton", nb::overload_cast<>(&IK::getSkeleton))
      .def("refreshIKHierarchy", &IK::refreshIKHierarchy)
      .def(
          "clone",
          [](const IK& self, const dynamics::SkeletonPtr& skeleton) {
            return dartnb::hierarchicalIKInstance(self.clone(skeleton));
          },
          nb::arg("newSkeleton"),
          nb::sig("def clone(self, newSkeleton: dartpy.dynamics.Skeleton) -> "
                  "dartpy.dynamics.WholeBodyIK"));

  using WholeBody = dynamics::WholeBodyIK;
  dartnb::dart_class<WholeBody, IK>(m, "WholeBodyIK")
      .def(
          dartnb::factory([](const dynamics::SkeletonPtr& skeleton) {
            return dartnb::HierarchicalIKOwner<WholeBody>{
                WholeBody::create(skeleton)};
          }),
          nb::arg("skeleton"))
      .def_static(
          "create",
          [](const dynamics::SkeletonPtr& skeleton) {
            return dartnb::hierarchicalIKInstance(WholeBody::create(skeleton));
          },
          nb::arg("skeleton"),
          nb::sig("def create(skeleton: dartpy.dynamics.Skeleton) -> "
                  "dartpy.dynamics.WholeBodyIK"))
      .def(
          "cloneWholeBodyIK",
          [](const WholeBody& self, const dynamics::SkeletonPtr& skeleton) {
            return dartnb::hierarchicalIKInstance(
                self.cloneWholeBodyIK(skeleton));
          },
          nb::arg("newSkeleton"),
          nb::sig("def cloneWholeBodyIK(self, newSkeleton: "
                  "dartpy.dynamics.Skeleton) -> dartpy.dynamics.WholeBodyIK"));
}

} // namespace python
} // namespace dart
