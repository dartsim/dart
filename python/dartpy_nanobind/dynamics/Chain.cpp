// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <nanobind/stl/vector.h>

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

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Chain.hpp>
#include <dart/dynamics/Linkage.hpp>
#include <dart/dynamics/MetaSkeleton.hpp>

#include <memory>
#include <string>
#include <vector>

namespace dart {
namespace python {

void Chain(nb::module_& m)
{
  dartnb::dart_class<dart::dynamics::Chain, dart::dynamics::Linkage>(m, "Chain")
      .def(
          dartnb::factory(
              +[](const dart::dynamics::Chain::Criteria& criteria)
                  -> dart::dynamics::ChainPtr {
                return dart::dynamics::Chain::create(criteria);
              }),
          nb::arg("criteria"))
      .def(
          dartnb::factory(
              +[](const dart::dynamics::Chain::Criteria& criteria,
                  const std::string& name) -> dart::dynamics::ChainPtr {
                return dart::dynamics::Chain::create(criteria, name);
              }),
          nb::arg("criteria"),
          nb::arg("name"))
      .def(
          dartnb::factory(
              +[](dart::dynamics::BodyNode* start,
                  dart::dynamics::BodyNode* target)
                  -> dart::dynamics::ChainPtr {
                return dart::dynamics::Chain::create(start, target);
              }),
          nb::arg("start").none(),
          nb::arg("target").none())
      .def(
          dartnb::factory(
              +[](dart::dynamics::BodyNode* start,
                  dart::dynamics::BodyNode* target,
                  const std::string& name) -> dart::dynamics::ChainPtr {
                return dart::dynamics::Chain::create(start, target, name);
              }),
          nb::arg("start").none(),
          nb::arg("target").none(),
          nb::arg("name"))
      .def(
          dartnb::factory(
              +[](dart::dynamics::BodyNode* start,
                  dart::dynamics::BodyNode* target,
                  bool includeUpstreamParentJoint) -> dart::dynamics::ChainPtr {
                if (includeUpstreamParentJoint)
                  return dart::dynamics::Chain::create(
                      start,
                      target,
                      dart::dynamics::Chain::IncludeUpstreamParentJoint);
                else
                  return dart::dynamics::Chain::create(start, target);
              }),
          nb::arg("start").none(),
          nb::arg("target").none(),
          nb::arg("includeUpstreamParentJoint"))
      .def(
          dartnb::factory(
              +[](dart::dynamics::BodyNode* start,
                  dart::dynamics::BodyNode* target,
                  bool includeUpstreamParentJoint,
                  const std::string& name) -> dart::dynamics::ChainPtr {
                if (includeUpstreamParentJoint)
                  return dart::dynamics::Chain::create(
                      start,
                      target,
                      dart::dynamics::Chain::IncludeUpstreamParentJoint,
                      name);
                else
                  return dart::dynamics::Chain::create(start, target, name);
              }),
          nb::arg("start").none(),
          nb::arg("target").none(),
          nb::arg("includeUpstreamParentJoint"),
          nb::arg("name"))
      .def(
          "cloneChain",
          +[](const dart::dynamics::Chain* self) -> dart::dynamics::ChainPtr {
            return self->cloneChain();
          })
      .def(
          "cloneChain",
          +[](const dart::dynamics::Chain* self,
              const std::string& cloneName) -> dart::dynamics::ChainPtr {
            return self->cloneChain(cloneName);
          },
          nb::arg("cloneName"))
      .def(
          "cloneMetaSkeleton",
          +[](const dart::dynamics::Chain* self,
              const std::string& cloneName) -> dart::dynamics::MetaSkeletonPtr {
            return self->cloneMetaSkeleton(cloneName);
          },
          nb::arg("cloneName"))
      .def(
          "isStillChain", +[](const dart::dynamics::Chain* self) -> bool {
            return self->isStillChain();
          });

  dartnb::dart_class<dart::dynamics::Chain::Criteria>(m, "ChainCriteria")
      .def(
          dartnb::init<dart::dynamics::BodyNode*, dart::dynamics::BodyNode*>(),
          nb::arg("start").none(),
          nb::arg("target").none())
      .def(
          dartnb::init<
              dart::dynamics::BodyNode*,
              dart::dynamics::BodyNode*,
              bool>(),
          nb::arg("start").none(),
          nb::arg("target").none(),
          nb::arg("includeBoth"))
      .def(
          "satisfy",
          +[](const dart::dynamics::Chain::Criteria* self)
              -> std::vector<dart::dynamics::BodyNode*> {
            return self->satisfy();
          })
      .def(
          "convert",
          +[](const dart::dynamics::Chain::Criteria* self)
              -> dart::dynamics::Linkage::Criteria { return self->convert(); })
      .def_static(
          "static_convert",
          +[](const dart::dynamics::Linkage::Criteria& criteria)
              -> dart::dynamics::Chain::Criteria {
            return dart::dynamics::Chain::Criteria::convert(criteria);
          },
          nb::arg("criteria"))
      .def_rw(
          "mStart",
          &dart::dynamics::Chain::Criteria::mStart,
          dartnb::setterArgument(&dart::dynamics::Chain::Criteria::mStart))
      .def_rw(
          "mTarget",
          &dart::dynamics::Chain::Criteria::mTarget,
          dartnb::setterArgument(&dart::dynamics::Chain::Criteria::mTarget))
      .def_rw(
          "mIncludeUpstreamParentJoint",
          &dart::dynamics::Chain::Criteria::mIncludeUpstreamParentJoint,
          dartnb::setterArgument(
              &dart::dynamics::Chain::Criteria::mIncludeUpstreamParentJoint));
}

} // namespace python
} // namespace dart
