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
#include <dart/dynamics/Linkage.hpp>
#include <dart/dynamics/MetaSkeleton.hpp>
#include <dart/dynamics/ReferentialSkeleton.hpp>

#include <memory>
#include <string>
#include <vector>

namespace dart {
namespace python {

void Linkage(nb::module_& m)
{
  dartnb::dart_class<
      dart::dynamics::Linkage,
      dart::dynamics::ReferentialSkeleton>(m, "Linkage")
      .def(
          dartnb::factory(
              +[](const dart::dynamics::Linkage::Criteria& criteria)
                  -> dart::dynamics::LinkagePtr {
                return dart::dynamics::Linkage::create(criteria);
              }),
          nb::arg("criteria"))
      .def(
          dartnb::factory(
              +[](const dart::dynamics::Linkage::Criteria& criteria,
                  const std::string& name) -> dart::dynamics::LinkagePtr {
                return dart::dynamics::Linkage::create(criteria, name);
              }),
          nb::arg("criteria"),
          nb::arg("name"))
      .def(
          "cloneLinkage",
          +[](const dart::dynamics::Linkage* self)
              -> dart::dynamics::LinkagePtr { return self->cloneLinkage(); })
      .def(
          "cloneLinkage",
          +[](const dart::dynamics::Linkage* self,
              const std::string& cloneName) -> dart::dynamics::LinkagePtr {
            return self->cloneLinkage(cloneName);
          },
          nb::arg("cloneName"))
      .def(
          "cloneMetaSkeleton",
          +[](const dart::dynamics::Linkage* self,
              const std::string& cloneName) -> dart::dynamics::MetaSkeletonPtr {
            return self->cloneMetaSkeleton(cloneName);
          },
          nb::arg("cloneName"))
      .def(
          "isAssembled",
          +[](const dart::dynamics::Linkage* self) -> bool {
            return self->isAssembled();
          })
      .def(
          "reassemble",
          +[](dart::dynamics::Linkage* self) { self->reassemble(); })
      .def(
          "satisfyCriteria",
          +[](dart::dynamics::Linkage* self) { self->satisfyCriteria(); });

  dartnb::dart_class<dart::dynamics::Linkage::Criteria>(m, "LinkageCriteria")
      .def(
          "satisfy",
          +[](const dart::dynamics::Linkage::Criteria* self)
              -> std::vector<dart::dynamics::BodyNode*> {
            return self->satisfy();
          })
      .def_rw(
          "mStart",
          &dart::dynamics::Linkage::Criteria::mStart,
          dartnb::setterArgument(&dart::dynamics::Linkage::Criteria::mStart))
      .def_rw(
          "mTargets",
          &dart::dynamics::Linkage::Criteria::mTargets,
          dartnb::setterArgument(&dart::dynamics::Linkage::Criteria::mTargets))
      .def_rw(
          "mTerminals",
          &dart::dynamics::Linkage::Criteria::mTerminals,
          dartnb::setterArgument(
              &dart::dynamics::Linkage::Criteria::mTerminals));

  nb::enum_<dart::dynamics::Linkage::Criteria::ExpansionPolicy>(
      m.attr("LinkageCriteria"), "ExpansionPolicy", nb::is_arithmetic())
      .value(
          "INCLUDE",
          dart::dynamics::Linkage::Criteria::ExpansionPolicy::INCLUDE)
      .value(
          "EXCLUDE",
          dart::dynamics::Linkage::Criteria::ExpansionPolicy::EXCLUDE)
      .value(
          "DOWNSTREAM",
          dart::dynamics::Linkage::Criteria::ExpansionPolicy::DOWNSTREAM)
      .value(
          "UPSTREAM",
          dart::dynamics::Linkage::Criteria::ExpansionPolicy::UPSTREAM)
      .export_values();

  dartnb::dart_class<dart::dynamics::Linkage::Criteria::Terminal>(
      m.attr("LinkageCriteria"), "Terminal")
      .def(dartnb::init<>())
      .def(
          dartnb::init<dart::dynamics::BodyNode*>(), nb::arg("terminal").none())
      .def(
          dartnb::init<dart::dynamics::BodyNode*, bool>(),
          nb::arg("terminal").none(),
          nb::arg("inclusive"))
      .def_rw(
          "mTerminal",
          &dart::dynamics::Linkage::Criteria::Terminal::mTerminal,
          dartnb::setterArgument(
              &dart::dynamics::Linkage::Criteria::Terminal::mTerminal))
      .def_rw(
          "mInclusive",
          &dart::dynamics::Linkage::Criteria::Terminal::mInclusive,
          dartnb::setterArgument(
              &dart::dynamics::Linkage::Criteria::Terminal::mInclusive));

  dartnb::dart_class<dart::dynamics::Linkage::Criteria::Target>(
      m.attr("LinkageCriteria"), "Target")
      .def(dartnb::init<>())
      .def(dartnb::init<dart::dynamics::BodyNode*>(), nb::arg("target").none())
      .def(
          dartnb::init<
              dart::dynamics::BodyNode*,
              dart::dynamics::Linkage::Criteria::ExpansionPolicy>(),
          nb::arg("target").none(),
          nb::arg("policy"))
      .def(
          dartnb::init<
              dart::dynamics::BodyNode*,
              dart::dynamics::Linkage::Criteria::ExpansionPolicy,
              bool>(),
          nb::arg("target").none(),
          nb::arg("policy"),
          nb::arg("chain"))
      .def_rw(
          "mNode",
          &dart::dynamics::Linkage::Criteria::Target::mNode,
          dartnb::setterArgument(
              &dart::dynamics::Linkage::Criteria::Target::mNode))
      .def_rw(
          "mPolicy",
          &dart::dynamics::Linkage::Criteria::Target::mPolicy,
          dartnb::setterArgument(
              &dart::dynamics::Linkage::Criteria::Target::mPolicy))
      .def_rw(
          "mChain",
          &dart::dynamics::Linkage::Criteria::Target::mChain,
          dartnb::setterArgument(
              &dart::dynamics::Linkage::Criteria::Target::mChain));
}

} // namespace python
} // namespace dart
