/*
 * Copyright (c) 2011-2021, The DART development contributors
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
#include <pybind11/pybind11.h>
#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

namespace py = pybind11;

namespace dart {
namespace python {

void Linkage(py::module& m)
{
  ::py::class_<
      dart::dynamics::Linkage,
      dart::dynamics::ReferentialSkeleton,
      std::shared_ptr<dart::dynamics::Linkage>>(m, "Linkage")
      .def(
          ::py::init(
              +[](const dart::dynamics::Linkage::Criteria& criteria)
                  -> dart::dynamics::LinkagePtr {
                return dart::dynamics::Linkage::create(criteria);
              }),
          ::py::arg("criteria"))
      .def(
          ::py::init(
              +[](const dart::dynamics::Linkage::Criteria& criteria,
                  const std::string& name) -> dart::dynamics::LinkagePtr {
                return dart::dynamics::Linkage::create(criteria, name);
              }),
          ::py::arg("criteria"),
          ::py::arg("name"))
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
          ::py::arg("cloneName"))
      .def(
          "cloneMetaSkeleton",
          +[](const dart::dynamics::Linkage* self,
              const std::string& cloneName) -> dart::dynamics::MetaSkeletonPtr {
            return self->cloneMetaSkeleton(cloneName);
          },
          ::py::arg("cloneName"))
      .def(
          "isAssembled",
          +[](const dart::dynamics::Linkage* self) -> bool {
            return self->isAssembled();
          })
      .def(
          "reassemble",
          +[](dart::dynamics::Linkage* self) { self->reassemble(); })
      .def("satisfyCriteria", +[](dart::dynamics::Linkage* self) {
        self->satisfyCriteria();
      });

  ::py::class_<dart::dynamics::Linkage::Criteria>(m, "LinkageCriteria")
      .def(
          "satisfy",
          +[](const dart::dynamics::Linkage::Criteria* self)
              -> std::vector<dart::dynamics::BodyNode*> {
            return self->satisfy();
          })
      .def_readwrite("mStart", &dart::dynamics::Linkage::Criteria::mStart)
      .def_readwrite("mTargets", &dart::dynamics::Linkage::Criteria::mTargets)
      .def_readwrite(
          "mTerminals", &dart::dynamics::Linkage::Criteria::mTerminals);

  ::py::enum_<dart::dynamics::Linkage::Criteria::ExpansionPolicy>(
      m.attr("LinkageCriteria"), "ExpansionPolicy")
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

  ::py::class_<dart::dynamics::Linkage::Criteria::Terminal>(
      m.attr("LinkageCriteria"), "Terminal")
      .def(::py::init<>())
      .def(::py::init<dart::dynamics::BodyNode*>(), ::py::arg("terminal"))
      .def(
          ::py::init<dart::dynamics::BodyNode*, bool>(),
          ::py::arg("terminal"),
          ::py::arg("inclusive"))
      .def_readwrite(
          "mTerminal", &dart::dynamics::Linkage::Criteria::Terminal::mTerminal)
      .def_readwrite(
          "mInclusive",
          &dart::dynamics::Linkage::Criteria::Terminal::mInclusive);

  ::py::class_<dart::dynamics::Linkage::Criteria::Target>(
      m.attr("LinkageCriteria"), "Target")
      .def(::py::init<>())
      .def(::py::init<dart::dynamics::BodyNode*>(), ::py::arg("target"))
      .def(
          ::py::init<
              dart::dynamics::BodyNode*,
              dart::dynamics::Linkage::Criteria::ExpansionPolicy>(),
          ::py::arg("target"),
          ::py::arg("policy"))
      .def(
          ::py::init<
              dart::dynamics::BodyNode*,
              dart::dynamics::Linkage::Criteria::ExpansionPolicy,
              bool>(),
          ::py::arg("target"),
          ::py::arg("policy"),
          ::py::arg("chain"))
      .def_readwrite("mNode", &dart::dynamics::Linkage::Criteria::Target::mNode)
      .def_readwrite(
          "mPolicy", &dart::dynamics::Linkage::Criteria::Target::mPolicy)
      .def_readwrite(
          "mChain", &dart::dynamics::Linkage::Criteria::Target::mChain);
}

} // namespace python
} // namespace dart
