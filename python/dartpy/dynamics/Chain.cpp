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
#include <pybind11/pybind11.h>
#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

namespace py = pybind11;

namespace dart {
namespace python {

void Chain(py::module& m)
{
  ::py::class_<
      dart::dynamics::Chain,
      dart::dynamics::Linkage,
      std::shared_ptr<dart::dynamics::Chain>>(m, "Chain")
      .def(
          ::py::init(
              +[](const dart::dynamics::Chain::Criteria& criteria)
                  -> dart::dynamics::ChainPtr {
                return dart::dynamics::Chain::create(criteria);
              }),
          ::py::arg("criteria"))
      .def(
          ::py::init(
              +[](const dart::dynamics::Chain::Criteria& criteria,
                  const std::string& name) -> dart::dynamics::ChainPtr {
                return dart::dynamics::Chain::create(criteria, name);
              }),
          ::py::arg("criteria"),
          ::py::arg("name"))
      .def(
          ::py::init(
              +[](dart::dynamics::BodyNode* start,
                  dart::dynamics::BodyNode* target)
                  -> dart::dynamics::ChainPtr {
                return dart::dynamics::Chain::create(start, target);
              }),
          ::py::arg("start"),
          ::py::arg("target"))
      .def(
          ::py::init(
              +[](dart::dynamics::BodyNode* start,
                  dart::dynamics::BodyNode* target,
                  const std::string& name) -> dart::dynamics::ChainPtr {
                return dart::dynamics::Chain::create(start, target, name);
              }),
          ::py::arg("start"),
          ::py::arg("target"),
          ::py::arg("name"))
      .def(
          ::py::init(
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
          ::py::arg("start"),
          ::py::arg("target"),
          ::py::arg("includeUpstreamParentJoint"))
      .def(
          ::py::init(
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
          ::py::arg("start"),
          ::py::arg("target"),
          ::py::arg("includeUpstreamParentJoint"),
          ::py::arg("name"))
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
          ::py::arg("cloneName"))
      .def(
          "cloneMetaSkeleton",
          +[](const dart::dynamics::Chain* self,
              const std::string& cloneName) -> dart::dynamics::MetaSkeletonPtr {
            return self->cloneMetaSkeleton(cloneName);
          },
          ::py::arg("cloneName"))
      .def("isStillChain", +[](const dart::dynamics::Chain* self) -> bool {
        return self->isStillChain();
      });

  ::py::class_<dart::dynamics::Chain::Criteria>(m, "ChainCriteria")
      .def(
          ::py::init<dart::dynamics::BodyNode*, dart::dynamics::BodyNode*>(),
          ::py::arg("start"),
          ::py::arg("target"))
      .def(
          ::py::init<
              dart::dynamics::BodyNode*,
              dart::dynamics::BodyNode*,
              bool>(),
          ::py::arg("start"),
          ::py::arg("target"),
          ::py::arg("includeBoth"))
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
          ::py::arg("criteria"))
      .def_readwrite("mStart", &dart::dynamics::Chain::Criteria::mStart)
      .def_readwrite("mTarget", &dart::dynamics::Chain::Criteria::mTarget)
      .def_readwrite(
          "mIncludeUpstreamParentJoint",
          &dart::dynamics::Chain::Criteria::mIncludeUpstreamParentJoint);
}

} // namespace python
} // namespace dart
