/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include <dart/io/io.hpp>

#include <dart/dart.hpp>

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void SdfParser(py::module& m)
{
  auto sm = m.def_submodule("SdfParser");

  ::py::enum_<io::SdfParser::RootJointType>(sm, "RootJointType")
      .value("FLOATING", io::SdfParser::RootJointType::FLOATING)
      .value("FIXED", io::SdfParser::RootJointType::FIXED);

  ::py::class_<io::SdfParser::Options>(sm, "Options")
      .def(
          ::py::init<
              common::ResourceRetrieverPtr,
              io::SdfParser::RootJointType>(),
          ::py::arg("resourceRetriever") = nullptr,
          ::py::arg("defaultRootJointType")
          = io::SdfParser::RootJointType::FLOATING)
      .def_readwrite(
          "mResourceRetriever", &io::SdfParser::Options::mResourceRetriever)
      .def_readwrite(
          "mDefaultRootJointType",
          &io::SdfParser::Options::mDefaultRootJointType);

  sm.def(
      "readWorld",
      +[](const common::Uri& uri, const common::ResourceRetrieverPtr& retriever)
          -> simulation::WorldPtr {
        DART_SUPPRESS_DEPRECATED_BEGIN
        return io::SdfParser::readWorld(uri, retriever);
        DART_SUPPRESS_DEPRECATED_END
      },
      ::py::arg("uri"),
      ::py::arg("retriever"));
  sm.def(
      "readWorld",
      ::py::overload_cast<const common::Uri&, const io::SdfParser::Options&>(
          &io::SdfParser::readWorld),
      ::py::arg("uri"),
      ::py::arg("options") = io::SdfParser::Options());
  sm.def(
      "readSkeleton",
      +[](const common::Uri& uri, const common::ResourceRetrieverPtr& retriever)
          -> dynamics::SkeletonPtr {
        DART_SUPPRESS_DEPRECATED_BEGIN
        return io::SdfParser::readSkeleton(uri, retriever);
        DART_SUPPRESS_DEPRECATED_END
      },
      ::py::arg("uri"),
      ::py::arg("retriever"));
  sm.def(
      "readSkeleton",
      ::py::overload_cast<const common::Uri&, const io::SdfParser::Options&>(
          &io::SdfParser::readSkeleton),
      ::py::arg("uri"),
      ::py::arg("options") = io::SdfParser::Options());
}

} // namespace python
} // namespace dart
