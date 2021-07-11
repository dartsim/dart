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

namespace py = pybind11;

namespace dart {
namespace python {

void DartLoader(py::module& m) {
  ::py::class_<dart::io::DartLoader>(m, "DartLoader")
      .def(::py::init<>())
      .def(
          "addPackageDirectory",
          +[](dart::io::DartLoader* self,
              const std::string& _packageName,
              const std::string& _packageDirectory) -> void {
            return self->addPackageDirectory(_packageName, _packageDirectory);
          },
          ::py::arg("packageName"),
          ::py::arg("packageDirectory"))
      .def(
          "parseSkeleton",
          +[](dart::io::DartLoader* self,
              const dart::common::Uri& _uri) -> dart::dynamics::SkeletonPtr {
            return self->parseSkeleton(_uri);
          },
          ::py::arg("uri"))
      .def(
          "parseSkeleton",
          +[](dart::io::DartLoader* self,
              const dart::common::Uri& _uri,
              const dart::common::ResourceRetrieverPtr& _resourceRetriever)
              -> dart::dynamics::SkeletonPtr {
            return self->parseSkeleton(_uri, _resourceRetriever);
          },
          ::py::arg("uri"),
          ::py::arg("resourceRetriever"))
      .def(
          "parseSkeletonString",
          +[](dart::io::DartLoader* self,
              const std::string& _urdfString,
              const dart::common::Uri& _baseUri)
              -> dart::dynamics::SkeletonPtr {
            return self->parseSkeletonString(_urdfString, _baseUri);
          },
          ::py::arg("urdfString"),
          ::py::arg("baseUri"))
      .def(
          "parseSkeletonString",
          +[](dart::io::DartLoader* self,
              const std::string& _urdfString,
              const dart::common::Uri& _baseUri,
              const dart::common::ResourceRetrieverPtr& _resourceRetriever)
              -> dart::dynamics::SkeletonPtr {
            return self->parseSkeletonString(
                _urdfString, _baseUri, _resourceRetriever);
          },
          ::py::arg("urdfString"),
          ::py::arg("baseUri"),
          ::py::arg("resourceRetriever"))
      .def(
          "parseWorld",
          +[](dart::io::DartLoader* self,
              const dart::common::Uri& _uri) -> dart::simulation::WorldPtr {
            return self->parseWorld(_uri);
          },
          ::py::arg("uri"))
      .def(
          "parseWorld",
          +[](dart::io::DartLoader* self,
              const dart::common::Uri& _uri,
              const dart::common::ResourceRetrieverPtr& _resourceRetriever)
              -> dart::simulation::WorldPtr {
            return self->parseWorld(_uri, _resourceRetriever);
          },
          ::py::arg("uri"),
          ::py::arg("resourceRetriever"))
      .def(
          "parseWorldString",
          +[](dart::io::DartLoader* self,
              const std::string& _urdfString,
              const dart::common::Uri& _baseUri) -> dart::simulation::WorldPtr {
            return self->parseWorldString(_urdfString, _baseUri);
          },
          ::py::arg("urdfString"),
          ::py::arg("baseUri"))
      .def(
          "parseWorldString",
          +[](dart::io::DartLoader* self,
              const std::string& _urdfString,
              const dart::common::Uri& _baseUri,
              const dart::common::ResourceRetrieverPtr& _resourceRetriever)
              -> dart::simulation::WorldPtr {
            return self->parseWorldString(
                _urdfString, _baseUri, _resourceRetriever);
          },
          ::py::arg("urdfString"),
          ::py::arg("baseUri"),
          ::py::arg("resourceRetriever"));
}

} // namespace python
} // namespace dart
