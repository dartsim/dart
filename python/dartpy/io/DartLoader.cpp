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

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void DartLoader(py::module& m)
{
  auto dartLoaderRootJointType
      = ::py::enum_<io::DartLoader::RootJointType>(m, "DartLoaderRootJointType")
            .value("FLOATING", io::DartLoader::RootJointType::FLOATING)
            .value("FIXED", io::DartLoader::RootJointType::FIXED);

  auto dartLoaderOptions
      = ::py::class_<io::DartLoader::Options>(m, "DartLoaderOptions")
            .def(
                ::py::init<
                    common::ResourceRetrieverPtr,
                    io::DartLoader::RootJointType,
                    const dynamics::Inertia&>(),
                ::py::arg("resourceRetriever") = nullptr,
                ::py::arg("defaultRootJointType")
                = io::DartLoader::RootJointType::FLOATING,
                ::py::arg("defaultInertia") = dynamics::Inertia())
            .def_readwrite(
                "mResourceRetriever",
                &io::DartLoader::Options::mResourceRetriever)
            .def_readwrite(
                "mDefaultRootJointType",
                &io::DartLoader::Options::mDefaultRootJointType)
            .def_readwrite(
                "mDefaultInertia", &io::DartLoader::Options::mDefaultInertia);

  auto dartLoader
      = ::py::class_<io::DartLoader>(m, "DartLoader")
            .def(::py::init<>())
            .def(
                "setOptions",
                &io::DartLoader::setOptions,
                ::py::arg("options") = io::DartLoader::Options())
            .def("getOptions", &io::DartLoader::getOptions)
            .def(
                "addPackageDirectory",
                &io::DartLoader::addPackageDirectory,
                ::py::arg("packageName"),
                ::py::arg("packageDirectory"))
            .def(
                "parseSkeleton",
                ::py::overload_cast<const common::Uri&>(
                    &io::DartLoader::parseSkeleton),
                ::py::arg("uri"))
            .def(
                "parseSkeletonString",
                ::py::overload_cast<const std::string&, const common::Uri&>(
                    &io::DartLoader::parseSkeletonString),
                ::py::arg("urdfString"),
                ::py::arg("baseUri"))
            .def(
                "parseWorld",
                ::py::overload_cast<const common::Uri&>(
                    &io::DartLoader::parseWorld),
                ::py::arg("uri"))
            .def(
                "parseWorldString",
                ::py::overload_cast<const std::string&, const common::Uri&>(
                    &io::DartLoader::parseWorldString),
                ::py::arg("urdfString"),
                ::py::arg("baseUri"));

  dartLoader.attr("RootJointType") = dartLoaderRootJointType;
  dartLoader.attr("Options") = dartLoaderOptions;
}

} // namespace python
} // namespace dart
