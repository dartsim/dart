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

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void DartLoader(py::module& m)
{
  auto dartLoaderFlags
      = ::py::enum_<utils::DartLoader::Flags>(m, "DartLoaderFlags")
            .value("NONE", utils::DartLoader::Flags::NONE)
            .value("FIXED_BASE_LINK", utils::DartLoader::Flags::FIXED_BASE_LINK)
            .value("DEFAULT", utils::DartLoader::Flags::DEFAULT);

  auto dartLoaderRootJointType
      = ::py::enum_<utils::DartLoader::RootJointType>(
            m, "DartLoaderRootJointType")
            .value("FLOATING", utils::DartLoader::RootJointType::FLOATING)
            .value("FIXED", utils::DartLoader::RootJointType::FIXED);

  auto dartLoaderOptions
      = ::py::class_<utils::DartLoader::Options>(m, "DartLoaderOptions")
            .def(
                ::py::init<
                    common::ResourceRetrieverPtr,
                    utils::DartLoader::RootJointType,
                    const dynamics::Inertia&>(),
                ::py::arg("resourceRetriever") = nullptr,
                ::py::arg("defaultRootJointType")
                = utils::DartLoader::RootJointType::FLOATING,
                ::py::arg("defaultInertia") = dynamics::Inertia())
            .def_readwrite(
                "mResourceRetriever",
                &utils::DartLoader::Options::mResourceRetriever)
            .def_readwrite(
                "mDefaultRootJointType",
                &utils::DartLoader::Options::mDefaultRootJointType)
            .def_readwrite(
                "mDefaultInertia",
                &utils::DartLoader::Options::mDefaultInertia);

  auto dartLoader
      = ::py::class_<utils::DartLoader>(m, "DartLoader")
            .def(::py::init<>())
            .def(
                "setOptions",
                &utils::DartLoader::setOptions,
                ::py::arg("options") = utils::DartLoader::Options())
            .def("getOptions", &utils::DartLoader::getOptions)
            .def(
                "addPackageDirectory",
                &utils::DartLoader::addPackageDirectory,
                ::py::arg("packageName"),
                ::py::arg("packageDirectory"))
            .def(
                "parseSkeleton",
                +[](dart::utils::DartLoader* self,
                    const dart::common::Uri& uri,
                    const common::ResourceRetrieverPtr& resourceRetriever,
                    unsigned int flags) -> dart::dynamics::SkeletonPtr {
                  DART_SUPPRESS_DEPRECATED_BEGIN
                  return self->parseSkeleton(uri, resourceRetriever, flags);
                  DART_SUPPRESS_DEPRECATED_END
                },
                ::py::arg("uri"),
                ::py::arg("resourceRetriever"),
                ::py::arg("flags") = utils::DartLoader::DEFAULT)
            .def(
                "parseSkeleton",
                ::py::overload_cast<const common::Uri&>(
                    &utils::DartLoader::parseSkeleton),
                ::py::arg("uri"))
            .def(
                "parseSkeletonString",
                +[](utils::DartLoader* self,
                    const std::string& urdfString,
                    const common::Uri& baseUri,
                    const common::ResourceRetrieverPtr& resourceRetriever,
                    unsigned int flags) -> dynamics::SkeletonPtr {
                  DART_SUPPRESS_DEPRECATED_BEGIN
                  return self->parseSkeletonString(
                      urdfString, baseUri, resourceRetriever, flags);
                  DART_SUPPRESS_DEPRECATED_END
                },
                ::py::arg("urdfString"),
                ::py::arg("baseUri"),
                ::py::arg("resourceRetriever"),
                ::py::arg("flags") = utils::DartLoader::DEFAULT)
            .def(
                "parseSkeletonString",
                ::py::overload_cast<const std::string&, const common::Uri&>(
                    &utils::DartLoader::parseSkeletonString),
                ::py::arg("urdfString"),
                ::py::arg("baseUri"))
            .def(
                "parseWorld",
                +[](utils::DartLoader* self,
                    const common::Uri& _uri,
                    const common::ResourceRetrieverPtr& resourceRetriever,
                    unsigned int flags) -> simulation::WorldPtr {
                  DART_SUPPRESS_DEPRECATED_BEGIN
                  return self->parseWorld(_uri, resourceRetriever, flags);
                  DART_SUPPRESS_DEPRECATED_END
                },
                ::py::arg("uri"),
                ::py::arg("resourceRetriever"),
                ::py::arg("flags") = utils::DartLoader::DEFAULT)
            .def(
                "parseWorld",
                ::py::overload_cast<const common::Uri&>(
                    &utils::DartLoader::parseWorld),
                ::py::arg("uri"))
            .def(
                "parseWorldString",
                +[](utils::DartLoader* self,
                    const std::string& urdfString,
                    const common::Uri& baseUri,
                    const common::ResourceRetrieverPtr& resourceRetriever,
                    unsigned int flags) -> simulation::WorldPtr {
                  DART_SUPPRESS_DEPRECATED_BEGIN
                  return self->parseWorldString(
                      urdfString, baseUri, resourceRetriever, flags);
                  DART_SUPPRESS_DEPRECATED_END
                },
                ::py::arg("urdfString"),
                ::py::arg("baseUri"),
                ::py::arg("resourceRetriever"),
                ::py::arg("flags") = utils::DartLoader::DEFAULT)
            .def(
                "parseWorldString",
                ::py::overload_cast<const std::string&, const common::Uri&>(
                    &utils::DartLoader::parseWorldString),
                ::py::arg("urdfString"),
                ::py::arg("baseUri"));

  dartLoader.attr("Flags") = dartLoaderFlags;
  dartLoader.attr("RootJointType") = dartLoaderRootJointType;
  dartLoader.attr("Options") = dartLoaderOptions;
}

} // namespace python
} // namespace dart
