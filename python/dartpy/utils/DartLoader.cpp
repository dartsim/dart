// clang-format off
#include "detail/dart_nb.hpp"
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

#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/Inertia.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/common/Macros.hpp>
#include <dart/common/ResourceRetriever.hpp>
#include <dart/common/Uri.hpp>

#include <string>

namespace dart {
namespace python {

void DartLoader(nb::module_& m)
{
  static_assert(dartnb::GcOwner<utils::DartLoader::Options>::value);
  static_assert(dartnb::GcOwner<utils::DartLoader>::value);
  auto dartLoaderFlags
      = nb::enum_<utils::DartLoader::Flags>(
            m, "DartLoaderFlags", nb::is_arithmetic(), nb::is_flag())
            .value("NONE", utils::DartLoader::Flags::NONE)
            .value("FIXED_BASE_LINK", utils::DartLoader::Flags::FIXED_BASE_LINK)
            .value("DEFAULT", utils::DartLoader::Flags::DEFAULT);

  auto dartLoaderRootJointType
      = nb::enum_<utils::DartLoader::RootJointType>(
            m, "DartLoaderRootJointType", nb::is_arithmetic())
            .value("FLOATING", utils::DartLoader::RootJointType::FLOATING)
            .value("FIXED", utils::DartLoader::RootJointType::FIXED);

  auto dartLoaderOptions
      = dartnb::dart_class<utils::DartLoader::Options>(m, "DartLoaderOptions")
            .def(
                dartnb::init<
                    common::ResourceRetrieverPtr,
                    utils::DartLoader::RootJointType,
                    const dynamics::Inertia&>(),
                nb::arg("resourceRetriever").none() = nullptr,
                nb::arg("defaultRootJointType")
                = utils::DartLoader::RootJointType::FLOATING,
                nb::arg("defaultInertia") = dynamics::Inertia())
            .def_rw(
                "mResourceRetriever",
                &utils::DartLoader::Options::mResourceRetriever,
                dartnb::setterArgument(
                    &utils::DartLoader::Options::mResourceRetriever))
            .def_rw(
                "mDefaultRootJointType",
                &utils::DartLoader::Options::mDefaultRootJointType,
                dartnb::setterArgument(
                    &utils::DartLoader::Options::mDefaultRootJointType))
            .def_rw(
                "mDefaultInertia",
                &utils::DartLoader::Options::mDefaultInertia,
                dartnb::setterArgument(
                    &utils::DartLoader::Options::mDefaultInertia));

  auto dartLoader
      = dartnb::dart_class<utils::DartLoader>(m, "DartLoader")
            .def(dartnb::init<>())
            .def(
                "setOptions",
                &utils::DartLoader::setOptions,
                nb::arg("options") = utils::DartLoader::Options())
            .def("getOptions", &utils::DartLoader::getOptions)
            .def(
                "addPackageDirectory",
                &utils::DartLoader::addPackageDirectory,
                nb::arg("packageName"),
                nb::arg("packageDirectory"))
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
                nb::arg("uri"),
                nb::arg("resourceRetriever").none(),
                nb::arg("flags") = utils::DartLoader::DEFAULT)
            .def(
                "parseSkeleton",
                nb::overload_cast<const common::Uri&>(
                    &utils::DartLoader::parseSkeleton),
                nb::arg("uri"))
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
                nb::arg("urdfString"),
                nb::arg("baseUri"),
                nb::arg("resourceRetriever").none(),
                nb::arg("flags") = utils::DartLoader::DEFAULT)
            .def(
                "parseSkeletonString",
                nb::overload_cast<const std::string&, const common::Uri&>(
                    &utils::DartLoader::parseSkeletonString),
                nb::arg("urdfString"),
                nb::arg("baseUri"))
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
                nb::arg("uri"),
                nb::arg("resourceRetriever").none(),
                nb::arg("flags") = utils::DartLoader::DEFAULT)
            .def(
                "parseWorld",
                nb::overload_cast<const common::Uri&>(
                    &utils::DartLoader::parseWorld),
                nb::arg("uri"))
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
                nb::arg("urdfString"),
                nb::arg("baseUri"),
                nb::arg("resourceRetriever").none(),
                nb::arg("flags") = utils::DartLoader::DEFAULT)
            .def(
                "parseWorldString",
                nb::overload_cast<const std::string&, const common::Uri&>(
                    &utils::DartLoader::parseWorldString),
                nb::arg("urdfString"),
                nb::arg("baseUri"));

  dartLoader.attr("Flags") = dartLoaderFlags;
  dartLoader.attr("RootJointType") = dartLoaderRootJointType;
  dartLoader.attr("Options") = dartLoaderOptions;
}

} // namespace python
} // namespace dart
