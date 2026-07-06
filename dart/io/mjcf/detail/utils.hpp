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

#ifndef DART_IO_MJCF_DETAIL_UTILS_HPP_
#define DART_IO_MJCF_DETAIL_UTILS_HPP_

#include <dart/math/math_types.hpp>

#include <dart/io/export.hpp>
#include <dart/io/mjcf/detail/compiler.hpp>
#include <dart/io/mjcf/detail/error.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tinyxml2.h>

#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

/// Checks if orientation elements (i.e., <quat>, <axisangle>, <euler>,
/// <xyaxes>, <zaxis>) are properly set in @c element
DART_IO_API Errors
checkOrientationValidity(const tinyxml2::XMLElement* element);

/// Extracts rotation matrix from "pre-parsed" orientation elements and the
/// compiler settings
DART_IO_API Eigen::Matrix3d compileRotation(
    const Eigen::Quaterniond& quat,
    const std::optional<Eigen::Vector4d>& axisAngle,
    const std::optional<Eigen::Vector3d>& euler,
    const std::optional<Eigen::Vector6d>& xyAxes,
    const std::optional<Eigen::Vector3d>& zAxis,
    const Compiler& compiler);

/// Includes other MJCF model file into @c element
///
/// @param[in] baseUri The URI of the main MJCF model file
/// @param[in] retriever The resource retriever used for the main MJCF model
/// file
/// @return Errors occurred in handling <include> elements
DART_IO_API Errors handleInclude(
    tinyxml2::XMLElement* element,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

/// Logs warnings about child elements of @c parentElement that are not in
/// @c knownChildNames.
DART_IO_API void warnUnknownElements(
    const tinyxml2::XMLElement* parentElement,
    std::initializer_list<std::string_view> knownChildNames);

/// Logs warnings about attributes of @c element that are not in
/// @c knownAttrNames.
DART_IO_API void warnUnknownAttributes(
    const tinyxml2::XMLElement* element,
    std::initializer_list<std::string_view> knownAttrNames);

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_IO_MJCF_DETAIL_UTILS_HPP_
