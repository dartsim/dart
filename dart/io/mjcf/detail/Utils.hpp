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

#ifndef DART_UTILS_MJCF_DETAIL_UTILS_HPP_
#define DART_UTILS_MJCF_DETAIL_UTILS_HPP_

#include <dart/simulation/World.hpp>

#include <dart/io/mjcf/detail/Compiler.hpp>
#include <dart/io/mjcf/detail/Error.hpp>

#include <tinyxml2.h>

#include <optional>

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

/// Checks if orientation elements (i.e., <quat>, <axisangle>, <euler>,
/// <xyaxes>, <zaxis>) are properly set in \c element
Errors checkOrientationValidity(const tinyxml2::XMLElement* element);

/// Extracts rotation matrix from "pre-parsed" orientation elements and the
/// compiler settings
math::Matrix3d compileRotation(
    const math::Quaterniond& quat,
    const std::optional<math::Vector4d>& axisAngle,
    const std::optional<math::Vector3d>& euler,
    const std::optional<math::Vector6d>& xyAxes,
    const std::optional<math::Vector3d>& zAxis,
    const Compiler& compiler);

/// Includes other MJCF model file into \c element
///
/// \param[in] baseUri The URI of the main MJCF model file
/// \param[in] retriever The resource retriever used for the main MJCF model
/// file
/// \return Errors occured in hadling <include> elements
Errors handleInclude(
    tinyxml2::XMLElement* element,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

/// Finds all BodyNodes by name
std::vector<dynamics::BodyNode*> getBodyNodes(
    const simulation::World& world, const std::string& name);

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_UTILS_HPP_
