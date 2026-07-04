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

#ifndef DART_UTILS_SDF_DETAIL_SDFHELPERS_HPP_
#define DART_UTILS_SDF_DETAIL_SDFHELPERS_HPP_

#include <dart/config.hpp>

#include <dart/utils/export.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sdf/sdf.hh>

#include <string_view>

namespace dart::utils::SdfParser::detail {

using ElementPtr = sdf::ElementPtr;

DART_UTILS_API bool hasElement(const ElementPtr& parent, std::string_view name);
DART_UTILS_API bool hasAuthoredElement(
    const ElementPtr& parent, std::string_view name);
DART_UTILS_API ElementPtr
getElement(const ElementPtr& parent, std::string_view name);

DART_UTILS_API unsigned int getValueUInt(
    const ElementPtr& parentElement, std::string_view name);
DART_UTILS_API double getValueDouble(
    const ElementPtr& parentElement, std::string_view name);
DART_UTILS_API Eigen::Vector3d getValueVector3d(
    const ElementPtr& parentElement, std::string_view name);
DART_UTILS_API Eigen::Vector3i getValueVector3i(
    const ElementPtr& parentElement, std::string_view name);
DART_UTILS_API Eigen::Isometry3d getValueIsometry3dWithExtrinsicRotation(
    const ElementPtr& parentElement, std::string_view name);

} // namespace dart::utils::SdfParser::detail

#endif // DART_UTILS_SDF_DETAIL_SDFHELPERS_HPP_
