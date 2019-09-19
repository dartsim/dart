/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_UTILS_MJCF_DETAIL_MESHATTRIBUTES_HPP_
#define DART_UTILS_MJCF_DETAIL_MESHATTRIBUTES_HPP_

#include <Eigen/Core>
#include <tinyxml2.h>

#include "dart/common/Optional.hpp"
#include "dart/utils/mjcf/detail/Error.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

/// Intermediate raw data read from the XML file. For the details, see
/// http://www.mujoco.org/book/XMLreference.html#mesh
struct MeshAttributes final
{
  /// Name of the Asset
  common::optional<std::string> mName;

  common::optional<std::string> mFile;

  Eigen::Vector3d mScale{Eigen::Vector3d::Ones()};
};

Errors appendMeshAttributes(
    MeshAttributes& attributes, tinyxml2::XMLElement* element);

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_MESHATTRIBUTES_HPP_
