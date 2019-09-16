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

#ifndef DART_UTILS_MJCF_DETAIL_ASSET_HPP_
#define DART_UTILS_MJCF_DETAIL_ASSET_HPP_

#include <unordered_map>
#include <vector>

#include <tinyxml2.h>

#include "dart/utils/mjcf/detail/Compiler.hpp"
#include "dart/utils/mjcf/detail/Error.hpp"
#include "dart/utils/mjcf/detail/Mesh.hpp"
#include "dart/utils/mjcf/detail/Types.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

class Body;

class Asset final
{
public:
  /// Default constructor
  Asset() = default;

  /// \{ \name Elements

  /// Returns the number of <mesh> elements in <asset>.
  std::size_t getNumMeshes() const;

  /// Returns <mesh> element at \c index in <asset>.
  const Mesh& getMesh(std::size_t index) const;

  /// Finds <mesh> element by name
  const Mesh* getMesh(const std::string& name) const;

  /// \}

private:
  // Private members used by MujocoModel class
  friend class MujocoModel;
  Errors read(tinyxml2::XMLElement* element);

  /// Updates attributes and elements that doesn't require any other elements.
  Errors preprocess(const Compiler& compiler);

  /// Updates attributes and elements that require the preprocessed child
  /// elements of this <Asset>.
  Errors compile(const Compiler& compiler);

  /// Updates attributes and elements that require the compiled parent element.
  Errors postprocess(const Compiler& compiler);

private:
  std::vector<Mesh> mMeshes;
  std::unordered_map<std::string, Mesh*> mMeshMap;
};

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_ASSET_HPP_
