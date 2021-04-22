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

#ifndef DART_IO_MJCF_DETAIL_MESH_HPP_
#define DART_IO_MJCF_DETAIL_MESH_HPP_

#include <tinyxml2.h>

#include "dart/dynamics/MeshShape.hpp"
#include "dart/io/mjcf/detail/Compiler.hpp"
#include "dart/io/mjcf/detail/Error.hpp"
#include "dart/io/mjcf/detail/MeshAttributes.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

class Mesh final
{
public:
  /// Default constructor
  Mesh() = default;

  /// \{ \name Attributes

  const std::string& getName() const;
  const std::string& getFile() const;
  const Eigen::Vector3d& getScale() const;

  dynamics::MeshShapePtr getMeshShape() const;

  /// \}

private:
  // Private members used by Asset class
  friend class Asset;
  friend class Default;
  Errors read(tinyxml2::XMLElement* element);

  /// Updates attributes and elements that doesn't require any other elements.
  Errors preprocess(const Compiler& compiler);

  /// Updates attributes and elements that require the preprocessed child
  /// elements of this <Mesh>.
  Errors compile(const Compiler& compiler);

  /// Updates attributes and elements that require the compiled parent element.
  Errors postprocess(const Compiler& compiler);

  dynamics::MeshShapePtr createMeshShape() const;

private:
  MeshAttributes mAttributes;

  /// Name of this Mesh
  std::string mName{""};

  std::string mFile{""};

  Eigen::Vector3d mScale{Eigen::Vector3d::Ones()};

  common::Uri mMeshUri;
  common::ResourceRetrieverPtr mRetriever;
  mutable bool mTriedToParse{false};
  mutable dynamics::MeshShapePtr mMeshShape{nullptr};
};

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_IO_MJCF_DETAIL_MESH_HPP_
