/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_IO_MJCF_DETAIL_WORLDBODY_HPP_
#define DART_IO_MJCF_DETAIL_WORLDBODY_HPP_

#include <string>
#include <vector>

#include <tinyxml2.h>

#include "dart/io/mjcf/detail/Body.hpp"
#include "dart/io/mjcf/detail/Compiler.hpp"
#include "dart/io/mjcf/detail/Error.hpp"
#include "dart/io/mjcf/detail/Geom.hpp"
#include "dart/io/mjcf/detail/Site.hpp"
#include "dart/io/mjcf/detail/Size.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

class Worldbody final
{
public:
  /// Default constructor
  Worldbody() = default;

  /// Returns the number of root <body> elements in <worldbody>.
  std::size_t getNumRootBodies() const;

  /// Returns a root <body> elements at \c index in <worldbody>.
  const Body& getRootBody(std::size_t index) const;

  /// Returns the number of <geom> elements in <worldbody>.
  std::size_t getNumGeoms() const;

  /// Returns a root <geom> element at \c index in <worldbody>.
  const Geom& getGeom(std::size_t index) const;

  /// Returns the number of <site> elements in <worldbody>.
  std::size_t getNumSites() const;

  /// Returns a root <site> element at \c index in <worldbody>.
  const Site& getSite(std::size_t index) const;

private:
  // Private memebers used by MujocoModel class
  friend class MujocoModel;
  Errors read(
      tinyxml2::XMLElement* element,
      const common::optional<Size>& size,
      const Defaults& defaults,
      const Default* currentDefault,
      const common::Uri& baseUri,
      const common::ResourceRetrieverPtr& retriever);

  /// Updates attributes and elements that doesn't require any other elements.
  Errors preprocess(const Compiler& compiler);

  /// Updates attributes and elements that require the preprocessed child
  /// elements of this <worldbody>.
  Errors compile(const Compiler& compiler);

  /// Updates attributes and elements that require the compiled parent element.
  Errors postprocess(const Compiler& compiler);

private:
  common::optional<std::string> mChildClass;
  std::vector<Geom> mGeoms;
  std::vector<Site> mSites;
  std::vector<Body> mRootBodies;
};

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_IO_MJCF_DETAIL_WORLDBODY_HPP_
