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

#ifndef DART_UTILS_MJCF_DETAIL_MUJOCOMODEL_HPP_
#define DART_UTILS_MJCF_DETAIL_MUJOCOMODEL_HPP_

#include <tinyxml2.h>

#include "dart/common/ResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/utils/mjcf/detail/Asset.hpp"
#include "dart/utils/mjcf/detail/Compiler.hpp"
#include "dart/utils/mjcf/detail/Default.hpp"
#include "dart/utils/mjcf/detail/Equality.hpp"
#include "dart/utils/mjcf/detail/Error.hpp"
#include "dart/utils/mjcf/detail/Option.hpp"
#include "dart/utils/mjcf/detail/Size.hpp"
#include "dart/utils/mjcf/detail/Worldbody.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

/// Main class to parse MJCF model file
class MujocoModel final
{
public:
  MujocoModel() = default;

  /// Reads MJCF XML file
  ///
  /// \param[in] uri URI to the XML file
  /// \param[in] retrieverOrNull Retriever to acquire the XML file from \c uri
  /// \return Errors occurred in parsing
  Errors read(
      const common::Uri& uri,
      const common::ResourceRetrieverPtr& retrieverOrNull = nullptr);

  /// Returns the name of the model.
  const std::string& getModel() const;

  /// Returns the parsed <compiler> element
  const Compiler& getCompiler() const;

  /// Returns the parsed <option> element
  const Option& getOption() const;

  /// Returns the parsed <size> element
  const Size& getSize() const;

  /// Returns the parsed <asset> element
  const Asset& getAsset() const;

  /// Returns the parsed <worldbody> element
  const Worldbody& getWorldbody() const;

  /// Returns the parsed <equality> element
  const Equality& getEquality() const;

private:
  Errors read(
      tinyxml2::XMLElement* element,
      const common::Uri& baseUri,
      const common::ResourceRetrieverPtr& retriever);

  /// The name of the model.
  std::string mModel{"MuJoCo Model"};

  Compiler mCompiler;
  Option mOption;
  Size mSize;
  Defaults mDefaults;
  Asset mAsset;
  Worldbody mWorldbody;
  Equality mEquality;
};

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_MUJOCOMODEL_HPP_
