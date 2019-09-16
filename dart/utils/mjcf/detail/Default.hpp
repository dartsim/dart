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

#ifndef DART_UTILS_MJCF_DETAIL_DEFAULT_HPP_
#define DART_UTILS_MJCF_DETAIL_DEFAULT_HPP_

#include <unordered_map>

#include <tinyxml2.h>

#include "dart/common/Optional.hpp"
#include "dart/utils/mjcf/detail/Error.hpp"
#include "dart/utils/mjcf/detail/GeomAttributes.hpp"
#include "dart/utils/mjcf/detail/JointAttributes.hpp"
#include "dart/utils/mjcf/detail/MeshAttributes.hpp"
#include "dart/utils/mjcf/detail/Types.hpp"
#include "dart/utils/mjcf/detail/WeldAttributes.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

class Default final
{
public:
  /// Default constructor
  Default() = default;

  const GeomAttributes& getGeomAttributes() const;
  const JointAttributes& getJointAttributes() const;
  const MeshAttributes& getMeshAttributes() const;

  const WeldAttributes& getWeldAttributes() const;

private:
  // Private members used by Defaults class
  friend class Defaults;
  Errors read(tinyxml2::XMLElement* element, const Default* parent);

private:
  GeomAttributes mGeomAttributes;
  JointAttributes mJointAttributes;
  MeshAttributes mMeshAttributes;

  // Equality constraint attributes
  WeldAttributes mWeldAttributes;
};

class Defaults
{
public:
  bool hasDefault(const std::string& className) const;
  const Default* getDefault(const std::string& className) const;
  const Default* getRootDefault() const;

private:
  // Private members used by MujocoModel class
  friend class MujocoModel;
  Errors read(tinyxml2::XMLElement* element, const Default* parent);

private:
  //  Default mRootDefault;
  std::string mRootClassName;
  std::unordered_map<std::string, Default> mDefaultMap;
};

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_DEFAULT_HPP_
