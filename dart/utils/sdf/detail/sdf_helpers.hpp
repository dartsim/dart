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

#include <dart/common/logging.hpp>
#include <dart/common/macros.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#if __has_include(<gz/math/Color.hh>)
  #include <gz/math/Color.hh>
  #include <gz/math/Pose3.hh>
  #include <gz/math/Vector2.hh>
  #include <gz/math/Vector3.hh>
#else
  #include <gz/math9/gz/math/Color.hh>
  #include <gz/math9/gz/math/Pose3.hh>
  #include <gz/math9/gz/math/Vector2.hh>
  #include <gz/math9/gz/math/Vector3.hh>
#endif
#include <sdf/sdf.hh>

#include <algorithm>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <cctype>

namespace dart::utils::SdfParser::detail {

using ElementPtr = sdf::ElementPtr;

DART_UTILS_API std::string toLowerCopy(std::string_view text);
DART_UTILS_API std::string trimCopy(std::string_view text);

DART_UTILS_API std::string getElementText(const ElementPtr& element);
DART_UTILS_API std::string getChildElementText(
    const ElementPtr& parent, std::string_view name);
DART_UTILS_API std::string getValueText(
    const ElementPtr& parentElement,
    std::string_view name,
    const sdf::ParamPtr& param);

template <typename T>
bool parseScalar(std::string_view text, T& value)
{
  std::istringstream stream{std::string(text)};
  stream >> value;
  return !stream.fail();
}

template <>
inline bool parseScalar<bool>(std::string_view text, bool& value)
{
  const std::string lower = toLowerCopy(text);
  if (lower == "1" || lower == "true") {
    value = true;
    return true;
  }

  if (lower == "0" || lower == "false") {
    value = false;
    return true;
  }

  return false;
}

template <typename T>
std::vector<T> parseArray(std::string_view text)
{
  std::istringstream stream{std::string(text)};
  std::vector<T> values;
  T value{};
  while (stream >> value) {
    values.push_back(value);
  }
  return values;
}

DART_UTILS_API sdf::ParamPtr getAttributeParam(
    const ElementPtr& element, std::string_view attributeName);

DART_UTILS_API sdf::ParamPtr getChildValueParam(
    const ElementPtr& parentElement, std::string_view name);

template <typename T>
bool readScalarParam(const sdf::ParamPtr& param, T& value)
{
  if (!param) {
    return false;
  }

  if (param->Get(value)) {
    return true;
  }

  std::string text;
  try {
    text = param->GetAsString();
  } catch (const std::exception&) {
    return false;
  }

  return parseScalar(text, value);
}

DART_UTILS_API Eigen::Vector3d toEigen(const gz::math::Vector3d& vec);
DART_UTILS_API Eigen::Vector2d toEigen(const gz::math::Vector2d& vec);
DART_UTILS_API Eigen::Vector3i toEigen(const gz::math::Vector3i& vec);
DART_UTILS_API Eigen::VectorXd colorToVector(const gz::math::Color& color);
DART_UTILS_API Eigen::Isometry3d poseToIsometry(const gz::math::Pose3d& pose);

DART_UTILS_API bool hasElement(const ElementPtr& parent, std::string_view name);
DART_UTILS_API ElementPtr
getElement(const ElementPtr& parent, std::string_view name);
DART_UTILS_API bool hasAttribute(
    const ElementPtr& element, std::string_view attributeName);

DART_UTILS_API std::string getAttributeString(
    const ElementPtr& element, std::string_view attributeName);

DART_UTILS_API std::string getValueString(
    const ElementPtr& parentElement, std::string_view name);
DART_UTILS_API bool getValueBool(
    const ElementPtr& parentElement, std::string_view name);
DART_UTILS_API unsigned int getValueUInt(
    const ElementPtr& parentElement, std::string_view name);
DART_UTILS_API double getValueDouble(
    const ElementPtr& parentElement, std::string_view name);
DART_UTILS_API Eigen::Vector2d getValueVector2d(
    const ElementPtr& parentElement, std::string_view name);
DART_UTILS_API Eigen::Vector3d getValueVector3d(
    const ElementPtr& parentElement, std::string_view name);
DART_UTILS_API Eigen::Vector3i getValueVector3i(
    const ElementPtr& parentElement, std::string_view name);
DART_UTILS_API Eigen::VectorXd getValueVectorXd(
    const ElementPtr& parentElement, std::string_view name);
DART_UTILS_API Eigen::Isometry3d getValueIsometry3dWithExtrinsicRotation(
    const ElementPtr& parentElement, std::string_view name);

class DART_UTILS_API ElementEnumerator
{
public:
  ElementEnumerator(const ElementPtr& parentElement, std::string_view name);
  bool next();
  ElementPtr get() const;

private:
  ElementPtr mParent;
  std::string mName;
  ElementPtr mCurrent;
  bool mInitialized;
};

} // namespace dart::utils::SdfParser::detail

#endif // DART_UTILS_SDF_DETAIL_SDFHELPERS_HPP_
