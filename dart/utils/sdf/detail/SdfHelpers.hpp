/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#if HAVE_SDFORMAT

#include <dart/common/Logging.hpp>
#include <dart/common/Macros.hpp>

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
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

namespace dart::utils::SdfParser::detail {

using ElementPtr = sdf::ElementPtr;

std::string toLowerCopy(std::string text);

template <typename T>
bool parseScalar(const std::string& text, T& value)
{
  std::istringstream stream(text);
  stream >> value;
  return !stream.fail();
}

template <>
inline bool parseScalar<bool>(const std::string& text, bool& value)
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
std::vector<T> parseArray(const std::string& text)
{
  std::istringstream stream(text);
  std::vector<T> values;
  T value{};
  while (stream >> value)
    values.push_back(value);
  return values;
}

sdf::ParamPtr getAttributeParam(
    const ElementPtr& element, const std::string& attributeName);

sdf::ParamPtr getChildValueParam(
    const ElementPtr& parentElement, const std::string& name);

template <typename T>
bool readScalarParam(const sdf::ParamPtr& param, T& value)
{
  if (!param)
    return false;

  if (param->Get(value))
    return true;

  std::string text;
  try {
    text = param->GetAsString();
  } catch (const std::exception&) {
    return false;
  }

  return parseScalar(text, value);
}

Eigen::Vector3d toEigen(const gz::math::Vector3d& vec);
Eigen::Vector2d toEigen(const gz::math::Vector2d& vec);
Eigen::Vector3i toEigen(const gz::math::Vector3i& vec);
Eigen::VectorXd colorToVector(const gz::math::Color& color);
Eigen::Isometry3d poseToIsometry(const gz::math::Pose3d& pose);

bool hasElement(const ElementPtr& parent, const std::string& name);
ElementPtr getElement(const ElementPtr& parent, const std::string& name);
bool hasAttribute(const ElementPtr& element, const std::string& attributeName);

std::string getAttributeString(
    const ElementPtr& element, const std::string& attributeName);

std::string getValueString(
    const ElementPtr& parentElement, const std::string& name);
bool getValueBool(
    const ElementPtr& parentElement, const std::string& name);
unsigned int getValueUInt(
    const ElementPtr& parentElement, const std::string& name);
double getValueDouble(
    const ElementPtr& parentElement, const std::string& name);
Eigen::Vector2d getValueVector2d(
    const ElementPtr& parentElement, const std::string& name);
Eigen::Vector3d getValueVector3d(
    const ElementPtr& parentElement, const std::string& name);
Eigen::Vector3i getValueVector3i(
    const ElementPtr& parentElement, const std::string& name);
Eigen::VectorXd getValueVectorXd(
    const ElementPtr& parentElement, const std::string& name);
Eigen::Isometry3d getValueIsometry3dWithExtrinsicRotation(
    const ElementPtr& parentElement, const std::string& name);

class ElementEnumerator
{
public:
  ElementEnumerator(const ElementPtr& parentElement, const std::string& name);
  bool next();
  ElementPtr get() const;

private:
  ElementPtr mParent;
  std::string mName;
  ElementPtr mCurrent;
  bool mInitialized;
};

} // namespace dart::utils::SdfParser::detail

#endif // HAVE_SDFORMAT

#endif // DART_UTILS_SDF_DETAIL_SDFHELPERS_HPP_
