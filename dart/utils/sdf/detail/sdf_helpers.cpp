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

#include "dart/utils/sdf/detail/sdf_helpers.hpp"

#include <dart/common/logging.hpp>

#include <sstream>
#include <string>
#include <vector>

#if __has_include(<gz/math/Pose3.hh>)
  #include <gz/math/Pose3.hh>
  #include <gz/math/Quaternion.hh>
  #include <gz/math/Vector3.hh>
#else
  #include <gz/math9/gz/math/Pose3.hh>
  #include <gz/math9/gz/math/Quaternion.hh>
  #include <gz/math9/gz/math/Vector3.hh>
#endif

namespace dart::utils::SdfParser::detail {

namespace {

std::string trimCopy(std::string_view text)
{
  const auto start = text.find_first_not_of(" \t\r\n");
  if (start == std::string_view::npos) {
    return std::string();
  }

  const auto end = text.find_last_not_of(" \t\r\n");
  return std::string(text.substr(start, end - start + 1));
}

std::string getElementText(const ElementPtr& element)
{
  if (!element) {
    return std::string();
  }

  sdf::Errors errors;
  const auto serialized = element->ToString(errors, "");
  if (!errors.empty()) {
    return std::string();
  }

  const auto open = serialized.find('>');
  const auto close = serialized.rfind('<');
  if (open == std::string::npos || close == std::string::npos
      || close <= open) {
    return std::string();
  }

  return trimCopy(serialized.substr(open + 1, close - open - 1));
}

std::string getChildElementText(const ElementPtr& parent, std::string_view name)
{
  if (!parent || name.empty()) {
    return std::string();
  }

  const auto child = getElement(parent, name);
  return getElementText(child);
}

std::string getValueText(
    const ElementPtr& parentElement,
    std::string_view name,
    const sdf::ParamPtr& param)
{
  const auto directText = getChildElementText(parentElement, name);
  if (!directText.empty()) {
    return directText;
  }

  if (param) {
    try {
      std::string text = trimCopy(param->GetAsString());
      if (text.find('<') == std::string::npos) {
        return text;
      }
    } catch (const std::exception& e) {
      DART_WARN(
          "[SdfParser] Failed to parse element <{}> under <{}> as string: {}",
          name,
          parentElement ? parentElement->GetName() : "unknown",
          e.what());
    }
  }

  return getChildElementText(parentElement, name);
}

template <typename T>
bool parseScalar(std::string_view text, T& value)
{
  std::istringstream stream{std::string(text)};
  stream >> value;
  return !stream.fail();
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

sdf::ParamPtr getChildValueParam(
    const ElementPtr& parentElement, std::string_view name)
{
  if (!parentElement || name.empty()) {
    return nullptr;
  }

  const std::string nameString(name);
  if (!parentElement->HasElement(nameString)) {
    return nullptr;
  }

  const auto child = parentElement->GetElement(nameString);
  if (!child) {
    return nullptr;
  }

  return child->GetValue();
}

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

Eigen::Vector3d toEigen(const gz::math::Vector3d& vec)
{
  return Eigen::Vector3d(vec.X(), vec.Y(), vec.Z());
}

Eigen::Isometry3d poseToIsometry(const gz::math::Pose3d& pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation()
      = Eigen::Vector3d(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
  Eigen::Quaterniond quat(
      pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());
  transform.linear() = quat.toRotationMatrix();
  return transform;
}

} // namespace

bool hasElement(const ElementPtr& parent, std::string_view name)
{
  return parent && !name.empty() && parent->HasElement(std::string(name));
}

ElementPtr getElement(const ElementPtr& parent, std::string_view name)
{
  if (!parent || name.empty()) {
    return nullptr;
  }

  const std::string nameString(name);
  if (!parent->HasElement(nameString)) {
    return nullptr;
  }

  return parent->GetElement(nameString);
}

unsigned int getValueUInt(
    const ElementPtr& parentElement, std::string_view name)
{
  unsigned int value = 0u;
  if (readScalarParam(getChildValueParam(parentElement, name), value)) {
    return value;
  }

  DART_WARN(
      "[SdfParser] Failed to parse element <{}> under <{}> as unsigned int.",
      name,
      parentElement ? parentElement->GetName() : "unknown");
  return 0u;
}

double getValueDouble(const ElementPtr& parentElement, std::string_view name)
{
  double value = 0.0;
  if (readScalarParam(getChildValueParam(parentElement, name), value)) {
    return value;
  }

  DART_WARN(
      "[SdfParser] Failed to parse element <{}> under <{}> as double.",
      name,
      parentElement ? parentElement->GetName() : "unknown");
  return 0.0;
}

Eigen::Vector3d getValueVector3d(
    const ElementPtr& parentElement, std::string_view name)
{
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  const auto param = getChildValueParam(parentElement, name);
  if (!param) {
    return result;
  }

  gz::math::Vector3d vec3;
  if (param->Get(vec3)) {
    return toEigen(vec3);
  }

  const auto text = getValueText(parentElement, name, param);
  if (text.empty()) {
    return result;
  }

  const auto values = parseArray<double>(text);
  if (values.size() >= 3) {
    result << values[0], values[1], values[2];
    return result;
  }

  DART_WARN(
      "[SdfParser] Element <{}> under <{}> expected 3 values but found {}.",
      name,
      parentElement ? parentElement->GetName() : "unknown",
      values.size());
  return result;
}

Eigen::Vector3i getValueVector3i(
    const ElementPtr& parentElement, std::string_view name)
{
  Eigen::Vector3i result = Eigen::Vector3i::Zero();
  const auto param = getChildValueParam(parentElement, name);
  if (!param) {
    return result;
  }

  const auto text = getValueText(parentElement, name, param);
  if (text.empty()) {
    return result;
  }

  const auto values = parseArray<int>(text);
  if (values.size() >= 3) {
    result << values[0], values[1], values[2];
    return result;
  }

  DART_WARN(
      "[SdfParser] Element <{}> under <{}> expected 3 integer values but found "
      "{}.",
      name,
      parentElement ? parentElement->GetName() : "unknown",
      values.size());
  return result;
}

Eigen::Isometry3d getValueIsometry3dWithExtrinsicRotation(
    const ElementPtr& parentElement, std::string_view name)
{
  const auto param = getChildValueParam(parentElement, name);
  if (!param) {
    return Eigen::Isometry3d::Identity();
  }

  gz::math::Pose3d pose;
  if (param->Get(pose)) {
    return poseToIsometry(pose);
  }

  const auto text = getValueText(parentElement, name, param);
  if (text.empty()) {
    return Eigen::Isometry3d::Identity();
  }

  const auto values = parseArray<double>(text);
  if (values.size() == 6) {
    gz::math::Pose3d fallbackPose(
        gz::math::Vector3d(values[0], values[1], values[2]),
        gz::math::Quaterniond(values[3], values[4], values[5]));
    return poseToIsometry(fallbackPose);
  }

  DART_WARN(
      "[SdfParser] Element <{}> under <{}> expected 6 pose values but found "
      "{}.",
      name,
      parentElement ? parentElement->GetName() : "unknown",
      values.size());
  return Eigen::Isometry3d::Identity();
}

} // namespace dart::utils::SdfParser::detail
