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

#include <string>

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
bool readParamValue(const sdf::ParamPtr& param, T& value)
{
  if (!param) {
    return false;
  }

  sdf::Errors errors;
  return param->Get(value, errors) && errors.empty();
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
  if (readParamValue(getChildValueParam(parentElement, name), value)) {
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
  if (readParamValue(getChildValueParam(parentElement, name), value)) {
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
  if (readParamValue(param, vec3)) {
    return toEigen(vec3);
  }

  DART_WARN(
      "[SdfParser] Failed to parse element <{}> under <{}> as vector3.",
      name,
      parentElement ? parentElement->GetName() : "unknown");
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

  gz::math::Vector3d vec3;
  if (readParamValue(param, vec3)) {
    result << static_cast<int>(vec3.X()), static_cast<int>(vec3.Y()),
        static_cast<int>(vec3.Z());
    return result;
  }

  DART_WARN(
      "[SdfParser] Failed to parse element <{}> under <{}> as vector3i.",
      name,
      parentElement ? parentElement->GetName() : "unknown");
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
  if (readParamValue(param, pose)) {
    return poseToIsometry(pose);
  }

  DART_WARN(
      "[SdfParser] Failed to parse element <{}> under <{}> as pose.",
      name,
      parentElement ? parentElement->GetName() : "unknown");
  return Eigen::Isometry3d::Identity();
}

} // namespace dart::utils::SdfParser::detail
