/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/io/mjcf/detail/Inertial.hpp"

#include "dart/io/XmlHelpers.hpp"
#include "dart/io/mjcf/detail/Utils.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors Inertial::read(tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "inertial")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <inertial> from the provided element");
    return errors;
  }

  // (required) pos
  if (!hasAttribute(element, "pos"))
  {
    errors.emplace_back(
        ErrorCode::ATTRIBUTE_MISSING,
        "Failed to find required attribute 'pos' in <inertial>");
    return errors;
  }
  mData.mPos = getAttributeVector3d(element, "pos");

  // Check if multiple orientation representations present
  const Errors orientationErrors = checkOrientationValidity(element);
  errors.insert(
      errors.end(), orientationErrors.begin(), orientationErrors.end());

  // quat
  if (hasAttribute(element, "quat"))
  {
    const Eigen::Vector4d vec4d = getAttributeVector4d(element, "quat");
    mData.mQuat.w() = vec4d[0];
    mData.mQuat.x() = vec4d[1];
    mData.mQuat.y() = vec4d[2];
    mData.mQuat.z() = vec4d[3];
  }

  // axisangle
  if (hasAttribute(element, "axisangle"))
  {
    mData.mAxisAngle = getAttributeVector4d(element, "axisangle");
  }

  // euler
  if (hasAttribute(element, "euler"))
  {
    mData.mEuler = getAttributeVector3d(element, "euler");
  }

  // xyaxes
  if (hasAttribute(element, "xyaxes"))
  {
    mData.mXYAxes = getAttributeVector6d(element, "xyaxes");
  }

  // zaxis
  if (hasAttribute(element, "zaxis"))
  {
    mData.mZAxis = getAttributeVector3d(element, "zaxis");
  }

  // (required) mass
  if (!hasAttribute(element, "mass"))
  {
    errors.emplace_back(
        ErrorCode::ATTRIBUTE_MISSING,
        "Failed to find required attribute 'mass' in <inertial>");
    return errors;
  }
  mData.mMass = getAttributeDouble(element, "mass");

  // (optional) diaginertia, fullinertia
  if (hasAttribute(element, "diaginertia"))
  {
    mData.mDiagInertia = getAttributeVector3d(element, "diaginertia");

    if (hasAttribute(element, "fullinertia"))
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_CONFLICT,
          "Not allowed to set both of 'diaginertia' and 'fullinertia' in "
          "<inertial>");
      return errors;
    }
  }
  else
  {
    // If diaginertia is omitted, the next attribute becomes required.
    if (!hasAttribute(element, "fullinertia"))
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_MISSING,
          "Failed to find required attribute 'diaginertia' or 'fullinertia' in "
          "<inertial>");
      return errors;
    }

    mData.mFullInertia = getAttributeVector6d(element, "fullinertia");
  }

  return errors;
}

//==============================================================================
Errors Inertial::compile(const Compiler& compiler)
{
  Errors errors;

  mPos = mData.mPos;

  if (compiler.getCoordinate() == Coordinate::LOCAL)
  {
    mRelativeTransform.translation() = mData.mPos;
    mRelativeTransform.linear() = compileRotation(
        mData.mQuat,
        mData.mAxisAngle,
        mData.mEuler,
        mData.mXYAxes,
        mData.mZAxis,
        compiler);
  }
  else
  {
    mWorldTransform.translation() = mData.mPos;
    mWorldTransform.linear() = compileRotation(
        mData.mQuat,
        mData.mAxisAngle,
        mData.mEuler,
        mData.mXYAxes,
        mData.mZAxis,
        compiler);
  }

  mMass = mData.mMass;

  if (mData.mDiagInertia)
  {
    mDiagonalInertia = *mData.mDiagInertia;
    mOffDiagonalInertia.setZero();
  }
  else
  {
    assert(mData.mFullInertia);
    mDiagonalInertia = mData.mFullInertia->head<3>();
    mOffDiagonalInertia = mData.mFullInertia->tail<3>();
  }

  return errors;
}

//==============================================================================
void Inertial::setMass(double mass)
{
  mMass = mass;
}

//==============================================================================
double Inertial::getMass() const
{
  return mMass;
}

//==============================================================================
void Inertial::setDiagInertia(const Eigen::Vector3d& inertia)
{
  mDiagonalInertia = inertia;
}

//==============================================================================
const Eigen::Vector3d& Inertial::getDiagInertia() const
{
  return mDiagonalInertia;
}

//==============================================================================
void Inertial::setOffDiagInertia(const Eigen::Vector3d& inertia)
{
  mOffDiagonalInertia = inertia;
}

//==============================================================================
const Eigen::Vector3d& Inertial::getOffDiagInertia() const
{
  return mOffDiagonalInertia;
}

//==============================================================================
void Inertial::setRelativeTransform(const Eigen::Isometry3d& tf)
{
  mRelativeTransform = tf;
}

//==============================================================================
const Eigen::Isometry3d& Inertial::getRelativeTransform() const
{
  return mRelativeTransform;
}

//==============================================================================
void Inertial::setWorldTransform(const Eigen::Isometry3d& tf)
{
  mWorldTransform = tf;
}

//==============================================================================
const Eigen::Isometry3d& Inertial::getWorldTransform() const
{
  return mWorldTransform;
}

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart
