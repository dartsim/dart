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

#include "dart/utils/mjcf/detail/Site.hpp"

#include "dart/utils/XmlHelpers.hpp"
#include "dart/utils/mjcf/detail/Body.hpp"
#include "dart/utils/mjcf/detail/Utils.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors Site::read(tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "site")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Site> from the provided element");
    return errors;
  }

  //-----------------
  // Read attributes
  //-----------------

  // name
  if (hasAttribute(element, "name"))
  {
    mData.mName = getAttributeString(element, "name");
  }

  // type
  if (hasAttribute(element, "type"))
  {
    const std::string type = getAttributeString(element, "type");
    if (type == "sphere")
    {
      mData.mType = GeomType::SPHERE;
    }
    else if (type == "capsule")
    {
      mData.mType = GeomType::CAPSULE;
    }
    else if (type == "ellipsoid")
    {
      mData.mType = GeomType::ELLIPSOID;
    }
    else if (type == "cylinder")
    {
      mData.mType = GeomType::CYLINDER;
    }
    else if (type == "box")
    {
      mData.mType = GeomType::BOX;
    }
    else
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'type': " + type);
      return errors;
    }
  }

  // group
  if (hasAttribute(element, "group"))
  {
    mData.mGroup = getAttributeInt(element, "group");
  }

  // size
  if (hasAttribute(element, "size"))
  {
    const Eigen::VectorXd size = getAttributeVectorXd(element, "size");
    if (size.size() == 0 || size.size() > 3)
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID, "Invalid attribute for 'size'");
      return errors;
    }
    mData.mSize.head(size.size()) = size;
  }

  // rgba
  if (hasAttribute(element, "rgba"))
  {
    mData.mRGBA = getAttributeVector4d(element, "rgba");
  }

  // fromto
  if (hasAttribute(element, "fromto"))
  {
    mData.mFromTo = getAttributeVector6d(element, "fromto");
  }

  // pos
  if (hasAttribute(element, "pos"))
  {
    mData.mPos = getAttributeVector3d(element, "pos");
  }

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

  return errors;
}

//==============================================================================
static bool canUseFromTo(
    GeomType type, const common::optional<Eigen::Vector6d>& fromto)
{
  if (not fromto)
    return false;

  switch (type)
  {
    case detail::GeomType::CAPSULE:
    case detail::GeomType::ELLIPSOID:
    case detail::GeomType::CYLINDER:
    case detail::GeomType::BOX:
      return true;
    default:
      return false;
  }
}

//==============================================================================
Errors Site::preprocess(const Compiler& compiler)
{
  Errors errors;

  if (mData.mName)
  {
    mName = *mData.mName;
  }

  mType = mData.mType;

  mGroup = mData.mGroup;
  mSize = mData.mSize;
  mRGBA = mData.mRGBA;

  // Size
  switch (mData.mType)
  {
    case GeomType::PLANE:
    case GeomType::HFIELD:
    case GeomType::SPHERE:
    {
      mSize = mData.mSize;
      break;
    }
    case GeomType::CAPSULE:
    case GeomType::CYLINDER:
    {
      if (mData.mFromTo)
      {
        const double radius = mData.mSize[0];
        mSize[0] = radius;

        const Eigen::Vector3d from = (*mData.mFromTo).head<3>();
        const Eigen::Vector3d to = (*mData.mFromTo).tail<3>();
        const double halfLength = 0.5 * (from - to).norm();
        mSize[1] = halfLength;
      }
      else
      {
        mSize = mData.mSize;
      }
      break;
    }
    case GeomType::ELLIPSOID:
    case GeomType::BOX:
    {
      if (mData.mFromTo)
      {
        const double halfLengthX = mData.mSize[0];
        mSize[0] = halfLengthX;

        const double halfLengthY = mData.mSize[1];
        mSize[1] = halfLengthY;

        const Eigen::Vector3d from = (*mData.mFromTo).head<3>();
        const Eigen::Vector3d to = (*mData.mFromTo).tail<3>();
        const double halfLengthZ = 0.5 * (from - to).norm();
        mSize[2] = halfLengthZ;
      }
      else
      {
        mSize = mData.mSize;
      }
      break;
    }
    case GeomType::MESH:
    {
      break;
    }
  }

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  if (canUseFromTo(mData.mType, mData.mFromTo))
  {
    assert(mData.mFromTo);
    const Eigen::Vector6d& fromto = *mData.mFromTo;
    const Eigen::Vector3d from = fromto.head<3>();
    const Eigen::Vector3d to = fromto.tail<3>();
    const Eigen::Vector3d dir = (to - from).normalized();
    tf.translation() = (from + to) / 2.0;
    tf.linear()
        = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), dir)
              .toRotationMatrix();
  }
  else
  {
    tf.translation() = mData.mPos;
    tf.linear() = compileRotation(
        mData.mQuat,
        mData.mAxisAngle,
        mData.mEuler,
        mData.mXYAxes,
        mData.mZAxis,
        compiler);
  }

  if (compiler.getCoordinate() == Coordinate::LOCAL)
  {
    mRelativeTransform = tf;
  }
  else
  {
    mWorldTransform = tf;
  }

  return errors;
}

//==============================================================================
Errors Site::compile(const Compiler& /*compiler*/)
{
  Errors errors;
  return errors;
}

//==============================================================================
Errors Site::postprocess(const Body* parent, const Compiler& compiler)
{
  Errors errors;

  if (compiler.getCoordinate() == Coordinate::LOCAL)
  {
    if (parent != nullptr)
    {
      mWorldTransform = parent->getWorldTransform() * mRelativeTransform;
    }
    else
    {
      mWorldTransform = mRelativeTransform;
    }
  }
  else
  {
    if (parent != nullptr)
    {
      mRelativeTransform
          = parent->getWorldTransform().inverse() * mWorldTransform;
    }
    else
    {
      mRelativeTransform = mWorldTransform;
    }
  }

  return errors;
}

//==============================================================================
const std::string& Site::getName() const
{
  return mName;
}

//==============================================================================
GeomType Site::getType() const
{
  return mType;
}

//==============================================================================
int Site::getGroup() const
{
  return mGroup;
}

//==============================================================================
const Eigen::Vector3d& Site::getSize() const
{
  return mSize;
}

//==============================================================================
double Site::getSphereRadius() const
{
  return mSize[0];
}

//==============================================================================
double Site::getCapsuleRadius() const
{
  return mSize[0];
}

//==============================================================================
double Site::getCapsuleHalfLength() const
{
  return mSize[1];
}

//==============================================================================
double Site::getCapsuleLength() const
{
  return 2.0 * mSize[1];
}

//==============================================================================
const Eigen::Vector3d& Site::getEllipsoidRadii() const
{
  return mSize;
}

//==============================================================================
Eigen::Vector3d Site::getEllipsoidDiameters() const
{
  return 2.0 * mSize;
}

//==============================================================================
double Site::getCylinderRadius() const
{
  return mSize[0];
}

//==============================================================================
double Site::getCylinderHalfLength() const
{
  return mSize[1];
}

//==============================================================================
double Site::getCylinderLength() const
{
  return 2.0 * mSize[1];
}

//==============================================================================
const Eigen::Vector3d& Site::getBoxHalfSize() const
{
  return mSize;
}

//==============================================================================
Eigen::Vector3d Site::getBoxSize() const
{
  return 2.0 * mSize;
}

//==============================================================================
const Eigen::Vector4d& Site::getRGBA() const
{
  return mRGBA;
}

//==============================================================================
void Site::setRelativeTransform(const Eigen::Isometry3d& tf)
{
  mRelativeTransform = tf;
}

//==============================================================================
const Eigen::Isometry3d& Site::getRelativeTransform() const
{
  return mRelativeTransform;
}

//==============================================================================
void Site::setWorldTransform(const Eigen::Isometry3d& tf)
{
  mWorldTransform = tf;
}

//==============================================================================
const Eigen::Isometry3d& Site::getWorldTransform() const
{
  return mWorldTransform;
}

//==============================================================================
double Site::computeVolume() const
{
  // TODO(JS): Not implemented
  return 1;
}

//==============================================================================
Eigen::Matrix3d Site::computeInertia() const
{
  // TODO(JS): Not implemented
  return Eigen::Matrix3d::Identity();
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
