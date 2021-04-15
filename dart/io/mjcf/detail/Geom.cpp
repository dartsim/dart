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

#include "dart/io/mjcf/detail/Geom.hpp"

#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/io/XmlHelpers.hpp"
#include "dart/io/mjcf/detail/Body.hpp"
#include "dart/io/mjcf/detail/Utils.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors Geom::read(
    tinyxml2::XMLElement* element,
    const Defaults& defaults,
    const GeomAttributes& defaultAttributes)
{
  Errors errors;

  if (std::string(element->Name()) != "geom")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Geom> from the provided element");
    return errors;
  }

  // Initialize the attributes from proper default
  if (hasAttribute(element, "class"))
  {
    const std::string className = getAttributeString(element, "class");
    const auto& defaultClass = defaults.getDefault(className);
    if (defaultClass)
    {
      mAttributes = defaultClass->getGeomAttributes();
    }
    else
    {
      errors.push_back(Error(
          ErrorCode::ATTRIBUTE_INVALID,
          "Failed to find default with class name '" + className + "'"));
    }
  }
  else
  {
    mAttributes = defaultAttributes;
  }

  // Read attributes
  const Errors attrErrors = appendGeomAttributes(mAttributes, element);
  errors.insert(errors.end(), attrErrors.begin(), attrErrors.end());

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
Errors Geom::preprocess(const Compiler& compiler)
{
  Errors errors;

  if (mAttributes.mName)
  {
    mName = *mAttributes.mName;
  }

  mType = mAttributes.mType;
  mConType = mAttributes.mConType;
  mConAffinity = mAttributes.mConAffinity;
  mConDim = mAttributes.mConDim;
  mGroup = mAttributes.mGroup;
  mPriority = mAttributes.mPriority;
  mSize = mAttributes.mSize;
  mRGBA = mAttributes.mRGBA;
  mFriction = mAttributes.mFriction;

  mSolMix = mAttributes.mSolMix;
  mMargin = mAttributes.mMargin;
  mGap = mAttributes.mGap;

  // Size
  switch (mType)
  {
    case GeomType::PLANE:
    case GeomType::HFIELD:
    case GeomType::SPHERE:
    {
      mSize = mAttributes.mSize;
      break;
    }
    case GeomType::CAPSULE:
    case GeomType::CYLINDER:
    {
      if (mAttributes.mFromTo)
      {
        const double radius = mAttributes.mSize[0];
        mSize[0] = radius;

        const Eigen::Vector3d from = (*mAttributes.mFromTo).head<3>();
        const Eigen::Vector3d to = (*mAttributes.mFromTo).tail<3>();
        const double halfLength = 0.5 * (from - to).norm();
        mSize[1] = halfLength;
      }
      else
      {
        mSize = mAttributes.mSize;
      }
      break;
    }
    case GeomType::ELLIPSOID:
    case GeomType::BOX:
    {
      if (mAttributes.mFromTo)
      {
        const double halfLengthX = mAttributes.mSize[0];
        mSize[0] = halfLengthX;

        const double halfLengthY = mAttributes.mSize[1];
        mSize[1] = halfLengthY;

        const Eigen::Vector3d from = (*mAttributes.mFromTo).head<3>();
        const Eigen::Vector3d to = (*mAttributes.mFromTo).tail<3>();
        const double halfLengthZ = 0.5 * (from - to).norm();
        mSize[2] = halfLengthZ;
      }
      else
      {
        mSize = mAttributes.mSize;
      }
      break;
    }
    case GeomType::MESH:
    {
      break;
    }
  }

  if (mAttributes.mMass)
  {
    mMass = *mAttributes.mMass;
    mVolume = computeVolume();
    if (mVolume > 1e-6)
    {
      mDensity = mMass / mVolume;
    }
    mInertia = computeInertia();
  }
  else
  {
    mDensity = mAttributes.mDensity;
    mVolume = computeVolume();
    mMass = mDensity * mVolume;
    mInertia = computeInertia();
  }

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  if (canUseFromTo(mType, mAttributes.mFromTo))
  {
    assert(mAttributes.mFromTo);
    const Eigen::Vector6d& fromto = *mAttributes.mFromTo;
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
    tf.translation() = mAttributes.mPos;
    tf.linear() = compileRotation(
        mAttributes.mQuat,
        mAttributes.mAxisAngle,
        mAttributes.mEuler,
        mAttributes.mXYAxes,
        mAttributes.mZAxis,
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

  if (mAttributes.mType == GeomType::HFIELD)
  {
    if (mAttributes.mHField)
    {
      mHField = *mAttributes.mHField;
    }
    else
    {
      errors.push_back(Error(
          ErrorCode::ATTRIBUTE_MISSING,
          "Failed to find 'hfield' attribute when the geom type is set to "
          "hfield."));
    }
  }

  if (mAttributes.mMesh)
  {
    mMesh = *mAttributes.mMesh;

    // When 'type' attribute is specified to a geometric primitive, namely one
    // of "sphere", "capsule", "cylinder", "ellipsoid", "box", then the
    // primitive is automatically fitted to the mesh asset referenced here.
    //
    // The fitting procedure uses either the equivalent inertia box or the
    // axis-aligned bounding box of the mesh, as determined by the attribute
    // fitaabb of compiler. The resulting size of the fitted geom is usually
    // what one would expect, but if not, it can be  further adjusted with the
    // fitscale attribute below. In the compiled mjModel the geom is represented
    // as a regular geom of the specified primitive type, and there is no
    // reference to the mesh used for fitting.
    if (mType == GeomType::SPHERE || mType == GeomType::CAPSULE
        || mType == GeomType::CYLINDER || mType == GeomType::ELLIPSOID
        || mType == GeomType::BOX)
    {
      errors.push_back(Error(
          ErrorCode::UNDEFINED_ERROR,
          "Fitting primitive shapes to mesh is not supported yet. Setting "
          "mass, volume, density, and size to predefined values for now."));

      mMass = 1;
      mVolume = 1;
      mDensity = 1000;
      mSize.setConstant(0.1);
    }
  }
  else
  {
    // If 'type' attribute is specified to "mesh", then 'mesh' attribute must
    // be specified.
    if (mAttributes.mType == GeomType::MESH)
    {
      errors.push_back(Error(
          ErrorCode::ATTRIBUTE_MISSING,
          "Failed to find 'mesh' attribute when the geom type is set to "
          "mesh."));
    }
  }

  mFitScale = mAttributes.mFitScale;

  return errors;
}

//==============================================================================
Errors Geom::compile(const Compiler& /*compiler*/)
{
  Errors errors;
  return errors;
}

//==============================================================================
Errors Geom::postprocess(const Body* parent, const Compiler& compiler)
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
const std::string& Geom::getName() const
{
  return mName;
}

//==============================================================================
GeomType Geom::getType() const
{
  return mType;
}

//==============================================================================
int Geom::getConType() const
{
  return mConType;
}

//==============================================================================
int Geom::getConAffinity() const
{
  return mConAffinity;
}

//==============================================================================
int Geom::getConDim() const
{
  return mConDim;
}

//==============================================================================
int Geom::getGroup() const
{
  return mGroup;
}

//==============================================================================
int Geom::getPriority() const
{
  return mPriority;
}

//==============================================================================
const Eigen::Vector3d& Geom::getSize() const
{
  return mSize;
}

//==============================================================================
Eigen::Vector2d Geom::getPlaneHalfSize() const
{
  return mSize.head<2>();
}

//==============================================================================
double Geom::getSphereRadius() const
{
  return mSize[0];
}

//==============================================================================
double Geom::getCapsuleRadius() const
{
  return mSize[0];
}

//==============================================================================
double Geom::getCapsuleHalfLength() const
{
  return mSize[1];
}

//==============================================================================
double Geom::getCapsuleLength() const
{
  return 2.0 * mSize[1];
}

//==============================================================================
const Eigen::Vector3d& Geom::getEllipsoidRadii() const
{
  return mSize;
}

//==============================================================================
Eigen::Vector3d Geom::getEllipsoidDiameters() const
{
  return 2.0 * mSize;
}

//==============================================================================
double Geom::getCylinderRadius() const
{
  return mSize[0];
}

//==============================================================================
double Geom::getCylinderHalfLength() const
{
  return mSize[1];
}

//==============================================================================
double Geom::getCylinderLength() const
{
  return 2.0 * mSize[1];
}

//==============================================================================
const Eigen::Vector3d& Geom::getBoxHalfSize() const
{
  return mSize;
}

//==============================================================================
Eigen::Vector3d Geom::getBoxSize() const
{
  return 2.0 * mSize;
}

//==============================================================================
const Eigen::Vector4d& Geom::getRGBA() const
{
  return mRGBA;
}

//==============================================================================
const Eigen::Vector3d& Geom::getFriction() const
{
  return mFriction;
}

//==============================================================================
double Geom::getMass() const
{
  return mMass;
}

//==============================================================================
double Geom::getDensity() const
{
  return mDensity;
}

//==============================================================================
double Geom::getVolume() const
{
  return mVolume;
}

//==============================================================================
const Eigen::Matrix3d& Geom::getInertia() const
{
  return mInertia;
}

//==============================================================================
double Geom::getSolMix() const
{
  return mSolMix;
}

//==============================================================================
double Geom::getMargine() const
{
  return mMargin;
}

//==============================================================================
double Geom::getGap() const
{
  return mGap;
}

//==============================================================================
const std::string& Geom::getHField() const
{
  return mHField;
}

//==============================================================================
const std::string& Geom::getMesh() const
{
  return mMesh;
}

//==============================================================================
void Geom::setRelativeTransform(const Eigen::Isometry3d& tf)
{
  mRelativeTransform = tf;
}

//==============================================================================
const Eigen::Isometry3d& Geom::getRelativeTransform() const
{
  return mRelativeTransform;
}

//==============================================================================
void Geom::setWorldTransform(const Eigen::Isometry3d& tf)
{
  mWorldTransform = tf;
}

//==============================================================================
const Eigen::Isometry3d& Geom::getWorldTransform() const
{
  return mWorldTransform;
}

//==============================================================================
double Geom::computeVolume() const
{
  switch (mType)
  {
    case GeomType::SPHERE:
      return dynamics::SphereShape::computeVolume(getSphereRadius());
    case GeomType::CAPSULE:
      return dynamics::CapsuleShape::computeVolume(
          getCapsuleRadius(), getCapsuleLength());
    case GeomType::ELLIPSOID:
      return dynamics::EllipsoidShape::computeVolume(getEllipsoidDiameters());
    case GeomType::CYLINDER:
      return dynamics::CylinderShape::computeVolume(
          getCylinderRadius(), getCylinderLength());
    case GeomType::BOX:
      return dynamics::BoxShape::computeVolume(getBoxSize());
    default:
      // TODO(JS): Error handle?
      break;
  }

  return 1;
}

//==============================================================================
Eigen::Matrix3d Geom::computeInertia() const
{
  switch (mType)
  {
    case GeomType::SPHERE:
      return dynamics::SphereShape::computeInertia(getSphereRadius(), mMass);
    case GeomType::CAPSULE:
      return dynamics::CapsuleShape::computeInertia(
          getCapsuleRadius(), getCapsuleLength(), mMass);
    case GeomType::ELLIPSOID:
      return dynamics::EllipsoidShape::computeInertia(
          getEllipsoidDiameters(), mMass);
    case GeomType::CYLINDER:
      return dynamics::CylinderShape::computeInertia(
          getCylinderRadius(), getCylinderLength(), mMass);
    case GeomType::BOX:
      return dynamics::BoxShape::computeInertia(getBoxSize(), mMass);
    default:
      // TODO(JS): Error handle?
      break;
  }

  return Eigen::Matrix3d::Identity();
}

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart
