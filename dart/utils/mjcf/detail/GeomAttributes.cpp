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

#include "dart/utils/mjcf/detail/GeomAttributes.hpp"

#include "dart/utils/XmlHelpers.hpp"
#include "dart/utils/mjcf/detail/Utils.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors appendGeomAttributes(
    GeomAttributes& attributes, tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "geom")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <geom> from the provided element");
    return errors;
  }

  //-----------------
  // Read attributes
  //-----------------

  // name
  if (hasAttribute(element, "name"))
  {
    attributes.mName = getAttributeString(element, "name");
  }

  // type
  if (hasAttribute(element, "type"))
  {
    const std::string type = getAttributeString(element, "type");
    if (type == "plane")
    {
      attributes.mType = GeomType::PLANE;
    }
    else if (type == "hfield")
    {
      attributes.mType = GeomType::HFIELD;
    }
    else if (type == "sphere")
    {
      attributes.mType = GeomType::SPHERE;
    }
    else if (type == "capsule")
    {
      attributes.mType = GeomType::CAPSULE;
    }
    else if (type == "ellipsoid")
    {
      attributes.mType = GeomType::ELLIPSOID;
    }
    else if (type == "cylinder")
    {
      attributes.mType = GeomType::CYLINDER;
    }
    else if (type == "box")
    {
      attributes.mType = GeomType::BOX;
    }
    else if (type == "mesh")
    {
      attributes.mType = GeomType::MESH;
    }
    else
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'type': " + type);
      return errors;
    }
  }

  // contype
  if (hasAttribute(element, "contype"))
  {
    attributes.mConType = getAttributeInt(element, "contype");
  }

  // conaffinity
  if (hasAttribute(element, "conaffinity"))
  {
    attributes.mConAffinity = getAttributeInt(element, "conaffinity");
  }

  // condim
  if (hasAttribute(element, "condim"))
  {
    attributes.mConDim = getAttributeInt(element, "condim");
  }

  // group
  if (hasAttribute(element, "group"))
  {
    attributes.mGroup = getAttributeInt(element, "group");
  }

  // priority
  if (hasAttribute(element, "priority"))
  {
    attributes.mPriority = getAttributeInt(element, "priority");
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
    attributes.mSize.head(size.size()) = size;
  }

  // rgba
  if (hasAttribute(element, "rgba"))
  {
    attributes.mRGBA = getAttributeVector4d(element, "rgba");
  }

  // friction
  if (hasAttribute(element, "friction"))
  {
    const Eigen::VectorXd friction = getAttributeVectorXd(element, "friction");
    if (friction.size() == 0 || friction.size() > 3)
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID, "Invalid attribute for 'size'");
      return errors;
    }
    attributes.mFriction.head(friction.size()) = friction;
  }

  // mass
  if (hasAttribute(element, "mass"))
  {
    attributes.mMass = getAttributeDouble(element, "mass");
  }

  // density
  if (hasAttribute(element, "density"))
  {
    attributes.mDensity = getAttributeDouble(element, "density");
  }

  // solmix
  if (hasAttribute(element, "solmix"))
  {
    attributes.mSolMix = getAttributeDouble(element, "solmix");
  }

  // margin
  if (hasAttribute(element, "margin"))
  {
    attributes.mMargin = getAttributeDouble(element, "margin");
  }

  // gap
  if (hasAttribute(element, "gap"))
  {
    attributes.mGap = getAttributeDouble(element, "gap");
  }

  // fromto
  if (hasAttribute(element, "fromto"))
  {
    attributes.mFromTo = getAttributeVector6d(element, "fromto");
  }

  // pos
  if (hasAttribute(element, "pos"))
  {
    attributes.mPos = getAttributeVector3d(element, "pos");
  }

  // Check if multiple orientation representations present
  const Errors orientationErrors = checkOrientationValidity(element);
  errors.insert(
      errors.end(), orientationErrors.begin(), orientationErrors.end());

  // quat
  if (hasAttribute(element, "quat"))
  {
    const Eigen::Vector4d vec4d = getAttributeVector4d(element, "quat");
    attributes.mQuat.w() = vec4d[0];
    attributes.mQuat.x() = vec4d[1];
    attributes.mQuat.y() = vec4d[2];
    attributes.mQuat.z() = vec4d[3];
  }

  // axisangle
  if (hasAttribute(element, "axisangle"))
  {
    attributes.mAxisAngle = getAttributeVector4d(element, "axisangle");
  }

  // euler
  if (hasAttribute(element, "euler"))
  {
    attributes.mEuler = getAttributeVector3d(element, "euler");
  }

  // xyaxes
  if (hasAttribute(element, "xyaxes"))
  {
    attributes.mXYAxes = getAttributeVector6d(element, "xyaxes");
  }

  // zaxis
  if (hasAttribute(element, "zaxis"))
  {
    attributes.mZAxis = getAttributeVector3d(element, "zaxis");
  }

  // hfield
  if (hasAttribute(element, "hfield"))
  {
    attributes.mHField = getAttributeString(element, "hfield");
  }

  // mesh
  if (hasAttribute(element, "mesh"))
  {
    attributes.mMesh = getAttributeString(element, "mesh");
  }

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
