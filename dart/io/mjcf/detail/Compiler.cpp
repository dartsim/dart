/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/io/mjcf/detail/Compiler.hpp"

#include <boost/filesystem.hpp>

#include "dart/io/XmlHelpers.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

//==============================================================================
void Compiler::setBaseUri(const common::Uri& baseUri)
{
  mBaseUri = baseUri;
}

//==============================================================================
const common::Uri& Compiler::getBaseUri() const
{
  return mBaseUri;
}

//==============================================================================
void Compiler::setResourceRetriever(
    const common::ResourceRetrieverPtr& retriever)
{
  mRetriever = retriever;
}

//==============================================================================
common::ResourceRetrieverPtr Compiler::getResourceRetriever() const
{
  return mRetriever;
}

//==============================================================================
Errors Compiler::read(tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "compiler")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <compiler> from the provided element");
    return errors;
  }

  // boundmass
  if (hasAttribute(element, "boundmass"))
  {
    mBoundMass = getAttributeDouble(element, "boundmass");
  }

  // boundinertia
  if (hasAttribute(element, "boundinertia"))
  {
    mBoundInertia = getAttributeDouble(element, "boundinertia");
  }

  // settotalmass
  if (hasAttribute(element, "settotalmass"))
  {
    mSetTotalMass = getAttributeDouble(element, "settotalmass");
  }

  // balanceinertia
  if (hasAttribute(element, "balanceinertia"))
  {
    mBalanceInertia = getAttributeBool(element, "balanceinertia");
  }

  // strippath
  if (hasAttribute(element, "strippath"))
  {
    mStripPath = getAttributeBool(element, "strippath");
  }

  // coordinate
  if (hasAttribute(element, "coordinate"))
  {
    const std::string coordiante = getAttributeString(element, "coordinate");
    if (coordiante == "local")
    {
      mCoordinate = Coordinate::LOCAL;
    }
    else if (coordiante == "global")
    {
      mCoordinate = Coordinate::GLOBAL;
    }
    else
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'coordinate': " + coordiante);
      return errors;
    }
  }

  // angle
  if (hasAttribute(element, "angle"))
  {
    const std::string angle = getAttributeString(element, "angle");
    if (angle == "degree")
    {
      mAngle = Angle::DEGREE;
    }
    else if (angle == "radian")
    {
      mAngle = Angle::RADIAN;
    }
    else
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'angle': " + angle);
      return errors;
    }
  }

  // fitaabb
  if (hasAttribute(element, "fitaabb"))
  {
    mFitAabb = getAttributeBool(element, "fitaabb");
  }
  if (mFitAabb)
  {
    dtwarn << "[MjcfParser] 'fitaabb' attribute is set to true, but DART does "
           << "not support this feature yet.\n";
  }

  // eulerseq
  if (hasAttribute(element, "eulerseq"))
  {
    mEulerSeq = getAttributeString(element, "eulerseq");
  }

  // meshdir
  if (hasAttribute(element, "meshdir"))
  {
    mMeshDir = getAttributeString(element, "meshdir");
  }

  // texturedir
  if (hasAttribute(element, "texturedir"))
  {
    mTextureDir = getAttributeString(element, "texturedir");
  }

  // discardvisual
  if (hasAttribute(element, "discardvisual"))
  {
    mDiscardVisual = getAttributeBool(element, "discardvisual");
  }

  // convexhull
  if (hasAttribute(element, "convexhull"))
  {
    mConvexHull = getAttributeBool(element, "convexhull");
  }

  // userthread
  if (hasAttribute(element, "userthread"))
  {
    mUserThread = getAttributeBool(element, "userthread");
  }

  // fusestatic
  if (hasAttribute(element, "fusestatic"))
  {
    mFuseStatic = getAttributeBool(element, "fusestatic");
  }

  // inertiafromgeom
  if (hasAttribute(element, "inertiafromgeom"))
  {
    const std::string inertiafromgeom
        = getAttributeString(element, "inertiafromgeom");
    if (inertiafromgeom == "false")
    {
#if DART_OS_WINDOWS
      mInertiaFromGeom = InertiaFromGeom::IFG_FALSE;
#else
      mInertiaFromGeom = InertiaFromGeom::FALSE;
#endif
    }
    else if (inertiafromgeom == "true")
    {
#if DART_OS_WINDOWS
      mInertiaFromGeom = InertiaFromGeom::IFG_TRUE;
#else
      mInertiaFromGeom = InertiaFromGeom::TRUE;
#endif
    }
    else if (inertiafromgeom == "auto")
    {
#if DART_OS_WINDOWS
      mInertiaFromGeom = InertiaFromGeom::IFG_AUTO;
#else
      mInertiaFromGeom = InertiaFromGeom::AUTO;
#endif
    }
    else
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'inertiafromgeom': " + inertiafromgeom);
      return errors;
    }
  }

  // inertiagrouprange
  if (hasAttribute(element, "inertiagrouprange"))
  {
    mInertiaGroupRange = getAttributeVector2i(element, "inertiagrouprange");
  }

  return errors;
}

//==============================================================================
double Compiler::getBoundInertia() const
{
  return mBoundInertia;
}

//==============================================================================
double Compiler::getSetTotalMass() const
{
  return mSetTotalMass;
}

//==============================================================================
bool Compiler::getBalanceInertia() const
{
  return mBalanceInertia;
}

//==============================================================================
bool Compiler::getStripPath() const
{
  return mStripPath;
}

//==============================================================================
double Compiler::getBoundMass() const
{
  return mBoundMass;
}

//==============================================================================
Coordinate Compiler::getCoordinate() const
{
  return mCoordinate;
}

//==============================================================================
Angle Compiler::getAngle() const
{
  return mAngle;
}

//==============================================================================
bool Compiler::getFitAabb() const
{
  return mFitAabb;
}

//==============================================================================
const std::string& Compiler::getEulerSeq() const
{
  return mEulerSeq;
}

//==============================================================================
const std::string& Compiler::getMeshDir() const
{
  return mMeshDir;
}

//==============================================================================
const std::string& Compiler::getTextureDir() const
{
  return mTextureDir;
}

//==============================================================================
bool Compiler::getDiscardVisual() const
{
  return mDiscardVisual;
}

//==============================================================================
bool Compiler::getConvexHull() const
{
  return mConvexHull;
}

//==============================================================================
bool Compiler::getUserThread() const
{
  return mUserThread;
}

//==============================================================================
bool Compiler::getFuseStatic() const
{
  return mFuseStatic;
}

//==============================================================================
InertiaFromGeom Compiler::getInertiaFromGeom() const
{
  return mInertiaFromGeom;
}

//==============================================================================
const Eigen::Vector2i& Compiler::getInertiaGroupRange() const
{
  return mInertiaGroupRange;
}

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart
