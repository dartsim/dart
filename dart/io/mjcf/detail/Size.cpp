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

#include "dart/io/mjcf/detail/Size.hpp"

#include "dart/io/XmlHelpers.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors Size::read(tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "size")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <size> from the provided element");
    return errors;
  }

  // mjmax
  if (hasAttribute(element, "mjmax"))
  {
    mMJMax = getAttributeInt(element, "mjmax");
  }

  // nconmax
  if (hasAttribute(element, "nconmax"))
  {
    mNConMax = getAttributeInt(element, "nconmax");
  }

  // nstack
  if (hasAttribute(element, "nstack"))
  {
    mNStack = getAttributeInt(element, "nstack");
  }

  // nuserdata
  if (hasAttribute(element, "nuserdata"))
  {
    mNUserData = getAttributeInt(element, "nuserdata");
  }

  // nkey
  if (hasAttribute(element, "nkey"))
  {
    mNKey = getAttributeInt(element, "nkey");
  }

  // nuser_body
  if (hasAttribute(element, "nuser_body"))
  {
    mNUserBody = getAttributeInt(element, "nuser_body");
  }

  // nuser_jnt
  if (hasAttribute(element, "nuser_jnt"))
  {
    mNUserJnt = getAttributeInt(element, "nuser_jnt");
  }

  // nuser_geom
  if (hasAttribute(element, "nuser_geom"))
  {
    mNUserGeom = getAttributeInt(element, "nuser_geom");
  }

  // nuser_site
  if (hasAttribute(element, "nuser_site"))
  {
    mNUserSite = getAttributeInt(element, "nuser_site");
  }

  // nuser_cam
  if (hasAttribute(element, "nuser_cam"))
  {
    mNUserCam = getAttributeInt(element, "nuser_cam");
  }

  // nuser_tendon
  if (hasAttribute(element, "nuser_tendon"))
  {
    mNUserTendon = getAttributeInt(element, "nuser_tendon");
  }

  // nuser_actuator
  if (hasAttribute(element, "nuser_actuator"))
  {
    mNUserActuator = getAttributeInt(element, "nuser_actuator");
  }

  // nuser_sensor
  if (hasAttribute(element, "nuser_sensor"))
  {
    mNUserSensor = getAttributeInt(element, "nuser_sensor");
  }

  return errors;
}

//==============================================================================
int Size::getMJMax() const
{
  return mMJMax;
}

//==============================================================================
int Size::getNConMax() const
{
  return mNConMax;
}

//==============================================================================
int Size::getNStack() const
{
  return mNStack;
}

//==============================================================================
int Size::getNUserData() const
{
  return mNUserData;
}

//==============================================================================
int Size::getNKey() const
{
  return mNKey;
}

//==============================================================================
int Size::getNUserBody() const
{
  return mNUserBody;
}

//==============================================================================
int Size::getNUserJnt() const
{
  return mNUserJnt;
}

//==============================================================================
int Size::getNUserGeom() const
{
  return mNUserGeom;
}

//==============================================================================
int Size::getNUserSite() const
{
  return mNUserSite;
}

//==============================================================================
int Size::getNUserCam() const
{
  return mNUserCam;
}

//==============================================================================
int Size::getNUserTendon() const
{
  return mNUserTendon;
}

//==============================================================================
int Size::getNUserActuator() const
{
  return mNUserActuator;
}

//==============================================================================
int Size::getNUserSensor() const
{
  return mNUserSensor;
}

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart
