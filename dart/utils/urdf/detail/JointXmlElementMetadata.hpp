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

#pragma once

// This header is private and not installed.

#include "dart/utils/urdf/DartLoader.hpp"

#include <tinyxml2.h>

#include <string>

namespace dart {
namespace utils {
namespace detail {

inline JointXmlElementMetadata collectJointXmlElementMetadata(
    const std::string& urdfString)
{
  JointXmlElementMetadata metadata;

  tinyxml2::XMLDocument xmlDoc;
  if (xmlDoc.Parse(urdfString.c_str()) != tinyxml2::XML_SUCCESS)
    return metadata;

  auto* robotXml = xmlDoc.FirstChildElement("robot");
  if (!robotXml)
    return metadata;

  for (auto* jointXml = robotXml->FirstChildElement("joint");
       jointXml != nullptr;
       jointXml = jointXml->NextSiblingElement("joint")) {
    const char* name = jointXml->Attribute("name");
    if (!name)
      continue;

    if (jointXml->FirstChildElement("axis"))
      metadata.jointsWithAxisElement.emplace(name);

    auto* limitXml = jointXml->FirstChildElement("limit");
    if (limitXml) {
      if (limitXml->Attribute("lower"))
        metadata.jointsWithLimitLowerAttribute.emplace(name);
      if (limitXml->Attribute("upper"))
        metadata.jointsWithLimitUpperAttribute.emplace(name);
    }
  }

  return metadata;
}

} // namespace detail
} // namespace utils
} // namespace dart
