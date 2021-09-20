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

#include "dart/utils/mjcf/detail/Utils.hpp"

#include "dart/utils/CompositeResourceRetriever.hpp"
#include "dart/utils/DartResourceRetriever.hpp"
#include "dart/utils/XmlHelpers.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors checkOrientationValidity(const tinyxml2::XMLElement* element)
{
  Errors errors;

  std::size_t numOrientationTypes = 0;
  std::string orientationTypes;

  if (hasAttribute(element, "quat"))
  {
    orientationTypes += "quat";
    ++numOrientationTypes;
  }

  if (hasAttribute(element, "axisangle"))
  {
    if (!orientationTypes.empty())
      orientationTypes += ", ";
    orientationTypes += "axisangle";
    ++numOrientationTypes;
  }

  if (hasAttribute(element, "euler"))
  {
    if (!orientationTypes.empty())
      orientationTypes += ", ";
    orientationTypes += "euler";
    ++numOrientationTypes;
  }

  if (hasAttribute(element, "xyaxes"))
  {
    if (!orientationTypes.empty())
      orientationTypes += ", ";
    orientationTypes += "xyaxes";
    ++numOrientationTypes;
  }

  if (hasAttribute(element, "zaxis"))
  {
    if (!orientationTypes.empty())
      orientationTypes += ", ";
    orientationTypes += "zaxis";
    ++numOrientationTypes;
  }

  if (numOrientationTypes > 1)
  {
    errors.push_back(Error(
        ErrorCode::ATTRIBUTE_CONFLICT,
        "More than one orientation representations present: "
            + orientationTypes));
  }

  return errors;
}

//==============================================================================
Eigen::Matrix3d compileRotation(
    const Eigen::Quaterniond& quat,
    const common::optional<Eigen::Vector4d>& axisAngle,
    const common::optional<Eigen::Vector3d>& euler,
    const common::optional<Eigen::Vector6d>& xyAxes,
    const common::optional<Eigen::Vector3d>& zAxis,
    const Compiler& compiler)
{
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();

  if (axisAngle)
  {
    const Eigen::Vector3d axis = axisAngle->head<3>().normalized();
    double angle = (*axisAngle)[3];
    if (compiler.getAngle() == Angle::DEGREE)
    {
      angle = math::toRadian(angle);
    }
    rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    assert(math::verifyRotation(rot));
  }
  else if (euler)
  {
    Eigen::Vector3d angles = *euler;
    if (compiler.getAngle() == Angle::DEGREE)
    {
      angles[0] = math::toRadian(angles[0]);
      angles[1] = math::toRadian(angles[1]);
      angles[2] = math::toRadian(angles[2]);
    }

    if (compiler.getEulerSeq() == "xyz")
    {
      rot = math::eulerXYZToMatrix(angles);
      assert(math::verifyRotation(rot));
    }
    else if (compiler.getEulerSeq() == "zyx")
    {
      rot = math::eulerZYXToMatrix(angles);
      assert(math::verifyRotation(rot));
    }
    else
    {
      dterr << "[MjcfParser] Unsupported Euler angle sequence: '"
            << compiler.getEulerSeq() << "'. Please report this error. "
            << "This should be an easy fix.\n";
      rot.setIdentity();
    }
  }
  else if (xyAxes)
  {
    rot.col(0) = (*xyAxes).head<3>().normalized();                    // X axis
    rot.col(1) = (*xyAxes).tail<3>().normalized();                    // Y axis
    rot.col(2).noalias() = rot.col(0).cross(rot.col(1)).normalized(); // Z axis
    assert(math::verifyRotation(rot));
  }
  else if (zAxis)
  {
    rot = Eigen::Quaterniond::FromTwoVectors(
              Eigen::Vector3d::UnitZ(), zAxis->normalized())
              .toRotationMatrix();
    assert(math::verifyRotation(rot));
  }
  else
  {
    rot = quat.normalized().toRotationMatrix();
    assert(math::verifyRotation(rot));
  }

  return rot;
}

//==============================================================================
Errors handleInclude(
    tinyxml2::XMLElement* element,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  Errors errors;

  ElementEnumerator includeElements(element, "include");
  while (includeElements.next())
  {
    const std::string fileAttribute
        = getAttributeString(includeElements.get(), "file");
    const common::Uri mjcfUri
        = common::Uri::createFromRelativeUri(baseUri, fileAttribute);
    tinyxml2::XMLDocument mjcfDoc;
    if (!readXmlFile(mjcfDoc, mjcfUri, retriever))
    {
      errors.emplace_back(
          ErrorCode::FILE_READ, "Failed to load '" + mjcfUri.toString() + "'.");
    }

    // Get root <mujoco> element
    tinyxml2::XMLElement* mujocoElement = mjcfDoc.FirstChildElement("mujoco");
    if (mujocoElement == nullptr)
    {
      errors.emplace_back(
          ErrorCode::ELEMENT_MISSING, "Failed to find <mujoco> at the root");
      return errors;
    }

    const bool copyResult = copyChildNodes(element, *mujocoElement);
    if (!copyResult)
    {
      errors.push_back(
          Error(ErrorCode::FILE_READ, "Failed to handle <include>"));
    }
  }

  return errors;
}

//==============================================================================
std::vector<dynamics::BodyNode*> getBodyNodes(
    const simulation::World& world, const std::string& name)
{
  std::vector<dynamics::BodyNode*> bodyNodes;

  for (std::size_t i = 0; i < world.getNumSkeletons(); ++i)
  {
    dynamics::SkeletonPtr skel = world.getSkeleton(i);
    if (dynamics::BodyNode* bodyNode = skel->getBodyNode(name))
    {
      bodyNodes.push_back(bodyNode);
    }
  }

  return bodyNodes;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
