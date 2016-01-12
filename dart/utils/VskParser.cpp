/*
 * Copyright (c) 2012-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Matthew Dutton <MatthewRDutton@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/utils/VskParser.h"

// Standard Library
#include <map>
#include <stdexcept>

#include <Eigen/Dense>

// Local Files
#include "dart/common/LocalResourceRetriever.h"
#include "dart/common/Uri.h"
#include "dart/dynamics/dynamics.h"
#include "dart/utils/XmlHelpers.h"

#define SCALE_VSK 1.0e-3

namespace kido {
namespace utils {

namespace VskParser {

namespace {

using BodyPropPtr  = std::shared_ptr<dynamics::BodyNode::Properties>;
using JointPropPtr = std::shared_ptr<dynamics::Joint::Properties>;

using ParameterMap     = std::map<std::string, double>;
using BodyNodeColorMap = std::map<dynamics::BodyNode*, Eigen::Vector3d>;

struct VskData
{
  ParameterMap       parameterMap;
  BodyNodeColorMap   bodyNodeColorMap;
  VskParser::Options options;
};

const double vsk_scale = 1.0e-3;

const std::vector<std::string> vskJointTypes
  = {"JointFree", "JointBall", "JointHardySpicer", "JointHinge", "JointDummy"};

bool readParameters(tinyxml2::XMLElement* parametersEle,
                    ParameterMap& paramMap);

bool readSkeletonElement(const tinyxml2::XMLElement* skeletonEle,
                         dynamics::SkeletonPtr& skel,
                         VskData& vskData);

// Parsing Helper Functions
bool readSegment(const tinyxml2::XMLElement* segment,
                 dynamics::BodyNode* parent,
                 const dynamics::SkeletonPtr& skel,
                 VskData& vskData);

bool readJoint(const std::string& jointType,
               const tinyxml2::XMLElement* jointEle,
               JointPropPtr& jointProperties,
               const Eigen::Isometry3d& tfFromParent,
               const VskData& vskData);

bool readJointFree(const tinyxml2::XMLElement* jointEle,
                   JointPropPtr& jointProperties,
                   const Eigen::Isometry3d& tfFromParent,
                   const VskData& vskData);

bool readJointBall(const tinyxml2::XMLElement* jointEle,
                   JointPropPtr& jointProperties,
                   const Eigen::Isometry3d& tfFromParent,
                   const VskData& vskData);

bool readJointHardySpicer(const tinyxml2::XMLElement* jointEle,
                          JointPropPtr& jointProperties,
                          const Eigen::Isometry3d& tfFromParent,
                          const VskData& vskData);

bool readJointHinge(const tinyxml2::XMLElement* jointEle,
                    JointPropPtr& jointProperties,
                    const Eigen::Isometry3d& tfFromParent,
                    const VskData& vskData);

bool readJointDummy(const tinyxml2::XMLElement* jointEle,
                    JointPropPtr& jointProperties,
                    const Eigen::Isometry3d& tfFromParent,
                    const VskData& vskData);

Eigen::Vector3d readColorAttribute(const tinyxml2::XMLElement* element,
                                   const ParameterMap& parameterMap,
                                   const Eigen::Vector3d& defaultColor);

template <typename JointType>
std::pair<dynamics::Joint*, dynamics::BodyNode*> createJointAndBodyNodePair(
    const std::string& jointType,
    const dynamics::SkeletonPtr& skeleton,
    dynamics::BodyNode* parentBodyNode,
    const dynamics::Joint::Properties* jointProperties,
    const dynamics::BodyNode::Properties& bodyNodeProperties);

bool readMarkerSet(const tinyxml2::XMLElement* markerSetEle,
                   const dynamics::SkeletonPtr& skel,
                   VskData& vskData);

bool readMarker(const tinyxml2::XMLElement* marker,
                const dynamics::SkeletonPtr& skel,
                VskData& vskData);

bool readStick(const tinyxml2::XMLElement* stickEle,
               const dynamics::SkeletonPtr& skel,
               VskData& vskData);

void generateShapes(const dynamics::SkeletonPtr& skel,
                    VskData& vskData);

common::ResourceRetrieverPtr getRetriever(
  const common::ResourceRetrieverPtr& retriever);

void tokenize(const std::string& str,
              std::vector<std::string>& tokens,
              const std::string& delimiters = " ");

} // anonymous namespace

//==============================================================================
VskParser::Options::Options(const Eigen::Vector3d& newDefaultEllipsoidSize,
                            double newThicknessRatio,
                            double newDensity,
                            double newJointPositionLowerLimit,
                            double newJointPositionUpperLimit,
                            double newJointDampingCoefficient,
                            double newJointFriction)
  : defaultEllipsoidSize(newDefaultEllipsoidSize),
    thicknessRatio(newThicknessRatio),
    density(newDensity),
    jointPositionLowerLimit(newJointPositionLowerLimit),
    jointPositionUpperLimit(newJointPositionUpperLimit),
    jointDampingCoefficient(newJointDampingCoefficient),
    jointFriction(newJointFriction)
{
  // Do nothing
}

//==============================================================================
dynamics::SkeletonPtr readSkeleton(
    const common::Uri& fileUri,
    const common::ResourceRetrieverPtr& retrieverOrNullptr,
    const Options options)
{
  const common::ResourceRetrieverPtr retriever
      = getRetriever(retrieverOrNullptr);

  // Load VSK file and create document
  tinyxml2::XMLDocument vskDocument;
  try
  {
    openXMLFile(vskDocument, fileUri, retriever);
  }
  catch (std::exception const& e)
  {
    dtwarn << "[VskParser::readSkeleton] Failed to load file '"
           << fileUri.toString() << "': "
           << e.what() << std::endl;
    return nullptr;
  }

  // Check if <KinematicModel> is included in the document
  tinyxml2::XMLElement* kinematicModelEle
      = vskDocument.FirstChildElement("KinematicModel");
  if (!kinematicModelEle)
  {
    dtwarn << "[VskParser::readSkeleton] This file '" << fileUri.toString()
           << "' doesn't include 'KinematicModel' tag. "
           << "Returning null pointer instead.\n";
    return nullptr;
  }

  VskData vskData;
  vskData.options = options;

  // Read <Parameters> element
  tinyxml2::XMLElement* parametersEle
      = kinematicModelEle->FirstChildElement("Parameters");
  if (parametersEle)
    readParameters(parametersEle, vskData.parameterMap);

  // Read <Skeleton> element
  dynamics::SkeletonPtr newSkeleton = nullptr;
  tinyxml2::XMLElement* skelEle
      = kinematicModelEle->FirstChildElement("Skeleton");
  if (skelEle)
  {
    const bool result = readSkeletonElement(skelEle, newSkeleton, vskData);

    if (!result)
    {
      dtwarn << "[VskParser::readSkeleton] Failed to parse a <Segment> element "
             << "from file '" << fileUri.getPath() << "'. "
             << "Returning null pointer.\n";
    }
  }
  else
  {
    dtwarn << "[VskParser::readSkeleton] Failed to find <Skeleton> element "
           << "under <KinematicModel> element "
           << "from file '" << fileUri.getPath() << "'. "
           << "Returning null pointer.\n";
    return nullptr;
  }

  // Read <MarkerSet> elements and add all the markers to newSkeleton
  ElementEnumerator markerSet(kinematicModelEle, "MarkerSet");
  while (markerSet.next())
  {
    const bool result = readMarkerSet(markerSet.get(), newSkeleton, vskData);

    if (!result)
    {
      dtwarn << "[VskParser::readSkeleton] Failed to parse a marker from "
             << "file '" << fileUri.toString() << "'. Ignoring the marker.\n";
    }
  }
  // TODO: Each Marker is belongs to a MarkerSet but KIDO doesn't store the
  // marker set information.

  // TODO: Read sticks

  // fill in the default if prims absent
  generateShapes(newSkeleton, vskData);

  return newSkeleton;
}

namespace {

//==============================================================================
bool readParameters(tinyxml2::XMLElement* parametersEle, ParameterMap& paramMap)
{
  if (nullptr == parametersEle)
    return false;

  ElementEnumerator param(parametersEle, "Parameter");
  while (param.next())
  {
    const std::string pname
        = getAttributeString(param.get(), "NAME");
    const double val = getAttributeDouble(param.get(), "VALUE");

    paramMap[pname] = val;
  }

  return true;
}

//==============================================================================
bool readSkeletonElement(const tinyxml2::XMLElement* skeletonEle,
                         dynamics::SkeletonPtr& skel,
                         VskData& vskData)
{
  skel = dynamics::Skeleton::create();

  // Read all segments
  ConstElementEnumerator segment(skeletonEle, "Segment");
  while (segment.next())
  {
    if (!readSegment(segment.get(), nullptr, skel, vskData))
      return false;
  }

  return true;
}

//==============================================================================
double getParameter(const ParameterMap& ParameterMap,
                    const std::string& paramNameOrValue)
{
  assert(!paramNameOrValue.empty());

  int sign = 1;
  std::string paramNameOrValueWithoutSign = paramNameOrValue;

  if (paramNameOrValueWithoutSign[0] == '-')
  {
    sign = -1;
    paramNameOrValueWithoutSign.erase(paramNameOrValueWithoutSign.begin());
  }

  ParameterMap::const_iterator result
      = ParameterMap.find(paramNameOrValueWithoutSign);
  const bool found = result != ParameterMap.end();

  if (found)
    return sign * result->second;
  else
    return toDouble(paramNameOrValue);
}

//==============================================================================
template <size_t NumParams>
Eigen::Matrix<double, NumParams, 1> getParameters(
    const ParameterMap& ParameterMap,
    const std::string& paramNamesOrValues)
{
  std::vector<std::string> tokens;
  tokenize(paramNamesOrValues, tokens);

  assert(tokens.size() == NumParams);

  Eigen::Matrix<double, NumParams, 1> result;

  for (size_t i = 0; i < NumParams; ++i)
    result[i] = getParameter(ParameterMap, tokens[i]);

  return result;
}

//==============================================================================
template <size_t NumParams>
Eigen::Matrix<double, NumParams, 1> readAttributeVector(
    const tinyxml2::XMLElement* element,
    const std::string& name,
    const ParameterMap& parameterMap)
{
  const std::string positionStr = getAttributeString(element, name);

  return getParameters<NumParams>(parameterMap, positionStr);
}

//==============================================================================
bool readSegment(const tinyxml2::XMLElement* segment,
                 dynamics::BodyNode* parentBodyNode,
                 const dynamics::SkeletonPtr& skel,
                 VskData& vskData)
{
  // Attribute: NAME
  const std::string name = getAttributeString(segment, "NAME");

  // Attribute: POSITION
  // The position of the segment's joint attaching it to its parent in the
  // reference coordinate system of the parent segment.
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  if (hasAttribute(segment, "POSITION"))
  {
    position = readAttributeVector<3>(segment, "POSITION",
                                      vskData.parameterMap);
    position *= vsk_scale;
  }

  // Attribute: ORIENTATION
  Eigen::Vector3d orientation = Eigen::Vector3d::Zero();
  if (hasAttribute(segment, "ORIENTATION"))
  {
    orientation = readAttributeVector<3>(segment, "ORIENTATION",
                                         vskData.parameterMap);
  }

  // Attribute: BOUNDS
  Eigen::Vector6d bounds = Eigen::Vector6d::Zero();
  if (hasAttribute(segment, "BOUNDS"))
    bounds = readAttributeVector<6>(segment, "BOUNDS", vskData.parameterMap);

  // Attribute: RGB
  Eigen::Vector3d rgb = readColorAttribute(segment, vskData.parameterMap,
                                           Eigen::Vector3d::Constant(0.5));

  dynamics::BodyNode::Properties bodyNodeProperties;
  bodyNodeProperties.mName = name;

  Eigen::Isometry3d tfFromParent;
  tfFromParent.translation() = position;
  tfFromParent.linear()      = math::expMapRot(orientation);

  // Joint
  const tinyxml2::XMLElement* jointEle = nullptr;
  JointPropPtr jointProperties;
  std::string foundJointType;

  for (const auto& jointType : vskJointTypes)
  {
    jointEle = segment->FirstChildElement(jointType.c_str());
    if (jointEle)
    {
      foundJointType = jointType;
      const bool res = readJoint(jointType, jointEle, jointProperties,
                                 tfFromParent, vskData);

      if (!res)
      {
        dtwarn << "[ParserVsk::readSegment] Faild to parse joint type.\n";
        return false;
      }

      break;
    }
  }

  jointProperties->mName = "Joint-" + bodyNodeProperties.mName;

  // Create joint and body node
  dynamics::BodyNode* bodyNode = nullptr;

  auto pair = createJointAndBodyNodePair<dynamics::FreeJoint>(
        foundJointType, skel, parentBodyNode, jointProperties.get(),
        bodyNodeProperties);
  bodyNode = pair.second;
  assert(bodyNode != nullptr);

  vskData.bodyNodeColorMap[bodyNode] = rgb;

  // Read the subtree segments
  ConstElementEnumerator childSegment(segment, "Segment");
  while (childSegment.next())
  {
    if (!readSegment(childSegment->ToElement(), bodyNode, skel, vskData))
      return false;
  }
  return true;
}

//==============================================================================
bool readJoint(const std::string& jointType,
               const tinyxml2::XMLElement* jointEle,
               JointPropPtr& jointProperties,
               const Eigen::Isometry3d& tfFromParent,
               const VskData& vskData)
{
  if (jointType == "JointFree")
  {
    return readJointFree(jointEle, jointProperties, tfFromParent,
                         vskData);
  }
  else if (jointType == "JointBall")
  {
    return readJointBall(jointEle, jointProperties, tfFromParent,
                         vskData);
  }
  else if (jointType == "JointHardySpicer")
  {
    return readJointHardySpicer(jointEle, jointProperties, tfFromParent,
                                vskData);
  }
  else if (jointType == "JointHinge")
  {
    return readJointHinge(jointEle, jointProperties, tfFromParent,
                          vskData);
  }
  else if (jointType == "JointDummy")
  {
    return readJointDummy(jointEle, jointProperties, tfFromParent,
                          vskData);
  }
  else
  {
    dtwarn << "[ParserVsk::readSegment] Faild to parse joint type.\n";
    return false;
  }
}

//==============================================================================
bool readJointFree(const tinyxml2::XMLElement* /*jointEle*/,
                   JointPropPtr& jointProperties,
                   const Eigen::Isometry3d& tfFromParent,
                   const VskData& /*vskData*/)
{
  dynamics::FreeJoint::Properties properties;

  properties.mT_ParentBodyToJoint = tfFromParent;
  properties.mT_ChildBodyToJoint = Eigen::Isometry3d::Identity();

  jointProperties
      = Eigen::make_aligned_shared<dynamics::FreeJoint::Properties>(
        properties);

  return true;
}

//==============================================================================
bool readJointBall(const tinyxml2::XMLElement* jointEle,
                   JointPropPtr& jointProperties,
                   const Eigen::Isometry3d& tfFromParent,
                   const VskData& vskData)
{
  dynamics::BallJoint::Properties properties;

  properties.mT_ParentBodyToJoint = tfFromParent;
  properties.mT_ChildBodyToJoint = Eigen::Isometry3d::Identity();
  properties.mDampingCoefficients = Eigen::Vector3d::Constant(
        vskData.options.jointDampingCoefficient);
  properties.mPositionLowerLimits = Eigen::Vector3d::Constant(
        vskData.options.jointPositionLowerLimit);
  properties.mPositionUpperLimits = Eigen::Vector3d::Constant(
        vskData.options.jointPositionUpperLimit);
  properties.mIsPositionLimited = true;
  properties.mFrictions = Eigen::Vector3d::Constant(
        vskData.options.jointFriction);

  jointProperties
      = Eigen::make_aligned_shared<dynamics::BallJoint::Properties>(
        properties);

  return true;
}

//==============================================================================
bool readJointHardySpicer(const tinyxml2::XMLElement* jointEle,
                          JointPropPtr& jointProperties,
                          const Eigen::Isometry3d& tfFromParent,
                          const VskData& vskData)
{
  dynamics::UniversalJoint::Properties properties;

  // Attribute: AXIS-PAIR
  Eigen::Vector3d axis1 = Eigen::Vector3d::UnitX();
  Eigen::Vector3d axis2 = Eigen::Vector3d::UnitY();
  if (hasAttribute(jointEle, "AXIS-PAIR"))
  {
    Eigen::Vector6d axisPair
        = readAttributeVector<6>(jointEle, "AXIS-PAIR", vskData.parameterMap);
    axis1 = axisPair.head<3>();
    axis2 = axisPair.tail<3>();
  }

  properties.mT_ParentBodyToJoint = tfFromParent;
  properties.mT_ChildBodyToJoint = Eigen::Isometry3d::Identity();
  properties.mAxis[0] = axis1;
  properties.mAxis[1] = axis2;
  properties.mDampingCoefficients = Eigen::Vector2d::Constant(
        vskData.options.jointDampingCoefficient);
  properties.mPositionLowerLimits = Eigen::Vector2d::Constant(
        vskData.options.jointPositionLowerLimit);
  properties.mPositionUpperLimits = Eigen::Vector2d::Constant(
        vskData.options.jointPositionUpperLimit);
  properties.mIsPositionLimited = true;
  properties.mFrictions = Eigen::Vector2d::Constant(
        vskData.options.jointFriction);

  jointProperties
      = Eigen::make_aligned_shared<dynamics::UniversalJoint::Properties>(
        properties);

  return true;
}

//==============================================================================
bool readJointHinge(const tinyxml2::XMLElement* jointEle,
                    JointPropPtr& jointProperties,
                    const Eigen::Isometry3d& tfFromParent,
                    const VskData& vskData)
{
  dynamics::RevoluteJoint::Properties properties;

  // Attribute: AXIS
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
  if (hasAttribute(jointEle, "AXIS"))
    axis = readAttributeVector<3>(jointEle, "AXIS", vskData.parameterMap);

  properties.mT_ParentBodyToJoint = tfFromParent;
  properties.mT_ChildBodyToJoint = Eigen::Isometry3d::Identity();
  properties.mAxis = axis;
  properties.mDampingCoefficient = vskData.options.jointDampingCoefficient;
  properties.mPositionLowerLimit = vskData.options.jointPositionLowerLimit;
  properties.mPositionUpperLimit = vskData.options.jointPositionUpperLimit;
  properties.mIsPositionLimited = true;
  properties.mFriction = vskData.options.jointFriction;

  jointProperties
      = Eigen::make_aligned_shared<dynamics::RevoluteJoint::Properties>(
        properties);

  return true;
}

//==============================================================================
bool readJointDummy(const tinyxml2::XMLElement* /*jointEle*/,
                    JointPropPtr& jointProperties,
                    const Eigen::Isometry3d& tfFromParent,
                    const VskData& /*vskData*/)
{
  dynamics::WeldJoint::Properties properties;

  properties.mT_ParentBodyToJoint = tfFromParent;
  properties.mT_ChildBodyToJoint = Eigen::Isometry3d::Identity();

  jointProperties
      = Eigen::make_aligned_shared<dynamics::WeldJoint::Properties>(
        properties);

  return true;
}

//==============================================================================
Eigen::Vector3d readColorAttribute(const tinyxml2::XMLElement* element,
                                   const ParameterMap& parameterMap,
                                   const Eigen::Vector3d& defaultColor)
{
  if (hasAttribute(element, "RGB"))
    return readAttributeVector<3>(element, "RGB", parameterMap) / 255.0;
  else
    return defaultColor;
}

//==============================================================================
template <typename JointType>
std::pair<dynamics::Joint*, dynamics::BodyNode*> createJointAndBodyNodePair(
    const std::string& jointType,
    const dynamics::SkeletonPtr& skeleton,
    dynamics::BodyNode* parentBodyNode,
    const dynamics::Joint::Properties* jointProperties,
    const dynamics::BodyNode::Properties& bodyNodeProperties)
{
  if (jointType == "JointFree")
  {
    return skeleton->createJointAndBodyNodePair<
        dynamics::FreeJoint, dynamics::BodyNode>(
          parentBodyNode,
          *static_cast<const dynamics::FreeJoint::Properties*>(
            jointProperties),
          bodyNodeProperties);
  }
  else if (jointType == "JointBall")
  {
    return skeleton->createJointAndBodyNodePair<
        dynamics::BallJoint, dynamics::BodyNode>(
          parentBodyNode,
          *static_cast<const dynamics::BallJoint::Properties*>(
            jointProperties),
          bodyNodeProperties);
  }
  else if (jointType == "JointHardySpicer")
  {
    return skeleton->createJointAndBodyNodePair<
        dynamics::UniversalJoint, dynamics::BodyNode>(
          parentBodyNode,
          *static_cast<const dynamics::UniversalJoint::Properties*>(
            jointProperties),
          bodyNodeProperties);
  }
  else if (jointType == "JointHinge")
  {
    return skeleton->createJointAndBodyNodePair<
        dynamics::RevoluteJoint, dynamics::BodyNode>(
          parentBodyNode,
          *static_cast<const dynamics::RevoluteJoint::Properties*>(
            jointProperties),
          bodyNodeProperties);
  }
  else if (jointType == "JointDummy")
  {
    return skeleton->createJointAndBodyNodePair<
        dynamics::WeldJoint, dynamics::BodyNode>(
          parentBodyNode,
          *static_cast<const dynamics::WeldJoint::Properties*>(
            jointProperties),
          bodyNodeProperties);
  }
  else
  {
    dtwarn << "[ParserVsk::readSegment] Attempting to parse unsupported joint "
           << "type.\n";

    return std::pair<dynamics::WeldJoint*, dynamics::BodyNode*>(nullptr,
                                                                nullptr);
  }
}

//==============================================================================
bool readMarkerSet(const tinyxml2::XMLElement* markerSetEle,
                   const dynamics::SkeletonPtr& skel,
                   VskData& vskData)
{
  // std::string name = getAttributeString(markerSetEle, "NAME");

  // Read all <Marker> elements in <Markers> element
  const tinyxml2::XMLElement* markersEle
      = markerSetEle->FirstChildElement("Markers");
  ConstElementEnumerator marker(markersEle, "Marker");
  while (marker.next())
  {
    if (!readMarker(marker.get(), skel, vskData))
      return false;
  }

  // Read all <Stick> elements in <Sticks> element
  const tinyxml2::XMLElement* sticksEle
      = markerSetEle->FirstChildElement("Sticks");
  ConstElementEnumerator stick(sticksEle, "Stick");
  while (stick.next())
  {
    if (!readStick(stick.get(), skel, vskData))
      return false;
  }

  return true;
}

//==============================================================================
bool readMarker(const tinyxml2::XMLElement* markerEle,
                const dynamics::SkeletonPtr& skel,
                VskData& vskData)
{
  // Attribute: NAME
  const std::string name = getAttributeString(markerEle, "NAME");

  // Attribute: POSITION
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  if (hasAttribute(markerEle, "POSITION"))
  {
    position = readAttributeVector<3>(markerEle, "POSITION",
                                      vskData.parameterMap);
    position *= vsk_scale;
  }

  // Attribute: SEGMENT
  const std::string segment = getAttributeString(markerEle, "SEGMENT");

  // Attribute: COVARIANCE
  // Eigen::VectorXd covariance
  //     = getAttribute<Eigen::VectorXd>(markerEle, "COVARIANCE");

  // Attribute: RGB
  Eigen::Vector4d rgba = Eigen::Vector4d::Ones();
  rgba.head<3>() = readColorAttribute(markerEle, vskData.parameterMap,
                                      Eigen::Vector3d::Constant(0.5));

  // Attribute: RADIUS
  // double radius = 0.01;
  // if (hasAttribute(markerEle, "RADIUS"))
  //   radius = getAttributeDouble(markerEle, "RADIUS");

  dynamics::BodyNode* bodyNode = skel->getBodyNode(segment);
  if (!bodyNode)
  {
    dtwarn << "[VskParser::readMarker] Failed to create a Marker ["
           << name << ": couldn't find a BodyNode [" << segment << "] in a"
           << "Skeleton [" << skel->getName() << "].\n";
    return false;
  }

  dynamics::Marker* marker = new dynamics::Marker(name, position, rgba,
                                                  bodyNode);
  bodyNode->addMarker(marker);

  return true;
}

//==============================================================================
bool readStick(const tinyxml2::XMLElement* /*stickEle*/,
               const dynamics::SkeletonPtr& /*skel*/,
               VskData& /*vskData*/)
{
  // std::string marker1 = getAttributeString(stickEle, "MARKER1");
  // std::string marker2 = getAttributeString(stickEle, "MARKER2");

  // Attribute: RGB
  // Eigen::Vector3d rgb = readColorAttribute(segment, vskData.parameterMap,
  //                                          Eigen::Vector3d(0.5, 0.5, 0.5));

  return true;
}

//==============================================================================
void generateShapes(const dynamics::SkeletonPtr& skel, VskData& vskData)
{
  // Generate shapes for bodies that have their parents
  for (size_t i = 0; i < skel->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* bodyNode = skel->getBodyNode(i);
    dynamics::Joint*    joint    = skel->getJoint(i);
    Eigen::Isometry3d   tf       = joint->getTransformFromParentBodyNode();
    dynamics::BodyNode* parent   = bodyNode->getParentBodyNode();

    // Don't add shape for a body doesn't have parent or a body is too close to
    // the parent.
    if (!parent || tf.translation().norm() < KIDO_EPSILON)
      continue;

    // Determine the diameters of the ellipsoid shape. The diameter along X-axis
    // is the distance between the parent body and the current body. Other
    // diameters are 35% of the distance.
    Eigen::Vector3d size;
    size[0] = tf.translation().norm();
    size[1] = size[2] = vskData.options.thicknessRatio * size[0];

    // Determine the local transform of the shape
    Eigen::Isometry3d localTransform = Eigen::Isometry3d::Identity();
    localTransform.linear() = math::computeRotation(tf.translation(),
                                                    math::AxisType::AXIS_X);
    localTransform.translation() = 0.5 * tf.translation();

    dynamics::ShapePtr shape(new dynamics::EllipsoidShape(size));
    shape->setLocalTransform(localTransform);
    shape->setColor(vskData.bodyNodeColorMap[parent]);

    parent->addVisualizationShape(shape);
    parent->addCollisionShape(shape);
  }

  // Generate shpae for bodies with no shape
  for (size_t i = 0; i < skel->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* bodyNode = skel->getBodyNode(i);

    if (bodyNode->getNumVisualizationShapes() > 0)
      continue;

    // Use hard-coded size ellipsoid
    const Eigen::Vector3d& size = vskData.options.defaultEllipsoidSize;

    dynamics::ShapePtr shape(new dynamics::EllipsoidShape(size));
    shape->setColor(vskData.bodyNodeColorMap[bodyNode]);

    bodyNode->addVisualizationShape(shape);
    bodyNode->addCollisionShape(shape);
  }

  // Update mass and moments of inertia of the bodies based on the their shapes
  const double& density = vskData.options.density;
  for (size_t i = 0; i < skel->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* bodyNode = skel->getBodyNode(i);

    // Now all the bodies should have at least one shape
    assert(bodyNode->getNumVisualizationShapes() > 0);

    double totalMass = 0.0;
    Eigen::Matrix3d totalMoi = Eigen::Matrix3d::Zero();
    const auto& shapes = bodyNode->getVisualizationShapes();

    for (const dynamics::ShapePtr& shape : shapes)
    {
      const double             mass    = density * shape->getVolume();
      const Eigen::Isometry3d& localTf = shape->getLocalTransform();
      const Eigen::Matrix3d    moi     = shape->computeInertia(mass);

      totalMass += mass;
      totalMoi  += math::parallelAxisTheorem(moi, localTf.translation(), mass);
      // TODO(JS): We should take the orientation of the inertia into account,
      // but Inertia class doens't support it for now. See #234.
    }

    const dynamics::Inertia inertia(totalMass, Eigen::Vector3d::Zero(),
                                    totalMoi);
    bodyNode->setInertia(inertia);
  }
}

//==============================================================================
common::ResourceRetrieverPtr getRetriever(
  const common::ResourceRetrieverPtr& retriever)
{
  if (retriever)
    return retriever;
  else
    return std::make_shared<common::LocalResourceRetriever>();
}

//==============================================================================
void tokenize(const std::string& str,
              std::vector<std::string>& tokens,
              const std::string& delimiters)
{
  // Skip delimiters at beginning.
  std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);

  // Find first "non-delimiter".
  std::string::size_type pos = str.find_first_of(delimiters, lastPos);

  while (std::string::npos != pos || std::string::npos != lastPos)
  {
    // Found a token, add it to the vector.
    tokens.push_back(str.substr(lastPos, pos - lastPos));

    // Skip delimiters.  Note the "not_of"
    lastPos = str.find_first_not_of(delimiters, pos);

    // Find next "non-delimiter"
    pos = str.find_first_of(delimiters, lastPos);
  }
}

} // anonymous namespace

} // namespace VskParser

} // namespace utils
} // namespace kido

