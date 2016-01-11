/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/utils/sdf/SoftSdfParser.h"

#include <map>
#include <iostream>
#include <fstream>

#include "dart/common/Console.h"
#include "dart/collision/dart/DARTCollisionDetector.h"
#include "dart/collision/fcl/FCLCollisionDetector.h"
// #include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"

#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
#include "dart/dynamics/SoftMeshShape.h"
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"

namespace kido {
namespace utils {

//==============================================================================
simulation::WorldPtr SoftSdfParser::readSoftSdfFile(
  const common::Uri& _fileUri, const common::ResourceRetrieverPtr& _retriever)
{
  return SdfParser::readSdfFile(_fileUri, getResourceRetriever(_retriever),
    static_cast<simulation::WorldPtr (*)(
      tinyxml2::XMLElement*, const std::string&,
      const common::ResourceRetrieverPtr&)>(&SoftSdfParser::readWorld));
}

//==============================================================================
dynamics::SkeletonPtr SoftSdfParser::readSkeleton(
    const common::Uri& _fileUri, const common::ResourceRetrieverPtr& _retriever)
{
  return SdfParser::readSkeleton(_fileUri, getResourceRetriever(_retriever),
    static_cast<dynamics::SkeletonPtr (*)(
      tinyxml2::XMLElement*, const std::string&,
      const common::ResourceRetrieverPtr&)>(&SoftSdfParser::readSkeleton));
}

bool SoftSdfParser::createSoftPair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const SDFJoint& newJoint,
    const SDFBodyNode& newBody)
{
  std::pair<dynamics::Joint*, dynamics::BodyNode*> pair;
  if(newBody.type.empty())
    pair = createJointAndNodePair<dynamics::BodyNode>(
          skeleton, parent, newJoint, newBody);
  else if(std::string("soft") == newBody.type)
    pair = createJointAndNodePair<dynamics::SoftBodyNode>(
          skeleton, parent, newJoint, newBody);
  else
  {
    dterr << "[SoftSdfParser::createSoftPair] Unsupported Link type: "
          << newBody.type << "\n";
    return false;
  }

  if(!pair.first || !pair.second)
    return false;

  return true;
}

simulation::WorldPtr SoftSdfParser::readWorld(
    tinyxml2::XMLElement* _worldElement,
    const std::string& _skelPath,
    const common::ResourceRetrieverPtr& _retriever)
{
  return SdfParser::readWorld(_worldElement, _skelPath, _retriever,
    static_cast<dynamics::SkeletonPtr (*)(
      tinyxml2::XMLElement*, const std::string&,
      const common::ResourceRetrieverPtr&)>(&SoftSdfParser::readSkeleton));
}

dynamics::SkeletonPtr SoftSdfParser::readSkeleton(
    tinyxml2::XMLElement* _skeletonElement,
    const std::string& _skelPath,
    const common::ResourceRetrieverPtr& _retriever)
{
  return SdfParser::readSkeleton(_skeletonElement, _skelPath, _retriever,
                                 &readSoftBodyNode, &createSoftPair);
}

SdfParser::SDFBodyNode SoftSdfParser::readSoftBodyNode(
    tinyxml2::XMLElement* _softBodyNodeElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const std::string& _skelPath,
    const common::ResourceRetrieverPtr& _retriever)
{
  //---------------------------------- Note ------------------------------------
  // SoftBodyNode is created if _softBodyNodeElement has <soft_shape>.
  // Otherwise, BodyNode is created.

  //----------------------------------------------------------------------------
  assert(_softBodyNodeElement != nullptr);

  // If _softBodyNodeElement has no <soft_shape>, return rigid body node
  if (!hasElement(_softBodyNodeElement, "soft_shape"))
  {
    return SdfParser::readBodyNode(
          _softBodyNodeElement, _skeletonFrame, _skelPath, _retriever);
  }

  SDFBodyNode standardSDF =
      SdfParser::readBodyNode(_softBodyNodeElement, _skeletonFrame, _skelPath, _retriever);

  BodyPropPtr standardProperties = standardSDF.properties;

  dynamics::SoftBodyNode::UniqueProperties softProperties;

  //----------------------------------------------------------------------------
  // Soft properties
  if (hasElement(_softBodyNodeElement, "soft_shape"))
  {
    tinyxml2::XMLElement* softShapeEle
        = getElement(_softBodyNodeElement, "soft_shape");

    // mass
    double totalMass = getValueDouble(softShapeEle, "total_mass");

    // pose
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (hasElement(softShapeEle, "pose"))
      T = getValueIsometry3dWithExtrinsicRotation(softShapeEle, "pose");

    // geometry
    tinyxml2::XMLElement* geometryEle = getElement(softShapeEle, "geometry");
    if (hasElement(geometryEle, "box"))
    {
      tinyxml2::XMLElement* boxEle = getElement(geometryEle, "box");
      Eigen::Vector3d size  = getValueVector3d(boxEle, "size");
      Eigen::Vector3i frags = getValueVector3i(boxEle, "frags");
      softProperties = dynamics::SoftBodyNodeHelper::makeBoxProperties(
            size, T, frags, totalMass);
    }
    else if (hasElement(geometryEle, "ellipsoid"))
    {
      tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "ellipsoid");
      Eigen::Vector3d size = getValueVector3d(ellipsoidEle, "size");
      double nSlices       = getValueDouble(ellipsoidEle, "num_slices");
      double nStacks       = getValueDouble(ellipsoidEle, "num_stacks");
      softProperties = dynamics::SoftBodyNodeHelper::makeEllipsoidProperties(
            size, nSlices, nStacks, totalMass);
    }
    else if (hasElement(geometryEle, "cylinder"))
    {
      tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "cylinder");
      double radius  = getValueDouble(ellipsoidEle, "radius");
      double height  = getValueDouble(ellipsoidEle, "height");
      double nSlices = getValueDouble(ellipsoidEle, "num_slices");
      double nStacks = getValueDouble(ellipsoidEle, "num_stacks");
      double nRings = getValueDouble(ellipsoidEle, "num_rings");
      softProperties = dynamics::SoftBodyNodeHelper::makeCylinderProperties(
            radius, height, nSlices, nStacks, nRings, totalMass);
    }
    else
    {
      dterr << "Unknown soft shape.\n";
    }

    // kv
    if (hasElement(softShapeEle, "kv"))
    {
      softProperties.mKv = getValueDouble(softShapeEle, "kv");
    }

    // ke
    if (hasElement(softShapeEle, "ke"))
    {
      softProperties.mKe = getValueDouble(softShapeEle, "ke");
    }

    // damp
    if (hasElement(softShapeEle, "damp"))
    {
      softProperties.mDampCoeff = getValueDouble(softShapeEle, "damp");
    }
  }

  SDFBodyNode sdfBodyNode;
  sdfBodyNode.properties =
      Eigen::make_aligned_shared<dynamics::SoftBodyNode::Properties>(
        *standardProperties, softProperties);
  sdfBodyNode.initTransform = standardSDF.initTransform;
  sdfBodyNode.type = "soft";

  return sdfBodyNode;
}

common::ResourceRetrieverPtr getResourceRetriever(
    const common::ResourceRetrieverPtr& _retriever);

}  // namespace utils
}  // namespace kido
