/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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

#include <algorithm>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dart/config.h"
#include "dart/common/Console.h"
#ifdef HAVE_BULLET_COLLISION
  #include "dart/collision/bullet/BulletCollisionDetector.h"
#endif
#include "dart/collision/dart/KIDOCollisionDetector.h"
#include "dart/collision/fcl/FCLCollisionDetector.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/SoftMeshShape.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/EulerJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/PlanarJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/Marker.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"
#include "dart/common/LocalResourceRetriever.h"
#include "dart/common/Uri.h"

namespace kido {
namespace utils {

//==============================================================================
static tinyxml2::XMLElement* checkFormatAndGetWorldElement(
    tinyxml2::XMLDocument& _document)
{
  //--------------------------------------------------------------------------
  // Check xml tag
  tinyxml2::XMLElement* skelElement = nullptr;
  skelElement = _document.FirstChildElement("skel");
  if (skelElement == nullptr)
  {
    dterr << "XML Document does not contain <skel> as the root element.\n";
    return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load World
  tinyxml2::XMLElement* worldElement = nullptr;
  worldElement = skelElement->FirstChildElement("world");
  if (worldElement == nullptr)
  {
    dterr << "XML Document does not contain a <world> element under the <skel> "
          << "element.\n";
    return nullptr;
  }

  return worldElement;
}

//==============================================================================
simulation::WorldPtr SkelParser::readWorld(
  const common::Uri& _uri,
  const common::ResourceRetrieverPtr& _retriever)
{
  const common::ResourceRetrieverPtr retriever = getRetriever(_retriever);

  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _dartFile;
  try
  {
    openXMLFile(_dartFile, _uri, retriever);
  }
  catch(std::exception const& e)
  {
    dterr << "[SkelParser::readWorld] LoadFile [" << _uri.toString()
          << "] Failed: " << e.what() << "\n";
    return nullptr;
  }

  tinyxml2::XMLElement* worldElement = checkFormatAndGetWorldElement(_dartFile);
  if(!worldElement)
  {
    dterr << "[SkelParser::readWorld] File named [" << _uri.toString()
          << "] could not be parsed!\n";
    return nullptr;
  }

  return readWorld(worldElement, _uri, retriever);
}

//==============================================================================
simulation::WorldPtr SkelParser::readWorldXML(
  const std::string& _xmlString,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _retriever)
{
  const common::ResourceRetrieverPtr retriever = getRetriever(_retriever);

  tinyxml2::XMLDocument _dartXML;
  if(_dartXML.Parse(_xmlString.c_str()) != tinyxml2::XML_SUCCESS)
  {
    _dartXML.PrintError();
    return nullptr;
  }

  tinyxml2::XMLElement* worldElement = checkFormatAndGetWorldElement(_dartXML);
  if(!worldElement)
  {
    dterr << "[SkelParser::readWorldXML] XML String could not be parsed!\n";
    return nullptr;
  }

  return readWorld(worldElement, _baseUri, retriever);
}

//==============================================================================
dynamics::SkeletonPtr SkelParser::readSkeleton(
  const common::Uri& _fileUri,
  const common::ResourceRetrieverPtr& _retriever)
{
  const common::ResourceRetrieverPtr retriever = getRetriever(_retriever);

  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _dartFile;
  try
  {
    openXMLFile(_dartFile, _fileUri, retriever);
  }
  catch(std::exception const& e)
  {
    std::cout << "LoadFile [" << _fileUri.toString() << "] Fails: "
              << e.what() << std::endl;
    return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load KIDO
  tinyxml2::XMLElement* skelElement = nullptr;
  skelElement = _dartFile.FirstChildElement("skel");
  if (skelElement == nullptr)
  {
    dterr << "Skel file[" << _fileUri.toString()
          << "] does not contain <skel> as the element.\n";
    return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load World
  tinyxml2::XMLElement* skeletonElement = nullptr;
  skeletonElement = skelElement->FirstChildElement("skeleton");
  if (skeletonElement == nullptr)
  {
    dterr << "Skel file[" << _fileUri.toString()
          << "] does not contain <skeleton> element "
          <<"under <skel> element.\n";
    return nullptr;
  }

  dynamics::SkeletonPtr newSkeleton = readSkeleton(
    skeletonElement, _fileUri, retriever);

  return newSkeleton;
}

//==============================================================================
simulation::WorldPtr SkelParser::readWorld(
  tinyxml2::XMLElement* _worldElement,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _retriever)
{
  assert(_worldElement != nullptr);

  // Create a world
  simulation::WorldPtr newWorld(new simulation::World);

  //--------------------------------------------------------------------------
  // Load physics
  tinyxml2::XMLElement* physicsElement
      = _worldElement->FirstChildElement("physics");
  if (physicsElement != nullptr)
  {
    // Time step
    tinyxml2::XMLElement* timeStepElement = nullptr;
    timeStepElement = physicsElement->FirstChildElement("time_step");
    if (timeStepElement != nullptr)
    {
      std::string strTimeStep = timeStepElement->GetText();
      double timeStep = toDouble(strTimeStep);
      newWorld->setTimeStep(timeStep);
    }

    // Gravity
    tinyxml2::XMLElement* gravityElement = nullptr;
    gravityElement = physicsElement->FirstChildElement("gravity");
    if (gravityElement != nullptr)
    {
      std::string strGravity = gravityElement->GetText();
      Eigen::Vector3d gravity = toVector3d(strGravity);
      newWorld->setGravity(gravity);
    }

    // Collision detector
    if (hasElement(physicsElement, "collision_detector"))
    {
      std::string strCD = getValueString(physicsElement, "collision_detector");
      if (strCD == "fcl_mesh")
      {
        newWorld->getConstraintSolver()->setCollisionDetector(
              new collision::FCLMeshCollisionDetector());
      }
      else if (strCD == "fcl")
      {
        newWorld->getConstraintSolver()->setCollisionDetector(
              new collision::FCLCollisionDetector());
      }
      else if (strCD == "dart")
      {
        newWorld->getConstraintSolver()->setCollisionDetector(
              new collision::KIDOCollisionDetector());
      }
#ifdef HAVE_BULLET_COLLISION
      else if (strCD == "bullet")
      {
        newWorld->getConstraintSolver()->setCollisionDetector(
              new collision::BulletCollisionDetector());
      }
#endif
      else
      {
        dtwarn << "Unknown collision detector[" << strCD << "]. "
               << "Default collision detector[fcl] will be loaded."
               << std::endl;
      }
    }
    else
    {
      newWorld->getConstraintSolver()->setCollisionDetector(
            new collision::FCLMeshCollisionDetector());
    }
  }

  //--------------------------------------------------------------------------
  // Load soft skeletons
  ElementEnumerator SkeletonElements(_worldElement, "skeleton");
  while (SkeletonElements.next())
  {
    dynamics::SkeletonPtr newSkeleton
        = readSkeleton(SkeletonElements.get(), _baseUri, _retriever);

    newWorld->addSkeleton(newSkeleton);
  }

  return newWorld;
}

//==============================================================================
enum NextResult
{
  VALID,
  CONTINUE,
  BREAK,
  CREATE_FREEJOINT_ROOT
};

//==============================================================================
static NextResult getNextJointAndNodePair(
    SkelParser::JointMap::iterator& it,
    SkelParser::BodyMap::const_iterator& child,
    dynamics::BodyNode*& parent,
    const dynamics::SkeletonPtr skeleton,
    SkelParser::JointMap& joints,
    const SkelParser::BodyMap& bodyNodes)
{
  NextResult result = VALID;
  const SkelParser::SkelJoint& joint = it->second;
  parent = skeleton->getBodyNode(joint.parentName);
  if(nullptr == parent && !joint.parentName.empty())
  {
    // Find the properties of the parent Joint of the current Joint, because it
    // does not seem to be created yet.
    SkelParser::JointMap::iterator check_parent_joint =
        joints.find(joint.parentName);
    if(check_parent_joint == joints.end())
    {
      SkelParser::BodyMap::const_iterator check_parent_node =
          bodyNodes.find(joint.parentName);
      if(check_parent_node == bodyNodes.end())
      {
        dterr << "[SkelParser::getNextJointAndNodePair] Could not find BodyNode "
              << "named [" << joint.parentName << "] requested as parent of "
              << "the Joint named [" << joint.properties->mName << "]. We will "
              << "now quit parsing.\n";
        return BREAK;
      }

      // If the current Joint has a parent BodyNode but does not have a parent
      // Joint, then we need to create a FreeJoint for the parent BodyNode.
      result = CREATE_FREEJOINT_ROOT;
    }
    else
    {
      it = check_parent_joint;
      return CONTINUE; // Create the parent before creating the current Joint
    }
  }

  // Find the child node of this Joint, so we can create them together
  child = bodyNodes.find(joint.childName);
  if(child == bodyNodes.end())
  {
    dterr << "[SkelParser::getNextJointAndNodePair] Could not find BodyNode "
          << "named [" << joint.childName << "] requested as child of Joint ["
          << joint.properties->mName << "]. This should not be possible! "
          << "We will now quit parsing. Please report this bug!\n";
    return BREAK;
  }

  return result;
}

//==============================================================================
template <typename BodyType>
std::pair<dynamics::Joint*, dynamics::BodyNode*> createJointAndNodePair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const SkelParser::SkelJoint& joint,
    const typename BodyType::Properties& body)
{
  if(std::string("weld") == joint.type)
    return skeleton->createJointAndBodyNodePair<dynamics::WeldJoint, BodyType>(parent,
      static_cast<const dynamics::WeldJoint::Properties&>(*joint.properties), body);

  else if(std::string("prismatic") == joint.type)
    return skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint, BodyType>(parent,
      static_cast<const dynamics::PrismaticJoint::Properties&>(*joint.properties), body);

  else if(std::string("revolute") == joint.type)
    return skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint, BodyType>(parent,
      static_cast<const dynamics::RevoluteJoint::Properties&>(*joint.properties), body);

  else if(std::string("universal") == joint.type)
    return skeleton->createJointAndBodyNodePair<dynamics::UniversalJoint, BodyType>(parent,
      static_cast<const dynamics::UniversalJoint::Properties&>(*joint.properties), body);

  else if(std::string("ball") == joint.type)
    return skeleton->createJointAndBodyNodePair<dynamics::BallJoint, BodyType>(parent,
      static_cast<const dynamics::BallJoint::Properties&>(*joint.properties), body);

  else if(std::string("euler") == joint.type)
    return skeleton->createJointAndBodyNodePair<dynamics::EulerJoint, BodyType>(parent,
      static_cast<const dynamics::EulerJoint::Properties&>(*joint.properties), body);

  else if(std::string("translational") == joint.type)
    return skeleton->createJointAndBodyNodePair<dynamics::TranslationalJoint, BodyType>(parent,
      static_cast<const dynamics::TranslationalJoint::Properties&>(*joint.properties), body);

  else if(std::string("planar") == joint.type)
    return skeleton->createJointAndBodyNodePair<dynamics::PlanarJoint, BodyType>(parent,
      static_cast<const dynamics::PlanarJoint::Properties&>(*joint.properties), body);

  else if(std::string("free") == joint.type)
    return skeleton->createJointAndBodyNodePair<dynamics::FreeJoint, BodyType>(parent,
      static_cast<const dynamics::FreeJoint::Properties&>(*joint.properties), body);

  else
  {
    dterr << "[SkelParser::createJointAndNodePair] Unsupported Joint type ("
          << joint.type << ") for Joint named [" << joint.properties->mName
          << "]! It will be discarded.\n";
    return std::pair<dynamics::Joint*, dynamics::BodyNode*>(nullptr, nullptr);
  }
}

//==============================================================================
bool createJointAndNodePair(dynamics::SkeletonPtr skeleton,
                            dynamics::BodyNode* parent,
                            const SkelParser::SkelJoint& joint,
                            const SkelParser::SkelBodyNode& body)
{
  std::pair<dynamics::Joint*, dynamics::BodyNode*> pair;
  if(body.type.empty())
    pair = createJointAndNodePair<dynamics::BodyNode>(skeleton, parent, joint,
      static_cast<const dynamics::BodyNode::Properties&>(*body.properties));
  else if(std::string("soft") == body.type)
    pair = createJointAndNodePair<dynamics::SoftBodyNode>(skeleton, parent, joint,
      static_cast<const dynamics::SoftBodyNode::Properties&>(*body.properties));
  else
  {
    dterr << "[SkelParser::createJointAndNodePair] Invalid type (" << body.type
          << ") for BodyNode named [" << body.properties->mName << "]\n";
    return false;
  }

  if(pair.first == nullptr || pair.second == nullptr)
    return false;

  dynamics::Joint* newJoint = pair.first;
  newJoint->setPositions(joint.position);
  newJoint->setVelocities(joint.velocity);
  newJoint->setAccelerations(joint.acceleration);
  newJoint->setForces(joint.force);

  return true;
}

//==============================================================================
dynamics::SkeletonPtr SkelParser::readSkeleton(
    tinyxml2::XMLElement* _skeletonElement,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever)
{
  assert(_skeletonElement != nullptr);

  dynamics::SkeletonPtr newSkeleton = dynamics::Skeleton::create();
  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttributeString(_skeletonElement, "name");
  newSkeleton->setName(name);

  //--------------------------------------------------------------------------
  // transformation
  if (hasElement(_skeletonElement, "transformation"))
  {
    Eigen::Isometry3d W =
        getValueIsometry3d(_skeletonElement, "transformation");
    skeletonFrame = W;
  }

  //--------------------------------------------------------------------------
  // immobile attribute
  tinyxml2::XMLElement* mobileElement = nullptr;
  mobileElement = _skeletonElement->FirstChildElement("mobile");
  if (mobileElement != nullptr)
  {
    newSkeleton->setMobile(toBool(mobileElement->GetText()));
  }

  //--------------------------------------------------------------------------
  // Bodies
  BodyMap bodyNodes;
  ElementEnumerator xmlBodies(_skeletonElement, "body");
  while (xmlBodies.next())
  {
    SkelBodyNode newBodyNode = readSoftBodyNode(
      xmlBodies.get(), skeletonFrame, _baseUri, _retriever);

    BodyMap::const_iterator it = bodyNodes.find(newBodyNode.properties->mName);
    if(it != bodyNodes.end())
    {
      dterr << "[SkelParser::readSkeleton] Skeleton named [" << name << "] has "
            << "multiple BodyNodes with the name ["
            << newBodyNode.properties->mName << "], but BodyNode names must be "
            << "unique! We will discard all BodyNodes with a repeated name.\n";
      continue;
    }

    bodyNodes[newBodyNode.properties->mName] = newBodyNode;
  }

  //--------------------------------------------------------------------------
  // Joints
  JointMap joints;
  IndexToJoint order;
  JointToIndex lookup;
  ElementEnumerator xmlJoints(_skeletonElement, "joint");
  while (xmlJoints.next())
    readJoint(xmlJoints.get(), bodyNodes, joints, order, lookup);

  //--------------------------------------------------------------------------
  // Assemble skeleton
  JointMap::iterator it = joints.find(order.begin()->second);
  BodyMap::const_iterator child;
  dynamics::BodyNode* parent;
  while(it != joints.end())
  {
    NextResult result = getNextJointAndNodePair(
          it, child, parent, newSkeleton, joints, bodyNodes);

    if(BREAK == result)
      break;
    else if(CONTINUE == result)
      continue;
    else if(CREATE_FREEJOINT_ROOT == result)
    {
      // If a root FreeJoint is needed for the parent of the current joint, then
      // create it
      BodyMap::const_iterator rootNode = bodyNodes.find(it->second.parentName);
      SkelJoint rootJoint;
      rootJoint.properties =
          Eigen::make_aligned_shared<dynamics::FreeJoint::Properties>(
            dynamics::Joint::Properties("root", rootNode->second.initTransform));
      rootJoint.type = "free";

      if(!createJointAndNodePair(newSkeleton, nullptr, rootJoint,
                                 rootNode->second))
        break;

      continue;
    }

    if(!createJointAndNodePair(newSkeleton, parent, it->second, child->second))
      break;

    JointToIndex::iterator index = lookup.find(it->first);
    order.erase(index->second);
    lookup.erase(index);
    joints.erase(it);

    IndexToJoint::iterator nextJoint = order.begin();
    if(nextJoint == order.end())
      break;

    it = joints.find(nextJoint->second);
  }

  newSkeleton->resetPositions();
  newSkeleton->resetVelocities();

  return newSkeleton;
}

//==============================================================================
SkelParser::SkelBodyNode SkelParser::readBodyNode(
    tinyxml2::XMLElement* _bodyNodeElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever)
{
  assert(_bodyNodeElement != nullptr);

  BodyPropPtr newBodyNode(new dynamics::BodyNode::Properties);
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  newBodyNode->mName = getAttributeString(_bodyNodeElement, "name");

  //--------------------------------------------------------------------------
  // gravity
  if (hasElement(_bodyNodeElement, "gravity"))
  {
    newBodyNode->mGravityMode = getValueBool(_bodyNodeElement, "gravity");
  }

  //--------------------------------------------------------------------------
  // self_collide
  //    if (hasElement(_bodyElement, "self_collide"))
  //    {
  //        bool gravityMode = getValueBool(_bodyElement, "self_collide");
  //    }

  //--------------------------------------------------------------------------
  // transformation
  if (hasElement(_bodyNodeElement, "transformation"))
  {
    Eigen::Isometry3d W =
        getValueIsometry3d(_bodyNodeElement, "transformation");
    initTransform = _skeletonFrame * W;
  }
  else
  {
    initTransform = _skeletonFrame;
  }

  //--------------------------------------------------------------------------
  // visualization_shape
  ElementEnumerator vizShapes(_bodyNodeElement, "visualization_shape");
  while (vizShapes.next())
  {
    dynamics::ShapePtr newShape = readShape(
      vizShapes.get(), newBodyNode->mName, _baseUri, _retriever);

    if(newShape)
      newBodyNode->mVizShapes.push_back(newShape);
  }

  //--------------------------------------------------------------------------
  // visualization_shape
  ElementEnumerator collShapes(_bodyNodeElement, "collision_shape");
  while (collShapes.next())
  {
    dynamics::ShapePtr newShape = readShape(
      collShapes.get(), newBodyNode->mName, _baseUri, _retriever);

    if(newShape)
      newBodyNode->mColShapes.push_back(newShape);
  }

  //--------------------------------------------------------------------------
  // inertia
  if (hasElement(_bodyNodeElement, "inertia"))
  {
    tinyxml2::XMLElement* inertiaElement =
        getElement(_bodyNodeElement, "inertia");

    // mass
    double mass = getValueDouble(inertiaElement, "mass");
    newBodyNode->mInertia.setMass(mass);

    // moment of inertia
    if (hasElement(inertiaElement, "moment_of_inertia"))
    {
      tinyxml2::XMLElement* moiElement
          = getElement(inertiaElement, "moment_of_inertia");

      double ixx = getValueDouble(moiElement, "ixx");
      double iyy = getValueDouble(moiElement, "iyy");
      double izz = getValueDouble(moiElement, "izz");

      double ixy = getValueDouble(moiElement, "ixy");
      double ixz = getValueDouble(moiElement, "ixz");
      double iyz = getValueDouble(moiElement, "iyz");

      newBodyNode->mInertia.setMoment(ixx, iyy, izz, ixy, ixz, iyz);
    }
    else if (newBodyNode->mVizShapes.size() > 0)
    {
      Eigen::Matrix3d Ic =
          newBodyNode->mVizShapes[0]->computeInertia(mass);

      newBodyNode->mInertia.setMoment(Ic);
    }

    // offset
    if (hasElement(inertiaElement, "offset"))
    {
      Eigen::Vector3d offset = getValueVector3d(inertiaElement, "offset");
      newBodyNode->mInertia.setLocalCOM(offset);
    }
  }

  //--------------------------------------------------------------------------
  // marker
  ElementEnumerator markers(_bodyNodeElement, "marker");
  while (markers.next())
  {
    newBodyNode->mMarkerProperties.push_back(readMarker(markers.get()));
  }

  SkelBodyNode skelBodyNode;
  skelBodyNode.properties = newBodyNode;
  skelBodyNode.initTransform = initTransform;

  return skelBodyNode;
}

//==============================================================================
SkelParser::SkelBodyNode SkelParser::readSoftBodyNode(
    tinyxml2::XMLElement* _softBodyNodeElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever)
{
  //---------------------------------- Note ------------------------------------
  // SoftBodyNode is created if _softBodyNodeElement has <soft_shape>.
  // Otherwise, BodyNode is created.

  //----------------------------------------------------------------------------
  assert(_softBodyNodeElement != nullptr);

  SkelBodyNode standardBodyNode = readBodyNode(
    _softBodyNodeElement, _skeletonFrame, _baseUri, _retriever);

  // If _softBodyNodeElement has no <soft_shape>, return rigid body node
  if (!hasElement(_softBodyNodeElement, "soft_shape"))
    return standardBodyNode;

  //----------------------------------------------------------------------------
  // Soft properties
  dynamics::SoftBodyNode::UniqueProperties newSoftBodyNode;

  if (hasElement(_softBodyNodeElement, "soft_shape"))
  {
    tinyxml2::XMLElement* softShapeEle
        = getElement(_softBodyNodeElement, "soft_shape");

    // mass
    double totalMass = getValueDouble(softShapeEle, "total_mass");

    // transformation
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (hasElement(softShapeEle, "transformation"))
      T = getValueIsometry3d(softShapeEle, "transformation");

    // geometry
    tinyxml2::XMLElement* geometryEle = getElement(softShapeEle, "geometry");
    if (hasElement(geometryEle, "box"))
    {
      tinyxml2::XMLElement* boxEle = getElement(geometryEle, "box");
      Eigen::Vector3d size  = getValueVector3d(boxEle, "size");
      Eigen::Vector3i frags = getValueVector3i(boxEle, "frags");
      newSoftBodyNode = dynamics::SoftBodyNodeHelper::makeBoxProperties(
            size, T, frags, totalMass);
    }
    else if (hasElement(geometryEle, "ellipsoid"))
    {
      tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "ellipsoid");
      Eigen::Vector3d size = getValueVector3d(ellipsoidEle, "size");
      double nSlices       = getValueDouble(ellipsoidEle, "num_slices");
      double nStacks       = getValueDouble(ellipsoidEle, "num_stacks");
      newSoftBodyNode = dynamics::SoftBodyNodeHelper::makeEllipsoidProperties(
            size,
            nSlices,
            nStacks,
            totalMass);
    }
    else if (hasElement(geometryEle, "cylinder"))
    {
      tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "cylinder");
      double radius  = getValueDouble(ellipsoidEle, "radius");
      double height  = getValueDouble(ellipsoidEle, "height");
      double nSlices = getValueDouble(ellipsoidEle, "num_slices");
      double nStacks = getValueDouble(ellipsoidEle, "num_stacks");
      double nRings = getValueDouble(ellipsoidEle, "num_rings");
      newSoftBodyNode = dynamics::SoftBodyNodeHelper::makeCylinderProperties(
            radius,
            height,
            nSlices,
            nStacks,
            nRings,
            totalMass);
    }
    else
    {
      dterr << "[SkelParser::readSoftBodyNode] Unknown soft shape in "
            << "SoftBodyNode named [" << standardBodyNode.properties->mName
            << "]\n";
    }

    // kv
    if (hasElement(softShapeEle, "kv"))
    {
      newSoftBodyNode.mKv = getValueDouble(softShapeEle, "kv");
    }

    // ke
    if (hasElement(softShapeEle, "ke"))
    {
      newSoftBodyNode.mKe = getValueDouble(softShapeEle, "ke");
    }

    // damp
    if (hasElement(softShapeEle, "damp"))
    {
      newSoftBodyNode.mDampCoeff = getValueDouble(softShapeEle, "damp");
    }
  }

  SkelBodyNode softBodyNode;
  softBodyNode.properties =
      Eigen::make_aligned_shared<dynamics::SoftBodyNode::Properties>(
          *standardBodyNode.properties, newSoftBodyNode);

  softBodyNode.initTransform = standardBodyNode.initTransform;
  softBodyNode.type = "soft";

  return softBodyNode;
}

//==============================================================================
dynamics::ShapePtr SkelParser::readShape(
    tinyxml2::XMLElement* vizEle,
    const std::string& bodyName,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever)
{
  dynamics::ShapePtr newShape;

  // Geometry
  assert(hasElement(vizEle, "geometry"));
  tinyxml2::XMLElement* geometryEle = getElement(vizEle, "geometry");

  if (hasElement(geometryEle, "box"))
  {
    tinyxml2::XMLElement* boxEle       = getElement(geometryEle, "box");
    Eigen::Vector3d       size         = getValueVector3d(boxEle, "size");
    newShape = dynamics::ShapePtr(new dynamics::BoxShape(size));
  }
  else if (hasElement(geometryEle, "ellipsoid"))
  {
    tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "ellipsoid");
    Eigen::Vector3d       size         = getValueVector3d(ellipsoidEle, "size");
    newShape = dynamics::ShapePtr(new dynamics::EllipsoidShape(size));
  }
  else if (hasElement(geometryEle, "cylinder"))
  {
    tinyxml2::XMLElement* cylinderEle  = getElement(geometryEle, "cylinder");
    double                radius       = getValueDouble(cylinderEle, "radius");
    double                height       = getValueDouble(cylinderEle, "height");
    newShape = dynamics::ShapePtr(new dynamics::CylinderShape(radius, height));
  }
  else if (hasElement(geometryEle, "plane"))
  {
    tinyxml2::XMLElement* planeEle = getElement(geometryEle, "plane");
    Eigen::Vector3d       normal   = getValueVector3d(planeEle, "normal");
    if (hasElement(planeEle, "offset")) {
      double offset = getValueDouble(planeEle, "offset");
      newShape = dynamics::ShapePtr(new dynamics::PlaneShape(normal, offset));
    } else if (hasElement(planeEle, "point")) {
      dtwarn << "[SkelParser::readShape] <point> element of <plane> is "
             << "deprecated as of KIDO 4.3. Please use <offset> element "
             << "instead." << std::endl;
      Eigen::Vector3d point = getValueVector3d(planeEle, "point");
      newShape = dynamics::ShapePtr(new dynamics::PlaneShape(normal, point));
    } else {
      dtwarn << "[SkelParser::readShape] <offset> element is not specified for "
             << "plane shape. KIDO will use 0.0." << std::endl;
      newShape = dynamics::ShapePtr(new dynamics::PlaneShape(normal, 0.0));
    }
  }
  else if (hasElement(geometryEle, "mesh"))
  {
    tinyxml2::XMLElement* meshEle      = getElement(geometryEle, "mesh");
    std::string           filename     = getValueString(meshEle, "file_name");
    Eigen::Vector3d       scale        = getValueVector3d(meshEle, "scale");

    const std::string meshUri = common::Uri::getRelativeUri(_baseUri, filename);
    const aiScene* model = dynamics::MeshShape::loadMesh(meshUri, _retriever);
    if (model)
    {
      newShape = std::make_shared<dynamics::MeshShape>(
        scale, model, meshUri, _retriever);
    }
    else
    {
      dterr << "Fail to load model[" << filename << "]." << std::endl;
    }
  }
  else
  {
    dterr << "[SkelParser::readShape] Unknown visualization shape in BodyNode "
          << "named [" << bodyName << "]\n";
    assert(0);
    return nullptr;
  }

  // transformation
  if (hasElement(vizEle, "transformation")) {
    Eigen::Isometry3d W = getValueIsometry3d(vizEle, "transformation");
    newShape->setLocalTransform(W);
  }

  // color
  if (hasElement(vizEle, "color")) {
    Eigen::Vector3d color = getValueVector3d(vizEle, "color");
    newShape->setColor(color);
  }

  return newShape;
}

//==============================================================================
dynamics::Marker::Properties SkelParser::readMarker(
    tinyxml2::XMLElement* _markerElement)
{
  // Name attribute
  std::string name = getAttributeString(_markerElement, "name");

  // offset
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();
  if (hasElement(_markerElement, "offset"))
    offset = getValueVector3d(_markerElement, "offset");

  dynamics::Marker::Properties newMarker(name, offset);

  return newMarker;
}

void SkelParser::readJoint(tinyxml2::XMLElement* _jointElement,
    const BodyMap& _bodyNodes,
    JointMap& _joints,
    IndexToJoint& _order,
    JointToIndex& _lookup)
{
  assert(_jointElement != nullptr);

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttributeString(_jointElement, "name");

  SkelJoint joint;

  //--------------------------------------------------------------------------
  // Type attribute
  joint.type = getAttributeString(_jointElement, "type");
  assert(!joint.type.empty());
  if (joint.type == std::string("weld"))
    joint.properties = readWeldJoint(_jointElement, joint, name);
  else if (joint.type == std::string("prismatic"))
    joint.properties = readPrismaticJoint(_jointElement, joint, name);
  else if (joint.type == std::string("revolute"))
    joint.properties = readRevoluteJoint(_jointElement, joint, name);
  else if (joint.type == std::string("universal"))
    joint.properties = readUniversalJoint(_jointElement, joint, name);
  else if (joint.type == std::string("ball"))
    joint.properties = readBallJoint(_jointElement, joint, name);
  else if (joint.type == std::string("euler"))
    joint.properties = readEulerJoint(_jointElement, joint, name);
  else if (joint.type == std::string("translational"))
    joint.properties = readTranslationalJoint(_jointElement, joint, name);
  else if (joint.type == std::string("planar"))
    joint.properties = readPlanarJoint(_jointElement, joint, name);
  else if (joint.type == std::string("free"))
    joint.properties = readFreeJoint(_jointElement, joint, name);
  else
  {
    dterr << "[SkelParser::readJoint] Unsupported joint type [" << joint.type
          << "] requested by Joint named [" << name << "]. This Joint will be "
          << "discarded.\n";
    return;
  }
  assert(joint.properties != nullptr);

  joint.properties->mName = name;

  //--------------------------------------------------------------------------
  // Actuator attribute
  if (hasAttribute(_jointElement, "actuator"))
  {
    const std::string actuator = getAttributeString(_jointElement, "actuator");

    if (actuator == "force")
      joint.properties->mActuatorType = dynamics::Joint::FORCE;
    else if (actuator == "passive")
      joint.properties->mActuatorType = dynamics::Joint::PASSIVE;
    else if (actuator == "servo")
      joint.properties->mActuatorType = dynamics::Joint::SERVO;
    else if (actuator == "acceleration")
      joint.properties->mActuatorType = dynamics::Joint::ACCELERATION;
    else if (actuator == "velocity")
      joint.properties->mActuatorType = dynamics::Joint::VELOCITY;
    else if (actuator == "locked")
      joint.properties->mActuatorType = dynamics::Joint::LOCKED;
    else
      dterr << "Joint named [" << name
            << "] contains invalid actuator attribute ["
            << actuator << "].\n";
  }
  else
  {
    joint.properties->mActuatorType = dynamics::Joint::DefaultActuatorType;
  }

  //--------------------------------------------------------------------------
  // parent
  BodyMap::const_iterator parent = _bodyNodes.end();
  if (hasElement(_jointElement, "parent"))
  {
    joint.parentName = getValueString(_jointElement, "parent");
    parent = _bodyNodes.find(joint.parentName);
  }
  else
  {
    dterr << "[SkelParser::readJoint] Joint named [" << name << "] is missing "
          << "a parent BodyNode!\n";
    assert(0);
  }

  // Use an empty string (rather than "world") to indicate that the joint has no parent
  if(joint.parentName == std::string("world")
     && _bodyNodes.find("world") == _bodyNodes.end())
    joint.parentName.clear();

  if(parent == _bodyNodes.end() && !joint.parentName.empty())
  {
    dterr << "[SkelParser::readJoint] Could not find a BodyNode named ["
          << joint.parentName << "] requested as the parent of Joint named ["
          << name << "]!\n";
    return;
  }

  //--------------------------------------------------------------------------
  // child
  BodyMap::const_iterator child = _bodyNodes.end();
  if (hasElement(_jointElement, "child"))
  {
    joint.childName = getValueString(_jointElement, "child");
    child = _bodyNodes.find(joint.childName);
  }
  else
  {
    dterr << "[SkelParser::readJoint] Joint named [" << name << "] is missing "
          << "a child BodyNode!\n";
    assert(0);
  }

  if(child == _bodyNodes.end())
  {
    dterr << "[SkelParser::readJoint] Could not find a BodyNode named ["
          << joint.childName << "] requested as the child of Joint named ["
          << name << "]!\n";
    return;
  }

  //--------------------------------------------------------------------------
  // transformation
  Eigen::Isometry3d parentWorld = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childWorld = child->second.initTransform;

  if (parent != _bodyNodes.end())
    parentWorld = parent->second.initTransform;

  if (hasElement(_jointElement, "transformation"))
    childToJoint = getValueIsometry3d(_jointElement, "transformation");

  Eigen::Isometry3d parentToJoint =
      parentWorld.inverse()*childWorld*childToJoint;

  joint.properties->mT_ParentBodyToJoint = parentToJoint;
  if(!math::verifyTransform(joint.properties->mT_ParentBodyToJoint))
    dterr << "[SkelParser::readJoint] Invalid parent to Joint transform for "
          << "Joint named [" << name << "]:\n"
          << joint.properties->mT_ParentBodyToJoint.matrix() << "\n";

  joint.properties->mT_ChildBodyToJoint = childToJoint;
  if(!math::verifyTransform(joint.properties->mT_ChildBodyToJoint))
    dterr << "[SkelParser::readJoint] Invalid child to Joint transform for "
          << "Joint named [" << name << "]:\n"
          << joint.properties->mT_ChildBodyToJoint.matrix() << "\n";

  JointMap::iterator it = _joints.find(joint.childName);
  if(it != _joints.end())
  {
    dterr << "[SkelParser::readJoint] BodyNode named [" << joint.childName
          << "] has been assigned two parent Joints: ["
          << it->second.properties->mName << "] and [" << name << "]. A "
          << "BodyNode must have exactly one parent Joint. [" << name << "] "
          << "will be discarded!\n";
    return;
  }

  _joints[joint.childName] = joint;

  // Keep track of when each joint appeared in the file
  size_t nextIndex;
  IndexToJoint::reverse_iterator lastIndex = _order.rbegin();
  if(lastIndex == _order.rend())
    nextIndex = 0;
  else
    nextIndex = lastIndex->first + 1;

  _order[nextIndex] = joint.childName;
  _lookup[joint.childName] = nextIndex;
}

//==============================================================================
static void getDofAttributeIfItExists(
    const std::string& _attribute,
    double* _value,
    const std::string& _element_type,
    const tinyxml2::XMLElement* _xmlElement,
    const std::string& _jointName,
    size_t _index)
{
  if (_xmlElement->QueryDoubleAttribute(_attribute.c_str(), _value)
      == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE)
  {
    dterr << "[SkelParser::getDofAttributeIfItExists] Invalid type for ["
          << _attribute << "] attribute of [" << _element_type
          << "] element in the [" << _index << "] dof of Joint ["
          << _jointName << "].\n";
  }
}

//==============================================================================
static void setDofLimitAttributes(tinyxml2::XMLElement* _dofElement,
                                  const std::string& _element_type,
                                  const std::string& _jointName,
                                  size_t _index,
                                  double* lower, double* upper, double* initial)
{
  const tinyxml2::XMLElement* xmlElement
      = getElement(_dofElement, _element_type);

  getDofAttributeIfItExists("lower", lower, _element_type, xmlElement, _jointName, _index);
  getDofAttributeIfItExists("upper", upper, _element_type, xmlElement, _jointName, _index);
  getDofAttributeIfItExists("initial", initial, _element_type, xmlElement, _jointName, _index);
}

//==============================================================================
// This structure exists to allow a common interface for setting values in both
// SingleDofJoint::Properties and MultiDofJoint::Properties
struct DofProxy
{
  size_t index;
  bool valid;

  double* lowerPosition;
  double* upperPosition;
  double* initalPosition;

  double* lowerVelocity;
  double* upperVelocity;
  double* initialVelocity;

  double* lowerAcceleration;
  double* upperAcceleration;
  double* initialAcceleration;

  double* lowerForce;
  double* upperForce;
  double* initialForce;

  double* springStiffness;
  double* restPosition;
  double* dampingCoefficient;
  double* friction;

  bool* preserveName;
  std::string* name;

  DofProxy(dynamics::SingleDofJoint::Properties& properties,
           SkelParser::SkelJoint& joint, size_t _index,
           const std::string& jointName)
    : index(_index),
      valid(true),

      lowerPosition(&properties.mPositionLowerLimit),
      upperPosition(&properties.mPositionUpperLimit),
      initalPosition(&properties.mInitialPosition),

      lowerVelocity(&properties.mVelocityLowerLimit),
      upperVelocity(&properties.mVelocityUpperLimit),
      initialVelocity(&properties.mInitialVelocity),

      lowerAcceleration(&properties.mAccelerationLowerLimit),
      upperAcceleration(&properties.mAccelerationUpperLimit),
      initialAcceleration(&joint.acceleration.data()[0]),

      lowerForce(&properties.mForceLowerLimit),
      upperForce(&properties.mForceUpperLimit),
      initialForce(&joint.force.data()[0]),

      springStiffness(&properties.mSpringStiffness),
      restPosition(&properties.mRestPosition),
      dampingCoefficient(&properties.mDampingCoefficient),
      friction(&properties.mFriction),

      preserveName(&properties.mPreserveDofName),
      name(&properties.mDofName)
  {
    if(index > 0)
    {
      dterr << "[SkelParser] Joint named [" << jointName << "] has a dof "
            << "element (" << index << ") which is out of bounds (max 0)\n";
      valid = false;
    }
  }

  template <typename PropertyType>
  DofProxy(PropertyType& properties,
           SkelParser::SkelJoint& joint, size_t _index,
           const std::string& jointName)
    : index(_index),
      valid(true),

      lowerPosition(&properties.mPositionLowerLimits.data()[index]),
      upperPosition(&properties.mPositionUpperLimits.data()[index]),
      initalPosition(&properties.mInitialPositions.data()[index]),

      lowerVelocity(&properties.mVelocityLowerLimits.data()[index]),
      upperVelocity(&properties.mVelocityUpperLimits.data()[index]),
      initialVelocity(&properties.mInitialVelocities.data()[index]),

      lowerAcceleration(&properties.mAccelerationLowerLimits.data()[index]),
      upperAcceleration(&properties.mAccelerationUpperLimits.data()[index]),
      initialAcceleration(&joint.acceleration.data()[index]),

      lowerForce(&properties.mForceLowerLimits.data()[index]),
      upperForce(&properties.mForceUpperLimits.data()[index]),
      initialForce(&joint.force.data()[index]),

      springStiffness(&properties.mSpringStiffnesses.data()[index]),
      restPosition(&properties.mRestPositions.data()[index]),
      dampingCoefficient(&properties.mDampingCoefficients.data()[index]),
      friction(&properties.mFrictions.data()[index]),

      preserveName(&properties.mPreserveDofNames[index]),
      name(&properties.mDofNames[index])
  {
    if((int)index >= properties.mPositionLowerLimits.size())
    {
      dterr << "[SkelParser] Joint named [" << jointName << "] has a dof "
            << "element (" << index << ") which is out of bounds (max "
            << properties.mPositionLowerLimits.size()-1 << ")\n";
      valid = false;
    }
  }
};

//==============================================================================
template <typename PropertyType>
static void readAllDegreesOfFreedom(tinyxml2::XMLElement* _jointElement,
                                    PropertyType& _properties,
                                    SkelParser::SkelJoint& _joint,
                                    const std::string& _jointName,
                                    size_t _numDofs)
{
  if(_joint.position.size() < (int)_numDofs)
  {
    _joint.position.resize(_numDofs);
    _joint.position.setZero();
  }

  if(_joint.velocity.size() < (int)_numDofs)
  {
    _joint.velocity.resize(_numDofs);
    _joint.velocity.setZero();
  }

  if(_joint.acceleration.size() < (int)_numDofs)
  {
    _joint.acceleration.resize((int)_numDofs);
    _joint.acceleration.setZero();
  }

  if(_joint.force.size() < (int)_numDofs)
  {
    _joint.force.resize(_numDofs);
    _joint.force.setZero();
  }

  ElementEnumerator DofElements(_jointElement, "dof");
  while(DofElements.next())
    readDegreeOfFreedom(DofElements.get(), _properties,
                        _joint, _jointName, _numDofs);
}

//==============================================================================
template <typename PropertyType>
static void readDegreeOfFreedom(tinyxml2::XMLElement* _dofElement,
                                PropertyType& properties,
                                SkelParser::SkelJoint& joint,
                                const std::string& jointName,
                                size_t numDofs)
{
  int localIndex = -1;
  int xml_err = _dofElement->QueryIntAttribute("local_index", &localIndex);

  // If the localIndex is out of bounds, quit
  if (localIndex >= (int)numDofs)
  {
    dterr << "[SkelParser::readDegreeOfFreedom] Joint named '"
          << jointName << "' contains dof element with invalid "
          << "number attribute [" << localIndex << "]. It must be less than "
          << numDofs << ".\n";
    return;
  }

  // If no localIndex was found, report an error and quit
  if (localIndex == -1 && numDofs > 1)
  {
    if (tinyxml2::XML_NO_ATTRIBUTE == xml_err)
    {
      dterr << "[SkelParser::readDegreeOfFreedom] Joint named ["
            << jointName << "] has [" << numDofs
            << "] DOFs, but the xml contains a dof element without its "
            << "local_index specified. For Joints with multiple DOFs, all dof "
            << "elements must specify their local_index attribute.\n";
    }
    else if (tinyxml2::XML_WRONG_ATTRIBUTE_TYPE == xml_err)
    {
      dterr << "[SkelParser::readDegreeOfFreedom] Joint named ["
            << jointName << "] has a dof element with a wrongly "
            << "formatted local_index attribute.\n";
    }

    return;
  }
  // Unless the joint is a single-dof joint
  else if (localIndex == -1 && numDofs == 1)
    localIndex = 0;

  DofProxy proxy(properties, joint, localIndex, jointName);

  const char* name = _dofElement->Attribute("name");
  if (name)
  {
    *proxy.name = std::string(name);
    *proxy.preserveName = true;
  }

  if (hasElement(_dofElement, "position"))
  {
    setDofLimitAttributes(_dofElement, "position", jointName, localIndex,
                          proxy.lowerPosition,
                          proxy.upperPosition,
                          proxy.initalPosition);
  }

  if (hasElement(_dofElement, "velocity"))
  {
    setDofLimitAttributes(_dofElement, "velocity", jointName, localIndex,
                          proxy.lowerVelocity,
                          proxy.upperVelocity,
                          proxy.initialVelocity);
  }

  if (hasElement(_dofElement, "acceleration"))
  {
    setDofLimitAttributes(_dofElement, "acceleration", jointName, localIndex,
                          proxy.lowerAcceleration,
                          proxy.upperAcceleration,
                          proxy.initialAcceleration);
  }

  if (hasElement(_dofElement, "force"))
  {
    setDofLimitAttributes(_dofElement, "force", jointName, localIndex,
                          proxy.lowerForce,
                          proxy.upperForce,
                          proxy.initialForce);
  }

  if (hasElement(_dofElement, "damping"))
    *proxy.dampingCoefficient = getValueDouble(_dofElement, "damping");

  if (hasElement(_dofElement, "friction"))
    *proxy.friction = getValueDouble(_dofElement, "friction");

  if (hasElement(_dofElement, "spring_rest_position"))
    *proxy.restPosition = getValueDouble(_dofElement, "spring_rest_position");

  if (hasElement(_dofElement, "spring_stiffness"))
    *proxy.springStiffness = getValueDouble(_dofElement, "spring_stiffness");
}

//==============================================================================
template <typename PropertyType>
static void readJointDynamicsAndLimit(tinyxml2::XMLElement* _jointElement,
                                      PropertyType& _properties,
                                      SkelParser::SkelJoint& _joint,
                                      const std::string& _name,
                                      size_t _numAxis)
{
  // TODO(MXG): Consider printing warnings for these tags that recommends using
  // the dof tag instead, because all functionality of these tags have been
  // moved to the dof tag

  assert(_jointElement != nullptr);
  assert(_numAxis <= 6);

  std::string axisName = "axis";

  // axis
  for (size_t i = 0; i < _numAxis; ++i)
  {
    if (i != 0)
      axisName = "axis" + std::to_string(i + 1);

    if (hasElement(_jointElement, axisName))
    {
      DofProxy proxy(_properties, _joint, i, _name);

      tinyxml2::XMLElement* axisElement = getElement(_jointElement, axisName);

      // damping
      if (hasElement(axisElement, "damping"))
      {
        dtwarn << "[SkelParser] <damping> tag is now an element under the "
               << "<dynamics> tag. Please see "
               << "(https://github.com/dartsim/dart/wiki/) for more details.\n";
        double damping = getValueDouble(axisElement, "damping");
        *proxy.dampingCoefficient = damping;
      }

      // dynamics
      if (hasElement(axisElement, "dynamics"))
      {
        tinyxml2::XMLElement* dynamicsElement
            = getElement(axisElement, "dynamics");

        // damping
        if (hasElement(dynamicsElement, "damping"))
        {
          double val = getValueDouble(dynamicsElement, "damping");
          *proxy.dampingCoefficient = val;
        }

        // friction
        if (hasElement(dynamicsElement, "friction"))
        {
          double val = getValueDouble(dynamicsElement, "friction");
          *proxy.friction = val;
        }

        // spring_rest_position
        if (hasElement(dynamicsElement, "spring_rest_position"))
        {
          double val = getValueDouble(dynamicsElement, "spring_rest_position");
          *proxy.restPosition = val;
        }

        // friction
        if (hasElement(dynamicsElement, "spring_stiffness"))
        {
          double val = getValueDouble(dynamicsElement, "spring_stiffness");
          *proxy.springStiffness = val;
        }
      }

      // limit
      if (hasElement(axisElement, "limit"))
      {
        tinyxml2::XMLElement* limitElement
            = getElement(axisElement, "limit");

        // lower
        if (hasElement(limitElement, "lower"))
        {
          double lower = getValueDouble(limitElement, "lower");
          *proxy.lowerPosition = lower;
        }

        // upper
        if (hasElement(limitElement, "upper"))
        {
          double upper = getValueDouble(limitElement, "upper");
          *proxy.upperPosition = upper;
        }
      }
    }
  }
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readWeldJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    const std::string&)
{
  assert(_jointElement != nullptr);

  return Eigen::make_aligned_shared<dynamics::WeldJoint::Properties>();
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readRevoluteJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    const std::string& _name)
{
  assert(_jointElement != nullptr);

  dynamics::RevoluteJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    properties.mAxis = xyz;
  }
  else
  {
    dterr << "[SkelParser::readRevoluteJoint] Revolute Joint named ["
          << _name << "] is missing axis information!\n";
    assert(0);
  }

  readJointDynamicsAndLimit<dynamics::SingleDofJoint::Properties>(
        _jointElement, properties, _joint, _name, 1);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    double init_pos = getValueDouble(_jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    _joint.position = ipos;
    properties.mInitialPosition = ipos[0];
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    double init_vel = getValueDouble(_jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    _joint.velocity = ivel;
    properties.mInitialVelocity = ivel[0];
  }

  readAllDegreesOfFreedom<dynamics::SingleDofJoint::Properties>(
        _jointElement, properties, _joint, _name, 1);

  return Eigen::make_aligned_shared<dynamics::RevoluteJoint::Properties>(
      properties);
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readPrismaticJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    const std::string& _name)
{
  assert(_jointElement != nullptr);

  dynamics::PrismaticJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    properties.mAxis = xyz;
  }
  else
  {
    dterr << "[SkelParser::readPrismaticJoint] Prismatic Joint named ["
          << _name << "] is missing axis information!\n";
    assert(0);
  }

  readJointDynamicsAndLimit<dynamics::SingleDofJoint::Properties>(
        _jointElement, properties, _joint, _name, 1);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    double init_pos = getValueDouble(_jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    _joint.position = ipos;
    properties.mInitialPosition = ipos[0];
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    double init_vel = getValueDouble(_jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    _joint.velocity = ivel;
    properties.mInitialVelocity = ivel[0];
  }

  readAllDegreesOfFreedom<dynamics::SingleDofJoint::Properties>(
        _jointElement, properties, _joint, _name, 1);

  return Eigen::make_aligned_shared<dynamics::PrismaticJoint::Properties>(
      properties);
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readScrewJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    const std::string& _name)
{
  assert(_jointElement != nullptr);

  dynamics::ScrewJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    properties.mAxis = xyz;

    // pitch
    if (hasElement(axisElement, "pitch"))
    {
      double pitch = getValueDouble(axisElement, "pitch");
      properties.mPitch = pitch;
    }
  }
  else
  {
    dterr << "[SkelParser::readScrewJoint] Screw Joint named [" << _name
          << "] is missing axis information!\n";
    assert(0);
  }

  readJointDynamicsAndLimit<dynamics::SingleDofJoint::Properties>(
        _jointElement, properties, _joint, _name, 1);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    double init_pos = getValueDouble(_jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    _joint.position = ipos;
    properties.mInitialPosition = ipos[0];
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    double init_vel = getValueDouble(_jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    _joint.velocity = ivel;
    properties.mInitialVelocity = ivel[0];
  }

  readAllDegreesOfFreedom<dynamics::SingleDofJoint::Properties>(
        _jointElement, properties, _joint, _name, 1);

  return Eigen::make_aligned_shared<dynamics::ScrewJoint::Properties>(
        properties);
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readUniversalJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    const std::string& _name)
{
  assert(_jointElement != nullptr);

  dynamics::UniversalJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    properties.mAxis[0] = xyz;
  }
  else
  {
    dterr << "[SkelParser::readUniversalJoint] Universal Joint named [" << _name
          << "] is missing axis information!\n";
    assert(0);
  }

  //--------------------------------------------------------------------------
  // axis2
  if (hasElement(_jointElement, "axis2"))
  {
    tinyxml2::XMLElement* axis2Element = getElement(_jointElement, "axis2");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axis2Element, "xyz");
    properties.mAxis[1] = xyz;
  }
  else
  {
    dterr << "[SkelParser::readUniversalJoint] Universal Joint named [" << _name
          << "] is missing axis2 information!\n";
    assert(0);
  }

  readJointDynamicsAndLimit(_jointElement, properties, _joint, _name, 2);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    Eigen::Vector2d init_pos = getValueVector2d(_jointElement, "init_pos");
    _joint.position = init_pos;
    properties.mInitialPositions = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    Eigen::Vector2d init_vel = getValueVector2d(_jointElement, "init_vel");
    _joint.velocity = init_vel;
    properties.mInitialVelocities = init_vel;
  }

  readAllDegreesOfFreedom(_jointElement, properties, _joint, _name, 2);

  return Eigen::make_aligned_shared<dynamics::UniversalJoint::Properties>(
      properties);
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readBallJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    const std::string& _name)
{
  assert(_jointElement != nullptr);

  dynamics::BallJoint::Properties properties;

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    _joint.position = init_pos;
    properties.mInitialPositions = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    _joint.velocity = init_vel;
    properties.mInitialVelocities = init_vel;
  }

  readAllDegreesOfFreedom(_jointElement, properties, _joint, _name, 3);

  return Eigen::make_aligned_shared<dynamics::BallJoint::Properties>(
      properties);
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readEulerJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    const std::string& _name)
{
  assert(_jointElement != nullptr);

  dynamics::EulerJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis order
  std::string order = getValueString(_jointElement, "axis_order");
  if (order == "xyz")
  {
    properties.mAxisOrder = dynamics::EulerJoint::AO_XYZ;
  }
  else if (order == "zyx")
  {
    properties.mAxisOrder = dynamics::EulerJoint::AO_ZYX;
  }
  else
  {
    dterr << "[SkelParser::readEulerJoint] Undefined Euler axis order for "
          << "Euler Joint named [" << _name << "]\n";
    assert(0);
  }

  //--------------------------------------------------------------------------
  // axis
  readJointDynamicsAndLimit(_jointElement, properties, _joint, _name, 3);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    _joint.position = init_pos;
    properties.mInitialPositions = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    _joint.velocity = init_vel;
    properties.mInitialVelocities = init_vel;
  }

  readAllDegreesOfFreedom(_jointElement, properties, _joint, _name, 3);

  return Eigen::make_aligned_shared<dynamics::EulerJoint::Properties>(
      properties);
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readTranslationalJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    const std::string& _name)
{
  assert(_jointElement != nullptr);

  dynamics::TranslationalJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  readJointDynamicsAndLimit(_jointElement, properties, _joint, _name, 3);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    _joint.position = init_pos;
    properties.mInitialPositions = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    _joint.velocity = init_vel;
    properties.mInitialVelocities = init_vel;
  }

  readAllDegreesOfFreedom(_jointElement, properties, _joint, _name, 3);

  return Eigen::make_aligned_shared<dynamics::TranslationalJoint::Properties>(
      properties);
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readPlanarJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    const std::string& _name)
{
  assert(_jointElement != nullptr);

  dynamics::PlanarJoint::Properties properties;

  //--------------------------------------------------------------------------
  // Plane
  if (hasElement(_jointElement, "plane"))
  {
    tinyxml2::XMLElement* planeElement = getElement(_jointElement, "plane");

    // Type attribute
    std::string type = getAttributeString(planeElement, "type");

    if (type == "xy")
    {
      properties.mPlaneType = dynamics::PlanarJoint::PT_XY;
    }
    else if (type == "yz")
    {
      properties.mPlaneType = dynamics::PlanarJoint::PT_YZ;
    }
    else if (type == "zx")
    {
      properties.mPlaneType = dynamics::PlanarJoint::PT_ZX;
    }
    else if (type == "arbitrary")
    {
      properties.mPlaneType = dynamics::PlanarJoint::PT_ARBITRARY;

      tinyxml2::XMLElement* transAxis1Element
          = getElement(planeElement, "translation_axis1");

      properties.mTransAxis1 = getValueVector3d(transAxis1Element, "xyz");

      tinyxml2::XMLElement* transAxis2Element
          = getElement(planeElement, "translation_axis2");

      properties.mTransAxis2 = getValueVector3d(transAxis2Element, "xyz");
    }
    else
    {
      dterr << "[SkelParser::readPlanarJoint] Planar Joint named [" << _name
            << "] is missing plane type information. Defaulting to XY-Plane.\n";
    }
  }
  else
  {
    dterr << "[SkelParser::readPlanarJoint] Planar Joint named [" << _name
          << "] is missing plane type information. Defaulting to XY-Plane.\n";
  }

  //--------------------------------------------------------------------------
  // axis
  readJointDynamicsAndLimit(_jointElement, properties, _joint, _name, 3);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    _joint.position = init_pos;
    properties.mInitialPositions = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    _joint.velocity = init_vel;
    properties.mInitialVelocities = init_vel;
  }

  readAllDegreesOfFreedom(_jointElement, properties, _joint, _name, 3);

  return Eigen::make_aligned_shared<dynamics::PlanarJoint::Properties>(
      properties);
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readFreeJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    const std::string& _name)
{
  assert(_jointElement != nullptr);

  dynamics::FreeJoint::Properties properties;

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    Eigen::Vector6d init_pos = getValueVector6d(_jointElement, "init_pos");
    _joint.position = init_pos;
    properties.mInitialPositions = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    Eigen::Vector6d init_vel = getValueVector6d(_jointElement, "init_vel");
    _joint.velocity = init_vel;
    properties.mInitialVelocities = init_vel;
  }

  readAllDegreesOfFreedom(_jointElement, properties, _joint, _name, 6);

  return Eigen::make_aligned_shared<dynamics::FreeJoint::Properties>(
      properties);
}

common::ResourceRetrieverPtr SkelParser::getRetriever(
  const common::ResourceRetrieverPtr& _retriever)
{
  if(_retriever)
    return _retriever;
  else
    return std::make_shared<common::LocalResourceRetriever>();
}

}  // namespace utils
}  // namespace kido
