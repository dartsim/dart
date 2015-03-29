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
//#include "dart/constraint/OldConstraintDynamics.h"
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

namespace dart {
namespace utils {

simulation::World* SoftSdfParser::readSoftSdfFile(const std::string& _filename)
{
  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _dartFile;
  try
  {
    openXMLFile(_dartFile, _filename.c_str());
  }
  catch(std::exception const& e)
  {
    std::cout << "LoadFile Fails: " << e.what() << std::endl;
    return NULL;
  }

  //--------------------------------------------------------------------------
  // Load DART
  tinyxml2::XMLElement* sdfElement = NULL;
  sdfElement = _dartFile.FirstChildElement("sdf");
  if (sdfElement == NULL)
    return NULL;

  //--------------------------------------------------------------------------
  // version attribute
  std::string version = getAttribute(sdfElement, "version");
  // We support 1.4 only for now.
  if (version != "1.4" && version != "1.5")
  {
    dterr << "The file format of ["
          << _filename
          << "] is not sdf 1.4 or 1.5."
          << std::endl;
    return NULL;
  }

  //--------------------------------------------------------------------------
  // Load World
  tinyxml2::XMLElement* worldElement = NULL;
  worldElement = sdfElement->FirstChildElement("world");
  if (worldElement == NULL)
    return NULL;

  // Change path to a Unix-style path if given a Windows one
  // Windows can handle Unix-style paths (apparently)
  std::string unixFileName = _filename;
  std::replace(unixFileName.begin(), unixFileName.end(), '\\' , '/' );
  std::string skelPath = unixFileName.substr(0, unixFileName.rfind("/") + 1);

  simulation::World* newWorld = readWorld(worldElement, skelPath);

  return newWorld;
}

dynamics::SkeletonPtr SoftSdfParser::readSkeleton(
    const std::string& _filename)
{
  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _dartFile;
  try
  {
    openXMLFile(_dartFile, _filename.c_str());
  }
  catch(std::exception const& e)
  {
    std::cout << "LoadFile Fails: " << e.what() << std::endl;
    return NULL;
  }

  //--------------------------------------------------------------------------
  // Load sdf
  tinyxml2::XMLElement* sdfElement = NULL;
  sdfElement = _dartFile.FirstChildElement("sdf");
  if (sdfElement == NULL)
    return NULL;

  //--------------------------------------------------------------------------
  // version attribute
  std::string version = getAttribute(sdfElement, "version");
  // TODO: We need version aware SDF parser (see #264)
  // We support 1.4 only for now.
  if (version != "1.4" && version != "1.5")
  {
    dterr << "The file format of ["
          << _filename
          << "] is not sdf 1.4 or 1.5."
          << std::endl;
    return NULL;
  }

  //--------------------------------------------------------------------------
  // Load skeleton
  tinyxml2::XMLElement* softSkelElement = NULL;
  softSkelElement = sdfElement->FirstChildElement("model");
  if (softSkelElement == NULL)
    return NULL;

  // Change path to a Unix-style path if given a Windows one
  // Windows can handle Unix-style paths (apparently)
  std::string unixFileName = _filename;
  std::replace(unixFileName.begin(), unixFileName.end(), '\\' , '/' );
  std::string skelPath = unixFileName.substr(0, unixFileName.rfind("/") + 1);

  dynamics::Skeleton* newSkeleton = readSkeleton(softSkelElement,
                                                             skelPath);

  return newSkeleton;
}

simulation::World* SoftSdfParser::readWorld(
    tinyxml2::XMLElement* _worldElement, const std::string& _skelPath)
{
  assert(_worldElement != NULL);

  // Create a world
  simulation::World* newWorld = new simulation::World;

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(_worldElement, "name");
  // World don't have name.
  //newWorld->setName(name);

  //--------------------------------------------------------------------------
  // Load physics
  if (hasElement(_worldElement, "physics"))
  {
    tinyxml2::XMLElement* physicsElement = _worldElement->FirstChildElement("physics");
    readPhysics(physicsElement, newWorld);
  }

  //--------------------------------------------------------------------------
  // Load skeletons
  ElementEnumerator skeletonElements(_worldElement, "model");
  while (skeletonElements.next())
  {
    dynamics::SkeletonPtr newSkeleton
        = readSkeleton(skeletonElements.get(), _skelPath);

    newWorld->addSkeleton(newSkeleton);
  }

  return newWorld;
}

dynamics::SkeletonPtr SoftSdfParser::readSkeleton(
    tinyxml2::XMLElement* _skeletonElement, const std::string& _skelPath)
{
  assert(_skeletonElement != NULL);

  dynamics::Skeleton* newSkeleton = new dynamics::Skeleton;
  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(_skeletonElement, "name");
  newSkeleton->setName(name);

  //--------------------------------------------------------------------------
  // immobile attribute
  if (hasElement(_skeletonElement, "static"))
  {
    bool isStatic= getValueBool(_skeletonElement, "static");
    newSkeleton->setMobile(!isStatic);
  }

  //--------------------------------------------------------------------------
  // transformation
  if (hasElement(_skeletonElement, "pose"))
  {
    Eigen::Isometry3d W = getValueIsometry3dWithExtrinsicRotation(_skeletonElement, "pose");
    skeletonFrame = W;
  }

  //--------------------------------------------------------------------------
  // Bodies
  ElementEnumerator bodies(_skeletonElement, "link");
  std::vector<SDFBodyNode, Eigen::aligned_allocator<SDFBodyNode> > sdfBodyNodes;
  while (bodies.next())
  {
    SDFBodyNode newSDFBodyNode
        = readSoftBodyNode(bodies.get(), newSkeleton, skeletonFrame,
                           _skelPath);
    sdfBodyNodes.push_back(newSDFBodyNode);
  }

  //--------------------------------------------------------------------------
  // Joints
  ElementEnumerator joints(_skeletonElement, "joint");
  while (joints.next())
  {
    readSoftJoint(joints.get(), sdfBodyNodes, skeletonFrame);
  }

  //--------------------------------------------------------------------------
  // Add FreeJoint to the body node that doesn't have parent joint
  for (unsigned int i = 0; i < sdfBodyNodes.size(); ++i)
  {
    dynamics::BodyNode* bodyNode = sdfBodyNodes[i].bodyNode;

    if (bodyNode->getParentJoint() == NULL)
    {
      // If this link has no parent joint, then we add 6-dof free joint.
      dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;

      newFreeJoint->setTransformFromParentBodyNode(
            sdfBodyNodes[i].initTransform);
      newFreeJoint->setTransformFromChildBodyNode(
            Eigen::Isometry3d::Identity());

      bodyNode->setParentJoint(newFreeJoint);
    }
  }

  for (std::vector<SDFBodyNode,
       Eigen::aligned_allocator<SDFBodyNode> >::iterator it
       = sdfBodyNodes.begin();
       it != sdfBodyNodes.end(); ++it)
  {
    newSkeleton->addBodyNode((*it).bodyNode);
  }

  return newSkeleton;
}

SdfParser::SDFBodyNode SoftSdfParser::readSoftBodyNode(
    tinyxml2::XMLElement* _softBodyNodeElement,
    dynamics::Skeleton* _Skeleton,
    const Eigen::Isometry3d& _skeletonFrame,
    const std::string& _skelPath)
{
  //---------------------------------- Note ------------------------------------
  // SoftBodyNode is created if _softBodyNodeElement has <soft_shape>.
  // Otherwise, BodyNode is created.

  //----------------------------------------------------------------------------
  assert(_softBodyNodeElement != NULL);
  assert(_Skeleton != NULL);

  // If _softBodyNodeElement has no <soft_shape>, return rigid body node
  if (!hasElement(_softBodyNodeElement, "soft_shape"))
  {
    return SdfParser::readBodyNode(_softBodyNodeElement,
                                   _Skeleton,
                                   _skeletonFrame,
                                   _skelPath);
  }

  dynamics::SoftBodyNode* newSoftBodyNode = new dynamics::SoftBodyNode;
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  std::string name = getAttribute(_softBodyNodeElement, "name");
  newSoftBodyNode->setName(name);

  //--------------------------------------------------------------------------
  // gravity
  if (hasElement(_softBodyNodeElement, "gravity"))
  {
    bool gravityMode = getValueBool(_softBodyNodeElement, "gravity");
    newSoftBodyNode->setGravityMode(gravityMode);
  }

  //--------------------------------------------------------------------------
  // self_collide
  //    if (hasElement(_bodyElement, "self_collide"))
  //    {
  //        bool gravityMode = getValueBool(_bodyElement, "self_collide");
  //    }

  //--------------------------------------------------------------------------
  // transformation
  if (hasElement(_softBodyNodeElement, "pose"))
  {
    Eigen::Isometry3d W = getValueIsometry3dWithExtrinsicRotation(_softBodyNodeElement, "pose");
    initTransform = _skeletonFrame * W;
  }
  else
  {
    initTransform = _skeletonFrame;
  }

  //--------------------------------------------------------------------------
  // visual
  ElementEnumerator vizShapes(_softBodyNodeElement, "visual");
  while (vizShapes.next())
  {
    dynamics::ShapePtr newShape(readShape(vizShapes.get(), _skelPath));
    if (newShape)
      newSoftBodyNode->addVisualizationShape(newShape);
  }

  //--------------------------------------------------------------------------
  // collision
  ElementEnumerator collShapes(_softBodyNodeElement, "collision");
  while (collShapes.next())
  {
    dynamics::ShapePtr newShape(readShape(collShapes.get(), _skelPath));

    if (newShape)
      newSoftBodyNode->addCollisionShape(newShape);
  }

  //--------------------------------------------------------------------------
  // inertia
  if (hasElement(_softBodyNodeElement, "inertial"))
  {
    tinyxml2::XMLElement* inertiaElement = getElement(_softBodyNodeElement, "inertial");

    // mass
    if (hasElement(inertiaElement, "mass"))
    {
      double mass = getValueDouble(inertiaElement, "mass");
      newSoftBodyNode->setMass(mass);
    }

    // offset
    if (hasElement(inertiaElement, "pose"))
    {
      Eigen::Isometry3d T = getValueIsometry3dWithExtrinsicRotation(inertiaElement, "pose");
      newSoftBodyNode->setLocalCOM(T.translation());
    }

    // inertia
    if (hasElement(inertiaElement, "inertia"))
    {
      tinyxml2::XMLElement* moiElement
          = getElement(inertiaElement, "inertia");

      double ixx = getValueDouble(moiElement, "ixx");
      double iyy = getValueDouble(moiElement, "iyy");
      double izz = getValueDouble(moiElement, "izz");

      double ixy = getValueDouble(moiElement, "ixy");
      double ixz = getValueDouble(moiElement, "ixz");
      double iyz = getValueDouble(moiElement, "iyz");

      newSoftBodyNode->setMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
    }
    else if (newSoftBodyNode->getVisualizationShape(0) != NULL)
    {
      Eigen::Matrix3d Ic =
          newSoftBodyNode->getVisualizationShape(0)->computeInertia(
            newSoftBodyNode->getMass());

      newSoftBodyNode->setMomentOfInertia(Ic(0,0), Ic(1,1), Ic(2,2),
                                  Ic(0,1), Ic(0,2), Ic(1,2));
    }
  }

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
      dynamics::SoftBodyNodeHelper::setBox(
            newSoftBodyNode, size, T, frags, totalMass);

      // Visualization shape
      newSoftBodyNode->addVisualizationShape(
            dynamics::ShapePtr(new dynamics::SoftMeshShape(newSoftBodyNode)));

      // Collision shape
      newSoftBodyNode->addCollisionShape(
            dynamics::ShapePtr(new dynamics::SoftMeshShape(newSoftBodyNode)));
    }
    else if (hasElement(geometryEle, "ellipsoid"))
    {
      tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "ellipsoid");
      Eigen::Vector3d size = getValueVector3d(ellipsoidEle, "size");
      double nSlices       = getValueDouble(ellipsoidEle, "num_slices");
      double nStacks       = getValueDouble(ellipsoidEle, "num_stacks");
      dynamics::SoftBodyNodeHelper::setEllipsoid(newSoftBodyNode,
                                                 size,
                                                 nSlices,
                                                 nStacks,
                                                 totalMass);

      // Visualization shape
      newSoftBodyNode->addVisualizationShape(
            std::shared_ptr<dynamics::Shape>(
              new dynamics::SoftMeshShape(newSoftBodyNode)));

      // Collision shape
      newSoftBodyNode->addCollisionShape(
            dynamics::ShapePtr(new dynamics::SoftMeshShape(newSoftBodyNode)));
    }
    else if (hasElement(geometryEle, "cylinder"))
    {
      tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "cylinder");
      double radius  = getValueDouble(ellipsoidEle, "radius");
      double height  = getValueDouble(ellipsoidEle, "height");
      double nSlices = getValueDouble(ellipsoidEle, "num_slices");
      double nStacks = getValueDouble(ellipsoidEle, "num_stacks");
      double nRings = getValueDouble(ellipsoidEle, "num_rings");
      dynamics::SoftBodyNodeHelper::setCylinder(newSoftBodyNode,
                                                radius,
                                                height,
                                                nSlices,
                                                nStacks,
                                                nRings,
                                                totalMass);

      // Visualization shape
      newSoftBodyNode->addVisualizationShape(
            std::shared_ptr<dynamics::Shape>(
              new dynamics::SoftMeshShape(newSoftBodyNode)));

      // Collision shape
      newSoftBodyNode->addCollisionShape(
            dynamics::ShapePtr(new dynamics::SoftMeshShape(newSoftBodyNode)));
    }
    else
    {
      dterr << "Unknown soft shape.\n";
    }

    // kv
    if (hasElement(softShapeEle, "kv"))
    {
      double kv = getValueDouble(softShapeEle, "kv");
      newSoftBodyNode->setVertexSpringStiffness(kv);
    }

    // ke
    if (hasElement(softShapeEle, "ke"))
    {
      double ke = getValueDouble(softShapeEle, "ke");
      newSoftBodyNode->setEdgeSpringStiffness(ke);
    }

    // damp
    if (hasElement(softShapeEle, "damp"))
    {
      double damp = getValueDouble(softShapeEle, "damp");
      newSoftBodyNode->setDampingCoefficient(damp);
    }
  }

  SDFBodyNode sdfBodyNode;
  sdfBodyNode.properties = newSoftBodyNode;
  sdfBodyNode.initTransform = initTransform;

  return sdfBodyNode;
}

dynamics::Joint* SoftSdfParser::readSoftJoint(tinyxml2::XMLElement* _jointElement,
                                              const std::vector<SDFBodyNode, Eigen::aligned_allocator<SDFBodyNode> >& _sdfBodyNodes,
                                              const Eigen::Isometry3d& _skeletonFrame)
{
  assert(_jointElement != NULL);

  dynamics::Joint* newJoint = NULL;

  //--------------------------------------------------------------------------
  // Type attribute
  std::string type = getAttribute(_jointElement, "type");
  assert(!type.empty());

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(_jointElement, "name");

  //--------------------------------------------------------------------------
  // parent
  SDFBodyNode sdfParentBodyNode;
  sdfParentBodyNode.properties = NULL;
  sdfParentBodyNode.initTransform = Eigen::Isometry3d::Identity();

  if (hasElement(_jointElement, "parent"))
  {
    std::string strParent = getValueString(_jointElement, "parent");

    if (strParent != std::string("world"))
    {
      for (std::vector<SDFBodyNode, Eigen::aligned_allocator<SDFBodyNode> >::const_iterator it =
           _sdfBodyNodes.begin(); it != _sdfBodyNodes.end(); ++it)
        if ((*it).bodyNode->getName() == strParent)
        {
          sdfParentBodyNode = (*it);
          break;
        }

      if (sdfParentBodyNode.properties == NULL)
      {
        dterr << "Can't find the parent body ["
              << strParent
              << "] of the joint ["
              << name
              << "]. " << std::endl;
        assert(0);
      }
    }
  }
  else
  {
    dterr << "Set parent body node for " << name << "."
          << std::endl;
    assert(0);
  }

  //--------------------------------------------------------------------------
  // child
  SDFBodyNode sdfChildBodyNode;
  sdfChildBodyNode.properties = NULL;
  sdfChildBodyNode.initTransform = Eigen::Isometry3d::Identity();

  if (hasElement(_jointElement, "child"))
  {
    std::string strChild = getValueString(_jointElement, "child");

    for (std::vector<SDFBodyNode, Eigen::aligned_allocator<SDFBodyNode> >::const_iterator it =
         _sdfBodyNodes.begin(); it != _sdfBodyNodes.end(); ++it)
    {
      if ((*it).bodyNode->getName() == strChild)
      {
        sdfChildBodyNode = (*it);
        break;
      }
    }

    if (sdfChildBodyNode.properties == NULL)
    {
      dterr << "Can't find the child body ["
            << strChild
            << "] of the joint ["
            << name
            << "]. " << std::endl;
      assert(0);
    }
  }
  else
  {
    dterr << "Set child body node for " << name << "."
          << std::endl;
    assert(0);
  }

  if (sdfParentBodyNode.properties)
    sdfParentBodyNode.properties->addChildBodyNode(sdfChildBodyNode.properties);

  //--------------------------------------------------------------------------
  // transformation
  Eigen::Isometry3d parentWorld = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childWorld = sdfChildBodyNode.initTransform;
  if (sdfParentBodyNode.properties)
    parentWorld = sdfParentBodyNode.initTransform;
  if (hasElement(_jointElement, "pose"))
    childToJoint = getValueIsometry3dWithExtrinsicRotation(_jointElement, "pose");
  Eigen::Isometry3d parentToJoint = parentWorld.inverse()*childWorld*childToJoint;

  // TODO: Workaround!!
  Eigen::Isometry3d jointFrame = childWorld * childToJoint;

  if (type == std::string("prismatic"))
    newJoint = readPrismaticJoint(_jointElement, _skeletonFrame, jointFrame);
  if (type == std::string("revolute"))
    newJoint = readRevoluteJoint(_jointElement, _skeletonFrame, jointFrame);
  if (type == std::string("screw"))
    newJoint = readScrewJoint(_jointElement, _skeletonFrame, jointFrame);
  if (type == std::string("revolute2"))
    newJoint = readUniversalJoint(_jointElement, _skeletonFrame, jointFrame);
  if (type == std::string("ball"))
    newJoint = readBallJoint(_jointElement, _skeletonFrame, jointFrame);
  assert(newJoint != NULL);

  newJoint->setName(name);
  sdfChildBodyNode.properties->setParentJoint(newJoint);
  newJoint->setTransformFromChildBodyNode(childToJoint);
  newJoint->setTransformFromParentBodyNode(parentToJoint);

  return newJoint;
}

}  // namespace utils
}  // namespace dart
