/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
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

#include "dart/utils/SoftParser.h"

#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>

#include <dart/common/Console.h>
#include <dart/collision/dart/DARTCollisionDetector.h>
#include <dart/collision/fcl/FCLCollisionDetector.h>
#include <dart/constraint/ConstraintDynamics.h>
// #include <dart/collision/fcl_mesh/FCLMeshCollisionDetector.h>
#include <dart/dynamics/BoxShape.h>
#include <dart/dynamics/CylinderShape.h>
#include <dart/dynamics/EllipsoidShape.h>
#include <dart/dynamics/WeldJoint.h>
#include <dart/dynamics/RevoluteJoint.h>
#include <dart/dynamics/PrismaticJoint.h>
#include <dart/dynamics/TranslationalJoint.h>
#include <dart/dynamics/BallJoint.h>
#include <dart/dynamics/FreeJoint.h>
#include <dart/simulation/World.h>
#include <dart/utils/SkelParser.h>

#include "dart/collision/fcl_mesh/SoftFCLMeshCollisionDetector.h"
#include "dart/dynamics/SoftMeshShape.h"
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/SoftSkeleton.h"
#include "dart/simulation/SoftWorld.h"

namespace dart {
namespace utils {

simulation::SoftWorld* SoftSkelParser::readSoftFile(
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
    std::cout << "LoadFile [" << _filename << "] Fails: "
              << e.what() << std::endl;
    return NULL;
  }

  //--------------------------------------------------------------------------
  // Load DART
  tinyxml2::XMLElement* skelElement = NULL;
  skelElement = _dartFile.FirstChildElement("skel");
  if (skelElement == NULL)
  {
    dterr << "Skel file[" << _filename << "] does not contain <skel> as the "
          << "element.\n";
    return NULL;
  }

  //--------------------------------------------------------------------------
  // Load World
  tinyxml2::XMLElement* worldElement = NULL;
  worldElement = skelElement->FirstChildElement("world");
  if (worldElement == NULL)
  {
    dterr << "Skel file[" << _filename << "] does not contain <world> element "
          <<"under <skel> element.\n";
    return NULL;
  }

  simulation::SoftWorld* newWorld = readSoftWorld(worldElement);

  return newWorld;
}

simulation::SoftWorld* SoftSkelParser::readSoftWorld(
    tinyxml2::XMLElement* _worldElement)
{
  assert(_worldElement != NULL);

  // Create a world
  simulation::SoftWorld* newSoftWorld = new simulation::SoftWorld;

  //--------------------------------------------------------------------------
  // Load physics
  tinyxml2::XMLElement* physicsElement
      = _worldElement->FirstChildElement("physics");
  if (physicsElement != NULL)
  {
    // Time step
    tinyxml2::XMLElement* timeStepElement = NULL;
    timeStepElement = physicsElement->FirstChildElement("time_step");
    if (timeStepElement != NULL)
    {
      std::string strTimeStep = timeStepElement->GetText();
      double timeStep = toDouble(strTimeStep);
      newSoftWorld->setTimeStep(timeStep);
    }

    // Gravity
    tinyxml2::XMLElement* gravityElement = NULL;
    gravityElement = physicsElement->FirstChildElement("gravity");
    if (gravityElement != NULL)
    {
      std::string strGravity = gravityElement->GetText();
      Eigen::Vector3d gravity = toVector3d(strGravity);
      newSoftWorld->setGravity(gravity);
    }

    // Collision detector
    if (hasElement(physicsElement, "collision_detector"))
    {
      std::string strCD = getValueString(physicsElement, "collision_detector");
      if (strCD == "fcl_mesh")
      {
        newSoftWorld->getConstraintHandler()->setCollisionDetector(
              new collision::SoftFCLMeshCollisionDetector());
      }
      else if (strCD == "fcl")
      {
        newSoftWorld->getConstraintHandler()->setCollisionDetector(
              new collision::FCLCollisionDetector());
      }
      else if (strCD == "dart")
      {
        newSoftWorld->getConstraintHandler()->setCollisionDetector(
              new collision::DARTCollisionDetector());
      }
      else
      {
        dtwarn << "Unknown collision detector[" << strCD << "]. "
               << "Default collision detector[fcl] will be loaded."
               << std::endl;
      }
    }
    else
    {
      newSoftWorld->getConstraintHandler()->setCollisionDetector(
            new collision::SoftFCLMeshCollisionDetector());
    }
  }

  //--------------------------------------------------------------------------
  // Load soft skeletons
  ElementEnumerator softSkeletonElements(_worldElement, "skeleton");
  while (softSkeletonElements.next())
  {
    dynamics::SoftSkeleton* newSoftSkeleton
        = readSoftSkeleton(softSkeletonElements.get(), newSoftWorld);

    newSoftWorld->addSkeleton(newSoftSkeleton);
  }

  return newSoftWorld;
}

dynamics::SoftSkeleton* SoftSkelParser::readSoftSkeleton(
    tinyxml2::XMLElement *_softSkeletonElement,
    simulation::World *_softWorld)
{
  assert(_softSkeletonElement != NULL);
  assert(_softWorld != NULL);

  dynamics::SoftSkeleton* newSoftSkeleton = new dynamics::SoftSkeleton;
  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(_softSkeletonElement, "name");
  newSoftSkeleton->setName(name);

  //--------------------------------------------------------------------------
  // transformation
  if (hasElement(_softSkeletonElement, "transformation"))
  {
    Eigen::Isometry3d W =
        getValueIsometry3d(_softSkeletonElement, "transformation");
    skeletonFrame = W;
  }

  //--------------------------------------------------------------------------
  // immobile attribute
//  tinyxml2::XMLElement* immobileElement = NULL;
//  immobileElement = _softSkeletonElement->FirstChildElement("immobile");
//  if (immobileElement != NULL)
//  {
//    std::string stdImmobile = immobileElement->GetText();
//    bool immobile = toBool(stdImmobile);
//    newSoftSkeleton->setImmobileState(immobile);
//  }

  //--------------------------------------------------------------------------
  // Bodies
  ElementEnumerator bodies(_softSkeletonElement, "body");
  std::vector<SkelBodyNode, Eigen::aligned_allocator<SkelBodyNode> >
      softBodyNodes;
  while (bodies.next())
  {
    SkelBodyNode newSoftBodyNode
        = readSoftBodyNode(bodies.get(),
                           newSoftSkeleton,
                           skeletonFrame);
    assert(newSoftBodyNode.bodyNode);
    softBodyNodes.push_back(newSoftBodyNode);
  }

  //--------------------------------------------------------------------------
  // Joints
  ElementEnumerator joints(_softSkeletonElement, "joint");
  while (joints.next())
    readSoftJoint(joints.get(), softBodyNodes);

  //--------------------------------------------------------------------------
  // Add FreeJoint to the body node that doesn't have parent joint
//  for (unsigned int i = 0; i < skelBodyNodes.size(); ++i)
//  {
//    dynamics::BodyNode* bodyNode = skelBodyNodes[i].bodyNode;

//    if (bodyNode->getParentJoint() == NULL)
//    {
//      // If this link has no parent joint, then we add 6-dof free joint.
//      dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;

//      newFreeJoint->setTransformFromParentBodyNode(
//            bodyNode->getWorldTransform());
//      newFreeJoint->setTransformFromChildBodyNode(
//            Eigen::Isometry3d::Identity());

//      bodyNode->setParentJoint(newFreeJoint);
//    }
//  }

  for (std::vector<SkelBodyNode,
       Eigen::aligned_allocator<SkelBodyNode> >::iterator it =
       softBodyNodes.begin();
       it != softBodyNodes.end(); ++it)
  {
    dynamics::SoftBodyNode* soft
        = dynamic_cast<dynamics::SoftBodyNode*>((*it).bodyNode);
    if (soft)
      newSoftSkeleton->addSoftBodyNode(soft);
    else
      newSoftSkeleton->addBodyNode((*it).bodyNode);
  }

  return newSoftSkeleton;
}

SkelParser::SkelBodyNode SoftSkelParser::readSoftBodyNode(
    tinyxml2::XMLElement*    _softBodyNodeElement,
    dynamics::SoftSkeleton*  _softSkeleton,
    const Eigen::Isometry3d& _skeletonFrame)
{
  //---------------------------------- Note ------------------------------------
  // SoftBodyNode is created if _softBodyNodeElement has <soft_shape>.
  // Otherwise, BodyNode is created.

  //----------------------------------------------------------------------------
  assert(_softBodyNodeElement != NULL);
  assert(_softSkeleton != NULL);

  // If _softBodyNodeElement has no <soft_shape>, return rigid body node
  if (!hasElement(_softBodyNodeElement, "soft_shape"))
  {
    return SkelParser::readBodyNode(_softBodyNodeElement,
                                    _softSkeleton,
                                    _skeletonFrame);
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
  if (hasElement(_softBodyNodeElement, "transformation"))
  {
    Eigen::Isometry3d W =
        getValueIsometry3d(_softBodyNodeElement, "transformation");
    initTransform = _skeletonFrame * W;
  }
  else
  {
    initTransform = _skeletonFrame;
  }

  // visualization_shape
  if (hasElement(_softBodyNodeElement, "visualization_shape"))
  {
    tinyxml2::XMLElement* vizElement
        = getElement(_softBodyNodeElement, "visualization_shape");

    dynamics::Shape* shape = NULL;

    // type
    assert(hasElement(vizElement, "geometry"));
    tinyxml2::XMLElement* geometryElement = getElement(vizElement, "geometry");

    // FIXME: Assume that type has only one shape type.
    if (hasElement(geometryElement, "box"))
    {
      tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

      Eigen::Vector3d size = getValueVector3d(boxElement, "size");

      shape = new dynamics::BoxShape(size);
    }
    else if (hasElement(geometryElement, "ellipsoid"))
    {
      tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement,
                                                          "ellipsoid");

      Eigen::Vector3d size = getValueVector3d(ellipsoidElement, "size");

      shape = new dynamics::EllipsoidShape(size);
    }
    else if (hasElement(geometryElement, "cylinder"))
    {
      tinyxml2::XMLElement* cylinderElement = getElement(geometryElement,
                                                         "cylinder");

      double radius = getValueDouble(cylinderElement, "radius");
      double height = getValueDouble(cylinderElement, "height");

      shape = new dynamics::CylinderShape(radius, height);
    }
    else
    {
      dterr << "Unknown visualization shape.\n";
      assert(0);
    }
    newSoftBodyNode->addVisualizationShape(shape);

    // transformation
    if (hasElement(vizElement, "transformation"))
    {
      Eigen::Isometry3d W = getValueIsometry3d(vizElement, "transformation");
      shape->setLocalTransform(W);
    }
  }

  // collision_shape
  if (hasElement(_softBodyNodeElement, "collision_shape"))
  {
    tinyxml2::XMLElement* colElement
        = getElement(_softBodyNodeElement, "collision_shape");

    dynamics::Shape* shape = NULL;

    // type
    assert(hasElement(colElement, "geometry"));
    tinyxml2::XMLElement* geometryElement = getElement(colElement, "geometry");

    // FIXME: Assume that type has only one shape type.
    if (hasElement(geometryElement, "box"))
    {
      tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

      Eigen::Vector3d size = getValueVector3d(boxElement, "size");

      shape = new dynamics::BoxShape(size);
    }
    else if (hasElement(geometryElement, "ellipsoid"))
    {
      tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement,
                                                          "ellipsoid");

      Eigen::Vector3d size = getValueVector3d(ellipsoidElement, "size");

      shape = new dynamics::EllipsoidShape(size);
    }
    else if (hasElement(geometryElement, "cylinder"))
    {
      tinyxml2::XMLElement* cylinderElement = getElement(geometryElement,
                                                         "cylinder");

      double radius = getValueDouble(cylinderElement, "radius");
      double height = getValueDouble(cylinderElement, "height");

      shape = new dynamics::CylinderShape(radius, height);
    }
    else
    {
      dterr << "Unknown visualization shape.\n";
      assert(0);
    }
    newSoftBodyNode->addCollisionShape(shape);

    // transformation
    if (hasElement(colElement, "transformation"))
    {
      Eigen::Isometry3d W = getValueIsometry3d(colElement, "transformation");
      shape->setLocalTransform(W);
    }
  }

  //--------------------------------------------------------------------------
  // inertia
  if (hasElement(_softBodyNodeElement, "inertia"))
  {
    tinyxml2::XMLElement* inertiaElement = getElement(_softBodyNodeElement,
                                                      "inertia");

    // mass
    double mass = getValueDouble(inertiaElement, "mass");
    newSoftBodyNode->setMass(mass);

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

      newSoftBodyNode->setInertia(ixx, iyy, izz, ixy, ixz, iyz);
    }
    else if (newSoftBodyNode->getVisualizationShape(0) != 0)
    {
      Eigen::Matrix3d Ic =
          newSoftBodyNode->getVisualizationShape(0)->computeInertia(mass);

      newSoftBodyNode->setInertia(Ic(0, 0), Ic(1, 1), Ic(2, 2),
                                  Ic(0, 1), Ic(0, 2), Ic(1, 2));
    }

    // offset
    if (hasElement(inertiaElement, "offset"))
    {
      Eigen::Vector3d offset = getValueVector3d(inertiaElement, "offset");
      newSoftBodyNode->setLocalCOM(offset);
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
//      Eigen::Vector3i frags = getValueVector3i(boxEle, "frags");
      dynamics::SoftBodyNodeHelper::setBox(newSoftBodyNode, size, T, totalMass);
//      dynamics::SoftBodyNodeHelper::setBox(newSoftBodyNode, size, frags,
//                                           totalMass);

      // Visualization shape
      newSoftBodyNode->addVisualizationShape(
            new dynamics::SoftMeshShape(newSoftBodyNode));

      // Collision shape
      newSoftBodyNode->addCollisionShape(
            new dynamics::SoftMeshShape(newSoftBodyNode));
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
            new dynamics::SoftMeshShape(newSoftBodyNode));

      // Collision shape
      newSoftBodyNode->addCollisionShape(
            new dynamics::SoftMeshShape(newSoftBodyNode));
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

  SkelBodyNode softBodyNode;
  softBodyNode.bodyNode = newSoftBodyNode;
  softBodyNode.initTransform = initTransform;

  return softBodyNode;
}

dynamics::Joint* SoftSkelParser::readSoftJoint(
    tinyxml2::XMLElement* _jointElement,
    const std::vector<SkelBodyNode,
    Eigen::aligned_allocator<SkelBodyNode> >& _softBodyNodes)
{
  assert(_jointElement != NULL);

  dynamics::Joint* newJoint = NULL;

  //--------------------------------------------------------------------------
  // Type attribute
  std::string type = getAttribute(_jointElement, "type");
  assert(!type.empty());
  if (type == std::string("weld"))
    newJoint = SkelParser::readWeldJoint(_jointElement);
  if (type == std::string("revolute"))
    newJoint = SkelParser::readRevoluteJoint(_jointElement);
  if (type == std::string("prismatic"))
    newJoint = SkelParser::readPrismaticJoint(_jointElement);
  if (type == std::string("ball"))
    newJoint = SkelParser::readBallJoint(_jointElement);
  if (type == std::string("translational"))
    newJoint = SkelParser::readTranslationalJoint(_jointElement);
  if (type == std::string("free"))
    newJoint = SkelParser::readFreeJoint(_jointElement);
  assert(newJoint != NULL);

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(_jointElement, "name");
  newJoint->setName(name);

  //--------------------------------------------------------------------------
  // parent
  SkelBodyNode softParentBodyNode;
  softParentBodyNode.bodyNode = NULL;
  softParentBodyNode.initTransform = Eigen::Isometry3d::Identity();

  if (hasElement(_jointElement, "parent"))
  {
    std::string strParent = getValueString(_jointElement, "parent");

    if (strParent != std::string("world"))
    {
      for (std::vector<SkelBodyNode,
           Eigen::aligned_allocator<SkelBodyNode> >::const_iterator it =
           _softBodyNodes.begin(); it != _softBodyNodes.end(); ++it)
        if ((*it).bodyNode->getName() == strParent)
        {
          softParentBodyNode = (*it);
          break;
        }

      if (softParentBodyNode.bodyNode == NULL)
      {
        dterr << "Can't find the parent body ["
              << strParent
              << "] of the joint ["
              << newJoint->getName()
              << "]. " << std::endl;
        assert(0);
      }
    }
  }
  else
  {
    dterr << "No parent body.\n";
    assert(0);
  }

  //--------------------------------------------------------------------------
  // child
  SkelBodyNode softChildBodyNode;
  softChildBodyNode.bodyNode = NULL;
  softChildBodyNode.initTransform = Eigen::Isometry3d::Identity();

  if (hasElement(_jointElement, "child"))
  {
    std::string strChild = getValueString(_jointElement, "child");

    for (std::vector<SkelBodyNode,
         Eigen::aligned_allocator<SkelBodyNode> >::const_iterator it =
         _softBodyNodes.begin(); it != _softBodyNodes.end(); ++it)
    {
      if ((*it).bodyNode->getName() == strChild)
      {
        softChildBodyNode = (*it);
        break;
      }
    }

    if (softChildBodyNode.bodyNode == NULL)
    {
      dterr << "Can't find the child body ["
            << strChild
            << "] of the joint ["
            << newJoint->getName()
            << "]. " << std::endl;
      assert(0);
    }
  }
  else
  {
    dterr << "Set child body node for " << newJoint->getName() << "."
          << std::endl;
    assert(0);
  }

  softChildBodyNode.bodyNode->setParentJoint(newJoint);

  if (softParentBodyNode.bodyNode)
    softParentBodyNode.bodyNode->addChildBodyNode(
          softChildBodyNode.bodyNode);

  //--------------------------------------------------------------------------
  // transformation
  Eigen::Isometry3d parentWorld = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childWorld = softChildBodyNode.initTransform;
  if (softParentBodyNode.bodyNode)
    parentWorld = softParentBodyNode.initTransform;
  if (hasElement(_jointElement, "transformation"))
    childToJoint = getValueIsometry3d(_jointElement, "transformation");
  Eigen::Isometry3d parentToJoint =
      parentWorld.inverse()*childWorld*childToJoint;
  newJoint->setTransformFromChildBodyNode(childToJoint);
  newJoint->setTransformFromParentBodyNode(parentToJoint);

  return newJoint;
}

}  // namespace utils
}  // namespace dart
