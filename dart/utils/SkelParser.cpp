/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#include "dart/common/Console.h"
#include "dart/collision/dart/DARTCollisionDetector.h"
#include "dart/collision/fcl/FCLCollisionDetector.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
#include "dart/constraint/ConstraintDynamics.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/EulerJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/Paths.h"

namespace dart {
namespace utils {

simulation::World* SkelParser::readSkelFile(const std::string& _filename) {
  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _dartFile;
  try {
    openXMLFile(_dartFile, _filename.c_str());
  } catch(std::exception const& e) {
    printf("LoadFile Fails: %s\n", e.what());
    return NULL;
  }

  //--------------------------------------------------------------------------
  // Load DART
  tinyxml2::XMLElement* dartElement = NULL;
  dartElement = _dartFile.FirstChildElement("skel");
  if (dartElement == NULL)
    return NULL;

  //--------------------------------------------------------------------------
  // Load World
  tinyxml2::XMLElement* worldElement = NULL;
  worldElement = dartElement->FirstChildElement("world");
  if (worldElement == NULL)
    return NULL;

  simulation::World* newWorld = readWorld(worldElement);

  return newWorld;
}

simulation::World* SkelParser::readWorld(tinyxml2::XMLElement* _worldElement) {
  assert(_worldElement != NULL);

  // Create a world
  simulation::World* newWorld = new simulation::World;

  //--------------------------------------------------------------------------
  // Load physics
  tinyxml2::XMLElement* physicsElement = NULL;
  physicsElement = _worldElement->FirstChildElement("physics");
  if (physicsElement != NULL) {
    // Time step
    tinyxml2::XMLElement* timeStepElement = NULL;
    timeStepElement = physicsElement->FirstChildElement("time_step");
    if (timeStepElement != NULL) {
      std::string strTimeStep = timeStepElement->GetText();
      double timeStep = toDouble(strTimeStep);
      newWorld->setTimeStep(timeStep);
    }

    // Gravity
    tinyxml2::XMLElement* gravityElement = NULL;
    gravityElement = physicsElement->FirstChildElement("gravity");
    if (gravityElement != NULL) {
      std::string strGravity = gravityElement->GetText();
      Eigen::Vector3d gravity = toVector3d(strGravity);
      newWorld->setGravity(gravity);
    }

    // Collision detector
    if (hasElement(physicsElement, "collision_detector")) {
      std::string strCD = getValueString(physicsElement, "collision_detector");
      if (strCD == "fcl_mesh") {
        newWorld->getConstraintHandler()->setCollisionDetector(
              new collision::FCLMeshCollisionDetector());
      } else if (strCD == "fcl") {
        newWorld->getConstraintHandler()->setCollisionDetector(
              new collision::FCLCollisionDetector());
      } else if (strCD == "dart") {
        newWorld->getConstraintHandler()->setCollisionDetector(
              new collision::DARTCollisionDetector());
      } else {
        dtwarn << "Unknown collision detector[" << strCD << "]. "
               << "Default collision detector[fcl] will be loaded."
               << std::endl;
      }
    }
  }

  //--------------------------------------------------------------------------
  // Load skeletons
  ElementEnumerator skeletonElements(_worldElement, "skeleton");
  while (skeletonElements.next()) {
    dynamics::Skeleton* newSkeleton
        = readSkeleton(skeletonElements.get(), newWorld);

    newWorld->addSkeleton(newSkeleton);
  }

  return newWorld;
}

dynamics::Skeleton* SkelParser::readSkeleton(
    tinyxml2::XMLElement* _skeletonElement,
    simulation::World* _world) {
  assert(_skeletonElement != NULL);
  assert(_world != NULL);

  dynamics::Skeleton* newSkeleton = new dynamics::Skeleton;
  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(_skeletonElement, "name");
  newSkeleton->setName(name);

  //--------------------------------------------------------------------------
  // transformation
  if (hasElement(_skeletonElement, "transformation")) {
    Eigen::Isometry3d W =
        getValueIsometry3d(_skeletonElement, "transformation");
    skeletonFrame = W;
  }

  //--------------------------------------------------------------------------
  // immobile attribute
  tinyxml2::XMLElement* mobileElement = NULL;
  mobileElement = _skeletonElement->FirstChildElement("mobile");
  if (mobileElement != NULL) {
    std::string stdMobile = mobileElement->GetText();
    bool mobile = toBool(stdMobile);
    newSkeleton->setMobile(mobile);
  }

  //--------------------------------------------------------------------------
  // Bodies
  ElementEnumerator bodies(_skeletonElement, "body");
  std::vector<SkelBodyNode,
      Eigen::aligned_allocator<SkelBodyNode> > skelBodyNodes;
  while (bodies.next()) {
    SkelBodyNode newSkelBodyNode
        = readBodyNode(bodies.get(), newSkeleton, skeletonFrame);
    assert(newSkelBodyNode.bodyNode);
    skelBodyNodes.push_back(newSkelBodyNode);
  }

  //--------------------------------------------------------------------------
  // Joints
  ElementEnumerator joints(_skeletonElement, "joint");
  while (joints.next()) {
    readJoint(joints.get(), skelBodyNodes);
  }

  //--------------------------------------------------------------------------
  // Add FreeJoint to the body node that doesn't have parent joint
  for (unsigned int i = 0; i < skelBodyNodes.size(); ++i) {
    dynamics::BodyNode* bodyNode = skelBodyNodes[i].bodyNode;

    if (bodyNode->getParentJoint() == NULL) {
      // If this link has no parent joint, then we add 6-dof free joint.
      dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;

      newFreeJoint->setTransformFromParentBodyNode(
            bodyNode->getWorldTransform());
      newFreeJoint->setTransformFromChildBodyNode(
            Eigen::Isometry3d::Identity());

      bodyNode->setParentJoint(newFreeJoint);
    }
  }

  for (std::vector<SkelBodyNode,
       Eigen::aligned_allocator<SkelBodyNode> >::iterator it =
       skelBodyNodes.begin(); it != skelBodyNodes.end(); ++it)
    newSkeleton->addBodyNode((*it).bodyNode);

  return newSkeleton;
}

SkelParser::SkelBodyNode SkelParser::readBodyNode(
    tinyxml2::XMLElement* _bodyNodeElement,
    dynamics::Skeleton* _skeleton,
    const Eigen::Isometry3d& _skeletonFrame) {
  assert(_bodyNodeElement != NULL);
  assert(_skeleton != NULL);

  dynamics::BodyNode* newBodyNode = new dynamics::BodyNode;
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  std::string name = getAttribute(_bodyNodeElement, "name");
  newBodyNode->setName(name);

  //--------------------------------------------------------------------------
  // gravity
  if (hasElement(_bodyNodeElement, "gravity")) {
    bool gravityMode = getValueBool(_bodyNodeElement, "gravity");
    newBodyNode->setGravityMode(gravityMode);
  }

  //--------------------------------------------------------------------------
  // self_collide
  //    if (hasElement(_bodyElement, "self_collide"))
  //    {
  //        bool gravityMode = getValueBool(_bodyElement, "self_collide");
  //    }

  //--------------------------------------------------------------------------
  // transformation
  if (hasElement(_bodyNodeElement, "transformation")) {
    Eigen::Isometry3d W =
        getValueIsometry3d(_bodyNodeElement, "transformation");
    initTransform = _skeletonFrame * W;
  } else {
    initTransform = _skeletonFrame;
  }

  //--------------------------------------------------------------------------
  // visualization_shape
  ElementEnumerator vizShapes(_bodyNodeElement, "visualization_shape");
  while (vizShapes.next()) {
    dynamics::Shape* newShape
        = readShape(vizShapes.get());

    newBodyNode->addVisualizationShape(newShape);
  }

  //--------------------------------------------------------------------------
  // visualization_shape
  ElementEnumerator collShapes(_bodyNodeElement, "collision_shape");
  while (collShapes.next()) {
    dynamics::Shape* newShape
        = readShape(collShapes.get());

    newBodyNode->addCollisionShape(newShape);
  }

  //--------------------------------------------------------------------------
  // inertia
  if (hasElement(_bodyNodeElement, "inertia")) {
    tinyxml2::XMLElement* inertiaElement =
        getElement(_bodyNodeElement, "inertia");

    // mass
    double mass = getValueDouble(inertiaElement, "mass");
    newBodyNode->setMass(mass);

    // moment of inertia
    if (hasElement(inertiaElement, "moment_of_inertia")) {
      tinyxml2::XMLElement* moiElement
          = getElement(inertiaElement, "moment_of_inertia");

      double ixx = getValueDouble(moiElement, "ixx");
      double iyy = getValueDouble(moiElement, "iyy");
      double izz = getValueDouble(moiElement, "izz");

      double ixy = getValueDouble(moiElement, "ixy");
      double ixz = getValueDouble(moiElement, "ixz");
      double iyz = getValueDouble(moiElement, "iyz");

      newBodyNode->setInertia(ixx, iyy, izz, ixy, ixz, iyz);
    } else if (newBodyNode->getVisualizationShape(0) != 0) {
      Eigen::Matrix3d Ic =
          newBodyNode->getVisualizationShape(0)->computeInertia(mass);

      newBodyNode->setInertia(Ic(0, 0), Ic(1, 1), Ic(2, 2),
                              Ic(0, 1), Ic(0, 2), Ic(1, 2));
    }

    // offset
    if (hasElement(inertiaElement, "offset")) {
      Eigen::Vector3d offset = getValueVector3d(inertiaElement, "offset");
      newBodyNode->setLocalCOM(offset);
    }
  }

  SkelBodyNode skelBodyNode;
  skelBodyNode.bodyNode = newBodyNode;
  skelBodyNode.initTransform = initTransform;

  return skelBodyNode;
}

dynamics::Shape* SkelParser::readShape(tinyxml2::XMLElement* vizEle) {
  dynamics::Shape* newShape = NULL;

  // Geometry
  assert(hasElement(vizEle, "geometry"));
  tinyxml2::XMLElement* geometryEle = getElement(vizEle, "geometry");

  if (hasElement(geometryEle, "box")) {
    tinyxml2::XMLElement* boxEle       = getElement(geometryEle, "box");
    Eigen::Vector3d       size         = getValueVector3d(boxEle, "size");
    newShape = new dynamics::BoxShape(size);
  } else if (hasElement(geometryEle, "ellipsoid")) {
    tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "ellipsoid");
    Eigen::Vector3d       size         = getValueVector3d(ellipsoidEle, "size");
    newShape = new dynamics::EllipsoidShape(size);
  } else if (hasElement(geometryEle, "cylinder")) {
    tinyxml2::XMLElement* cylinderEle  = getElement(geometryEle, "cylinder");
    double                radius       = getValueDouble(cylinderEle, "radius");
    double                height       = getValueDouble(cylinderEle, "height");
    newShape = new dynamics::CylinderShape(radius, height);
  } else if (hasElement(geometryEle, "plane")) {
    tinyxml2::XMLElement* planeEle     = getElement(geometryEle, "plane");
    Eigen::Vector3d       normal       = getValueVector3d(planeEle, "normal");
    Eigen::Vector3d       point        = getValueVector3d(planeEle, "point");
    newShape = new dynamics::PlaneShape(normal, point);
  } else if (hasElement(geometryEle, "mesh")) {
    tinyxml2::XMLElement* meshEle      = getElement(geometryEle, "mesh");
    std::string           filename     = getValueString(meshEle, "file_name");
    Eigen::Vector3d       scale        = getValueVector3d(meshEle, "scale");
    // TODO(JS): Do we assume that all mesh files place at DART_DATA_PATH?
    const aiScene* model = dynamics::MeshShape::loadMesh(DART_DATA_PATH +
                                                         filename);
    if (model) {
      newShape = new dynamics::MeshShape(scale, model);
    } else {
      dterr << "Fail to load model[" << filename << "]." << std::endl;
    }
  } else {
    dterr << "Unknown visualization shape.\n";
    assert(0);
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

dynamics::Joint* SkelParser::readJoint(
    tinyxml2::XMLElement* _jointElement,
    const std::vector<SkelBodyNode,
        Eigen::aligned_allocator<SkelBodyNode> >& _skelBodyNodes) {
  assert(_jointElement != NULL);

  dynamics::Joint* newJoint = NULL;

  //--------------------------------------------------------------------------
  // Type attribute
  std::string type = getAttribute(_jointElement, "type");
  assert(!type.empty());
  if (type == std::string("weld"))
    newJoint = readWeldJoint(_jointElement);
  if (type == std::string("prismatic"))
    newJoint = readPrismaticJoint(_jointElement);
  if (type == std::string("revolute"))
    newJoint = readRevoluteJoint(_jointElement);
  if (type == std::string("universal"))
    newJoint = readUniversalJoint(_jointElement);
  if (type == std::string("ball"))
    newJoint = readBallJoint(_jointElement);
  if (type == std::string("euler"))
    newJoint = readEulerJoint(_jointElement);
  if (type == std::string("translational"))
    newJoint = readTranslationalJoint(_jointElement);
  if (type == std::string("free"))
    newJoint = readFreeJoint(_jointElement);
  assert(newJoint != NULL);

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(_jointElement, "name");
  newJoint->setName(name);

  //--------------------------------------------------------------------------
  // parent
  SkelBodyNode skelParentBodyNode;
  skelParentBodyNode.bodyNode = NULL;
  skelParentBodyNode.initTransform = Eigen::Isometry3d::Identity();

  if (hasElement(_jointElement, "parent")) {
    std::string strParent = getValueString(_jointElement, "parent");

    if (strParent != std::string("world")) {
      for (std::vector<SkelBodyNode,
           Eigen::aligned_allocator<SkelBodyNode> >::const_iterator it =
           _skelBodyNodes.begin(); it != _skelBodyNodes.end(); ++it)
        if ((*it).bodyNode->getName() == strParent) {
          skelParentBodyNode = (*it);
          break;
        }

      if (skelParentBodyNode.bodyNode == NULL) {
        dterr << "Can't find the parent body ["
              << strParent
              << "] of the joint ["
              << newJoint->getName()
              << "]. " << std::endl;
        assert(0);
      }
    }
  } else {
    dterr << "No parent body.\n";
    assert(0);
  }

  //--------------------------------------------------------------------------
  // child
  SkelBodyNode skelChildBodyNode;
  skelChildBodyNode.bodyNode = NULL;
  skelChildBodyNode.initTransform = Eigen::Isometry3d::Identity();

  if (hasElement(_jointElement, "child")) {
    std::string strChild = getValueString(_jointElement, "child");

    for (std::vector<SkelBodyNode,
         Eigen::aligned_allocator<SkelBodyNode> >::const_iterator it =
         _skelBodyNodes.begin(); it != _skelBodyNodes.end(); ++it) {
      if ((*it).bodyNode->getName() == strChild) {
        skelChildBodyNode = (*it);
        break;
      }
    }

    if (skelChildBodyNode.bodyNode == NULL) {
      dterr << "Can't find the child body ["
            << strChild
            << "] of the joint ["
            << newJoint->getName()
            << "]. " << std::endl;
      assert(0);
    }
  } else {
    dterr << "Set child body node for " << newJoint->getName() << "."
          << std::endl;
    assert(0);
  }

  skelChildBodyNode.bodyNode->setParentJoint(newJoint);

  if (skelParentBodyNode.bodyNode)
    skelParentBodyNode.bodyNode->addChildBodyNode(skelChildBodyNode.bodyNode);

  //--------------------------------------------------------------------------
  // transformation
  Eigen::Isometry3d parentWorld = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childWorld = skelChildBodyNode.initTransform;
  if (skelParentBodyNode.bodyNode)
    parentWorld = skelParentBodyNode.initTransform;
  if (hasElement(_jointElement, "transformation"))
    childToJoint = getValueIsometry3d(_jointElement, "transformation");
  Eigen::Isometry3d parentToJoint =
      parentWorld.inverse()*childWorld*childToJoint;
  newJoint->setTransformFromChildBodyNode(childToJoint);
  newJoint->setTransformFromParentBodyNode(parentToJoint);

  return newJoint;
}

dynamics::WeldJoint* SkelParser::readWeldJoint(
    tinyxml2::XMLElement* _jointElement) {
  assert(_jointElement != NULL);

  dynamics::WeldJoint* newWeldJoint = new dynamics::WeldJoint;

  return newWeldJoint;
}

dynamics::RevoluteJoint* SkelParser::readRevoluteJoint(
    tinyxml2::XMLElement* _jointElement) {
  assert(_jointElement != NULL);

  dynamics::RevoluteJoint* newRevoluteJoint = new dynamics::RevoluteJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    tinyxml2::XMLElement* axisElement
        = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    newRevoluteJoint->setAxis(xyz);

    // damping
    if (hasElement(axisElement, "damping")) {
      double damping = getValueDouble(axisElement, "damping");
      newRevoluteJoint->setDampingCoefficient(0, damping);
    }

    // limit
    if (hasElement(axisElement, "limit")) {
      tinyxml2::XMLElement* limitElement
          = getElement(axisElement, "limit");

      // lower
      if (hasElement(limitElement, "lower")) {
        double lower = getValueDouble(limitElement, "lower");
        newRevoluteJoint->getGenCoord(0)->set_qMin(lower);
      }

      // upper
      if (hasElement(limitElement, "upper")) {
        double upper = getValueDouble(limitElement, "upper");
        newRevoluteJoint->getGenCoord(0)->set_qMax(upper);
      }
    }
  } else {
    assert(0);
  }

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    double init_pos = getValueDouble(_jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    newRevoluteJoint->set_q(ipos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    double init_vel = getValueDouble(_jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    newRevoluteJoint->set_q(ivel);
  }

  return newRevoluteJoint;
}

dynamics::PrismaticJoint* SkelParser::readPrismaticJoint(
    tinyxml2::XMLElement* _jointElement) {
  assert(_jointElement != NULL);

  dynamics::PrismaticJoint* newPrismaticJoint = new dynamics::PrismaticJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    tinyxml2::XMLElement* axisElement
        = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    newPrismaticJoint->setAxis(xyz);

    // damping
    if (hasElement(axisElement, "damping")) {
      double damping = getValueDouble(axisElement, "damping");
      newPrismaticJoint->setDampingCoefficient(0, damping);
    }

    // limit
    if (hasElement(axisElement, "limit")) {
      tinyxml2::XMLElement* limitElement
          = getElement(axisElement, "limit");

      // lower
      if (hasElement(limitElement, "lower")) {
        double lower = getValueDouble(limitElement, "lower");
        newPrismaticJoint->getGenCoord(0)->set_qMin(lower);
      }

      // upper
      if (hasElement(limitElement, "upper")) {
        double upper = getValueDouble(limitElement, "upper");
        newPrismaticJoint->getGenCoord(0)->set_qMax(upper);
      }
    }
  } else {
    assert(0);
  }

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    double init_pos = getValueDouble(_jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    newPrismaticJoint->set_q(ipos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    double init_vel = getValueDouble(_jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    newPrismaticJoint->set_q(ivel);
  }

  return newPrismaticJoint;
}

dynamics::ScrewJoint* SkelParser::readScrewJoint(
    tinyxml2::XMLElement* _jointElement) {
  assert(_jointElement != NULL);

  dynamics::ScrewJoint* newScrewJoint = new dynamics::ScrewJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    tinyxml2::XMLElement* axisElement
        = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    newScrewJoint->setAxis(xyz);

    // pitch
    if (hasElement(axisElement, "pitch")) {
      double pitch = getValueDouble(axisElement, "pitch");
      newScrewJoint->setPitch(pitch);
    }

    // damping
    if (hasElement(axisElement, "damping")) {
      double damping = getValueDouble(axisElement, "damping");
      newScrewJoint->setDampingCoefficient(0, damping);
    }

    // limit
    if (hasElement(axisElement, "limit")) {
      tinyxml2::XMLElement* limitElement
          = getElement(axisElement, "limit");

      // lower
      if (hasElement(limitElement, "lower")) {
        double lower = getValueDouble(limitElement, "lower");
        newScrewJoint->getGenCoord(0)->set_qMin(lower);
      }

      // upper
      if (hasElement(limitElement, "upper")) {
        double upper = getValueDouble(limitElement, "upper");
        newScrewJoint->getGenCoord(0)->set_qMax(upper);
      }
    }
  } else {
    assert(0);
  }

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    double init_pos = getValueDouble(_jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    newScrewJoint->set_q(ipos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    double init_vel = getValueDouble(_jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    newScrewJoint->set_q(ivel);
  }

  return newScrewJoint;
}

dynamics::UniversalJoint* SkelParser::readUniversalJoint(
    tinyxml2::XMLElement* _jointElement) {
  assert(_jointElement != NULL);

  dynamics::UniversalJoint* newUniversalJoint = new dynamics::UniversalJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    tinyxml2::XMLElement* axisElement
        = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    newUniversalJoint->setAxis1(xyz);

    // damping
    if (hasElement(axisElement, "damping")) {
      double damping = getValueDouble(axisElement, "damping");
      newUniversalJoint->setDampingCoefficient(0, damping);
    }

    // limit
    if (hasElement(axisElement, "limit")) {
      tinyxml2::XMLElement* limitElement
          = getElement(axisElement, "limit");

      // lower
      if (hasElement(limitElement, "lower")) {
        double lower = getValueDouble(limitElement, "lower");
        newUniversalJoint->getGenCoord(0)->set_qMin(lower);
      }

      // upper
      if (hasElement(limitElement, "upper")) {
        double upper = getValueDouble(limitElement, "upper");
        newUniversalJoint->getGenCoord(0)->set_qMax(upper);
      }
    }
  } else {
    assert(0);
  }

  //--------------------------------------------------------------------------
  // axis2
  if (hasElement(_jointElement, "axis2")) {
    tinyxml2::XMLElement* axis2Element
        = getElement(_jointElement, "axis2");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axis2Element, "xyz");
    newUniversalJoint->setAxis2(xyz);

    // damping
    if (hasElement(axis2Element, "damping")) {
      double damping = getValueDouble(axis2Element, "damping");
      newUniversalJoint->setDampingCoefficient(1, damping);
    }

    // limit
    if (hasElement(axis2Element, "limit")) {
      tinyxml2::XMLElement* limitElement
          = getElement(axis2Element, "limit");

      // lower
      if (hasElement(limitElement, "lower")) {
        double lower = getValueDouble(limitElement, "lower");
        newUniversalJoint->getGenCoord(1)->set_qMin(lower);
      }

      // upper
      if (hasElement(limitElement, "upper")) {
        double upper = getValueDouble(limitElement, "upper");
        newUniversalJoint->getGenCoord(1)->set_qMax(upper);
      }
    }
  } else {
    assert(0);
  }

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    Eigen::Vector2d init_pos = getValueVector2d(_jointElement, "init_pos");
    newUniversalJoint->set_q(init_pos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    Eigen::Vector2d init_vel = getValueVector2d(_jointElement, "init_vel");
    newUniversalJoint->set_q(init_vel);
  }

  return newUniversalJoint;
}

dynamics::BallJoint* SkelParser::readBallJoint(
    tinyxml2::XMLElement* _jointElement) {
  assert(_jointElement != NULL);

  dynamics::BallJoint* newBallJoint = new dynamics::BallJoint;

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    newBallJoint->set_q(init_pos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    newBallJoint->set_q(init_vel);
  }

  return newBallJoint;
}

dynamics::EulerJoint* SkelParser::readEulerJoint(
    tinyxml2::XMLElement* _jointElement) {
  assert(_jointElement != NULL);

  dynamics::EulerJoint* newEulerJoint = new dynamics::EulerJoint;

  //--------------------------------------------------------------------------
  // axis order
  std::string order = getValueString(_jointElement, "axis_order");
  if (order == "xyz") {
    newEulerJoint->setAxisOrder(dynamics::EulerJoint::AO_XYZ);
  } else if (order == "zyx") {
    newEulerJoint->setAxisOrder(dynamics::EulerJoint::AO_ZYX);
  } else {
    dterr << "Undefined Euler axis order\n";
    assert(0);
  }

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    tinyxml2::XMLElement* axisElement
        = getElement(_jointElement, "axis");

    // damping
    if (hasElement(axisElement, "damping")) {
      double damping = getValueDouble(axisElement, "damping");
      newEulerJoint->setDampingCoefficient(0, damping);
    }

    // limit
    if (hasElement(axisElement, "limit")) {
      tinyxml2::XMLElement* limitElement
          = getElement(axisElement, "limit");

      // lower
      if (hasElement(limitElement, "lower")) {
        double lower = getValueDouble(limitElement, "lower");
        newEulerJoint->getGenCoord(0)->set_qMin(lower);
      }

      // upper
      if (hasElement(limitElement, "upper")) {
        double upper = getValueDouble(limitElement, "upper");
        newEulerJoint->getGenCoord(0)->set_qMax(upper);
      }
    }
  }

  //--------------------------------------------------------------------------
  // axis2
  if (hasElement(_jointElement, "axis2")) {
    tinyxml2::XMLElement* axis2Element
        = getElement(_jointElement, "axis2");

    // damping
    if (hasElement(axis2Element, "damping")) {
      double damping = getValueDouble(axis2Element, "damping");
      newEulerJoint->setDampingCoefficient(1, damping);
    }

    // limit
    if (hasElement(axis2Element, "limit")) {
      tinyxml2::XMLElement* limitElement
          = getElement(axis2Element, "limit");

      // lower
      if (hasElement(limitElement, "lower")) {
        double lower = getValueDouble(limitElement, "lower");
        newEulerJoint->getGenCoord(1)->set_qMin(lower);
      }

      // upper
      if (hasElement(limitElement, "upper")) {
        double upper = getValueDouble(limitElement, "upper");
        newEulerJoint->getGenCoord(1)->set_qMax(upper);
      }
    }
  }

  //--------------------------------------------------------------------------
  // axis3
  if (hasElement(_jointElement, "axis3")) {
    tinyxml2::XMLElement* axis3Element
        = getElement(_jointElement, "axis3");

    // damping
    if (hasElement(axis3Element, "damping")) {
      double damping = getValueDouble(axis3Element, "damping");
      newEulerJoint->setDampingCoefficient(2, damping);
    }

    // limit
    if (hasElement(axis3Element, "limit")) {
      tinyxml2::XMLElement* limitElement
          = getElement(axis3Element, "limit");

      // lower
      if (hasElement(limitElement, "lower")) {
        double lower = getValueDouble(limitElement, "lower");
        newEulerJoint->getGenCoord(2)->set_qMin(lower);
      }

      // upper
      if (hasElement(limitElement, "upper")) {
        double upper = getValueDouble(limitElement, "upper");
        newEulerJoint->getGenCoord(2)->set_qMax(upper);
      }
    }
  }

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    newEulerJoint->set_q(init_pos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    newEulerJoint->set_q(init_vel);
  }

  return newEulerJoint;
}

dynamics::TranslationalJoint* SkelParser::readTranslationalJoint(
    tinyxml2::XMLElement* _jointElement) {
  assert(_jointElement != NULL);

  dynamics::TranslationalJoint* newTranslationalJoint
      = new dynamics::TranslationalJoint;

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    newTranslationalJoint->set_q(init_pos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    newTranslationalJoint->set_q(init_vel);
  }

  return newTranslationalJoint;
}

dynamics::FreeJoint* SkelParser::readFreeJoint(
    tinyxml2::XMLElement* _jointElement) {
  assert(_jointElement != NULL);

  dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    Eigen::Vector6d init_pos = getValueVector6d(_jointElement, "init_pos");
    newFreeJoint->set_q(init_pos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    Eigen::Vector6d init_vel = getValueVector6d(_jointElement, "init_vel");
    newFreeJoint->set_q(init_vel);
  }

  return newFreeJoint;
}

}  // namespace utils
}  // namespace dart
