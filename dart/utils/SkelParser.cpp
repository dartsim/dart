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
#include "dart/collision/dart/DARTCollisionDetector.h"
#include "dart/collision/fcl/FCLCollisionDetector.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
//#include "dart/constraint/OldConstraintDynamics.h"
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
#include "dart/utils/Paths.h"

namespace dart {
namespace utils {

simulation::World* SkelParser::readWorld(const std::string& _filename)
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

  simulation::World* newWorld = readWorld(worldElement);

  return newWorld;
}

dynamics::SkeletonPtr SkelParser::readSkeleton(const std::string& _filename)
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
    return nullptr;
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
  tinyxml2::XMLElement* skeletonElement = NULL;
  skeletonElement = skelElement->FirstChildElement("skeleton");
  if (skeletonElement == NULL)
  {
    dterr << "Skel file[" << _filename
          << "] does not contain <skeleton> element "
          <<"under <skel> element.\n";
    return NULL;
  }

  dynamics::SkeletonPtr newSkeleton = readSkeleton(skeletonElement);

  // Initialize skeleto to be ready for use
  newSkeleton->init();

  return newSkeleton;
}

simulation::World* SkelParser::readWorld(tinyxml2::XMLElement* _worldElement) {
  assert(_worldElement != NULL);

  // Create a world
  simulation::World* newWorld = new simulation::World;

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
      newWorld->setTimeStep(timeStep);
    }

    // Gravity
    tinyxml2::XMLElement* gravityElement = NULL;
    gravityElement = physicsElement->FirstChildElement("gravity");
    if (gravityElement != NULL)
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
              new collision::DARTCollisionDetector());
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
        = readSkeleton(SkeletonElements.get());

    newWorld->addSkeleton(newSkeleton);
  }

  return newWorld;
}

dynamics::SkeletonPtr SkelParser::readSkeleton(
    tinyxml2::XMLElement* _skeletonElement) {
  assert(_skeletonElement != NULL);

  dynamics::SkeletonPtr newSkeleton(new dynamics::Skeleton);
  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(_skeletonElement, "name");
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
  std::vector<SkelBodyNode, Eigen::aligned_allocator<SkelBodyNode> >
      softBodyNodes;
  while (bodies.next())
  {
    SkelBodyNode newSoftBodyNode
        = readSoftBodyNode(bodies.get(),
                           newSkeleton,
                           skeletonFrame);
    assert(newSoftBodyNode.bodyNode);
    softBodyNodes.push_back(newSoftBodyNode);
  }

  //--------------------------------------------------------------------------
  // Joints
  ElementEnumerator joints(_skeletonElement, "joint");
  while (joints.next())
    readJoint(joints.get(), softBodyNodes);

  //--------------------------------------------------------------------------
  // Add FreeJoint to the body node that doesn't have parent joint
  for (unsigned int i = 0; i < softBodyNodes.size(); ++i) {
    dynamics::BodyNode* bodyNode = softBodyNodes[i].bodyNode;

    if (bodyNode->getParentJoint() == NULL) {
      // If this link has no parent joint, then we add 6-dof free joint.
      dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;

      newFreeJoint->setTransformFromParentBodyNode(
            bodyNode->getTransform());
      newFreeJoint->setTransformFromChildBodyNode(
            Eigen::Isometry3d::Identity());

      bodyNode->setParentJoint(newFreeJoint);
    }
  }

  for (std::vector<SkelBodyNode,
       Eigen::aligned_allocator<SkelBodyNode> >::iterator it =
       softBodyNodes.begin();
       it != softBodyNodes.end(); ++it)
  {
    newSkeleton->addBodyNode((*it).bodyNode);
  }

  return newSkeleton;
}

SkelParser::SkelBodyNode SkelParser::readBodyNode(
    tinyxml2::XMLElement* _bodyNodeElement,
    dynamics::SkeletonPtr _skeleton,
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
    std::shared_ptr<dynamics::Shape> newShape(readShape(vizShapes.get()));

    newBodyNode->addVisualizationShape(newShape);
  }

  //--------------------------------------------------------------------------
  // visualization_shape
  ElementEnumerator collShapes(_bodyNodeElement, "collision_shape");
  while (collShapes.next()) {
    dynamics::ShapePtr newShape(readShape(collShapes.get()));

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

      newBodyNode->setMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
    } else if (newBodyNode->getVisualizationShape(0) != 0) {
      Eigen::Matrix3d Ic =
          newBodyNode->getVisualizationShape(0)->computeInertia(mass);

      newBodyNode->setMomentOfInertia(Ic(0, 0), Ic(1, 1), Ic(2, 2),
                                      Ic(0, 1), Ic(0, 2), Ic(1, 2));
    }

    // offset
    if (hasElement(inertiaElement, "offset")) {
      Eigen::Vector3d offset = getValueVector3d(inertiaElement, "offset");
      newBodyNode->setLocalCOM(offset);
    }
  }

  //--------------------------------------------------------------------------
  // marker
  ElementEnumerator markers(_bodyNodeElement, "marker");
  while (markers.next())
  {
    dynamics::Marker* newMarker = readMarker(markers.get(), newBodyNode);
    newBodyNode->addMarker(newMarker);
  }

  SkelBodyNode skelBodyNode;
  skelBodyNode.bodyNode = newBodyNode;
  skelBodyNode.initTransform = initTransform;

  return skelBodyNode;
}

SkelParser::SkelBodyNode SkelParser::readSoftBodyNode(
    tinyxml2::XMLElement* _softBodyNodeElement,
    dynamics::SkeletonPtr _Skeleton,
    const Eigen::Isometry3d& _skeletonFrame)
{
  //---------------------------------- Note ------------------------------------
  // SoftBodyNode is created if _softBodyNodeElement has <soft_shape>.
  // Otherwise, BodyNode is created.

  //----------------------------------------------------------------------------
  assert(_softBodyNodeElement != NULL);
  assert(_Skeleton != NULL);

  // If _softBodyNodeElement has no <soft_shape>, return rigid body node
  if (!hasElement(_softBodyNodeElement, "soft_shape"))
    return readBodyNode(_softBodyNodeElement, _Skeleton, _skeletonFrame);

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

    std::shared_ptr<dynamics::Shape> shape;


    // type
    assert(hasElement(vizElement, "geometry"));
    tinyxml2::XMLElement* geometryElement = getElement(vizElement, "geometry");

    // FIXME: Assume that type has only one shape type.
    if (hasElement(geometryElement, "box"))
    {
      tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

      Eigen::Vector3d size = getValueVector3d(boxElement, "size");

      shape = std::shared_ptr<dynamics::Shape>(new dynamics::BoxShape(size));
    }
    else if (hasElement(geometryElement, "ellipsoid"))
    {
      tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement,
                                                          "ellipsoid");

      Eigen::Vector3d size = getValueVector3d(ellipsoidElement, "size");

      shape = std::shared_ptr<dynamics::Shape>(
            new dynamics::EllipsoidShape(size));
    }
    else if (hasElement(geometryElement, "cylinder"))
    {
      tinyxml2::XMLElement* cylinderElement = getElement(geometryElement,
                                                         "cylinder");

      double radius = getValueDouble(cylinderElement, "radius");
      double height = getValueDouble(cylinderElement, "height");

      shape = std::shared_ptr<dynamics::Shape>(
            new dynamics::CylinderShape(radius, height));
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

    dynamics::ShapePtr shape;

    // type
    assert(hasElement(colElement, "geometry"));
    tinyxml2::XMLElement* geometryElement = getElement(colElement, "geometry");

    // FIXME: Assume that type has only one shape type.
    if (hasElement(geometryElement, "box"))
    {
      tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

      Eigen::Vector3d size = getValueVector3d(boxElement, "size");

      shape = dynamics::ShapePtr(new dynamics::BoxShape(size));
    }
    else if (hasElement(geometryElement, "ellipsoid"))
    {
      tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement,
                                                          "ellipsoid");

      Eigen::Vector3d size = getValueVector3d(ellipsoidElement, "size");

      shape = dynamics::ShapePtr(new dynamics::EllipsoidShape(size));
    }
    else if (hasElement(geometryElement, "cylinder"))
    {
      tinyxml2::XMLElement* cylinderElement = getElement(geometryElement,
                                                         "cylinder");

      double radius = getValueDouble(cylinderElement, "radius");
      double height = getValueDouble(cylinderElement, "height");

      shape = dynamics::ShapePtr(new dynamics::CylinderShape(radius, height));
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

      newSoftBodyNode->setMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
    }
    else if (newSoftBodyNode->getVisualizationShape(0) != 0)
    {
      Eigen::Matrix3d Ic =
          newSoftBodyNode->getVisualizationShape(0)->computeInertia(mass);

      newSoftBodyNode->setMomentOfInertia(Ic(0, 0), Ic(1, 1), Ic(2, 2),
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
      Eigen::Vector3i frags = getValueVector3i(boxEle, "frags");
      dynamics::SoftBodyNodeHelper::setBox(
            newSoftBodyNode, size, T, frags, totalMass);

      // Visualization shape
      newSoftBodyNode->addVisualizationShape(
            std::shared_ptr<dynamics::Shape>(
              new dynamics::SoftMeshShape(newSoftBodyNode)));

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

  //--------------------------------------------------------------------------
  // marker
  ElementEnumerator markers(_softBodyNodeElement, "marker");
  while (markers.next())
  {
    dynamics::Marker* newMarker = readMarker(markers.get(), newSoftBodyNode);
    newSoftBodyNode->addMarker(newMarker);
  }

  SkelBodyNode softBodyNode;
  softBodyNode.bodyNode = newSoftBodyNode;
  softBodyNode.initTransform = initTransform;

  return softBodyNode;
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

dynamics::Marker* SkelParser::readMarker(tinyxml2::XMLElement* _markerElement,
                                         dynamics::BodyNode* _bodyNode)
{
  // Name attribute
  std::string name = getAttribute(_markerElement, "name");

  // offset
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();
  if (hasElement(_markerElement, "offset"))
    offset = getValueVector3d(_markerElement, "offset");

  dynamics::Marker* newMarker = new dynamics::Marker(name, offset, _bodyNode);

  return newMarker;
}

dynamics::Joint* SkelParser::readJoint(
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
  if (type == std::string("planar"))
    newJoint = readPlanarJoint(_jointElement);
  if (type == std::string("free"))
    newJoint = readFreeJoint(_jointElement);
  assert(newJoint != NULL);

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(_jointElement, "name");
  newJoint->setName(name);

  //--------------------------------------------------------------------------
  // Actuator attribute
  if (hasAttribute(_jointElement, "actuator"))
  {
    const std::string actuator = getAttribute(_jointElement, "actuator");

    if (actuator == "force")
      newJoint->setActuatorType(dynamics::Joint::FORCE);
    else if (actuator == "passive")
      newJoint->setActuatorType(dynamics::Joint::PASSIVE);
    else if (actuator == "servo")
      newJoint->setActuatorType(dynamics::Joint::SERVO);
    else if (actuator == "acceleration")
      newJoint->setActuatorType(dynamics::Joint::ACCELERATION);
    else if (actuator == "velocity")
      newJoint->setActuatorType(dynamics::Joint::VELOCITY);
    else if (actuator == "locked")
      newJoint->setActuatorType(dynamics::Joint::LOCKED);
    else
      dterr << "Joint named [" << newJoint->getName()
            << "] contains invalid actuator attribute ["
            << actuator << "]." << std::endl;
  }
  else
  {
    newJoint->setActuatorType(dynamics::Joint::DefaultActuatorType);
  }

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

  //--------------------------------------------------------------------------
  // Degrees of Freedom
  ElementEnumerator DofElements(_jointElement, "dof");
  while(DofElements.next())
    readDegreeOfFreedom(DofElements.get(), newJoint);

  return newJoint;
}

//==============================================================================
static void getDofAttributeIfItExists(
    const std::string& _attribute,
    double& _value,
    const std::string& _element_type,
    const tinyxml2::XMLElement* _xmlElement,
    const dynamics::DegreeOfFreedom* _dof)
{
  if (_xmlElement->QueryDoubleAttribute(_attribute.c_str(), &_value)
      == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE)
  {
    dterr << "Invalid type for '" << _attribute << "' attribute of '"
          << _element_type << "' element in the '"
          << _dof->getName() << "' dof of '" << _dof->getJoint()->getName()
          << "'.\n";
  }
}

//==============================================================================
template <
    void (dynamics::DegreeOfFreedom::*setLimits)(double,double),
    std::pair<double,double> (dynamics::DegreeOfFreedom::*getLimits)() const,
    void (dynamics::DegreeOfFreedom::*setValue)(double),
    double (dynamics::DegreeOfFreedom::*getValue)() const>
static void setDofAttributes(tinyxml2::XMLElement* _dofElement,
                             dynamics::DegreeOfFreedom* _dof,
                             const std::string& _element_type)
{
  const tinyxml2::XMLElement* xmlElement
      = getElement(_dofElement, _element_type);

  std::pair<double,double> limit_vals = (_dof->*getLimits)();

  double defaultLower = limit_vals.first;
  double defaultUpper = limit_vals.second;
  double defaultInitial = (_dof->*getValue)();

  getDofAttributeIfItExists("lower", defaultLower,
                            _element_type, xmlElement, _dof);
  getDofAttributeIfItExists("upper", defaultUpper,
                            _element_type, xmlElement, _dof);
  getDofAttributeIfItExists("initial", defaultInitial,
                            _element_type, xmlElement, _dof);

  (_dof->*setLimits)(defaultLower, defaultUpper);
  (_dof->*setValue)(defaultInitial);
}

//==============================================================================
void SkelParser::readDegreeOfFreedom(tinyxml2::XMLElement* _dofElement,
                                     dynamics::Joint* _dartJoint)
{
  if (NULL == _dartJoint || NULL == _dofElement)
    return;

  int localIndex = -1;
  int xml_err = _dofElement->QueryIntAttribute("local_index", &localIndex);

  // If the localIndex is out of bounds, quit
  if (localIndex >= (int)_dartJoint->getNumDofs())
  {
    dterr << "[SkelParser::readDegreeOfFreedom] Joint named '"
          << _dartJoint->getName() << "' contains dof element with invalid "
          << "number attribute [" << localIndex << "]. It must be less than "
          << _dartJoint->getNumDofs() << ".\n";
    return;
  }

  // If no localIndex was found, report an error and quit
  if (localIndex == -1 && _dartJoint->getNumDofs() > 1)
  {
    if (tinyxml2::XML_NO_ATTRIBUTE == xml_err)
    {
      dterr << "[SkelParser::readDegreeOfFreedom] Joint named '"
            << _dartJoint->getName() << "' has " << _dartJoint->getNumDofs()
            << " DOFs, but the xml contains a dof element without its "
            << "local_index specified. For Joints with multiple DOFs, all dof "
            << "elements must specify their local_index attribute.\n";
    }
    else if (tinyxml2::XML_WRONG_ATTRIBUTE_TYPE == xml_err)
    {
      dterr << "[SkelParser::readDegreeOfFreedom] Joint named '"
            << _dartJoint->getName() << "' has a dof element with a wrongly "
            << "formatted local_index attribute.\n";
    }

    return;
  }
  // Unless the joint is a single-dof joint
  else if (localIndex == -1 && _dartJoint->getNumDofs() == 1)
    localIndex = 0;

  dart::dynamics::DegreeOfFreedom* dof = _dartJoint->getDof(localIndex);
  const char* name = _dofElement->Attribute("name");
  if (name)
  {
    dof->setName(std::string(name));
  }

  if (hasElement(_dofElement, "position"))
  {
    setDofAttributes<
        &dart::dynamics::DegreeOfFreedom::setPositionLimits,
        &dart::dynamics::DegreeOfFreedom::getPositionLimits,
        &dart::dynamics::DegreeOfFreedom::setPosition,
        &dart::dynamics::DegreeOfFreedom::getPosition>(_dofElement, dof,
                                                       "position");
  }

  if (hasElement(_dofElement, "velocity"))
  {
    setDofAttributes<
        &dart::dynamics::DegreeOfFreedom::setVelocityLimits,
        &dart::dynamics::DegreeOfFreedom::getVelocityLimits,
        &dart::dynamics::DegreeOfFreedom::setVelocity,
        &dart::dynamics::DegreeOfFreedom::getVelocity>(_dofElement, dof,
                                                       "velocity");
  }

  if (hasElement(_dofElement, "acceleration"))
  {
    setDofAttributes<
        &dart::dynamics::DegreeOfFreedom::setAccelerationLimits,
        &dart::dynamics::DegreeOfFreedom::getAccelerationLimits,
        &dart::dynamics::DegreeOfFreedom::setAcceleration,
        &dart::dynamics::DegreeOfFreedom::getAcceleration>(_dofElement, dof,
                                                           "acceleration");
  }

  if (hasElement(_dofElement, "force"))
  {
    setDofAttributes<
        &dart::dynamics::DegreeOfFreedom::setForceLimits,
        &dart::dynamics::DegreeOfFreedom::getForceLimits,
        &dart::dynamics::DegreeOfFreedom::setForce,
        &dart::dynamics::DegreeOfFreedom::getForce>(_dofElement, dof,
                                                    "force");
  }
}

//==============================================================================
dynamics::WeldJoint* SkelParser::readWeldJoint(
    tinyxml2::XMLElement* _jointElement)
{
  assert(_jointElement != NULL);

  dynamics::WeldJoint* newWeldJoint = new dynamics::WeldJoint;

  return newWeldJoint;
}

//==============================================================================
dynamics::RevoluteJoint* SkelParser::readRevoluteJoint(
    tinyxml2::XMLElement* _jointElement)
{
  assert(_jointElement != NULL);

  dynamics::RevoluteJoint* newRevoluteJoint = new dynamics::RevoluteJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    newRevoluteJoint->setAxis(xyz);
  }
  else
  {
    assert(0);
  }

  readJointDynamicsAndLimit(_jointElement, newRevoluteJoint, 1);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    double init_pos = getValueDouble(_jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    newRevoluteJoint->setPositions(ipos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    double init_vel = getValueDouble(_jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    newRevoluteJoint->setVelocities(ivel);
  }

  return newRevoluteJoint;
}

//==============================================================================
dynamics::PrismaticJoint* SkelParser::readPrismaticJoint(
    tinyxml2::XMLElement* _jointElement)
{
  assert(_jointElement != NULL);

  dynamics::PrismaticJoint* newPrismaticJoint = new dynamics::PrismaticJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    newPrismaticJoint->setAxis(xyz);
  }
  else
  {
    assert(0);
  }

  readJointDynamicsAndLimit(_jointElement, newPrismaticJoint, 1);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    double init_pos = getValueDouble(_jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    newPrismaticJoint->setPositions(ipos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    double init_vel = getValueDouble(_jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    newPrismaticJoint->setVelocities(ivel);
  }

  return newPrismaticJoint;
}
//==============================================================================
dynamics::ScrewJoint* SkelParser::readScrewJoint(
    tinyxml2::XMLElement* _jointElement)
{
  assert(_jointElement != NULL);

  dynamics::ScrewJoint* newScrewJoint = new dynamics::ScrewJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    newScrewJoint->setAxis(xyz);

    // pitch
    if (hasElement(axisElement, "pitch"))
    {
      double pitch = getValueDouble(axisElement, "pitch");
      newScrewJoint->setPitch(pitch);
    }
  }
  else
  {
    assert(0);
  }

  readJointDynamicsAndLimit(_jointElement, newScrewJoint, 1);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    double init_pos = getValueDouble(_jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    newScrewJoint->setPositions(ipos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    double init_vel = getValueDouble(_jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    newScrewJoint->setVelocities(ivel);
  }

  return newScrewJoint;
}

//==============================================================================
dynamics::UniversalJoint* SkelParser::readUniversalJoint(
    tinyxml2::XMLElement* _jointElement)
{
  assert(_jointElement != NULL);

  dynamics::UniversalJoint* newUniversalJoint = new dynamics::UniversalJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    newUniversalJoint->setAxis1(xyz);
  }
  else
  {
    assert(0);
  }

  //--------------------------------------------------------------------------
  // axis2
  if (hasElement(_jointElement, "axis2"))
  {
    tinyxml2::XMLElement* axis2Element = getElement(_jointElement, "axis2");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axis2Element, "xyz");
    newUniversalJoint->setAxis2(xyz);
  }
  else
  {
    assert(0);
  }

  readJointDynamicsAndLimit(_jointElement, newUniversalJoint, 2);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    Eigen::Vector2d init_pos = getValueVector2d(_jointElement, "init_pos");
    newUniversalJoint->setPositions(init_pos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    Eigen::Vector2d init_vel = getValueVector2d(_jointElement, "init_vel");
    newUniversalJoint->setVelocities(init_vel);
  }

  return newUniversalJoint;
}

//==============================================================================
dynamics::BallJoint* SkelParser::readBallJoint(
    tinyxml2::XMLElement* _jointElement)
{
  assert(_jointElement != NULL);

  dynamics::BallJoint* newBallJoint = new dynamics::BallJoint;

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    newBallJoint->setPositions(init_pos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    newBallJoint->setVelocities(init_vel);
  }

  return newBallJoint;
}

//==============================================================================
dynamics::EulerJoint* SkelParser::readEulerJoint(
    tinyxml2::XMLElement* _jointElement)
{
  assert(_jointElement != NULL);

  dynamics::EulerJoint* newEulerJoint = new dynamics::EulerJoint;

  //--------------------------------------------------------------------------
  // axis order
  std::string order = getValueString(_jointElement, "axis_order");
  if (order == "xyz")
  {
    newEulerJoint->setAxisOrder(dynamics::EulerJoint::AO_XYZ);
  }
  else if (order == "zyx")
  {
    newEulerJoint->setAxisOrder(dynamics::EulerJoint::AO_ZYX);
  }
  else
  {
    dterr << "Undefined Euler axis order\n";
    assert(0);
  }

  //--------------------------------------------------------------------------
  // axis
  readJointDynamicsAndLimit(_jointElement, newEulerJoint, 3);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    newEulerJoint->setPositions(init_pos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    newEulerJoint->setVelocities(init_vel);
  }

  return newEulerJoint;
}

//==============================================================================
dynamics::TranslationalJoint* SkelParser::readTranslationalJoint(
    tinyxml2::XMLElement* _jointElement)
{
  assert(_jointElement != NULL);

  dynamics::TranslationalJoint* newTranslationalJoint
      = new dynamics::TranslationalJoint;

  //--------------------------------------------------------------------------
  // axis
  readJointDynamicsAndLimit(_jointElement, newTranslationalJoint, 3);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    newTranslationalJoint->setPositions(init_pos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    newTranslationalJoint->setVelocities(init_vel);
  }

  return newTranslationalJoint;
}

//==============================================================================
dynamics::PlanarJoint* SkelParser::readPlanarJoint(
    tinyxml2::XMLElement* _jointElement)
{
  assert(_jointElement != NULL);

  dynamics::PlanarJoint* newPlanarJoint = new dynamics::PlanarJoint;

  //--------------------------------------------------------------------------
  // Plane
  if (hasElement(_jointElement, "plane"))
  {
    tinyxml2::XMLElement* planeElement = getElement(_jointElement, "plane");

    // Type attribute
    std::string type = getAttribute(planeElement, "type");

    if (type == "xy")
    {
      newPlanarJoint->setXYPlane();
    }
    else if (type == "yz")
    {
      newPlanarJoint->setYZPlane();
    }
    else if (type == "zx")
    {
      newPlanarJoint->setZXPlane();
    }
    else if (type == "arbitrary")
    {
      tinyxml2::XMLElement* transAxis1Element
          = getElement(planeElement, "translation_axis1");

      Eigen::Vector3d transAxis1 = getValueVector3d(transAxis1Element, "xyz");

      tinyxml2::XMLElement* transAxis2Element
          = getElement(planeElement, "translation_axis2");

      Eigen::Vector3d transAxis2 = getValueVector3d(transAxis2Element, "xyz");

      newPlanarJoint->setArbitraryPlane(transAxis1, transAxis2);
    }
    else
    {
      newPlanarJoint->setXYPlane();
      dterr << "Undefined plane type. XY-plane is set as default.\n";
    }
  }
  else
  {
    newPlanarJoint->setXYPlane();
    dterr << "Plane type is not specified. XY-plane is set as default.\n";
  }

  //--------------------------------------------------------------------------
  // axis
  readJointDynamicsAndLimit(_jointElement, newPlanarJoint, 3);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    newPlanarJoint->setPositions(init_pos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    newPlanarJoint->setVelocities(init_vel);
  }

  return newPlanarJoint;
}

//==============================================================================
dynamics::FreeJoint* SkelParser::readFreeJoint(
    tinyxml2::XMLElement* _jointElement)
{
  assert(_jointElement != NULL);

  dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos"))
  {
    Eigen::Vector6d init_pos = getValueVector6d(_jointElement, "init_pos");
    newFreeJoint->setPositions(init_pos);
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel"))
  {
    Eigen::Vector6d init_vel = getValueVector6d(_jointElement, "init_vel");
    newFreeJoint->setVelocities(init_vel);
  }

  return newFreeJoint;
}

//==============================================================================
void SkelParser::readJointDynamicsAndLimit(tinyxml2::XMLElement* _jointElement,
                                           dynamics::Joint* _joint,
                                           size_t _numAxis)
{
  assert(_jointElement != NULL);
  assert(_numAxis <= 6);

  std::string axisName = "axis";

  // axis
  for (size_t i = 0; i < _numAxis; ++i)
  {
    if (i != 0)
      axisName = "axis" + std::to_string(i + 1);

    if (hasElement(_jointElement, axisName))
    {
      tinyxml2::XMLElement* axisElement = getElement(_jointElement, axisName);

      // damping
      if (hasElement(axisElement, "damping"))
      {
        dtwarn << "<damping> tag is moved to under <dynamics> tag. "
               << "Please see (https://github.com/dartsim/dart/wiki/) "
               << "for the detail." << std::endl;
        double damping = getValueDouble(axisElement, "damping");
        _joint->setDampingCoefficient(i, damping);
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
          _joint->setDampingCoefficient(i, val);
        }

        // friction
        if (hasElement(dynamicsElement, "friction"))
        {
          double val = getValueDouble(dynamicsElement, "friction");
          _joint->setCoulombFriction(i, val);
        }

        // spring_rest_position
        if (hasElement(dynamicsElement, "spring_rest_position"))
        {
          double val = getValueDouble(dynamicsElement, "spring_rest_position");
          _joint->setRestPosition(i, val);
        }

        // friction
        if (hasElement(dynamicsElement, "spring_stiffness"))
        {
          double val = getValueDouble(dynamicsElement, "spring_stiffness");
          _joint->setSpringStiffness(i, val);
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
          _joint->setPositionLowerLimit(i, lower);
        }

        // upper
        if (hasElement(limitElement, "upper"))
        {
          double upper = getValueDouble(limitElement, "upper");
          _joint->setPositionUpperLimit(i, upper);
        }
      }
    }
  }
}

}  // namespace utils
}  // namespace dart
