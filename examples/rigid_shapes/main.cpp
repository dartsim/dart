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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "dart/common/macros.hpp"
#include "dart/common/string.hpp"

#include <dart/gui/all.hpp>

#include <dart/utils/All.hpp>

#include <dart/All.hpp>
#include <dart/io/read.hpp>

#include <CLI/CLI.hpp>
#include <fcl/config.h>

#include <iostream>
#include <span>
#include <string>
#include <vector>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui;
using namespace dart::utils;
using namespace dart::math;

namespace {

bool tryParseCollisionDetector(
    const std::string& value, CollisionDetectorType& detector)
{
  if (value == "dart") {
    detector = CollisionDetectorType::Dart;
    return true;
  }
  if (value == "fcl") {
    detector = CollisionDetectorType::Fcl;
    return true;
  }
  if (value == "bullet") {
    detector = CollisionDetectorType::Bullet;
    return true;
  }
  if (value == "ode") {
    detector = CollisionDetectorType::Ode;
    return true;
  }
  return false;
}

bool updateGroundThickness(const WorldPtr& world, double thickness)
{
  if (!world) {
    return false;
  }

  SkeletonPtr groundSkeleton = world->getSkeleton("ground skeleton");
  if (!groundSkeleton) {
    for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
      auto skeleton = world->getSkeleton(i);
      if (skeleton && skeleton->getBodyNode("ground")) {
        groundSkeleton = skeleton;
        break;
      }
    }
  }

  if (!groundSkeleton) {
    return false;
  }

  auto* groundBody = groundSkeleton->getBodyNode("ground");
  if (!groundBody) {
    return false;
  }

  auto updateShapeNode = [thickness](ShapeNode* shapeNode) {
    if (!shapeNode) {
      return;
    }

    auto box = std::dynamic_pointer_cast<BoxShape>(shapeNode->getShape());
    if (!box) {
      return;
    }

    const Eigen::Vector3d originalSize = box->getSize();
    const Eigen::Vector3d originalTranslation
        = shapeNode->getRelativeTranslation();
    const double originalTop = originalTranslation.y() + 0.5 * originalSize.y();

    Eigen::Vector3d size = originalSize;
    size.y() = thickness;
    box->setSize(size);

    Eigen::Vector3d translation = originalTranslation;
    translation.y() = originalTop - 0.5 * thickness;
    shapeNode->setRelativeTranslation(translation);
  };

  groundBody->eachShapeNodeWith<VisualAspect>(updateShapeNode);
  groundBody->eachShapeNodeWith<CollisionAspect>(updateShapeNode);

  return true;
}

} // namespace

class CustomWorldNode : public RealTimeWorldNode
{
public:
  CustomWorldNode(
      const WorldPtr& world,
      const std::shared_ptr<PointCloudShape>& contactShape,
      VisualAspect* contactVisual)
    : RealTimeWorldNode(world),
      mContactShape(contactShape),
      mContactVisual(contactVisual),
      mContactPointsVisible(false)
  {
    if (mContactVisual) {
      mContactVisual->setHidden(true);
    }
  }

  void toggleContactPoints()
  {
    setContactPointsVisible(!mContactPointsVisible);
  }

  void setContactPointsVisible(bool visible)
  {
    mContactPointsVisible = visible;
    if (mContactVisual) {
      mContactVisual->setHidden(!visible);
    }
    if (!visible && mContactShape) {
      mContactShape->removeAllPoints();
    }
  }

  bool getContactPointsVisible() const
  {
    return mContactPointsVisible;
  }

  void customPreRefresh() override
  {
    if (!mContactPointsVisible || !mContactShape) {
      return;
    }

    const auto& contacts = getWorld()->getLastCollisionResult().getContacts();
    mContactPoints.clear();
    mContactPoints.reserve(contacts.size());
    for (const auto& contact : contacts) {
      mContactPoints.push_back(contact.point);
    }

    mContactShape->setPoint(
        std::span<const Eigen::Vector3d>(
            mContactPoints.data(), mContactPoints.size()));
  }

private:
  std::shared_ptr<PointCloudShape> mContactShape;
  VisualAspect* mContactVisual;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      mContactPoints;
  bool mContactPointsVisible;
};

class RigidShapesEventHandler : public ::osgGA::GUIEventHandler
{
public:
  RigidShapesEventHandler(const WorldPtr& world, CustomWorldNode* worldNode)
    : mWorld(world), mWorldNode(worldNode)
  {
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN) {
      switch (ea.getKey()) {
        case 'q':
        case 'Q':
          spawnBox(
              getRandomTransform(),
              Random::uniform<Eigen::Vector3d>(0.05, 0.25));
          return true;
        case 'w':
        case 'W':
          spawnEllipsoid(
              getRandomTransform(),
              Random::uniform<Eigen::Vector3d>(0.025, 0.125));
          return true;
        case 'e':
        case 'E': {
          const double radius = Random::uniform(0.05, 0.25);
          const double height = Random::uniform(0.1, 0.5);
          spawnCylinder(getRandomTransform(), radius, height);
          return true;
        }
        case 'r':
        case 'R': {
          const int vertexCount = Random::uniform<int>(16, 32);
          const double bound = Random::uniform(0.1, 0.25);
          spawnConvexMesh(getRandomTransform(), vertexCount, bound);
          return true;
        }
        case 'a':
        case 'A':
          if (mWorld->getNumSkeletons() > 1) {
            mWorld->removeSkeleton(
                mWorld->getSkeleton(mWorld->getNumSkeletons() - 1));
          }
          return true;
        case 'c':
        case 'C':
          if (mWorldNode) {
            mWorldNode->toggleContactPoints();
          }
          return true;
        default:
          return false;
      }
    }
    return false;
  }

private:
  std::string nextSkeletonName(const std::string& prefix)
  {
    return prefix + "_" + std::to_string(mSpawnCount++);
  }

  Eigen::Isometry3d getRandomTransform()
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    const Eigen::Vector3d rotation = Random::uniform<Eigen::Vector3d>(-pi, pi);
    const Eigen::Vector3d position = Eigen::Vector3d(
        Random::uniform(-1.0, 1.0),
        Random::uniform(0.5, 1.0),
        Random::uniform(-1.0, 1.0));

    T.translation() = position;
    T.linear() = expMapRot(rotation);

    return T;
  }

  void spawnBox(
      const Eigen::Isometry3d& _T,
      const Eigen::Vector3d& _size,
      double _mass = 10)
  {
    SkeletonPtr newSkeleton = Skeleton::create(nextSkeletonName("box"));

    ShapePtr newShape(new BoxShape(_size));

    BodyNode::Properties bodyProp;
    bodyProp.mName = "box_link";
    bodyProp.mInertia.setMass(_mass);

    FreeJoint::Properties jointProp;
    jointProp.mName = "box_joint";
    jointProp.mT_ParentBodyToJoint = _T;

    auto pair = newSkeleton->createJointAndBodyNodePair<FreeJoint>(
        nullptr, jointProp, bodyProp);
    auto shapeNode = pair.second->createShapeNodeWith<
        VisualAspect,
        CollisionAspect,
        DynamicsAspect>(newShape);
    shapeNode->getVisualAspect()->setColor(
        Random::uniform<Eigen::Vector3d>(0.0, 1.0));

    mWorld->addSkeleton(newSkeleton);
  }

  void spawnEllipsoid(
      const Eigen::Isometry3d& _T,
      const Eigen::Vector3d& _radii,
      double _mass = 10)
  {
    SkeletonPtr newSkeleton = Skeleton::create(nextSkeletonName("ellipsoid"));

    ShapePtr newShape(new EllipsoidShape(_radii * 2.0));

    BodyNode::Properties bodyProp;
    bodyProp.mName = "ellipsoid_link";
    bodyProp.mInertia.setMass(_mass);

    FreeJoint::Properties jointProp;
    jointProp.mName = "ellipsoid_joint";
    jointProp.mT_ParentBodyToJoint = _T;

    auto pair = newSkeleton->createJointAndBodyNodePair<FreeJoint>(
        nullptr, jointProp, bodyProp);
    auto shapeNode = pair.second->createShapeNodeWith<
        VisualAspect,
        CollisionAspect,
        DynamicsAspect>(newShape);
    shapeNode->getVisualAspect()->setColor(
        Random::uniform<Eigen::Vector3d>(0.0, 1.0));

    mWorld->addSkeleton(newSkeleton);
  }

  void spawnCylinder(
      const Eigen::Isometry3d& _T,
      double _radius,
      double _height,
      double _mass = 10)
  {
    SkeletonPtr newSkeleton = Skeleton::create(nextSkeletonName("cylinder"));

    ShapePtr newShape(new CylinderShape(_radius, _height));

    BodyNode::Properties bodyProp;
    bodyProp.mName = "cylinder_link";
    bodyProp.mInertia.setMass(_mass);

    FreeJoint::Properties jointProp;
    jointProp.mName = "cylinder_joint";
    jointProp.mT_ParentBodyToJoint = _T;

    auto pair = newSkeleton->createJointAndBodyNodePair<FreeJoint>(
        nullptr, jointProp, bodyProp);
    auto shapeNode = pair.second->createShapeNodeWith<
        VisualAspect,
        CollisionAspect,
        DynamicsAspect>(newShape);
    shapeNode->getVisualAspect()->setColor(
        Random::uniform<Eigen::Vector3d>(0.0, 1.0));

    mWorld->addSkeleton(newSkeleton);
  }

  void spawnConvexMesh(
      const Eigen::Isometry3d& _T,
      int _vertexCount,
      double _bound,
      double _mass = 10)
  {
    SkeletonPtr newSkeleton = Skeleton::create(nextSkeletonName("convex_mesh"));

    auto mesh = std::make_shared<ConvexMeshShape::TriMeshType>();
    mesh->reserveVertices(_vertexCount + 4);

    const double seed = _bound * 0.6;
    mesh->addVertex(Eigen::Vector3d(-seed, -seed, -seed));
    mesh->addVertex(Eigen::Vector3d(seed, -seed, seed));
    mesh->addVertex(Eigen::Vector3d(-seed, seed, seed));
    mesh->addVertex(Eigen::Vector3d(seed, seed, -seed));

    for (int i = 0; i < _vertexCount; ++i) {
      mesh->addVertex(Random::uniform<Eigen::Vector3d>(-_bound, _bound));
    }

    ShapePtr newShape = ConvexMeshShape::fromMesh(mesh, true);

    BodyNode::Properties bodyProp;
    bodyProp.mName = "convex_mesh_link";
    bodyProp.mInertia.setMass(_mass);

    FreeJoint::Properties jointProp;
    jointProp.mName = "convex_mesh_joint";
    jointProp.mT_ParentBodyToJoint = _T;

    auto pair = newSkeleton->createJointAndBodyNodePair<FreeJoint>(
        nullptr, jointProp, bodyProp);
    auto shapeNode = pair.second->createShapeNodeWith<
        VisualAspect,
        CollisionAspect,
        DynamicsAspect>(newShape);
    shapeNode->getVisualAspect()->setColor(
        Random::uniform<Eigen::Vector3d>(0.0, 1.0));

    mWorld->addSkeleton(newSkeleton);
  }

protected:
  WorldPtr mWorld;
  CustomWorldNode* mWorldNode;
  std::size_t mSpawnCount{0};
};

int main(int argc, char* argv[])
{
  CLI::App app("Rigid shapes example");
  std::string collisionDetector = "file";
  std::size_t maxContacts = 1000;
  double groundThickness = 0.0;
  app.add_option(
      "--collision-detector",
      collisionDetector,
      "Collision detector backend: file, fcl, bullet, ode, dart");
  app.add_option(
      "--max-contacts",
      maxContacts,
      "Maximum number of contacts per collision pass (0 disables collision)");
  app.add_option(
      "--ground-thickness",
      groundThickness,
      "Override ground box thickness in meters (0 keeps the file value)");
  CLI11_PARSE(app, argc, argv);

  WorldPtr myWorld = dart::io::readWorld("dart://sample/skel/shapes.skel");
  DART_ASSERT(myWorld != nullptr);

  const std::string detectorLower = toLower(collisionDetector);
  if (detectorLower != "file") {
    CollisionDetectorType detectorType;
    if (!tryParseCollisionDetector(detectorLower, detectorType)) {
      std::cerr << "Unsupported collision detector: " << collisionDetector
                << std::endl;
      return 1;
    }
    myWorld->setCollisionDetector(detectorType);
  }

  if (const auto detector = myWorld->getCollisionDetector()) {
    std::cout << "Collision detector: " << detector->getTypeView() << std::endl;
  }

  auto& collisionOption = myWorld->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = maxContacts;

  if (groundThickness < 0.0) {
    std::cerr << "--ground-thickness must be non-negative." << std::endl;
    return 1;
  }

  if (groundThickness > 0.0) {
    if (!updateGroundThickness(myWorld, groundThickness)) {
      std::cerr << "Failed to update ground thickness." << std::endl;
      return 1;
    }
  }

  auto contactShape = std::make_shared<PointCloudShape>(0.02);
  contactShape->setDataVariance(Shape::DYNAMIC);
  contactShape->setPointShapeType(PointCloudShape::BILLBOARD_CIRCLE);

  auto contactFrame
      = SimpleFrame::createShared(Frame::World(), "contact_points");
  contactFrame->setShape(contactShape);
  auto* contactVisual = contactFrame->createVisualAspect();
  contactVisual->setRGBA(Eigen::Vector4d(0.9, 0.1, 0.1, 1.0));
  contactVisual->setHidden(true);
  myWorld->addSimpleFrame(contactFrame);

  ::osg::ref_ptr<CustomWorldNode> node
      = new CustomWorldNode(myWorld, contactShape, contactVisual);

  auto handler = new RigidShapesEventHandler(myWorld, node.get());

  auto viewer = Viewer();
  viewer.addWorldNode(node);
  viewer.addEventHandler(handler);

  viewer.addInstructionText("space bar: simulation on/off\n");
  viewer.addInstructionText("'q': spawn a random cube\n");
  viewer.addInstructionText("'w': spawn a random ellipsoid\n");
  viewer.addInstructionText("'e': spawn a random cylinder\n");
  viewer.addInstructionText("'r': spawn a random convex mesh\n");
  viewer.addInstructionText("'a': delete a spawned object at last\n");
  viewer.addInstructionText("'c': toggle contact points\n");
  std::cout << viewer.getInstructions() << std::endl;

  viewer.setUpViewInWindow(0, 0, 640, 480);

  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.0f, 2.0f, 2.0f),
      ::osg::Vec3(0.0f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 1.0f, 0.0f));

  viewer.setCameraManipulator(viewer.getCameraManipulator());

  viewer.run();

  return 0;
}
