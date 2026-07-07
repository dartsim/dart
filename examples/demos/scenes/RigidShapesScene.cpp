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

// Ported from examples/rigid_shapes: shapes.skel (assorted primitives resting
// on a ground plane) plus keyboard spawning of random boxes ('q'), ellipsoids
// ('w'), and cylinders ('e'); 'a' deletes the most recently spawned skeleton.
//
// Deviation from the original: the original unconditionally constructs a
// Bullet collision detector. dart-demos links dart-collision-bullet when that
// target exists in the build (see examples/demos/CMakeLists.txt), so this port
// opts into Bullet only if the host's collision-detector factory can create
// one (the graceful pattern AddDeleteSkelsScene also uses), falling back to
// DART's default (FCL) detector when the backend was not built; spawn behavior
// is otherwise unchanged.

#include "Scenes.hpp"
#include "ZUp.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <memory>
#include <stdexcept>

namespace dart_demos {

namespace {

//==============================================================================
Eigen::Isometry3d randomTransform()
{
  using dart::math::constantsd;
  using dart::math::expMapRot;
  using dart::math::Random;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();

  const Eigen::Vector3d rotation
      = Random::uniform<Eigen::Vector3d>(-constantsd::pi(), constantsd::pi());
  // Spawn above the ground plane; the vertical axis is Z here (the world was
  // reoriented to Z-up), where the original used Y.
  const Eigen::Vector3d position(
      Random::uniform(-1.0, 1.0),
      Random::uniform(-1.0, 1.0),
      Random::uniform(0.5, 1.0));

  tf.translation() = position;
  tf.linear() = expMapRot(rotation);
  return tf;
}

//==============================================================================
void spawnBox(const dart::simulation::WorldPtr& world)
{
  using namespace dart::dynamics;
  using dart::math::Random;

  auto skel = Skeleton::create();
  ShapePtr shape(new BoxShape(Random::uniform<Eigen::Vector3d>(0.05, 0.25)));

  BodyNode::Properties bodyProp;
  bodyProp.mName = "box_link";
  bodyProp.mInertia.setMass(10);

  FreeJoint::Properties jointProp;
  jointProp.mName = "box_joint";
  jointProp.mT_ParentBodyToJoint = randomTransform();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, jointProp, bodyProp);
  auto shapeNode = pair.second->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(
      Random::uniform<Eigen::Vector3d>(0.0, 1.0));

  world->addSkeleton(skel);
}

//==============================================================================
void spawnEllipsoid(const dart::simulation::WorldPtr& world)
{
  using namespace dart::dynamics;
  using dart::math::Random;

  auto skel = Skeleton::create();
  ShapePtr shape(
      new EllipsoidShape(Random::uniform<Eigen::Vector3d>(0.025, 0.125) * 2.0));

  BodyNode::Properties bodyProp;
  bodyProp.mName = "ellipsoid_link";
  bodyProp.mInertia.setMass(10);

  FreeJoint::Properties jointProp;
  jointProp.mName = "ellipsoid_joint";
  jointProp.mT_ParentBodyToJoint = randomTransform();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, jointProp, bodyProp);
  auto shapeNode = pair.second->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(
      Random::uniform<Eigen::Vector3d>(0.0, 1.0));

  world->addSkeleton(skel);
}

//==============================================================================
void spawnCylinder(const dart::simulation::WorldPtr& world)
{
  using namespace dart::dynamics;
  using dart::math::Random;

  auto skel = Skeleton::create();
  const double radius = Random::uniform(0.05, 0.25);
  const double height = Random::uniform(0.1, 0.5);
  ShapePtr shape(new CylinderShape(radius, height));

  BodyNode::Properties bodyProp;
  bodyProp.mName = "cylinder_link";
  bodyProp.mInertia.setMass(10);

  FreeJoint::Properties jointProp;
  jointProp.mName = "cylinder_joint";
  jointProp.mT_ParentBodyToJoint = randomTransform();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, jointProp, bodyProp);
  auto shapeNode = pair.second->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(
      Random::uniform<Eigen::Vector3d>(0.0, 1.0));

  world->addSkeleton(skel);
}

} // namespace

//==============================================================================
DemoScene makeRigidShapesScene()
{
  DemoScene scene;
  scene.id = "rigid_shapes";
  scene.title = "Rigid Shapes";
  scene.category = "Rigid Body";
  scene.summary
      = "Spawn assorted rigid shapes (boxes, ellipsoids, cylinders) onto a "
        "ground plane.";

  scene.factory = [] {
    auto world
        = dart::utils::SkelParser::readWorld("dart://sample/skel/shapes.skel");
    if (!world)
      throw std::runtime_error("failed to load dart://sample/skel/shapes.skel");
    reorientWorldToZUp(world);

    // The original unconditionally requires Bullet
    // (BulletCollisionDetector::create()); use the same graceful, string-keyed
    // factory check as AddDeleteSkelsScene instead. This picks up Bullet when
    // the host links dart-collision-bullet (it does when the backend is built)
    // and otherwise falls back to DART's default (FCL) detector.
    if (dart::collision::CollisionDetector::getFactory()->canCreate("bullet")) {
      world->getConstraintSolver()->setCollisionDetector(
          dart::collision::CollisionDetector::getFactory()->create("bullet"));
    }

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(2.0, -2.0, 2.0),
        ::osg::Vec3d(0.0, 0.0, 0.0),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.keyActions.push_back(KeyAction{'q', "Spawn box", [world] {
                                           spawnBox(world);
                                         }});
    setup.keyActions.push_back(KeyAction{'w', "Spawn ellipsoid", [world] {
                                           spawnEllipsoid(world);
                                         }});
    setup.keyActions.push_back(KeyAction{'e', "Spawn cylinder", [world] {
                                           spawnCylinder(world);
                                         }});
    setup.keyActions.push_back(
        KeyAction{'a', "Delete last skeleton", [world] {
                    if (world->getNumSkeletons() > 1)
                      world->removeSkeleton(
                          world->getSkeleton(world->getNumSkeletons() - 1));
                  }});

    setup.renderPanel = [world] {
      ImGui::Text("Skeletons: %zu", world->getNumSkeletons());
      ImGui::TextWrapped(
          "'q'/'w'/'e' spawn a random box/ellipsoid/cylinder above the "
          "ground; 'a' deletes the most recently spawned one.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
