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

// Ported from examples/add_delete_skels: a ground plane onto which random
// cubes are spawned ('q') and removed from the end of the list ('w').
//
// Deviation from the original: the original opportunistically switches to a
// Bullet collision detector when available. dart-demos links
// dart-collision-bullet when that target exists in the build (see
// examples/demos/CMakeLists.txt), so `canCreate("bullet")` succeeds and Bullet
// is used whenever the backend was built; otherwise the runtime lookup fails
// gracefully and DART's default (FCL) detector is used instead. Either way the
// spawn/delete behavior is unchanged, and the string-keyed factory check keeps
// this a faithful, forward-compatible port.

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
struct AddDeleteSkelsState
{
  int spawnCount = 0;
};

//==============================================================================
void spawnCube(
    const dart::simulation::WorldPtr& world, AddDeleteSkelsState& state)
{
  using dart::dynamics::BoxShape;
  using dart::dynamics::CollisionAspect;
  using dart::dynamics::DynamicsAspect;
  using dart::dynamics::FreeJoint;
  using dart::dynamics::Skeleton;
  using dart::dynamics::VisualAspect;
  using dart::math::Random;

  // Spawn position/size use the same random ranges as the original example;
  // the vertical axis is Z here (the world was reoriented to Z-up), where the
  // original used Y.
  const Eigen::Vector3d position(
      Random::uniform(-1.0, 1.0),
      Random::uniform(-1.0, 1.0),
      Random::uniform(0.5, 1.0));
  const Eigen::Vector3d size(
      Random::uniform(0.1, 0.5),
      Random::uniform(0.1, 0.5),
      Random::uniform(0.1, 0.5));
  constexpr double mass = 0.1;

  auto cubeSkel = Skeleton::create("cube" + std::to_string(state.spawnCount++));

  dart::dynamics::BodyNode::Properties body;
  body.mName = "cube_link";
  body.mInertia.setMass(mass);
  body.mInertia.setMoment(BoxShape::computeInertia(size, mass));
  auto boxShape = std::make_shared<BoxShape>(size);

  FreeJoint::Properties joint;
  joint.mName = "cube_joint";
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(position);

  auto pair
      = cubeSkel->createJointAndBodyNodePair<FreeJoint>(nullptr, joint, body);
  auto shapeNode = pair.second->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(
      Random::uniform<Eigen::Vector3d>(0.0, 1.0));

  world->addSkeleton(cubeSkel);
}

} // namespace

//==============================================================================
DemoScene makeAddDeleteSkelsScene()
{
  DemoScene scene;
  scene.id = "add_delete_skels";
  scene.title = "Add / Delete Skeletons";
  scene.category = "Rigid Body";
  scene.summary = "Spawn and delete dynamic cubes at runtime.";

  scene.factory = [] {
    auto world
        = dart::utils::SkelParser::readWorld("dart://sample/skel/ground.skel");
    if (!world)
      throw std::runtime_error("failed to load dart://sample/skel/ground.skel");
    world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
    reorientWorldToZUp(world);

    // Opportunistically use Bullet if the host links it in (see the file
    // comment above), matching the original's best-effort collision-detector
    // selection.
    if (dart::collision::CollisionDetector::getFactory()->canCreate("bullet")) {
      world->getConstraintSolver()->setCollisionDetector(
          dart::collision::CollisionDetector::getFactory()->create("bullet"));
    }

    auto state = std::make_shared<AddDeleteSkelsState>();

    DemoSceneSetup setup;
    setup.world = world;
    // Elevated three-quarter view, pulled back enough for the whole 4x4 m
    // ground.skel plane and a growing cube pile to remain readable inside the
    // demo host's center viewport.
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(7.0, 7.0, 8.0),
        ::osg::Vec3d(0.0, 0.0, 1.0),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.keyActions.push_back(KeyAction{'q', "Spawn cube", [world, state] {
                                           spawnCube(world, *state);
                                         }});
    setup.keyActions.push_back(
        KeyAction{'w', "Delete last skeleton", [world] {
                    if (world->getNumSkeletons() > 1)
                      world->removeSkeleton(
                          world->getSkeleton(world->getNumSkeletons() - 1));
                  }});

    setup.renderPanel = [world] {
      const std::size_t numCubes
          = world->getNumSkeletons() > 0 ? world->getNumSkeletons() - 1 : 0;
      ImGui::Text("Spawned cubes: %zu", numCubes);
      ImGui::TextWrapped(
          "'q' spawns a random cube above the ground; 'w' deletes the most "
          "recently added skeleton.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
