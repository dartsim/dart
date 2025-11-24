/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "scenes/basic_dynamics/BasicDynamicsScene.hpp"

#include <dart/gui/osg/IncludeImGui.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Inertia.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <array>
#include <string>

namespace dart::demo {

namespace {

Eigen::Isometry3d makeTransform(const Eigen::Vector3d& position)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  return tf;
}

} // namespace

BasicDynamicsScene::BasicDynamicsScene()
{
  mMetadata.key = "basic_dynamics";
  mMetadata.name = "Basic dynamics";
  mMetadata.description
      = "Stacked rigid bodies with a flat ground plane. Spawn new shapes to "
        "see contact, friction, and restitution in action.";
  mMetadata.tags = {"dynamics", "contacts"};
  mMetadata.eye = osg::Vec3d(5.5, 3.0, 2.5);
  mMetadata.center = osg::Vec3d(0.0, 0.0, 0.6);

  mWorld = dart::simulation::World::create("basic_dynamics_world");
  mWorld->setGravity(Eigen::Vector3d(0, 0, -9.81));
  mWorld->setTimeStep(0.001);

  addGround();
  addInitialStack();

  mNode = new HookedWorldNode(mWorld, [this](double dt) { this->step(dt); });
}

const SceneMetadata& BasicDynamicsScene::metadata() const
{
  return mMetadata;
}

dart::simulation::WorldPtr BasicDynamicsScene::world() const
{
  return mWorld;
}

osg::ref_ptr<dart::gui::osg::RealTimeWorldNode> BasicDynamicsScene::node()
{
  return mNode;
}

void BasicDynamicsScene::reset()
{
  if (!mWorld)
    return;

  mWorld->reset();
  applyInitialTransforms();
}

void BasicDynamicsScene::step(double /*dt*/)
{
  // Scene is controller-free; nothing to do between steps.
}

void BasicDynamicsScene::renderImGui()
{
  ImGui::Text("Objects: %zu", mBodies.size());
  ImGui::SameLine();
  ImGui::Text("Sim time: %.2f s", mWorld ? mWorld->getTime() : 0.0);

  if (ImGui::Button("Spawn shape"))
    spawnObject();

  ImGui::SameLine();
  if (ImGui::Button("Reset layout"))
    reset();

  ImGui::Spacing();
  ImGui::TextUnformatted(
      "Use the controls above to drop more shapes and explore stacking "
      "stability. The reset button restores the original arrangement.");
}

void BasicDynamicsScene::addGround()
{
  using namespace dart::dynamics;

  auto ground = Skeleton::create("ground");
  auto pair = ground->createJointAndBodyNodePair<WeldJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.05);
  joint->setTransformFromParentBodyNode(tf);

  const Eigen::Vector3d size(10.0, 10.0, 0.1);
  auto shape = std::make_shared<BoxShape>(size);
  auto node = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  node->getVisualAspect()->setColor(Eigen::Vector3d(0.7, 0.7, 0.7));

  mWorld->addSkeleton(ground);
}

void BasicDynamicsScene::addInitialStack()
{
  const std::array<Eigen::Vector3d, 3> colors
      = {Eigen::Vector3d(0.8, 0.3, 0.3),
         Eigen::Vector3d(0.2, 0.6, 0.9),
         Eigen::Vector3d(0.3, 0.8, 0.4)};

  auto boxA = createBox(
      "box_a",
      Eigen::Vector3d(0.0, 0.0, 0.6),
      Eigen::Vector3d(0.5, 0.5, 0.5),
      colors[0],
      1.5);
  auto sphereB = createSphere(
      "sphere_b", Eigen::Vector3d(0.5, 0.1, 1.2), 0.25, colors[1], 0.8);
  auto boxC = createBox(
      "box_c",
      Eigen::Vector3d(-0.4, -0.2, 1.6),
      Eigen::Vector3d(0.35, 0.35, 0.35),
      colors[2],
      1.0);

  mBodies.push_back(boxA);
  mBodies.push_back(sphereB);
  mBodies.push_back(boxC);

  mInitialTransforms.push_back(makeTransform(Eigen::Vector3d(0.0, 0.0, 0.6)));
  mInitialTransforms.push_back(makeTransform(Eigen::Vector3d(0.5, 0.1, 1.2)));
  mInitialTransforms.push_back(makeTransform(Eigen::Vector3d(-0.4, -0.2, 1.6)));

  mWorld->addSkeleton(boxA);
  mWorld->addSkeleton(sphereB);
  mWorld->addSkeleton(boxC);
}

void BasicDynamicsScene::applyInitialTransforms()
{
  using dart::dynamics::FreeJoint;

  for (std::size_t i = 0; i < mBodies.size(); ++i) {
    if (!mBodies[i] || i >= mInitialTransforms.size())
      continue;

    FreeJoint::setTransformOf(
        mBodies[i].get(),
        mInitialTransforms[i],
        dart::dynamics::Frame::World());
    mBodies[i]->setVelocities(Eigen::VectorXd::Zero(mBodies[i]->getNumDofs()));
  }
}

dart::dynamics::SkeletonPtr BasicDynamicsScene::createBox(
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& color,
    double mass)
{
  using namespace dart::dynamics;

  auto skeleton = Skeleton::create(name);
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;

  joint->setTransformFromParentBodyNode(makeTransform(position));

  auto shape = std::make_shared<BoxShape>(size);
  auto node = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  node->getVisualAspect()->setColor(color);

  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);

  return skeleton;
}

dart::dynamics::SkeletonPtr BasicDynamicsScene::createSphere(
    const std::string& name,
    const Eigen::Vector3d& position,
    double radius,
    const Eigen::Vector3d& color,
    double mass)
{
  using namespace dart::dynamics;

  auto skeleton = Skeleton::create(name);
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;

  joint->setTransformFromParentBodyNode(makeTransform(position));

  auto shape = std::make_shared<SphereShape>(radius);
  auto node = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  node->getVisualAspect()->setColor(color);

  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);

  return skeleton;
}

void BasicDynamicsScene::spawnObject()
{
  const std::array<Eigen::Vector3d, 3> colors
      = {Eigen::Vector3d(0.9, 0.5, 0.2),
         Eigen::Vector3d(0.4, 0.7, 0.9),
         Eigen::Vector3d(0.5, 0.9, 0.5)};

  const Eigen::Vector3d position(
      -0.4 + 0.2 * static_cast<double>(mSpawnCount % 5),
      -0.2 + 0.15 * static_cast<double>((mSpawnCount / 2) % 3),
      1.4 + 0.25 * static_cast<double>(mSpawnCount % 4));

  const bool spawnSphere = (mSpawnCount % 2) != 0;
  dart::dynamics::SkeletonPtr body;
  Eigen::Isometry3d tf = makeTransform(position);

  if (spawnSphere) {
    body = createSphere(
        "spawned_sphere_" + std::to_string(mSpawnCount),
        position,
        0.22,
        colors[mSpawnCount % colors.size()],
        0.8);
  } else {
    body = createBox(
        "spawned_box_" + std::to_string(mSpawnCount),
        position,
        Eigen::Vector3d(0.3, 0.3, 0.3),
        colors[mSpawnCount % colors.size()],
        1.0);
  }

  if (!body)
    return;

  mWorld->addSkeleton(body);
  mBodies.push_back(body);
  mInitialTransforms.push_back(tf);
  ++mSpawnCount;
}

std::shared_ptr<Scene> createBasicDynamicsScene()
{
  return std::make_shared<BasicDynamicsScene>();
}

} // namespace dart::demo
