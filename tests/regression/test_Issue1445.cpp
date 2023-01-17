/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/test/io/TestHelpers.hpp>

#include <dart/dart.hpp>

#include <gtest/gtest.h>

// This test is adapted from @azeey's work here:
// https://github.com/ignitionrobotics/ign-physics/pull/31
TEST(Issue1445, Collision)
{
  std::string model1Str = R"(
  <sdf version="1.6">
    <model name="M1">
      <pose>0 0 0.1 0 0 0</pose>
      <link name="body">
        <collision name="coll_box">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </sdf>)";

  std::string model2Str = R"(
  <sdf version="1.6">
    <model name="M2">
      <pose>1 0 0.1 0 0 0</pose>
      <link name="chassis">
        <collision name="coll_sphere">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
        </collision>
      </link>
    </model>
  </sdf>)";

  auto world = std::make_shared<dart::simulation::World>();

  auto ground = dart::dynamics::Skeleton::create("ground");
  {
    auto* bn = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>()
                   .second;
    bn->createShapeNodeWith<
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(
        std::make_shared<dart::dynamics::BoxShape>(
            math::Vector3d(10, 10, 0.2)));
    auto sn = bn->getShapeNode(0);
    ASSERT_TRUE(sn);
    sn->setRelativeTranslation({0, 0, -0.1});
    world->addSkeleton(ground);
  }

  auto model1 = dart::dynamics::Skeleton::create("M1");
  {
    auto pair = model1->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
    auto* joint = pair.first;
    auto* bn = pair.second;
    joint->setName("joint1");
    bn->setName("body1");

    bn->createShapeNodeWith<
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(
        std::make_shared<dart::dynamics::BoxShape>(
            math::Vector3d(0.2, 0.2, 0.2)));

    auto tf = math::Isometry3d::Identity();
    tf.translation()[2] = 0.1;
    joint->setTransform(tf);

    world->addSkeleton(model1);
  }

  world->step();

  auto model2 = dart::dynamics::Skeleton::create("M2");
  {
    auto pair = model2->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
    auto* joint = pair.first;
    auto* bn = pair.second;
    joint->setName("joint2");
    bn->setName("body2");

    bn->createShapeNodeWith<
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(
        std::make_shared<dart::dynamics::SphereShape>(0.1));

    auto tf = math::Isometry3d::Identity();
    tf.translation() = math::Vector3d(1.0, 0.0, 0.1);
    joint->setTransform(tf);
  }

  auto* model1Body = model1->getRootBodyNode();
  auto* model2Body = model2->getRootBodyNode();

  const auto poseParent = model1Body->getTransform();
  const auto poseChild = model2Body->getTransform();

  // Commenting out the following `step` call makes this test fail
  // world->Step(output, state, input);
  auto fixedJoint = model2Body->moveTo<dart::dynamics::WeldJoint>(model1Body);

  // Pose of child relative to parent
  auto poseParentChild = poseParent.inverse() * poseChild;

  // We let the joint be at the origin of the child link.
  fixedJoint->setTransformFromParentBodyNode(poseParentChild);

  const std::size_t numSteps = 100;

  for (std::size_t i = 0; i < numSteps; ++i)
    world->step();

  // Expect both bodies to hit the ground and stop
  EXPECT_NEAR(0.0, model1Body->getLinearVelocity().z(), 1e-3);
  EXPECT_NEAR(0.0, model2Body->getLinearVelocity().z(), 1e-3);

  auto temp1 = dart::dynamics::Skeleton::create("temp1");
  world->addSkeleton(temp1);
  model2Body->moveTo<dart::dynamics::FreeJoint>(temp1, nullptr);

  for (std::size_t i = 0; i < numSteps; ++i)
    world->step();

  // Expect both bodies to remain in contact with the ground with zero velocity.
  EXPECT_NEAR(0.0, model1Body->getLinearVelocity().z(), 1e-3);
  EXPECT_NEAR(0.0, model2Body->getLinearVelocity().z(), 1e-3);

  auto* groundBody = ground->getRootBodyNode();
  auto temp2 = groundBody->remove();

  for (std::size_t i = 0; i < numSteps; ++i)
    world->step();

  // Expect both bodies to be falling after the BodyNode ofthe the ground is
  // removed
  EXPECT_LE(model1Body->getLinearVelocity().z(), -1e-2);
  EXPECT_LE(model2Body->getLinearVelocity().z(), -1e-2);
}
