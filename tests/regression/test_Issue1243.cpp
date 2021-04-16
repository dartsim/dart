/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include <TestHelpers.hpp>
#include <gtest/gtest.h>

#include <dart/dart.hpp>

dart::dynamics::SkeletonPtr create_box(
    const Eigen::Vector3d& dims,
    double mass,
    const Eigen::Vector4d& color,
    const std::string& box_name)
{
  dart::dynamics::SkeletonPtr box_skel
      = dart::dynamics::Skeleton::create(box_name);

  // Give the box a body
  dart::dynamics::BodyNodePtr body;
  body
      = box_skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(nullptr)
            .second;
  body->setName(box_name);

  // Give the body a shape
  auto box = std::make_shared<dart::dynamics::BoxShape>(dims);
  auto box_node = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(box);
  box_node->getVisualAspect()->setColor(color);
  // Set up inertia
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(box->computeInertia(mass));
  body->setInertia(inertia);

  return box_skel;
}

dart::dynamics::SkeletonPtr create_floor()
{
  double floor_width = 10.;
  double floor_height = 0.1;

  dart::dynamics::SkeletonPtr floor_skel
      = dart::dynamics::Skeleton::create("floor");

  // Give the floor a body
  dart::dynamics::BodyNodePtr body
      = floor_skel
            ->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr)
            .second;

  // Give the body a shape
  auto box = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(floor_width, floor_width, floor_height));
  auto box_node = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(box);
  box_node->getVisualAspect()->setColor(dart::Color::Gray());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation()[2] -= floor_height / 2.0;
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor_skel;
}

//==============================================================================
TEST(Issue1243, State)
{
  auto world = std::make_shared<dart::simulation::World>();

  auto box_skel = create_box({0.1, 0.1, 0.1}, 1., {1., 0., 0., 1.}, "box");
  auto floor_skel = create_floor();

  box_skel->setPosition(5, 0.2);

  world->addSkeleton(box_skel);
  world->addSkeleton(floor_skel);

  dart::dynamics::Skeleton::State bookmark_state;
  Eigen::Isometry3d bookmark_tf;
  for (size_t i = 0; i < 20; i++)
  {
    if (i == 10)
    {
      bookmark_state = box_skel->getState();
      bookmark_tf = box_skel->getRootBodyNode()->getTransform();
    }
    world->step();
  }

  const Eigen::Isometry3d final_tf
      = box_skel->getRootBodyNode()->getTransform();

  box_skel->setState(bookmark_state);
  const Eigen::Isometry3d rewind_tf
      = box_skel->getRootBodyNode()->getTransform();

  EXPECT_FALSE(equals(bookmark_tf, final_tf));
  EXPECT_TRUE(equals(bookmark_tf, rewind_tf));
}
