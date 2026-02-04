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

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;

TEST(ShapeFrameTest, SetAndGetShape)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "test_frame");
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1, 2, 3));

  frame->setShape(box);
  EXPECT_EQ(frame->getShape(), box);

  auto newBox = std::make_shared<BoxShape>(Eigen::Vector3d(4, 5, 6));
  frame->setShape(newBox);
  EXPECT_EQ(frame->getShape(), newBox);
}

TEST(ShapeFrameTest, AsShapeFrame)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "sf_test");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));

  ShapeFrame* sf = frame->asShapeFrame();
  EXPECT_NE(sf, nullptr);
  EXPECT_EQ(sf, frame.get());

  const ShapeFrame* constSf
      = static_cast<const SimpleFrame*>(frame.get())->asShapeFrame();
  EXPECT_NE(constSf, nullptr);
}

TEST(ShapeFrameTest, IsShapeNode)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "node_test");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));

  EXPECT_FALSE(frame->isShapeNode());
}

TEST(VisualAspectTest, SetAndGetRGBA)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "rgba_test");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->createVisualAspect();

  auto* visual = frame->getVisualAspect();
  ASSERT_NE(visual, nullptr);

  Eigen::Vector4d color(1.0, 0.5, 0.25, 0.8);
  visual->setRGBA(color);

  Eigen::Vector4d retrieved = visual->getRGBA();
  EXPECT_TRUE(retrieved.isApprox(color));
}

TEST(VisualAspectTest, SetAndGetColor)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "color_test");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->createVisualAspect();

  auto* visual = frame->getVisualAspect();
  ASSERT_NE(visual, nullptr);

  Eigen::Vector3d rgb(0.2, 0.4, 0.6);
  visual->setColor(rgb);

  Eigen::Vector3d retrievedRgb = visual->getColor();
  EXPECT_TRUE(retrievedRgb.isApprox(rgb));
}

TEST(VisualAspectTest, SetAndGetRGB)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "rgb_test");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->createVisualAspect();

  auto* visual = frame->getVisualAspect();
  ASSERT_NE(visual, nullptr);

  Eigen::Vector3d rgb(0.1, 0.2, 0.3);
  visual->setRGB(rgb);

  Eigen::Vector3d retrievedRgb = visual->getRGB();
  EXPECT_TRUE(retrievedRgb.isApprox(rgb));
}

TEST(VisualAspectTest, SetAndGetAlpha)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "alpha_test");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->createVisualAspect();

  auto* visual = frame->getVisualAspect();
  ASSERT_NE(visual, nullptr);

  visual->setAlpha(0.5);
  EXPECT_DOUBLE_EQ(visual->getAlpha(), 0.5);

  visual->setAlpha(1.0);
  EXPECT_DOUBLE_EQ(visual->getAlpha(), 1.0);
}

TEST(VisualAspectTest, HideAndShow)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "hide_test");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->createVisualAspect();

  auto* visual = frame->getVisualAspect();
  ASSERT_NE(visual, nullptr);

  EXPECT_FALSE(visual->isHidden());

  visual->hide();
  EXPECT_TRUE(visual->isHidden());

  visual->show();
  EXPECT_FALSE(visual->isHidden());
}

TEST(VisualAspectTest, ResetColorAndUsesDefaultColor)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "reset_test");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->createVisualAspect();

  auto* visual = frame->getVisualAspect();
  ASSERT_NE(visual, nullptr);

  EXPECT_TRUE(visual->usesDefaultColor());

  visual->setRGBA(Eigen::Vector4d(1.0, 0.0, 0.0, 1.0));
  EXPECT_FALSE(visual->usesDefaultColor());

  visual->resetColor();
  EXPECT_TRUE(visual->usesDefaultColor());
}

TEST(VisualAspectTest, GetDefaultRGBA)
{
  Eigen::Vector4d defaultColor = VisualAspect::getDefaultRGBA();
  EXPECT_TRUE(defaultColor.allFinite());
  EXPECT_GE(defaultColor[3], 0.0);
  EXPECT_LE(defaultColor[3], 1.0);
}

TEST(CollisionAspectTest, IsCollidable)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "coll_test");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->createCollisionAspect();

  auto* collision = frame->getCollisionAspect();
  ASSERT_NE(collision, nullptr);

  EXPECT_TRUE(collision->isCollidable());

  collision->setCollidable(false);
  EXPECT_FALSE(collision->isCollidable());

  collision->setCollidable(true);
  EXPECT_TRUE(collision->isCollidable());
}

TEST(DynamicsAspectTest, SetAndGetFrictionCoeff)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "friction_test");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->createDynamicsAspect();

  auto* dynamics = frame->getDynamicsAspect();
  ASSERT_NE(dynamics, nullptr);

  dynamics->setFrictionCoeff(0.5);
  EXPECT_NEAR(dynamics->getFrictionCoeff(), 0.5, 1e-10);
}

TEST(DynamicsAspectTest, SetAndGetPrimaryFrictionCoeff)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "prim_friction_test");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->createDynamicsAspect();

  auto* dynamics = frame->getDynamicsAspect();
  ASSERT_NE(dynamics, nullptr);

  dynamics->setPrimaryFrictionCoeff(0.8);
  EXPECT_DOUBLE_EQ(dynamics->getPrimaryFrictionCoeff(), 0.8);
}

TEST(DynamicsAspectTest, SetFirstFrictionDirectionFrame)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "dir_frame_test");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->createDynamicsAspect();

  auto* dynamics = frame->getDynamicsAspect();
  ASSERT_NE(dynamics, nullptr);

  dynamics->setFirstFrictionDirectionFrame(nullptr);
  EXPECT_EQ(dynamics->getFirstFrictionDirectionFrame(), nullptr);

  dynamics->setFirstFrictionDirectionFrame(Frame::World());
  EXPECT_EQ(dynamics->getFirstFrictionDirectionFrame(), Frame::World());
}

TEST(DynamicsAspectTest, SetAndGetRestitutionCoeff)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "restitution_test");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->createDynamicsAspect();

  auto* dynamics = frame->getDynamicsAspect();
  ASSERT_NE(dynamics, nullptr);

  dynamics->setRestitutionCoeff(0.7);
  EXPECT_DOUBLE_EQ(dynamics->getRestitutionCoeff(), 0.7);
}
