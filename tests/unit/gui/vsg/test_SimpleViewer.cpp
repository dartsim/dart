/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/gui/vsg/DebugDraw.hpp>
#include <dart/gui/vsg/SimpleViewer.hpp>

#include <gtest/gtest.h>

namespace vsg = dart::gui::vsg;

TEST(VsgSimpleViewer, HeadlessFactory)
{
  auto viewer = vsg::SimpleViewer::headless(640, 480);
  EXPECT_TRUE(viewer.isHeadless());
}

TEST(VsgSimpleViewer, HeadlessTagConstruction)
{
  vsg::SimpleViewer viewer(vsg::SimpleViewer::HeadlessTag{}, 320, 240);
  EXPECT_TRUE(viewer.isHeadless());
}

TEST(VsgSimpleViewer, GetRoot)
{
  auto viewer = vsg::SimpleViewer::headless(640, 480);
  auto root = viewer.getRoot();
  ASSERT_NE(root, nullptr);
}

TEST(VsgSimpleViewer, AddNode)
{
  auto viewer = vsg::SimpleViewer::headless(640, 480);
  auto axes = vsg::createAxes(1.0);

  viewer.addNode(axes);
  SUCCEED();
}

TEST(VsgSimpleViewer, AddGrid)
{
  auto viewer = vsg::SimpleViewer::headless(640, 480);
  viewer.addGrid(5.0, 0.5);
  SUCCEED();
}

TEST(VsgSimpleViewer, AddAxes)
{
  auto viewer = vsg::SimpleViewer::headless(640, 480);
  viewer.addAxes(2.0);
  SUCCEED();
}

TEST(VsgSimpleViewer, LookAt)
{
  auto viewer = vsg::SimpleViewer::headless(640, 480);
  viewer.lookAt(
      Eigen::Vector3d(5.0, 5.0, 5.0),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d::UnitZ());
  SUCCEED();
}

TEST(VsgSimpleViewer, SetBackgroundColor)
{
  auto viewer = vsg::SimpleViewer::headless(640, 480);
  viewer.setBackgroundColor(Eigen::Vector4d(0.1, 0.2, 0.3, 1.0));
  SUCCEED();
}

TEST(VsgSimpleViewer, SetScene)
{
  auto viewer = vsg::SimpleViewer::headless(640, 480);
  auto axes = vsg::createAxes(1.0);
  viewer.setScene(axes);
  SUCCEED();
}

TEST(VsgSimpleViewer, Clear)
{
  auto viewer = vsg::SimpleViewer::headless(640, 480);
  viewer.addAxes(1.0);
  viewer.addGrid(5.0, 1.0);
  viewer.clear();
  SUCCEED();
}
