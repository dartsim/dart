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
#include <dart/gui/vsg/Materials.hpp>

#include <gtest/gtest.h>

namespace vsg = dart::gui::vsg;

TEST(VsgDebugDraw, CreatePoint)
{
  auto node
      = vsg::createPoint(Eigen::Vector3d(1, 2, 3), 0.05, vsg::colors::Red);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreatePoints)
{
  std::vector<Eigen::Vector3d> positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  auto node = vsg::createPoints(positions, 0.03, vsg::colors::Green);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateLine)
{
  auto node = vsg::createLine(
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1), vsg::colors::Blue);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateLines)
{
  std::vector<Eigen::Vector3d> starts = {{0, 0, 0}, {1, 0, 0}};
  std::vector<Eigen::Vector3d> ends = {{0, 1, 0}, {1, 1, 0}};
  auto node = vsg::createLines(starts, ends, vsg::colors::White);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateArrow)
{
  auto node = vsg::createArrow(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, 0, 1),
      0.5,
      vsg::colors::Yellow);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateArrows)
{
  std::vector<Eigen::Vector3d> starts = {{0, 0, 0}, {1, 0, 0}};
  std::vector<Eigen::Vector3d> directions = {{0, 0, 1}, {0, 1, 0}};
  auto node = vsg::createArrows(starts, directions, 0.3, vsg::colors::Cyan);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateAxes)
{
  auto node = vsg::createAxes(2.0, 0.05);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateGrid)
{
  auto node = vsg::createGrid(10.0, 1.0, vsg::colors::Gray);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateWireframeBox)
{
  auto node = vsg::createWireframeBox(
      Eigen::Vector3d(-1, -1, -1),
      Eigen::Vector3d(1, 1, 1),
      vsg::colors::Orange);
  ASSERT_NE(node, nullptr);
}
