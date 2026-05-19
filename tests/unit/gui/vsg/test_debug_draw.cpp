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

#include <dart/gui/vsg/debug_draw.hpp>
#include <dart/gui/vsg/materials.hpp>

#include <gtest/gtest.h>

namespace dart_vsg = dart::gui::vsg;

TEST(VsgDebugDraw, CreatePoint)
{
  auto node = dart_vsg::createPoint(
      Eigen::Vector3d(1, 2, 3), 0.05, dart_vsg::colors::Red);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreatePoints)
{
  std::vector<Eigen::Vector3d> positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  auto node = dart_vsg::createPoints(positions, 0.03, dart_vsg::colors::Green);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateLine)
{
  auto node = dart_vsg::createLine(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(1, 1, 1),
      dart_vsg::colors::Blue);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateLines)
{
  std::vector<Eigen::Vector3d> starts = {{0, 0, 0}, {1, 0, 0}};
  std::vector<Eigen::Vector3d> ends = {{0, 1, 0}, {1, 1, 0}};
  auto node = dart_vsg::createLines(starts, ends, dart_vsg::colors::White);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateArrow)
{
  auto node = dart_vsg::createArrow(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, 0, 1),
      0.5,
      dart_vsg::colors::Yellow);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateArrows)
{
  std::vector<Eigen::Vector3d> starts = {{0, 0, 0}, {1, 0, 0}};
  std::vector<Eigen::Vector3d> directions = {{0, 0, 1}, {0, 1, 0}};
  auto node
      = dart_vsg::createArrows(starts, directions, 0.3, dart_vsg::colors::Cyan);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateAxes)
{
  auto node = dart_vsg::createAxes(2.0, 0.05);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateGrid)
{
  auto node = dart_vsg::createGrid(10.0, 1.0, dart_vsg::colors::Gray);
  ASSERT_NE(node, nullptr);
}

TEST(VsgDebugDraw, CreateWireframeBox)
{
  auto node = dart_vsg::createWireframeBox(
      Eigen::Vector3d(-1, -1, -1),
      Eigen::Vector3d(1, 1, 1),
      dart_vsg::colors::Orange);
  ASSERT_NE(node, nullptr);
}
