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

#include <dart/gui/render/convex_mesh_shape_node.hpp>
#include <dart/gui/shape_frame_node.hpp>

#include <dart/dynamics/convex_mesh_shape.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <gtest/gtest.h>
#include <osg/ref_ptr>

TEST(ConvexMeshShapeNodeTest, CreatesRenderNode)
{
  dart::dynamics::ConvexMeshShape::Vertices vertices
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitZ()};
  dart::dynamics::ConvexMeshShape::Triangles triangles
      = {dart::dynamics::ConvexMeshShape::TriMeshType::Triangle(0, 1, 2),
         dart::dynamics::ConvexMeshShape::TriMeshType::Triangle(0, 1, 3),
         dart::dynamics::ConvexMeshShape::TriMeshType::Triangle(0, 2, 3),
         dart::dynamics::ConvexMeshShape::TriMeshType::Triangle(1, 2, 3)};

  auto shape
      = std::make_shared<dart::dynamics::ConvexMeshShape>(vertices, triangles);

  dart::dynamics::SimpleFrame frame;
  frame.setShape(shape);
  frame.getVisualAspect(true);

  osg::ref_ptr<dart::gui::ShapeFrameNode> frameNode
      = new dart::gui::ShapeFrameNode(&frame, nullptr);
  ASSERT_GT(frameNode->getNumChildren(), 0u);
  auto* convexNode = dynamic_cast<dart::gui::render::ConvexMeshShapeNode*>(
      frameNode->getChild(0));
  EXPECT_NE(convexNode, nullptr);
}
