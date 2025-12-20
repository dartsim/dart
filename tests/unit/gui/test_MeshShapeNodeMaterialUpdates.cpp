/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/gui/ShapeFrameNode.hpp>
#include <dart/gui/render/MeshShapeNode.hpp>

#include <dart/dynamics/ArrowShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>

#include <gtest/gtest.h>
#include <osg/Material>
#include <osg/ref_ptr>

TEST(MeshShapeNodeTest, UpdatesMaterialOnColorChange)
{
  const Eigen::Vector4d initialColor(0.1, 0.2, 0.3, 0.4);
  auto arrow = std::make_shared<dart::dynamics::ArrowShape>(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitX(),
      dart::dynamics::ArrowShape::Properties(),
      initialColor);
  arrow->setDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);

  dart::dynamics::SimpleFrame frame;
  frame.setShape(arrow);
  frame.getVisualAspect(true)->setRGBA(initialColor);

  osg::ref_ptr<dart::gui::ShapeFrameNode> frameNode
      = new dart::gui::ShapeFrameNode(&frame, nullptr);
  ASSERT_GT(frameNode->getNumChildren(), 0u);
  auto* meshNode
      = dynamic_cast<dart::gui::render::MeshShapeNode*>(frameNode->getChild(0));
  ASSERT_NE(meshNode, nullptr);

  const auto* material = meshNode->getMaterial(0);
  ASSERT_NE(material, nullptr);
  osg::Vec4 diffuse = material->getDiffuse(osg::Material::FRONT_AND_BACK);
  EXPECT_FLOAT_EQ(diffuse.r(), static_cast<float>(initialColor[0]));
  EXPECT_FLOAT_EQ(diffuse.g(), static_cast<float>(initialColor[1]));
  EXPECT_FLOAT_EQ(diffuse.b(), static_cast<float>(initialColor[2]));
  EXPECT_FLOAT_EQ(diffuse.a(), static_cast<float>(initialColor[3]));

  const Eigen::Vector4d updatedColor(0.8, 0.7, 0.6, 0.5);
  frame.getVisualAspect()->setRGBA(updatedColor);
  meshNode->refresh();

  material = meshNode->getMaterial(0);
  ASSERT_NE(material, nullptr);
  diffuse = material->getDiffuse(osg::Material::FRONT_AND_BACK);
  EXPECT_FLOAT_EQ(diffuse.r(), static_cast<float>(updatedColor[0]));
  EXPECT_FLOAT_EQ(diffuse.g(), static_cast<float>(updatedColor[1]));
  EXPECT_FLOAT_EQ(diffuse.b(), static_cast<float>(updatedColor[2]));
  EXPECT_FLOAT_EQ(diffuse.a(), static_cast<float>(updatedColor[3]));
}
