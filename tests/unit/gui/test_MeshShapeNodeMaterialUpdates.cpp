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

#include <dart/config.hpp>

#include <dart/gui/render/mesh_shape_node.hpp>
#include <dart/gui/shape_frame_node.hpp>

#include <dart/dynamics/arrow_shape.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <dart/common/diagnostics.hpp>
#include <dart/common/local_resource_retriever.hpp>
#include <dart/common/uri.hpp>

#include <gtest/gtest.h>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Material>
#include <osg/PrimitiveSet>
#include <osg/ref_ptr>

#include <string>
#include <vector>

namespace {

void collectGeometries(osg::Node* node, std::vector<osg::Geometry*>& geometries)
{
  if (!node)
    return;

  if (auto* geode = node->asGeode()) {
    const unsigned int numDrawables = geode->getNumDrawables();
    for (unsigned int i = 0; i < numDrawables; ++i) {
      auto* geometry = dynamic_cast<osg::Geometry*>(geode->getDrawable(i));
      if (geometry)
        geometries.push_back(geometry);
    }
  }

  if (auto* group = node->asGroup()) {
    const unsigned int numChildren = group->getNumChildren();
    for (unsigned int i = 0; i < numChildren; ++i)
      collectGeometries(group->getChild(i), geometries);
  }
}

} // namespace

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

TEST(MeshShapeNodeTest, TessellatesConcavePolygons)
{
  const std::string filePath = dart::config::dataPath("obj/Concave.obj");
  const dart::common::Uri fileUri = dart::common::Uri::createFromPath(filePath);
  const std::string fileUriString = fileUri.toString();
  ASSERT_FALSE(fileUriString.empty());

  auto retriever = std::make_shared<dart::common::LocalResourceRetriever>();
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene
      = dart::dynamics::MeshShape::loadMesh(fileUriString, retriever);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);

  DART_SUPPRESS_DEPRECATED_BEGIN
  auto meshShape = std::make_shared<dart::dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), scene, fileUri, retriever);
  DART_SUPPRESS_DEPRECATED_END

  dart::dynamics::SimpleFrame frame;
  frame.setShape(meshShape);
  frame.getVisualAspect(true);

  osg::ref_ptr<dart::gui::ShapeFrameNode> frameNode
      = new dart::gui::ShapeFrameNode(&frame, nullptr);
  ASSERT_GT(frameNode->getNumChildren(), 0u);

  dart::gui::render::MeshShapeNode* meshNode = nullptr;
  for (unsigned int i = 0; i < frameNode->getNumChildren(); ++i) {
    meshNode = dynamic_cast<dart::gui::render::MeshShapeNode*>(
        frameNode->getChild(i));
    if (meshNode)
      break;
  }
  ASSERT_NE(meshNode, nullptr);

  std::vector<osg::Geometry*> geometries;
  collectGeometries(meshNode, geometries);
  ASSERT_FALSE(geometries.empty());

  bool hasPolygons = false;
  bool hasTriangles = false;
  for (const auto* geometry : geometries) {
    const unsigned int primitiveCount = geometry->getNumPrimitiveSets();
    for (unsigned int i = 0; i < primitiveCount; ++i) {
      const osg::PrimitiveSet* primitive = geometry->getPrimitiveSet(i);
      if (!primitive)
        continue;

      const auto mode = primitive->getMode();
      if (mode == osg::PrimitiveSet::POLYGON)
        hasPolygons = true;
      if (mode == osg::PrimitiveSet::TRIANGLES
          || mode == osg::PrimitiveSet::TRIANGLE_FAN
          || mode == osg::PrimitiveSet::TRIANGLE_STRIP)
        hasTriangles = true;
    }
  }

  EXPECT_FALSE(hasPolygons);
  EXPECT_TRUE(hasTriangles);
}
