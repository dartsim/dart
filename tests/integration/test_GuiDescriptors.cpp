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

#include <dart/gui/DebugOverlay.hpp>
#include <dart/gui/GridVisual.hpp>
#include <dart/gui/SupportPolygonVisual.hpp>
#include <dart/gui/detail/debug.hpp>
#include <dart/gui/detail/geometry.hpp>
#include <dart/gui/detail/overlay_adapter.hpp>
#include <dart/gui/detail/renderable.hpp>
#include <dart/gui/detail/viewer_types.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/ConeShape.hpp>
#include <dart/dynamics/ConvexMeshShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/EndEffector.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/HeightmapShape.hpp>
#include <dart/dynamics/LineSegmentShape.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/MultiSphereConvexHullShape.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/PointCloudShape.hpp>
#include <dart/dynamics/PointMass.hpp>
#include <dart/dynamics/PyramidShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>
#include <dart/dynamics/SoftMeshShape.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/VoxelGridShape.hpp>

#include <dart/common/Deprecated.hpp>

#include <assimp/material.h>
#include <assimp/scene.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace {
using namespace dart;
using namespace dart::gui;

std::shared_ptr<const aiScene> makeGuiMeshScene(bool secondHasColors = true)
{
  auto scene = std::make_shared<aiScene>();
  scene->mNumMaterials = 2;
  scene->mMaterials = new aiMaterial* [2] {
    new aiMaterial(), new aiMaterial()
  };
  aiColor4D first(0.2f, 0.4f, 0.6f, 0.25f);
  aiColor4D second(0.8f, 0.6f, 0.4f, 0.75f);
  scene->mMaterials[0]->AddProperty(&first, 1, AI_MATKEY_COLOR_DIFFUSE);
  scene->mMaterials[1]->AddProperty(&second, 1, AI_MATKEY_COLOR_DIFFUSE);
  aiString texture("textures/diffuse.png");
  scene->mMaterials[0]->AddProperty(&texture, AI_MATKEY_TEXTURE_DIFFUSE(0));
  scene->mNumMeshes = 2;
  scene->mMeshes = new aiMesh*[2];
  for (unsigned int i = 0; i < 2; ++i) {
    auto* mesh = scene->mMeshes[i] = new aiMesh();
    mesh->mMaterialIndex = i;
    mesh->mNumVertices = 3;
    mesh->mVertices = new aiVector3D[3]{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    mesh->mNormals = new aiVector3D[3]{{0, 0, 1}, {0, 0, 1}, {0, 0, 1}};
    mesh->mNumFaces = 1;
    mesh->mFaces = new aiFace[1];
    mesh->mFaces[0].mNumIndices = 3;
    mesh->mFaces[0].mIndices = new unsigned int[3]{0, 1, 2};
    mesh->mTextureCoords[0]
        = new aiVector3D[3]{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    mesh->mNumUVComponents[0] = 2;
    mesh->mColors[0]
        = new aiColor4D[3]{{1, 0, 0, 0.2f}, {0, 1, 0, 0.3f}, {0, 0, 1, 0.4f}};
    mesh->mColors[2]
        = new aiColor4D[3]{{0, 1, 1, 0.5f}, {1, 0, 1, 0.6f}, {1, 1, 0, 0.7f}};
  }
  if (!secondHasColors) {
    delete[] scene->mMeshes[1]->mColors[0];
    delete[] scene->mMeshes[1]->mColors[2];
    scene->mMeshes[1]->mColors[0] = nullptr;
    scene->mMeshes[1]->mColors[2] = nullptr;
  }
  scene->mRootNode = new aiNode();
  scene->mRootNode->mTransformation.a4 = 1.0f;
  scene->mRootNode->mNumMeshes = 1;
  scene->mRootNode->mMeshes = new unsigned int[1]{0};
  scene->mRootNode->mNumChildren = 1;
  scene->mRootNode->mChildren = new aiNode* [1] {
    new aiNode()
  };
  auto* child = scene->mRootNode->mChildren[0];
  child->mParent = scene->mRootNode;
  child->mTransformation.b4 = 2.0f;
  child->mNumMeshes = 1;
  child->mMeshes = new unsigned int[1]{1};
  return scene;
}

std::shared_ptr<dynamics::MeshShape> makeGuiMeshShape()
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  auto shape = std::shared_ptr<dynamics::MeshShape>(new dynamics::MeshShape(
      Eigen::Vector3d(2, 3, 4),
      makeGuiMeshScene(),
      "/tmp/gui-model/model.obj"));
  DART_SUPPRESS_DEPRECATED_END
  return shape;
}
} // namespace

TEST(GuiDescriptors, SupportedPrimitiveDispatch)
{
  using namespace dart::dynamics;
  const std::vector<std::pair<ShapePtr, ShapeKind>> fixtures{
      {std::make_shared<BoxShape>(Eigen::Vector3d(1, 2, 3)), ShapeKind::Box},
      {std::make_shared<SphereShape>(0.5), ShapeKind::Sphere},
      {std::make_shared<EllipsoidShape>(Eigen::Vector3d(1, 2, 3)),
       ShapeKind::Ellipsoid},
      {std::make_shared<CylinderShape>(0.5, 2), ShapeKind::Cylinder},
      {std::make_shared<CapsuleShape>(0.5, 2), ShapeKind::Capsule},
      {std::make_shared<ConeShape>(0.5, 2), ShapeKind::Cone},
      {std::make_shared<PyramidShape>(1, 2, 3), ShapeKind::Pyramid},
      {std::make_shared<MultiSphereConvexHullShape>(
           MultiSphereConvexHullShape::Spheres{
               {0.5, Eigen::Vector3d(1, 0, 0)}}),
       ShapeKind::MultiSphere},
      {std::make_shared<LineSegmentShape>(
           Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 2.0f),
       ShapeKind::LineSegments},
      {std::make_shared<PlaneShape>(Eigen::Vector3d::UnitY(), 2.0),
       ShapeKind::Plane}};
  for (const auto& [shape, expected] : fixtures) {
    SCOPED_TRACE(shape->getType());
    const auto geometry = describeShape(*shape);
    ASSERT_TRUE(geometry);
    EXPECT_EQ(geometry->kind, expected);
    EXPECT_TRUE(geometry->hasLocalBounds);
    EXPECT_TRUE(geometry->localBoundsMin.allFinite());
    EXPECT_TRUE(geometry->localBoundsMax.allFinite());
    EXPECT_TRUE(geometry->unsupportedReason.empty());
  }
  const auto plane = describeShape(*fixtures.back().first);
  EXPECT_DOUBLE_EQ(plane->offset, 2.0);
  EXPECT_EQ(plane->normal, Eigen::Vector3d::UnitY());
}

TEST(GuiDescriptors, ConvexHeightmapAndPointCloudDispatch)
{
  using namespace dart::dynamics;
  ConvexMeshShape convex(
      ConvexMeshShape::Vertices{
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::UnitX(),
          Eigen::Vector3d::UnitY()},
      ConvexMeshShape::Triangles{
          ConvexMeshShape::TriMeshType::Triangle(0, 1, 2)});
  const auto convexGeometry = describeShape(convex);
  ASSERT_TRUE(convexGeometry);
  EXPECT_EQ(convexGeometry->kind, ShapeKind::ConvexMesh);
  EXPECT_EQ(convexGeometry->triangleVertices.size(), 3u);
  EXPECT_EQ(convexGeometry->triangleIndices.size(), 1u);

  HeightmapShapef heightmap;
  HeightmapShapef::HeightField heights(2, 3);
  heights << 0, 1, 2, 3, 4, 5;
  heightmap.setHeightField(heights);
  heightmap.setScale(Eigen::Vector3f(2, 3, 4));
  const auto grid = describeShape(heightmap);
  ASSERT_TRUE(grid);
  EXPECT_EQ(grid->kind, ShapeKind::Heightmap);
  EXPECT_EQ(grid->triangleVertices.size(), 6u);
  EXPECT_EQ(grid->triangleIndices.size(), 4u);
  ASSERT_EQ(grid->triangleNormals.size(), 6u);
  const Eigen::Vector3d expectedNormal = Eigen::Vector3d(-2, 4, 1).normalized();
  for (const auto& normal : grid->triangleNormals)
    EXPECT_TRUE(normal.isApprox(expectedNormal, 1e-12));
  EXPECT_EQ(grid->triangleVertices.front(), Eigen::Vector3d::Zero());
  EXPECT_EQ(grid->triangleVertices.back(), Eigen::Vector3d(4, -3, 20));
  EXPECT_EQ(grid->localBoundsMin, Eigen::Vector3d(0, -3, 0));
  EXPECT_EQ(grid->localBoundsMax, Eigen::Vector3d(4, 0, 20));

  heightmap.setScale(Eigen::Vector3f(2, 3, 4) * 1e-4f);
  const auto smallGrid = describeShape(heightmap);
  ASSERT_TRUE(smallGrid);
  ASSERT_EQ(smallGrid->triangleNormals.size(), 6u);
  for (const auto& normal : smallGrid->triangleNormals) {
    EXPECT_TRUE(normal.allFinite());
    EXPECT_TRUE(normal.isApprox(expectedNormal, 1e-6));
    EXPECT_NEAR(normal.norm(), 1.0, 1e-12);
  }

  PointCloudShape cloud(0.2);
  cloud.addPoint(Eigen::Vector3d(1, 2, 3));
  cloud.setOverallColor(Eigen::Vector4d(0.2, 0.4, 0.6, 0.8));
  cloud.setColorMode(PointCloudShape::BIND_OVERALL);
  for (int mode = PointCloudShape::BOX; mode <= PointCloudShape::POINT;
       ++mode) {
    cloud.setPointShapeType(static_cast<PointCloudShape::PointShapeType>(mode));
    const auto geometry = describeShape(cloud);
    ASSERT_TRUE(geometry);
    EXPECT_EQ(geometry->kind, ShapeKind::PointCloud);
    EXPECT_EQ(static_cast<int>(geometry->pointShapeType), mode);
    EXPECT_EQ(geometry->pointCloudPoints.size(), 1u);
    ASSERT_EQ(geometry->pointCloudColors.size(), 1u);
    EXPECT_EQ(
        geometry->pointCloudColors.front(),
        Eigen::Vector4d(0.2, 0.4, 0.6, 0.8));
  }
}

TEST(GuiDescriptors, HeightmapNormalsUseCardinalNeighbors)
{
  dart::dynamics::HeightmapShapef heightmap;
  auto heights
      = dart::dynamics::HeightmapShapef::HeightField::Zero(3, 3).eval();
  heights(0, 2) = 1;
  heightmap.setHeightField(heights);
  const auto grid = describeShape(heightmap);
  ASSERT_TRUE(grid);
  ASSERT_EQ(grid->triangleVertices.size(), 9u);
  ASSERT_EQ(grid->triangleIndices.size(), 8u);
  ASSERT_EQ(grid->triangleNormals.size(), 9u);
  EXPECT_TRUE(grid->triangleNormals[4].isApprox(Eigen::Vector3d::UnitZ()));
  for (const auto& normal : grid->triangleNormals) {
    EXPECT_TRUE(normal.allFinite());
    EXPECT_NEAR(normal.norm(), 1.0, 1e-12);
  }
}

TEST(GuiDescriptors, LegacySpecularSanityRetainsValidAndPbrCoefficients)
{
  for (const aiColor4D& source :
       {aiColor4D(1, 1, 1, 1),
        aiColor4D(2, 1.5, 1.2, 2),
        aiColor4D(1, .999, 1, 1),
        aiColor4D(1, 1, 1, .5),
        aiColor4D(.08, .002, .002, 1),
        aiColor4D(.4, .2, .1, 0)}) {
    const auto scene = makeGuiMeshScene();
    ASSERT_EQ(
        scene->mMaterials[0]->AddProperty(&source, 1, AI_MATKEY_COLOR_SPECULAR),
        AI_SUCCESS);
    DART_SUPPRESS_DEPRECATED_BEGIN
    dynamics::MeshShape shape(Eigen::Vector3d::Ones(), scene);
    DART_SUPPRESS_DEPRECATED_END
    const auto geometry = describeShape(shape);
    ASSERT_TRUE(geometry);
    const auto& material = geometry->meshMaterials[0];
    const bool rejected
        = source.r >= 1 && source.g >= 1 && source.b >= 1 && source.a >= 1;
    const Eigen::Vector4d expected
        = rejected ? Eigen::Vector4d(0, 0, 0, 1)
                   : Eigen::Vector4d(source.r, source.g, source.b, source.a);
    EXPECT_TRUE(material.specular.isApprox(expected, 1e-6));
    EXPECT_NEAR(material.diffuse.w(), .25, 1e-6);
  }
  for (const unsigned pbr : {0u, 1u, 2u}) {
    SCOPED_TRACE(pbr);
    const auto scene = makeGuiMeshScene();
    const aiColor4D white(1, 1, 1, 1);
    ASSERT_EQ(
        scene->mMaterials[0]->AddProperty(&white, 1, AI_MATKEY_COLOR_SPECULAR),
        AI_SUCCESS);
    const float factor = .5;
    if (pbr == 0)
      ASSERT_EQ(
          scene->mMaterials[0]->AddProperty(
              &factor, 1, AI_MATKEY_METALLIC_FACTOR),
          AI_SUCCESS);
    else if (pbr == 1)
      ASSERT_EQ(
          scene->mMaterials[0]->AddProperty(
              &factor, 1, AI_MATKEY_ROUGHNESS_FACTOR),
          AI_SUCCESS);
    else {
      const aiString texture("metallic.png");
      ASSERT_EQ(
          scene->mMaterials[0]->AddProperty(
              &texture, AI_MATKEY_TEXTURE(aiTextureType_METALNESS, 0)),
          AI_SUCCESS);
    }
    DART_SUPPRESS_DEPRECATED_BEGIN
    dynamics::MeshShape shape(Eigen::Vector3d::Ones(), scene);
    DART_SUPPRESS_DEPRECATED_END
    const auto geometry = describeShape(shape);
    ASSERT_TRUE(geometry);
    EXPECT_EQ(geometry->meshMaterials[0].specular, Eigen::Vector4d::Ones());
  }
}

TEST(GuiDescriptors, AssimpHierarchyUvMaterialsColorsAndAlphaModes)
{
  auto shape = makeGuiMeshShape();
  auto geometry = describeShape(*shape);
  ASSERT_TRUE(geometry);
  EXPECT_EQ(geometry->kind, ShapeKind::Mesh);
  ASSERT_EQ(geometry->triangleVertices.size(), 6u);
  EXPECT_EQ(geometry->triangleVertices[0], Eigen::Vector3d(2, 0, 0));
  EXPECT_EQ(geometry->triangleVertices[3], Eigen::Vector3d(2, 6, 0));
  EXPECT_EQ(geometry->triangleVertices[5], Eigen::Vector3d(2, 9, 0));
  EXPECT_EQ(geometry->localBoundsMin, Eigen::Vector3d(2, 0, 0));
  EXPECT_EQ(geometry->localBoundsMax, Eigen::Vector3d(4, 9, 0));
  ASSERT_EQ(geometry->meshParts.size(), 2u);
  EXPECT_EQ(geometry->meshParts[1].vertexOffset, 3u);
  EXPECT_EQ(geometry->meshParts[1].triangleOffset, 1u);
  EXPECT_EQ(geometry->meshParts[1].materialIndex, 1u);
  ASSERT_EQ(geometry->meshMaterials.size(), 2u);
  EXPECT_NEAR(geometry->meshMaterials[0].diffuse.w(), 0.25, 1e-6);
  // Windows joins texture paths with its preferred separator.
  EXPECT_EQ(
      std::filesystem::path(geometry->meshMaterials[0].baseColorTexturePath)
          .generic_string(),
      "/tmp/gui-model/textures/diffuse.png");
  ASSERT_EQ(geometry->meshTextureCoordinates.size(), 6u);
  EXPECT_EQ(geometry->meshTextureCoordinates[4], Eigen::Vector3d::UnitX());
  EXPECT_TRUE(geometry->meshUsesMaterialColors);

  shape->setColorMode(dart::dynamics::MeshShape::COLOR_INDEX);
  shape->setColorIndex(3);
  geometry = describeShape(*shape);
  ASSERT_TRUE(geometry->meshUsesVertexColors);
  EXPECT_FALSE(geometry->meshUsesMaterialColors);
  EXPECT_TRUE(
      geometry->meshVertexColors[0].isApprox(Eigen::Vector4d(0, 1, 1, 0.5)));
  shape->setColorIndex(1);
  geometry = describeShape(*shape);
  EXPECT_TRUE(geometry->meshVertexColors[0].isApprox(
      Eigen::Vector4d(1, 0, 0, 0.2), 1e-6));
  shape->setAlphaMode(dart::dynamics::MeshShape::AUTO);
  EXPECT_EQ(describeShape(*shape)->meshAlphaMode, MeshAlphaMode::Auto);
  shape->setAlphaMode(dart::dynamics::MeshShape::SHAPE_ALPHA);
  EXPECT_EQ(describeShape(*shape)->meshAlphaMode, MeshAlphaMode::ShapeAlpha);
  shape->setColorMode(dart::dynamics::MeshShape::SHAPE_COLOR);
  geometry = describeShape(*shape);
  EXPECT_FALSE(geometry->meshUsesMaterialColors);
  EXPECT_FALSE(geometry->meshUsesVertexColors);
}

TEST(GuiDescriptors, MissingVertexColorChannelsUsePerPartFallback)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  auto mesh = std::shared_ptr<dart::dynamics::MeshShape>(
      new dart::dynamics::MeshShape(
          Eigen::Vector3d::Ones(), makeGuiMeshScene(false)));
  DART_SUPPRESS_DEPRECATED_END
  mesh->setColorMode(dart::dynamics::MeshShape::COLOR_INDEX);
  const auto geometry = describeShape(*mesh);
  ASSERT_TRUE(geometry);
  ASSERT_EQ(geometry->meshParts.size(), 2u);
  EXPECT_TRUE(geometry->meshUsesVertexColors);
  EXPECT_TRUE(geometry->meshParts[0].usesVertexColors);
  EXPECT_FALSE(geometry->meshParts[1].usesVertexColors);
}

TEST(GuiDescriptors, WorldTraversalAndLiveCacheInvalidation)
{
  using namespace dart::dynamics;
  dart::simulation::World world("gui");
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = SimpleFrame::createShared(parent.get(), "child");
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  child->setShape(box);
  child->createVisualAspect();
  world.addSimpleFrame(parent);
  world.addSimpleFrame(child);
  RenderableExtractor extractor;
  auto before = extractor.extract(world);
  ASSERT_EQ(before.size(), 1u);
  EXPECT_EQ(before[0].shapeFrame, child.get());
  auto transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(3, 4, 5);
  parent->setRelativeTransform(transform);
  const auto moved = extractor.extract(world);
  ASSERT_EQ(moved.size(), 1u);
  EXPECT_EQ(moved[0].worldTransform.translation(), Eigen::Vector3d(3, 4, 5));
  EXPECT_EQ(moved[0].renderResourceVersion, before[0].renderResourceVersion);
  box->setSize(Eigen::Vector3d(2, 3, 4));
  const auto resized = extractor.extract(world);
  EXPECT_EQ(resized[0].geometry.size, Eigen::Vector3d(2, 3, 4));
  EXPECT_NE(resized[0].renderResourceVersion, before[0].renderResourceVersion);
  child->getVisualAspect()->setAlpha(0.5);
  const auto alpha = extractor.extract(world);
  EXPECT_DOUBLE_EQ(alpha[0].material.rgba.w(), 0.5);
  EXPECT_NE(alpha[0].renderResourceVersion, resized[0].renderResourceVersion);
  child->getVisualAspect()->hide();
  EXPECT_FALSE(extractor.extract(world)[0].material.visible);
  child->setShape(nullptr);
  EXPECT_TRUE(extractor.extract(world).empty());
  child->setShape(makeGuiMeshShape());
  auto mesh = std::dynamic_pointer_cast<MeshShape>(child->getShape());
  const auto material = extractor.extract(world);
  mesh->setColorMode(MeshShape::COLOR_INDEX);
  mesh->setColorIndex(2);
  const auto vertex = extractor.extract(world);
  EXPECT_TRUE(vertex[0].geometry.meshUsesVertexColors);
  EXPECT_NE(material[0].renderResourceVersion, vertex[0].renderResourceVersion);
  mesh->setAlphaMode(MeshShape::SHAPE_ALPHA);
  EXPECT_EQ(
      extractor.extract(world)[0].geometry.meshAlphaMode,
      MeshAlphaMode::ShapeAlpha);
  world.removeSimpleFrame(parent);
  world.removeSimpleFrame(child);
  EXPECT_TRUE(extractor.extract(world).empty());
}

TEST(GuiDescriptors, PersistentGeometryBuffersAndMultipleWorlds)
{
  using namespace dart::dynamics;
  auto firstWorld = dart::simulation::World::create();
  auto secondWorld = dart::simulation::World::create();
  auto frame = SimpleFrame::createShared(Frame::World(), "static_mesh");
  auto mesh = makeGuiMeshShape();
  frame->setShape(mesh);
  frame->createVisualAspect();
  firstWorld->addSimpleFrame(frame);
  auto other = SimpleFrame::createShared(Frame::World(), "other");
  other->setShape(std::make_shared<SphereShape>(0.3));
  other->createVisualAspect();
  secondWorld->addSimpleFrame(other);
  std::vector<std::shared_ptr<dart::simulation::World>> worlds{
      firstWorld, secondWorld};
  RenderableExtractor extractor;
  const auto& before = extractor.extract(worlds);
  ASSERT_EQ(before.size(), 2u);
  const auto* vertices = before[0].geometry.triangleVertices.data();
  const auto* indices = before[0].geometry.triangleIndices.data();
  const auto version = before[0].renderResourceVersion;
  frame->setTranslation(Eigen::Vector3d(1, 2, 3));
  const auto& after = extractor.extract(worlds);
  EXPECT_EQ(after[0].geometry.triangleVertices.data(), vertices);
  EXPECT_EQ(after[0].geometry.triangleIndices.data(), indices);
  EXPECT_EQ(after[0].renderResourceVersion, version);
  EXPECT_EQ(after[0].worldTransform.translation(), Eigen::Vector3d(1, 2, 3));
  mesh->setScale(Eigen::Vector3d(4, 3, 2));
  const auto& changed = extractor.extract(worlds);
  EXPECT_NE(changed[0].renderResourceVersion, version);
  EXPECT_EQ(
      changed[0].geometry.triangleVertices.front(), Eigen::Vector3d(4, 0, 0));
  worlds.erase(worlds.begin());
  const auto& remaining = extractor.extract(worlds);
  ASSERT_EQ(remaining.size(), 1u);
  EXPECT_EQ(remaining[0].shapeFrame, other.get());
  worlds.clear();
  EXPECT_TRUE(extractor.extract(worlds).empty());
}

TEST(GuiDescriptors, LineEditsRebuildResourcesWithoutShapeVersionChange)
{
  using namespace dart::dynamics;
  dart::simulation::World world;
  auto frame = SimpleFrame::createShared(Frame::World(), "line");
  auto line = std::make_shared<LineSegmentShape>(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  frame->setShape(line);
  frame->createVisualAspect();
  world.addSimpleFrame(frame);
  RenderableExtractor extractor;
  const auto before = extractor.extract(world);
  const auto version = line->getVersion();
  line->setVertex(1, Eigen::Vector3d::UnitY());
  line->setThickness(3.0f);
  EXPECT_EQ(line->getVersion(), version);
  const auto after = extractor.extract(world);
  ASSERT_EQ(after.size(), 1u);
  EXPECT_EQ(after[0].geometry.lineVertices[1], Eigen::Vector3d::UnitY());
  EXPECT_DOUBLE_EQ(after[0].geometry.lineThickness, 3.0);
  EXPECT_NE(before[0].renderResourceVersion, after[0].renderResourceVersion);
}

TEST(GuiDescriptors, DynamicTriangleGeometryEditsWithoutShapeVersionChange)
{
  using namespace dart::dynamics;
  dart::simulation::World world;
  auto frame = SimpleFrame::createShared(Frame::World(), "dynamic_mesh");
  auto heightmap = std::make_shared<HeightmapShapef>();
  heightmap->setHeightField(HeightmapShapef::HeightField::Zero(2, 2));
  heightmap->setDataVariance(Shape::STATIC);
  frame->setShape(heightmap);
  frame->createVisualAspect();
  world.addSimpleFrame(frame);
  RenderableExtractor extractor;
  const auto before = extractor.extract(world);
  ASSERT_EQ(before.size(), 1u);
  ASSERT_EQ(before[0].geometry.kind, ShapeKind::Heightmap);
  ASSERT_EQ(before[0].geometry.triangleVertices.size(), 4u);
  ASSERT_EQ(before[0].geometry.triangleNormals.size(), 4u);
  const auto heightmapVersion = heightmap->getVersion();
  heightmap->getHeightFieldModifiable()(0, 0) = 2.0;
  const auto cached = extractor.extract(world);
  EXPECT_EQ(cached[0].renderResourceVersion, before[0].renderResourceVersion);
  EXPECT_EQ(cached[0].geometry.triangleVertices[0], Eigen::Vector3d::Zero());

  heightmap->setDataVariance(Shape::DYNAMIC_VERTICES);
  const auto heights = extractor.extract(world);
  ASSERT_EQ(heights.size(), 1u);
  EXPECT_EQ(heightmap->getVersion(), heightmapVersion);
  EXPECT_NE(heights[0].renderResourceVersion, before[0].renderResourceVersion);
  EXPECT_EQ(heights[0].geometry.triangleVertices[0], Eigen::Vector3d(0, 0, 2));
  EXPECT_DOUBLE_EQ(heights[0].geometry.localBoundsMax.z(), 2.0);
  EXPECT_FALSE(heights[0].geometry.triangleNormals[0].isApprox(
      before[0].geometry.triangleNormals[0]));
  EXPECT_EQ(
      extractor.extract(world)[0].renderResourceVersion,
      heights[0].renderResourceVersion);

  auto convex = std::make_shared<ConvexMeshShape>(
      ConvexMeshShape::Vertices{
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::UnitX(),
          Eigen::Vector3d::UnitY()},
      ConvexMeshShape::Triangles{
          ConvexMeshShape::TriMeshType::Triangle(0, 1, 2)});
  convex->setDataVariance(Shape::DYNAMIC_VERTICES | Shape::DYNAMIC_ELEMENTS);
  convex->getMesh()->computeVertexNormals();
  frame->setShape(convex);
  const auto meshBefore = extractor.extract(world);
  ASSERT_EQ(meshBefore.size(), 1u);
  ASSERT_EQ(meshBefore[0].geometry.triangleVertices.size(), 3u);
  ASSERT_EQ(meshBefore[0].geometry.triangleNormals.size(), 3u);
  const auto convexVersion = convex->getVersion();
  convex->getMesh()->getVertices()[0].z() = 2.0;
  const auto vertices = extractor.extract(world);
  EXPECT_EQ(convex->getVersion(), convexVersion);
  EXPECT_NE(
      vertices[0].renderResourceVersion, meshBefore[0].renderResourceVersion);
  EXPECT_EQ(vertices[0].geometry.triangleVertices[0], Eigen::Vector3d(0, 0, 2));
  EXPECT_DOUBLE_EQ(vertices[0].geometry.localBoundsMax.z(), 2.0);

  convex->getMesh()->addTriangle(0, 1, 2);
  const auto indices = extractor.extract(world);
  EXPECT_EQ(convex->getVersion(), convexVersion);
  EXPECT_NE(
      indices[0].renderResourceVersion, vertices[0].renderResourceVersion);
  ASSERT_EQ(indices[0].geometry.triangleIndices.size(), 2u);
  EXPECT_EQ(indices[0].geometry.triangleIndices[1], Eigen::Vector3i(0, 1, 2));

  convex->getMesh()->getVertexNormals()[0] = Eigen::Vector3d::UnitX();
  const auto normals = extractor.extract(world);
  EXPECT_EQ(convex->getVersion(), convexVersion);
  EXPECT_NE(normals[0].renderResourceVersion, indices[0].renderResourceVersion);
  EXPECT_EQ(normals[0].geometry.triangleNormals[0], Eigen::Vector3d::UnitX());
}

TEST(GuiDescriptors, SkeletonAndSoftGeometryRemainLive)
{
  using namespace dart::dynamics;
  auto skeleton = Skeleton::create("soft");
  const auto softProperties = SoftBodyNodeHelper::makeBoxProperties(
      Eigen::Vector3d::Ones(), Eigen::Isometry3d::Identity(), 1.0);
  const SoftBodyNode::Properties properties(
      BodyNode::Properties(), softProperties);
  auto* body = skeleton
                   ->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
                       nullptr, FreeJoint::Properties(), properties)
                   .second;
  auto softMesh = std::make_shared<SoftMeshShape>(body);
  auto* node = body->createShapeNodeWith<VisualAspect>(softMesh);
  dart::simulation::World world;
  world.addSkeleton(skeleton);
  RenderableExtractor extractor;
  const auto before = extractor.extract(world);
  ASSERT_FALSE(before.empty());
  const auto first = describeShapeFrame(*node);
  ASSERT_TRUE(first);
  EXPECT_EQ(first->geometry.kind, ShapeKind::SoftMesh);
  ASSERT_FALSE(first->geometry.triangleVertices.empty());
  const auto original = first->geometry.triangleVertices.front();
  body->getPointMass(0)->setPositions(Eigen::Vector3d(0.2, 0.3, 0.4));
  const auto second = describeShapeFrame(*node);
  ASSERT_TRUE(second);
  EXPECT_TRUE(second->geometry.triangleVertices.front().isApprox(
      original + Eigen::Vector3d(0.2, 0.3, 0.4)));
  EXPECT_NE(first->renderResourceVersion, second->renderResourceVersion);
  EXPECT_FALSE(extractor.extract(world).empty());
}

#if HAVE_OCTOMAP
TEST(GuiDescriptors, VoxelOccupancyAndLeafSizes)
{
  dart::dynamics::VoxelGridShape voxels(0.25);
  voxels.updateOccupancy(Eigen::Vector3d(0.1, 0.1, 0.1));
  auto geometry = describeShape(voxels);
  ASSERT_TRUE(geometry);
  EXPECT_EQ(geometry->kind, ShapeKind::VoxelGrid);
  ASSERT_EQ(geometry->voxelCenters.size(), 1u);
  ASSERT_EQ(geometry->voxelSizes.size(), 1u);
  EXPECT_DOUBLE_EQ(geometry->voxelSizes[0], 0.25);
  EXPECT_TRUE(
      geometry->localBoundsMax.isApprox(Eigen::Vector3d::Constant(0.25)));
}
#endif

TEST(GuiDescriptors, ResourcePlanRetainsChangesVisibilityAndDuplicates)
{
  RenderableDescriptor stable;
  stable.id = 1;
  stable.renderResourceVersion = 10;
  RenderableDescriptor changed = stable;
  changed.id = 2;
  changed.renderResourceVersion = 20;
  RenderableDescriptor hidden = stable;
  hidden.id = 3;
  hidden.material.visible = false;
  const std::vector<RenderableDescriptor> descriptors{
      stable, changed, changed, hidden};
  const std::vector<ActiveRenderableState> active{
      {1, 0, 10}, {1, 0, 10}, {2, 0, 15}, {3, 0, 10}, {4, 0, 0}};
  const auto plan = planRenderableSetUpdate(descriptors, active);
  EXPECT_EQ(
      plan.activeRenderableIndicesToRemove,
      (std::vector<std::size_t>{1, 2, 3, 4}));
  EXPECT_EQ(plan.descriptorIndicesToAdd, (std::vector<std::size_t>{1}));
}

TEST(GuiDescriptors, PrimitiveMeshesAndCameraMathAreFinite)
{
  const std::vector<MeshGeometry> meshes{
      makeBoxMeshGeometry(Eigen::Vector3d(1, 2, 3)),
      makeEllipsoidMeshGeometry(Eigen::Vector3d(1, 2, 3)),
      makeCylinderMeshGeometry(0.5, 2),
      makeCapsuleMeshGeometry(0.5, 2),
      makeConeMeshGeometry(0.5, 2),
      makePyramidMeshGeometry(Eigen::Vector3d(1, 2, 3)),
      makeMultiSphereMeshGeometry({Eigen::Vector3d::Zero()}, {0.5})};
  for (const auto& mesh : meshes) {
    ASSERT_FALSE(mesh.vertices.empty());
    EXPECT_TRUE(mesh.hasBounds);
    for (const auto index : mesh.indices)
      EXPECT_LT(index, mesh.vertices.size());
    for (const auto& vertex : mesh.vertices) {
      EXPECT_TRUE(vertex.position.allFinite());
      EXPECT_TRUE(vertex.normal.allFinite());
      EXPECT_NEAR(vertex.normal.norm(), 1.0, 1e-5);
    }
  }
  const auto camera = fitOrbitCameraToViewport(
      BoundingSphere{Eigen::Vector3d::Zero(), 1.0, true},
      200,
      800,
      45,
      -45,
      25);
  const auto projected = projectToPixels(camera, 200, 800, camera.target);
  EXPECT_NEAR(projected.x(), 100, 1e-6);
  EXPECT_NEAR(projected.y(), 400, 1e-6);
  const auto ray = makePerspectivePickRay(camera, 100, 400, 200, 800);
  EXPECT_TRUE(
      ray.direction.isApprox((camera.target - cameraEye(camera)).normalized()));
  EXPECT_FALSE(makeGridDebugLines().empty());
}

TEST(GuiDescriptors, OverlayStylesWorldAndScreenLabelsAndGridPlanes)
{
  auto overlay = std::make_shared<DebugOverlay>();
  overlay->setLineWidth(5);
  overlay->setFont("/tmp/fixture-font.ttf");
  overlay->setCharacterSize(0.2);
  overlay->setTextOutlineWidth(2);
  overlay->setTextOutlineColor(Eigen::Vector4d(0.1, 0.2, 0.3, 1));
  EXPECT_EQ(
      overlay->addLine(
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::UnitX(),
          Eigen::Vector4d(1, 0, 0, 0.5)),
      0u);
  overlay->addLabel(Eigen::Vector3d::UnitZ(), "world", Eigen::Vector4d::Ones());
  overlay->addScreenLabel(
      Eigen::Vector2d(30, 40), "screen", Eigen::Vector4d::Ones(), 18);
  overlay->addLabelPixels(
      Eigen::Vector3d::UnitY(), "world pixels", Eigen::Vector4d::Ones(), 22);
  auto grid = std::make_shared<GridVisual>();
  grid->setNumCells(2);
  grid->setMinorLineStepSize(0.5);
  grid->setNumMinorLinesPerMajorLine(2);
  grid->setPlaneType(GridVisual::PlaneType::ZX);
  grid->setOffset(Eigen::Vector3d(1, 2, 3));
  grid->setMajorLineWidth(4);
  grid->setMinorLineWidth(1);
  grid->refresh();
  DebugScene scene;
  detail::collectDebugScene({overlay, grid}, scene);
  ASSERT_EQ(scene.lines.size(), 13u);
  EXPECT_TRUE(scene.lines.front().alwaysOnTop);
  EXPECT_DOUBLE_EQ(scene.lines.front().pixelWidth, 5);
  EXPECT_TRUE(scene.lines[1].from.isApprox(grid->getOffset()));
  EXPECT_TRUE(scene.lines[1].to.isApprox(Eigen::Vector3d(1, 2, 4)));
  for (std::size_t i = 1; i < scene.lines.size(); ++i) {
    EXPECT_FALSE(scene.lines[i].alwaysOnTop);
    EXPECT_DOUBLE_EQ(scene.lines[i].from.y(), 2);
    EXPECT_DOUBLE_EQ(scene.lines[i].to.y(), 2);
  }
  ASSERT_EQ(scene.labels.size(), 3u);
  EXPECT_EQ(scene.labels[0].fontPath, "/tmp/fixture-font.ttf");
  EXPECT_FALSE(scene.labels[0].screenSpace);
  EXPECT_DOUBLE_EQ(scene.labels[0].characterSize, 0.2);
  EXPECT_DOUBLE_EQ(scene.labels[0].outlineWidth, 2);
  EXPECT_TRUE(
      scene.labels[0].outlineColor.isApprox(overlay->getTextOutlineColor()));
  EXPECT_TRUE(scene.labels[1].screenSpace);
  EXPECT_TRUE(
      scene.labels[1].position.head<2>().isApprox(Eigen::Vector2d(30, 40)));
  EXPECT_DOUBLE_EQ(scene.labels[1].characterSize, 18);
  EXPECT_TRUE(scene.labels[2].pixelSize);
  EXPECT_FALSE(scene.labels[2].screenSpace);
  EXPECT_DOUBLE_EQ(scene.labels[2].characterSize, 22);
  grid->display(false);
  overlay->clear();
  detail::collectDebugScene({overlay, grid}, scene);
  EXPECT_TRUE(scene.lines.empty());
  EXPECT_TRUE(scene.labels.empty());
}

struct EmptyFontFile
{
  std::filesystem::path path;
  EmptyFontFile()
  {
    static std::atomic<unsigned> sequence{0};
    const auto now
        = std::chrono::steady_clock::now().time_since_epoch().count();
    path = std::filesystem::temp_directory_path()
           / ("dart-empty-overlay-font-" + std::to_string(now) + "-"
              + std::to_string(sequence++) + ".ttf");
    std::ofstream font(path, std::ios::binary | std::ios::trunc);
  }
  ~EmptyFontFile()
  {
    std::error_code error;
    std::filesystem::remove(path, error);
  }
};

TEST(GuiDescriptors, OverlayEmptyFontFileThrowsBeforeImGui)
{
  const EmptyFontFile fontFile;
  const auto& fontPath = fontFile.path;
  ASSERT_TRUE(std::filesystem::is_regular_file(fontPath));
  ASSERT_EQ(std::filesystem::file_size(fontPath), 0u);
  auto overlay = std::make_shared<DebugOverlay>();
  overlay->setFont(fontPath.string());
  overlay->addLabel(
      Eigen::Vector3d::UnitZ(), "empty font", Eigen::Vector4d::Ones());
  DebugScene scene;
  detail::collectDebugScene({overlay}, scene);
  detail::OverlayRenderer renderer;
  try {
    renderer.prepareFonts(scene);
    FAIL() << "prepareFonts accepted a zero-byte font file";
  } catch (const std::runtime_error& e) {
    EXPECT_EQ(
        std::string(e.what()),
        "Cannot load overlay font: " + fontPath.string());
  }
}

TEST(GuiDescriptors, DepthTestedPixelWidthLinesPreservePerspectiveWidth)
{
  DebugScene scene;
  DebugLineDescriptor line;
  line.from = {-1, 0, 0};
  line.to = {1, 0, -1};
  line.pixelWidth = 6;
  line.alwaysOnTop = false;
  scene.lines.push_back(line);
  line.alwaysOnTop = true;
  scene.lines.push_back(line);
  const Eigen::Vector3d eye(0, 0, 4), center(0, 0, 0), up(0, 1, 0);
  detail::prepareDebugScene(scene, eye, center, up, 60, 800, 600);
  ASSERT_EQ(scene.lines.size(), 1u);
  EXPECT_TRUE(scene.lines.front().alwaysOnTop);
  ASSERT_EQ(scene.triangles.size(), 2u);
  const auto& first = scene.triangles.front();
  EXPECT_FALSE(first.alwaysOnTop);
  const double focal = 300 / std::tan(3.14159265358979323846 / 6);
  auto pixels = [&](const Eigen::Vector3d& p) -> Eigen::Vector2d {
    return {
        400 + p.x() * focal / (4 - p.z()), 300 - p.y() * focal / (4 - p.z())};
  };
  EXPECT_NEAR((pixels(first.a) - pixels(first.b)).norm(), 6, 1e-9);
  EXPECT_NEAR((pixels(scene.triangles[1].c) - pixels(first.c)).norm(), 6, 1e-9);
  for (const auto& triangle : scene.triangles) {
    EXPECT_TRUE(triangle.a.allFinite());
    EXPECT_TRUE(triangle.b.allFinite());
    EXPECT_TRUE(triangle.c.allFinite());
  }
  scene = {};
  line.alwaysOnTop = false;
  line.from = {0, 0, 5};
  line.to = {0, 1, 6};
  scene.lines.push_back(line);
  detail::prepareDebugScene(scene, eye, center, up, 60, 800, 600);
  EXPECT_TRUE(scene.lines.empty());
  EXPECT_TRUE(scene.triangles.empty());
}

TEST(GuiDescriptors, SupportPolygonTracksSkeletonAndTreeLifetime)
{
  auto skeleton = dynamics::Skeleton::create();
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  skeleton->setGravity(Eigen::Vector3d(0, 0, -9.81));
  auto* support = body->createEndEffector()->getSupport(true);
  support->setGeometry(
      {Eigen::Vector3d(-1, -1, 0),
       Eigen::Vector3d(1, -1, 0),
       Eigen::Vector3d(1, 1, 0),
       Eigen::Vector3d(-1, 1, 0)});
  support->setActive(true);
  auto visual = std::make_shared<SupportPolygonVisual>(skeleton, 0u, 0.03);
  visual->displayCentroid(false);
  visual->displayCenterOfMass(false);
  visual->refresh();
  DebugScene scene;
  detail::collectDebugScene({visual}, scene);
  ASSERT_EQ(scene.triangles.size(), 2u);
  for (const auto& triangle : scene.triangles) {
    EXPECT_FALSE(triangle.alwaysOnTop);
    EXPECT_NEAR(triangle.a.z(), 0.03, 1e-9);
    EXPECT_TRUE(triangle.rgba.isApprox(visual->getPolygonColor()));
  }
  visual->displayPolygon(false);
  visual->displayCenterOfMass(true);
  visual->refresh();
  scene = {};
  detail::collectDebugScene({visual}, scene);
  ASSERT_FALSE(scene.triangles.empty());
  EXPECT_TRUE(
      scene.triangles.front().rgba.isApprox(visual->getValidCOMColor()));
  visual->visualizeTree(100);
  visual->refresh();
  scene = {};
  detail::collectDebugScene({visual}, scene);
  EXPECT_TRUE(scene.triangles.empty());
  visual->visualizeWholeSkeleton();
  skeleton.reset();
  visual->refresh();
  scene = {};
  detail::collectDebugScene({visual}, scene);
  EXPECT_EQ(visual->getSkeleton(), nullptr);
  EXPECT_TRUE(scene.triangles.empty());
}

TEST(GuiDescriptors, ResourcePlanDetectsFirstExplicitResourceKey)
{
  RenderableDescriptor descriptor;
  descriptor.id = 1;
  descriptor.shapeVersion = 0;
  descriptor.renderResourceVersion = 1;
  const auto plan = planRenderableSetUpdate(
      {descriptor}, std::vector<ActiveRenderableState>{{1, 0, 0}});
  EXPECT_EQ(
      plan.activeRenderableIndicesToRemove, (std::vector<std::size_t>{0}));
  EXPECT_EQ(plan.descriptorIndicesToAdd, (std::vector<std::size_t>{0}));
  descriptor.renderResourceVersion = 0;
  const auto legacy = planRenderableSetUpdate(
      {descriptor}, std::vector<ActiveRenderableState>{{1, 0, 0}});
  EXPECT_TRUE(legacy.activeRenderableIndicesToRemove.empty());
  EXPECT_TRUE(legacy.descriptorIndicesToAdd.empty());
}

TEST(GuiDescriptors, CapsuleFacesMatchOutwardAuthoredNormals)
{
  for (const double height : {0.0, 1.0}) {
    const auto mesh = makeCapsuleMeshGeometry(.5, height);
    std::size_t checked = 0;
    for (const auto& face : mesh.triangles) {
      const auto& a = mesh.vertices[face.a];
      const auto& b = mesh.vertices[face.b];
      const auto& c = mesh.vertices[face.c];
      const Eigen::Vector3f normal
          = (b.position - a.position).cross(c.position - a.position);
      if (normal.squaredNorm() < 1e-12f)
        continue; // The latitude grid duplicates the two pole positions.
      EXPECT_GT(normal.dot(a.normal + b.normal + c.normal), 0.0f);
      ++checked;
    }
    EXPECT_GT(checked, 500u);
  }
}
