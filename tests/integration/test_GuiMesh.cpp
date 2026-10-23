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

#include <dart/gui/Viewer.hpp>
#include <dart/gui/WorldNode.hpp>
#include <dart/gui/detail/geometry.hpp>
#include <dart/gui/detail/interaction.hpp>
#include <dart/gui/detail/renderable.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>

#include <dart/common/Deprecated.hpp>

#include <assimp/material.h>
#include <assimp/scene.h>
#include <gtest/gtest.h>
#include <png.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cmath>
#include <cstdlib>

namespace {
using namespace dart;
using namespace dart::gui;

std::shared_ptr<aiScene> meshScene(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::vector<unsigned int>>& faces)
{
  auto scene = std::make_shared<aiScene>();
  scene->mNumMeshes = 1;
  scene->mMeshes = new aiMesh* [1] {
    new aiMesh()
  };
  auto& mesh = *scene->mMeshes[0];
  mesh.mNumVertices = vertices.size();
  mesh.mVertices = new aiVector3D[vertices.size()];
  for (std::size_t i = 0; i < vertices.size(); ++i)
    mesh.mVertices[i]
        = aiVector3D(vertices[i].x(), vertices[i].y(), vertices[i].z());
  mesh.mNumFaces = faces.size();
  mesh.mFaces = new aiFace[faces.size()];
  for (std::size_t i = 0; i < faces.size(); ++i) {
    mesh.mFaces[i].mNumIndices = faces[i].size();
    mesh.mFaces[i].mIndices = new unsigned int[faces[i].size()];
    std::copy(faces[i].begin(), faces[i].end(), mesh.mFaces[i].mIndices);
  }
  scene->mRootNode = new aiNode();
  scene->mRootNode->mNumMeshes = 1;
  scene->mRootNode->mMeshes = new unsigned int[1]{0};
  return scene;
}

std::shared_ptr<dynamics::MeshShape> meshShape(
    const std::shared_ptr<aiScene>& scene)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  auto shape = std::shared_ptr<dynamics::MeshShape>(
      new dynamics::MeshShape(Eigen::Vector3d::Ones(), scene));
  DART_SUPPRESS_DEPRECATED_END
  return shape;
}

RenderableDescriptor descriptor(const dynamics::Shape& shape)
{
  RenderableDescriptor result;
  result.id = 1;
  result.geometry = *describeShape(shape);
  return result;
}

VisualPickOptions viewport()
{
  VisualPickOptions result;
  result.eye = {0, 0, 4};
  result.center = {0, 0, 0};
  result.up = {0, 1, 0};
  result.fovyDegrees = 60;
  result.width = 800;
  result.height = 600;
  return result;
}
PickRay rayFromPixel(const VisualPickOptions& v, double x, double y)
{
  const double tangent = std::tan(v.fovyDegrees * 3.14159265358979323846 / 360);
  const Eigen::Vector3d forward = (v.center - v.eye).normalized();
  const Eigen::Vector3d right = forward.cross(v.up).normalized();
  const Eigen::Vector3d up = right.cross(forward);
  return {
      v.eye,
      (forward + right * ((2 * x / v.width - 1) * tangent * v.width / v.height)
       + up * ((1 - 2 * y / v.height) * tangent))
          .normalized()};
}
constexpr unsigned imageWidth = 320, imageHeight = 240;
const Eigen::Vector4d meshBackground(0.03, 0.05, 0.65, 1);

struct MeshImage
{
  std::vector<unsigned char> pixels;
  explicit MeshImage(const std::filesystem::path& path)
  {
    png_image image{};
    image.version = PNG_IMAGE_VERSION;
    if (!png_image_begin_read_from_file(&image, path.string().c_str()))
      throw std::runtime_error(image.message);
    if (image.width != imageWidth || image.height != imageHeight) {
      png_image_free(&image);
      throw std::runtime_error("Wrong physical mesh capture dimensions");
    }
    image.format = PNG_FORMAT_RGBA;
    pixels.resize(PNG_IMAGE_SIZE(image));
    if (!png_image_finish_read(&image, nullptr, pixels.data(), 0, nullptr)) {
      const std::string error = image.message;
      png_image_free(&image);
      throw std::runtime_error(error);
    }
    png_image_free(&image);
  }

  std::size_t count(
      bool green,
      unsigned x0 = 0,
      unsigned y0 = 0,
      unsigned x1 = imageWidth,
      unsigned y1 = imageHeight) const
  {
    std::size_t result = 0;
    for (unsigned y = y0; y < y1; ++y)
      for (unsigned x = x0; x < x1; ++x) {
        const auto* p = &pixels[(y * imageWidth + x) * 4];
        result += green ? p[1] > p[0] + 30 && p[1] > p[2] + 30
                        : p[0] > p[1] + 30 && p[0] > p[2] + 30;
      }
    return result;
  }
};

MeshImage captureMesh(Viewer& viewer, const std::string& name)
{
  const auto* configured = std::getenv("DART_FILAMENT_PARITY_DIR");
  const auto directory = configured ? std::filesystem::path(configured)
                                    : std::filesystem::temp_directory_path()
                                          / "dart-filament-mesh";
  std::filesystem::create_directories(directory);
  const auto path = directory / ("mesh-" + name + ".png");
  viewer.captureScreen(path.string());
  viewer.frame();
  return MeshImage(path);
}

void setUpMeshViewer(
    Viewer& viewer,
    const std::shared_ptr<simulation::World>& world,
    double distance = 4)
{
  viewer.addWorldNode(std::make_shared<WorldNode>(world));
  viewer.setVerticalFieldOfView(60);
  viewer.setCameraHome({0, 0, distance}, {0, 0, 0}, {0, 1, 0});
  viewer.setUpViewForCapture(imageWidth, imageHeight);
}

std::shared_ptr<dynamics::SimpleFrame> addMeshFrame(
    const std::shared_ptr<simulation::World>& world,
    const std::shared_ptr<dynamics::MeshShape>& shape)
{
  auto frame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), "visual-mesh");
  frame->setShape(shape);
  frame->createVisualAspect()->setColor(Eigen::Vector4d(0.95, 0.02, 0.02, 1));
  world->addSimpleFrame(frame);
  return frame;
}

} // namespace

TEST(GuiMesh, ConcavePolygonPreservesAreaWindingAndPickNotch)
{
  const std::vector<Eigen::Vector3d> vertices{
      {-2, -2, 0},
      {2, -2, 0},
      {2, 2, 0},
      {1, 2, 0},
      {1, -1, 0},
      {-1, -1, 0},
      {-1, 2, 0},
      {-2, 2, 0}};
  std::vector<unsigned int> face{0, 1, 2, 3, 4, 5, 6, 7};
  for (bool reversed : {false, true}) {
    SCOPED_TRACE(reversed);
    if (reversed)
      std::reverse(face.begin(), face.end());
    const auto shape = meshShape(meshScene(vertices, {face}));
    const auto renderable = descriptor(*shape);
    const auto& geometry = renderable.geometry;
    ASSERT_EQ(geometry.triangleIndices.size(), 6u);
    ASSERT_EQ(geometry.meshParts.size(), 1u);
    EXPECT_EQ(geometry.meshParts[0].triangleCount, 6u);
    double area = 0;
    for (const auto& triangle : geometry.triangleIndices) {
      const auto& a = geometry.triangleVertices[triangle.x()];
      const auto& b = geometry.triangleVertices[triangle.y()];
      const auto& c = geometry.triangleVertices[triangle.z()];
      const double signedArea = (b - a).cross(c - a).z() / 2;
      EXPECT_GT(signedArea * (reversed ? -1 : 1), 0);
      area += signedArea;
    }
    EXPECT_NEAR(area, reversed ? -10 : 10, 1e-12);
    EXPECT_FALSE(intersectRenderableHit(renderable, {{0, 1, 3}, {0, 0, -1}}));
    EXPECT_TRUE(intersectRenderableHit(renderable, {{1.5, 1, 3}, {0, 0, -1}}));
    EXPECT_TRUE(intersectRenderableHit(renderable, {{0, -1.5, 3}, {0, 0, -1}}));
  }
}

TEST(GuiMesh, PointAndLineFacesRetainPartsColorsAndOnePixelPicking)
{
  auto scene = meshScene({{-1, 0, 0}, {0, 0, 0}, {1, 0, 0}}, {{0}, {1, 2}});
  scene->mNumMaterials = 1;
  scene->mMaterials = new aiMaterial* [1] {
    new aiMaterial()
  };
  aiColor4D diffuse(0.2f, 0.4f, 0.6f, 0.25f);
  scene->mMaterials[0]->AddProperty(&diffuse, 1, AI_MATKEY_COLOR_DIFFUSE);
  scene->mMeshes[0]->mColors[0]
      = new aiColor4D[3]{{1, 0, 0, 0.2f}, {0, 1, 0, 0.3f}, {0, 0, 1, 0.4f}};
  auto shape = meshShape(scene);
  shape->setColorMode(dynamics::MeshShape::COLOR_INDEX);
  auto renderable = descriptor(*shape);
  const auto& geometry = renderable.geometry;
  EXPECT_TRUE(geometry.unsupportedReason.empty());
  EXPECT_TRUE(geometry.triangleIndices.empty());
  ASSERT_EQ(geometry.meshPointIndices.size(), 1u);
  ASSERT_EQ(geometry.meshLineIndices.size(), 1u);
  EXPECT_EQ(geometry.meshPointIndices[0], 0);
  EXPECT_EQ(geometry.meshLineIndices[0], Eigen::Vector2i(1, 2));
  ASSERT_EQ(geometry.meshParts.size(), 1u);
  const auto& part = geometry.meshParts[0];
  EXPECT_EQ(part.pointCount, 1u);
  EXPECT_EQ(part.lineCount, 1u);
  EXPECT_EQ(part.materialIndex, 0u);
  EXPECT_TRUE(part.usesVertexColors);
  ASSERT_EQ(geometry.meshVertexColors.size(), 3u);
  EXPECT_TRUE(geometry.meshVertexColors[1].isApprox(
      Eigen::Vector4d(0, 1, 0, 0.3), 1e-6));
  EXPECT_TRUE(geometry.meshMaterials[0].diffuse.isApprox(
      Eigen::Vector4d(0.2, 0.4, 0.6, 0.25), 1e-6));
  const auto options = viewport();
  const auto visual = makeMeshPointLineGeometry(
      renderable,
      part,
      options.eye,
      options.center,
      options.up,
      options.fovyDegrees,
      options.height);
  ASSERT_EQ(visual.vertices.size(), 12u);
  EXPECT_EQ(visual.vertices[0].sourceVertex, 0u);
  EXPECT_EQ(visual.vertices[6].sourceVertex, 1u);
  EXPECT_EQ(visual.vertices[7].sourceVertex, 2u);
  EXPECT_EQ(visual.vertices[11].sourceVertex, 1u);
  const double pointX = 400 - 300 / std::tan(3.14159265358979323846 / 6) / 4;
  EXPECT_TRUE(intersectRenderableHit(
      renderable, rayFromPixel(options, pointX, 300), options));
  EXPECT_FALSE(intersectRenderableHit(
      renderable, rayFromPixel(options, pointX + 1, 300), options));
  const double lineX = 400 + 150 / std::tan(3.14159265358979323846 / 6) / 4;
  EXPECT_TRUE(intersectRenderableHit(
      renderable, rayFromPixel(options, lineX, 300.4), options));
  EXPECT_FALSE(intersectRenderableHit(
      renderable, rayFromPixel(options, lineX, 300.6), options));
  EXPECT_FALSE(
      intersectRenderableHit(renderable, rayFromPixel(options, pointX, 300)));
}

TEST(GuiMesh, InvalidPointLineAndPolygonFacesAreIgnored)
{
  const auto shape = meshShape(meshScene(
      {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}},
      {{9}, {0, 9}, {0, 1, 2, 9}, {0, 1, 2}}));
  const auto geometry = describeShape(*shape);
  ASSERT_TRUE(geometry);
  EXPECT_TRUE(geometry->meshPointIndices.empty());
  EXPECT_TRUE(geometry->meshLineIndices.empty());
  ASSERT_EQ(geometry->triangleIndices.size(), 1u);
  EXPECT_EQ(geometry->triangleIndices[0], Eigen::Vector3i(0, 1, 2));
}

TEST(GuiMesh, ExtractorRetainsOwningWorldAcrossRefreshAndRemoval)
{
  auto first = simulation::World::create();
  auto second = simulation::World::create();
  auto a = dynamics::SimpleFrame::createShared(dynamics::Frame::World(), "a");
  auto b = dynamics::SimpleFrame::createShared(dynamics::Frame::World(), "b");
  const auto shape = meshShape(meshScene({{0, 0, 0}}, {{0}}));
  for (const auto& frame : {a, b}) {
    frame->setShape(shape);
    frame->createVisualAspect();
  }
  first->addSimpleFrame(a);
  second->addSimpleFrame(b);
  RenderableExtractor extractor;
  const auto& both = extractor.extract(
      std::vector<std::shared_ptr<simulation::World>>{first, second});
  ASSERT_EQ(both.size(), 2u);
  EXPECT_EQ(both[0].world, first.get());
  EXPECT_EQ(both[1].world, second.get());
  EXPECT_EQ(describeShapeFrame(*a)->world, nullptr);
  a->setTranslation({1, 2, 3});
  const auto& refreshed = extractor.extract(
      std::vector<std::shared_ptr<simulation::World>>{second, first});
  ASSERT_EQ(refreshed.size(), 2u);
  EXPECT_EQ(refreshed[0].world, first.get());
  EXPECT_EQ(refreshed[1].world, second.get());
  second->removeSimpleFrame(b);
  const auto& remaining = extractor.extract(*first);
  ASSERT_EQ(remaining.size(), 1u);
  EXPECT_EQ(remaining[0].world, first.get());
}

TEST(GuiMesh, LegacyGlossAndExplicitPbrPresenceReachDescriptorsAndCache)
{
  auto scene = meshScene({{0, 0, 0}}, {{0}});
  scene->mNumMaterials = 1;
  scene->mMaterials = new aiMaterial* [1] {
    new aiMaterial()
  };
  auto shape = meshShape(scene);
  shape->setDataVariance(dynamics::Shape::DYNAMIC_COLOR);
  auto frame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World(), "gloss");
  frame->setShape(shape);
  frame->createVisualAspect();
  auto world = simulation::World::create();
  world->addSimpleFrame(frame);
  RenderableExtractor extractor;
  const auto absent = extractor.extract(*world)[0];
  EXPECT_FALSE(absent.geometry.meshMaterials[0].hasPbrMetallic);
  EXPECT_FALSE(absent.geometry.meshMaterials[0].hasPbrRoughness);
  EXPECT_DOUBLE_EQ(absent.geometry.meshMaterials[0].shininessStrength, 1);
  float shininess = 80, strength = 0.4f;
  scene->mMaterials[0]->AddProperty(&shininess, 1, AI_MATKEY_SHININESS);
  scene->mMaterials[0]->AddProperty(&strength, 1, AI_MATKEY_SHININESS_STRENGTH);
  const auto gloss = extractor.extract(*world)[0];
  EXPECT_DOUBLE_EQ(gloss.geometry.meshMaterials[0].shininess, 80);
  EXPECT_NEAR(gloss.geometry.meshMaterials[0].shininessStrength, 0.4, 1e-6);
  EXPECT_NE(absent.renderResourceVersion, gloss.renderResourceVersion);
  float metallic = 0, roughness = 1;
  scene->mMaterials[0]->AddProperty(&metallic, 1, AI_MATKEY_METALLIC_FACTOR);
  scene->mMaterials[0]->AddProperty(&roughness, 1, AI_MATKEY_ROUGHNESS_FACTOR);
  const auto explicitPbr = extractor.extract(*world)[0];
  EXPECT_TRUE(explicitPbr.geometry.meshMaterials[0].hasPbrMetallic);
  EXPECT_TRUE(explicitPbr.geometry.meshMaterials[0].hasPbrRoughness);
  EXPECT_NE(gloss.renderResourceVersion, explicitPbr.renderResourceVersion);
}

TEST(GuiMesh, ConcavePolygonNotchReachesPixels)
{
  const auto shape = meshShape(meshScene(
      {{-2, -2, 0},
       {2, -2, 0},
       {2, 2, 0},
       {1, 2, 0},
       {1, -1, 0},
       {-1, -1, 0},
       {-1, 2, 0},
       {-2, 2, 0}},
      {{0, 1, 2, 3, 4, 5, 6, 7}}));
  shape->setColorMode(dynamics::MeshShape::SHAPE_COLOR);
  auto world = simulation::World::create();
  addMeshFrame(world, shape);
  Viewer viewer(meshBackground);
  setUpMeshViewer(viewer, world, 8);
  const auto image = captureMesh(viewer, "concave-u");
  EXPECT_GT(image.count(false), 1000u);
  EXPECT_EQ(image.count(false, 145, 75, 175, 115), 0u);
  EXPECT_GT(image.count(false, 193, 80, 205, 110), 100u);
  EXPECT_GT(image.count(false, 145, 155, 175, 165), 100u);
  EXPECT_DOUBLE_EQ(world->getTime(), 0);
}

TEST(GuiMesh, ProgrammaticPointLineColorsAndCameraZoomReachPixels)
{
  const double scale = imageHeight / (8 * std::tan(3.14159265358979323846 / 6));
  auto scene = meshScene(
      {{-56.5 / scale, 0.5 / scale, 0},
       {20.5 / scale, 0.5 / scale, 0},
       {74.5 / scale, 0.5 / scale, 0}},
      {{0}, {1, 2}});
  scene->mMeshes[0]->mColors[0]
      = new aiColor4D[3]{{1, 0, 0, 1}, {0, 1, 0, 1}, {1, 0, 0, 1}};
  scene->mNumMaterials = 1;
  scene->mMaterials = new aiMaterial* [1] {
    new aiMaterial()
  };
  const aiColor4D diffuse(0, 1, 0, 1);
  scene->mMaterials[0]->AddProperty(&diffuse, 1, AI_MATKEY_COLOR_DIFFUSE);
  auto shape = meshShape(scene);
  shape->setColorMode(dynamics::MeshShape::COLOR_INDEX);
  auto world = simulation::World::create();
  addMeshFrame(world, shape);
  Viewer viewer(meshBackground);
  setUpMeshViewer(viewer, world);
  const auto original = captureMesh(viewer, "point-line-vertex-colors");
  EXPECT_GT(original.count(false, 99, 116, 108, 124), 0u);
  EXPECT_GT(original.count(true, 180, 116, 197, 124), 3u);
  EXPECT_GT(original.count(false, 224, 116, 237, 124), 2u);
  viewer.setCameraHome({0, 0, 8}, {0, 0, 0}, {0, 1, 0});
  const auto zoom = captureMesh(viewer, "point-line-camera-zoom");
  EXPECT_GT(zoom.count(false, 129, 116, 136, 123), 0u);
  EXPECT_GT(zoom.count(true, 169, 116, 178, 123), 0u);
  EXPECT_EQ(zoom.count(false, 99, 116, 108, 124), 0u);
  shape->setColorMode(dynamics::MeshShape::MATERIAL_COLOR);
  const auto material = captureMesh(viewer, "point-line-material-colors");
  EXPECT_EQ(material.count(false), 0u);
  EXPECT_GT(material.count(true), 10u);
  EXPECT_DOUBLE_EQ(world->getTime(), 0);
}

TEST(GuiMesh, MixedMeshWireframePreservesPointAndLinePixels)
{
  auto scene = meshScene(
      {{-1.3, -0.8, 0},
       {-0.3, -0.8, 0},
       {-0.8, 0.8, 0},
       {0.6, 0.8, 0},
       {0.3, -0.5, 0},
       {1.3, -0.5, 0}},
      {{0, 1, 2}, {3}, {4, 5}});
  scene->mMeshes[0]->mColors[0] = new aiColor4D[6]{
      {1, 0, 0, 1},
      {1, 0, 0, 1},
      {1, 0, 0, 1},
      {0, 1, 0, 1},
      {0, 1, 0, 1},
      {0, 1, 0, 1}};
  auto shape = meshShape(scene);
  shape->setColorMode(dynamics::MeshShape::COLOR_INDEX);
  auto world = simulation::World::create();
  addMeshFrame(world, shape);
  Viewer viewer(meshBackground);
  setUpMeshViewer(viewer, world);
  const auto solid = captureMesh(viewer, "mixed-solid");
  ASSERT_GT(solid.count(false), 500u);
  ASSERT_GT(solid.count(true), 20u);
  viewer.setWireframe(true);
  const auto wire = captureMesh(viewer, "mixed-wireframe");
  EXPECT_GT(wire.count(false), 20u);
  EXPECT_LT(wire.count(false), solid.count(false) / 2);
  EXPECT_GE(wire.count(true), solid.count(true) * 9 / 10);
  viewer.setWireframe(false);
  const auto restored = captureMesh(viewer, "mixed-solid-restored");
  EXPECT_GT(restored.count(false), solid.count(false) * 9 / 10);
  EXPECT_GE(restored.count(true), solid.count(true) * 9 / 10);
}
