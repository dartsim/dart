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

#include <dart/config.hpp>

#include <dart/gui/application.hpp>
#include <dart/gui/debug.hpp>
#include <dart/gui/detail/scenes.hpp>
#include <dart/gui/detail/simulation_stepper.hpp>
#include <dart/gui/geometry.hpp>
#include <dart/gui/gizmo.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/scene.hpp>

#include <dart/simulation/world.hpp>

#include <dart/collision/collision_group.hpp>
#include <dart/collision/collision_option.hpp>
#include <dart/collision/collision_result.hpp>
#include <dart/collision/contact.hpp>
#include <dart/collision/dart/dart_collision_detector.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/cone_shape.hpp>
#include <dart/dynamics/convex_mesh_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/degree_of_freedom.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/heightmap_shape.hpp>
#include <dart/dynamics/inverse_kinematics.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/mesh_material.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/multi_sphere_convex_hull_shape.hpp>
#include <dart/dynamics/plane_shape.hpp>
#include <dart/dynamics/point_cloud_shape.hpp>
#include <dart/dynamics/pyramid_shape.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/soft_body_node.hpp>
#include <dart/dynamics/soft_mesh_shape.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/voxel_grid_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/geometry.hpp>
#include <dart/math/tri_mesh.hpp>

#include <dart/common/uri.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdint>

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::BoxShape;
using dart::dynamics::CapsuleShape;
using dart::dynamics::ConeShape;
using dart::dynamics::ConvexMeshShape;
using dart::dynamics::CylinderShape;
using dart::dynamics::EllipsoidShape;
using dart::dynamics::FreeJoint;
using dart::dynamics::HeightmapShaped;
using dart::dynamics::LineSegmentShape;
using dart::dynamics::MeshMaterial;
using dart::dynamics::MeshShape;
using dart::dynamics::MultiSphereConvexHullShape;
using dart::dynamics::PlaneShape;
using dart::dynamics::PointCloudShape;
using dart::dynamics::PyramidShape;
using dart::dynamics::Shape;
using dart::dynamics::ShapePtr;
using dart::dynamics::SimpleFrame;
using dart::dynamics::Skeleton;
using dart::dynamics::SoftBodyNode;
using dart::dynamics::SoftBodyNodeHelper;
using dart::dynamics::SoftMeshShape;
using dart::dynamics::SphereShape;
using dart::dynamics::VisualAspect;
using dart::dynamics::WeldJoint;
using dart::gui::ActiveRenderableState;
using dart::gui::MeshAlphaMode;
using dart::gui::MeshGeometry;
using dart::gui::ShapeKind;
using dart::math::SupportGeometry;
using dart::simulation::World;

using dart::dynamics::VoxelGridShape;

class UnsupportedTestShape final : public Shape
{
public:
  UnsupportedTestShape() : Shape(Shape::UNSUPPORTED) {}

  std::string_view getType() const override
  {
    return "UnsupportedTestShape";
  }

  Eigen::Matrix3d computeInertia(double /*mass*/) const override
  {
    return Eigen::Matrix3d::Zero();
  }

  ShapePtr clone() const override
  {
    return std::make_shared<UnsupportedTestShape>();
  }

protected:
  void updateBoundingBox() const override
  {
    mBoundingBox.setMin(Eigen::Vector3d::Zero());
    mBoundingBox.setMax(Eigen::Vector3d::Zero());
    mIsBoundingBoxDirty = false;
  }

  void updateVolume() const override
  {
    mVolume = 0.0;
    mIsVolumeDirty = false;
  }
};

SupportGeometry makeSupportFootGeometry()
{
  SupportGeometry geometry;
  geometry.emplace_back(Eigen::Vector3d(0.12, 0.06, 0.0));
  geometry.emplace_back(Eigen::Vector3d(0.12, -0.06, 0.0));
  geometry.emplace_back(Eigen::Vector3d(-0.12, -0.06, 0.0));
  geometry.emplace_back(Eigen::Vector3d(-0.12, 0.06, 0.0));
  return geometry;
}

void addSupportEndEffector(
    BodyNode* parent,
    const std::string& name,
    const Eigen::Vector3d& translation,
    const SupportGeometry& geometry)
{
  auto* endEffector = parent->createEndEffector(name);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  endEffector->setDefaultRelativeTransform(transform);
  endEffector->resetRelativeTransform();

  auto* support = endEffector->getSupport(true);
  support->setGeometry(geometry);
  support->setActive(true);
}

class MeshDescriptorHarness final : public MeshShape
{
public:
  using MeshShape::MeshShape;
  using MeshShape::SubMeshRange;

  void setMaterials(std::vector<MeshMaterial> materials)
  {
    mMaterials = std::move(materials);
  }

  void setSubMeshRanges(std::vector<SubMeshRange> ranges)
  {
    mSubMeshRanges = std::move(ranges);
  }

  void setTextureCoords(std::vector<Eigen::Vector3d> coords, int components)
  {
    mTextureCoords = std::move(coords);
    mTextureCoordComponents = components;
  }
};

class RecordingPanelBuilder final : public dart::gui::PanelBuilder
{
public:
  void text(std::string_view value) override
  {
    events.emplace_back("text:" + std::string(value));
  }

  void separator() override
  {
    events.emplace_back("separator");
  }

  void sameLine() override
  {
    events.emplace_back("same-line");
  }

  bool button(std::string_view label) override
  {
    events.emplace_back("button:" + std::string(label));
    return buttonPressed;
  }

  bool checkbox(std::string_view label, bool& value) override
  {
    events.emplace_back("checkbox:" + std::string(label));
    value = checkboxValue;
    return true;
  }

  bool slider(
      std::string_view label,
      double& value,
      double minimum,
      double maximum) override
  {
    events.emplace_back("slider:" + std::string(label));
    value = (minimum + maximum) * 0.5;
    return true;
  }

  bool colorEdit(std::string_view label, Eigen::Vector4d& rgba) override
  {
    events.emplace_back("color:" + std::string(label));
    rgba = colorValue;
    return true;
  }

  void colorSwatch(std::string_view label, const Eigen::Vector4d& rgba) override
  {
    (void)rgba;
    events.emplace_back("color-swatch:" + std::string(label));
  }

  void plotLines(
      std::string_view label, std::span<const double> values) override
  {
    events.emplace_back(
        "plot-lines:" + std::string(label) + ":"
        + std::to_string(values.size()));
  }

  bool beginTable(
      std::string_view label,
      std::span<const std::string_view> columns) override
  {
    events.emplace_back(
        "begin-table:" + std::string(label) + ":"
        + std::to_string(columns.size()));
    for (std::string_view column : columns) {
      events.emplace_back("table-column:" + std::string(column));
    }
    return true;
  }

  void tableNextRow() override
  {
    events.emplace_back("table-row");
  }

  bool tableNextColumn() override
  {
    events.emplace_back("table-column-next");
    return true;
  }

  void endTable() override
  {
    events.emplace_back("end-table");
  }

  bool collapsingHeader(std::string_view label, bool defaultOpen) override
  {
    events.emplace_back(
        "collapsing:" + std::string(label) + ":"
        + (defaultOpen ? "open" : "closed"));
    return true;
  }

  bool beginMenuBar() override
  {
    events.emplace_back("begin-menu-bar");
    return true;
  }

  void endMenuBar() override
  {
    events.emplace_back("end-menu-bar");
  }

  bool beginMenu(std::string_view label) override
  {
    events.emplace_back("begin-menu:" + std::string(label));
    return true;
  }

  void endMenu() override
  {
    events.emplace_back("end-menu");
  }

  bool menuItem(std::string_view label) override
  {
    events.emplace_back("menu-item:" + std::string(label));
    return true;
  }

  void openModal(std::string_view label, bool& open) override
  {
    events.emplace_back("open-modal:" + std::string(label));
    open = true;
  }

  bool beginModal(std::string_view label, bool& open) override
  {
    events.emplace_back(
        "begin-modal:" + std::string(label) + ":" + (open ? "open" : "closed"));
    return open;
  }

  void endModal() override
  {
    events.emplace_back("end-modal");
  }

  bool buttonPressed = true;
  bool checkboxValue = true;
  Eigen::Vector4d colorValue = Eigen::Vector4d(0.25, 0.5, 0.75, 0.85);
  std::vector<std::string> events;
};

constexpr std::array<std::string_view, 18> kForbiddenBackendTokens
    = {"#include <filament/",
       "#include \"filament/",
       "filament::",
       "Filament::",
       "GLFW",
       "ImGui",
       "imgui",
       "OpenGL",
       "Vulkan",
       "Metal",
       "osg::",
       "::osg",
       "osg/",
       "osgGA",
       "osgViewer",
       "Raylib",
       "raylib",
       "rlgl"};

constexpr std::array<std::string_view, 2> kForbiddenFilamentIncludeTokens
    = {"#include <filament/", "#include \"filament/"};

constexpr std::array<std::string_view, 3> kForbiddenDetailTokens
    = {"detail/filament", "detail/detail", "experimental::filament"};

constexpr std::array<std::string_view, 4> kForbiddenPromotedGuiTokens
    = {"dart/gui/experimental",
       "dart::gui::experimental",
       "dart-gui-experimental",
       "gui-experimental"};

const std::filesystem::path kDartsimApplicationDirectory
    = std::filesystem::path("dartsim") / "app";

const std::filesystem::path kDartsimUiDirectory
    = std::filesystem::path("dartsim") / "ui";

const std::filesystem::path kDartGuiDirectory
    = std::filesystem::path("dart") / "gui";

struct BackendTokenViolation
{
  std::filesystem::path source;
  std::string_view token;
};

std::string readSourceFile(const std::filesystem::path& relativePath)
{
  const auto path
      = std::filesystem::path(dart::config::sourcePath()) / relativePath;
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    ADD_FAILURE() << "Failed to open source file: " << path;
    return {};
  }

  return {
      std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::size_t countOccurrences(std::string_view text, std::string_view needle)
{
  std::size_t count = 0;
  std::size_t position = 0;
  while ((position = text.find(needle, position)) != std::string_view::npos) {
    ++count;
    position += needle.size();
  }
  return count;
}

std::vector<std::filesystem::path> listPublicHeadersInDirectory(
    const std::filesystem::path& relativeDirectory)
{
  const auto directory
      = std::filesystem::path(dart::config::sourcePath()) / relativeDirectory;

  std::vector<std::filesystem::path> headers;
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".hpp") {
      continue;
    }

    headers.push_back(relativeDirectory / entry.path().filename());
  }

  std::sort(headers.begin(), headers.end());
  return headers;
}

std::vector<std::filesystem::path> listRegularFilesRecursively(
    const std::filesystem::path& relativeDirectory)
{
  const auto sourceRoot = std::filesystem::path(dart::config::sourcePath());
  const auto directory = sourceRoot / relativeDirectory;

  std::vector<std::filesystem::path> files;
  for (const auto& entry :
       std::filesystem::recursive_directory_iterator(directory)) {
    if (!entry.is_regular_file()) {
      continue;
    }

    files.push_back(std::filesystem::relative(entry.path(), sourceRoot));
  }

  std::sort(files.begin(), files.end());
  return files;
}

std::vector<std::filesystem::path> listCppSourceFilesRecursively(
    const std::filesystem::path& relativeDirectory)
{
  std::vector<std::filesystem::path> sources;
  for (const auto& file : listRegularFilesRecursively(relativeDirectory)) {
    const auto extension = file.extension();
    if (extension != ".cpp" && extension != ".cc" && extension != ".cxx"
        && extension != ".hpp" && extension != ".hh" && extension != ".h"
        && extension != ".hxx") {
      continue;
    }

    sources.push_back(file);
  }

  std::sort(sources.begin(), sources.end());
  return sources;
}

template <std::size_t TokenCount>
std::vector<BackendTokenViolation> scanSourceFilesForTokens(
    const std::vector<std::filesystem::path>& sources,
    const std::array<std::string_view, TokenCount>& tokens)
{
  std::vector<BackendTokenViolation> violations;
  for (const auto& source : sources) {
    const auto contents = readSourceFile(source);
    for (const auto token : tokens) {
      if (contents.find(token) != std::string::npos) {
        violations.push_back({source, token});
      }
    }
  }

  return violations;
}

std::vector<BackendTokenViolation> scanHeadersForBackendTokens(
    const std::vector<std::filesystem::path>& directories)
{
  std::vector<std::filesystem::path> headers;
  for (const auto& directory : directories) {
    const auto directoryHeaders = listPublicHeadersInDirectory(directory);
    headers.insert(
        headers.end(), directoryHeaders.begin(), directoryHeaders.end());
  }

  return scanSourceFilesForTokens(headers, kForbiddenBackendTokens);
}

std::vector<std::filesystem::path> guiHeaderDirectoriesForBackendTokenScan()
{
  std::vector<std::filesystem::path> directories
      = {std::filesystem::path("dart") / "gui"};
  return directories;
}

std::shared_ptr<const SoftMeshShape> findSoftMeshShape(
    const SoftBodyNode& softBody)
{
  for (std::size_t i = 0; i < softBody.getNumShapeNodes(); ++i) {
    auto* shapeNode = softBody.getShapeNode(i);
    if (shapeNode == nullptr) {
      continue;
    }

    auto softMesh
        = std::dynamic_pointer_cast<const SoftMeshShape>(shapeNode->getShape());
    if (softMesh != nullptr) {
      return softMesh;
    }
  }

  return nullptr;
}

void expectMeshGeometryIsWellFormed(const MeshGeometry& mesh)
{
  ASSERT_TRUE(mesh.hasBounds);
  ASSERT_FALSE(mesh.vertices.empty());
  ASSERT_FALSE(mesh.indices.empty());
  EXPECT_EQ(mesh.indices.size(), mesh.triangles.size() * 3u);
  EXPECT_EQ(mesh.indices.size() % 3u, 0u);

  for (const auto& vertex : mesh.vertices) {
    EXPECT_TRUE(vertex.position.allFinite());
    EXPECT_TRUE(vertex.normal.allFinite());
    EXPECT_TRUE(vertex.uv.allFinite());
    EXPECT_GT(vertex.normal.squaredNorm(), 0.9f);
    EXPECT_LT(vertex.normal.squaredNorm(), 1.1f);
  }

  for (std::uint32_t index : mesh.indices) {
    EXPECT_LT(index, mesh.vertices.size());
  }
}

TEST(FilamentSceneExtraction, PublicHeadersStayBackendHidden)
{
  const auto publicHeaderDirectories
      = guiHeaderDirectoriesForBackendTokenScan();
  ASSERT_FALSE(publicHeaderDirectories.empty());

  for (const auto& directory : publicHeaderDirectories) {
    const auto headers = listPublicHeadersInDirectory(directory);
    ASSERT_FALSE(headers.empty()) << directory;
  }

  const auto violations = scanHeadersForBackendTokens(publicHeaderDirectories);
  for (const auto& violation : violations) {
    ADD_FAILURE() << violation.source
                  << " exposes backend implementation token `"
                  << violation.token << "`";
  }
}

TEST(FilamentSceneExtraction, PublicAggregateUsesPromotedGuiHeaders)
{
  const auto aggregate
      = readSourceFile(std::filesystem::path("dart") / "gui" / "all.hpp");

  EXPECT_EQ(aggregate.find("dart/gui/experimental"), std::string::npos);
  EXPECT_NE(aggregate.find("#include <dart/gui/gizmo.hpp>"), std::string::npos);
  EXPECT_NE(aggregate.find("#include <dart/gui/panel.hpp>"), std::string::npos);
  EXPECT_NE(aggregate.find("#include <dart/gui/scene.hpp>"), std::string::npos);
  EXPECT_NE(
      aggregate.find("#include <dart/gui/viewer.hpp>"), std::string::npos);
}

TEST(FilamentSceneExtraction, ViewerInputAndLightingDefaultsStayUsable)
{
  const auto inputSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "input.cpp");
  const auto frameViewportSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "frame_viewport.cpp");
  const auto sceneFrameSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "scene_frame.cpp");
  const auto uiFrameSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "ui_frame.cpp");
  const auto frameRendererSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "frame_renderer.cpp");
  const auto applicationSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "application.cpp");
  const auto sceneFixturesSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "scene_fixtures.cpp");
  const auto selectionSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "selection.cpp");
  const auto debugOverlaySource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "debug_overlay.cpp");
  const auto renderableFactorySource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail"
      / "renderable_factory.cpp");
  const auto renderableResourcesSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail"
      / "renderable_resources.cpp");
  const auto panelSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "panel.cpp");
  const auto renderEnvironmentSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail"
      / "render_environment.cpp");
  const auto viewerHeader
      = readSourceFile(std::filesystem::path("dart") / "gui" / "viewer.hpp");
  const auto debugColorSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "materials"
      / "debug_color.mat");
  const auto texturedLitSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "materials"
      / "textured_lit.mat");
  const auto transparentTexturedLitSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "materials"
      / "transparent_textured_lit.mat");

  EXPECT_NE(
      inputSource.find("isSceneMouseInputCapturedByUi"), std::string::npos);
  EXPECT_NE(inputSource.find("io.WantCaptureMouse"), std::string::npos);
  EXPECT_NE(
      frameViewportSource.find(
          "isSceneMouseInputCapturedByUi(showUi, imguiIo)"),
      std::string::npos);
  EXPECT_NE(sceneFrameSource.find("uiCapturesMouse"), std::string::npos);
  EXPECT_NE(
      sceneFixturesSource.find("makeAtlasMeshVisualsReadable(atlas)"),
      std::string::npos);
  EXPECT_NE(
      sceneFixturesSource.find("mesh->setColorMode(MeshShape::MATERIAL_COLOR)"),
      std::string::npos);
  EXPECT_NE(
      sceneFixturesSource.find("Eigen::Vector3d(5.5, 4.0, groundThickness)"),
      std::string::npos);
  EXPECT_NE(
      sceneFixturesSource.find("transform.translation().z() = -0.95"),
      std::string::npos);
  EXPECT_EQ(
      sceneFixturesSource.find("atlas->setPosition(0, -halfPi)"),
      std::string::npos);
  EXPECT_NE(selectionSource.find("uiCapturesMouse"), std::string::npos);
  EXPECT_EQ(inputSource.find("isInsideStatusPanel"), std::string::npos);
  EXPECT_EQ(frameViewportSource.find("isInsideStatusPanel"), std::string::npos);
  EXPECT_NE(
      sceneFrameSource.find("selectionDebugRenderableId()"), std::string::npos);
  EXPECT_NE(
      sceneFrameSource.find(
          "DART_PROFILE_SCOPED_N(\"SceneFrameUpdater::update\")"),
      std::string::npos);
  EXPECT_NE(uiFrameSource.find("projectDebugLabel"), std::string::npos);
  EXPECT_NE(uiFrameSource.find("renderDebugLabels"), std::string::npos);
  EXPECT_NE(uiFrameSource.find("dartScene.debugLabels"), std::string::npos);
  EXPECT_NE(
      uiFrameSource.find("DART_PROFILE_SCOPED_N(\"updateFrameUi\")"),
      std::string::npos);
  EXPECT_NE(
      frameRendererSource.find(
          "DART_PROFILE_SCOPED_N(\"renderApplicationFrame\")"),
      std::string::npos);
  EXPECT_NE(
      frameRendererSource.find("DART_PROFILE_SCOPED_N(\"beginFilamentFrame\")"),
      std::string::npos);
  EXPECT_NE(
      frameRendererSource.find(
          "DART_PROFILE_SCOPED_N(\"renderFilamentViews\")"),
      std::string::npos);
  EXPECT_NE(applicationSource.find("DART_PROFILE_FRAME"), std::string::npos);
  EXPECT_NE(
      applicationSource.find("DART_PROFILE_SCOPED_N(\"GUI render frame\")"),
      std::string::npos);
  EXPECT_NE(
      applicationSource.find("appOptions.debugLabels"), std::string::npos);
  EXPECT_NE(
      selectionSource.find("mSelectionBoundsVisible = false"),
      std::string::npos);
  EXPECT_NE(selectionSource.find("kGizmoWorldScale = 1.0"), std::string::npos);
  EXPECT_NE(selectionSource.find("solveIkHandle"), std::string::npos);
  EXPECT_NE(selectionSource.find("handle.solveMode"), std::string::npos);
  EXPECT_NE(
      selectionSource.find("InverseKinematicsSolveMode::SkeletonHierarchy"),
      std::string::npos);
  EXPECT_NE(selectionSource.find("skeleton->getIK(true)"), std::string::npos);
  EXPECT_NE(
      debugOverlaySource.find("kGizmoWorldScale = 1.0"), std::string::npos);
  EXPECT_NE(debugColorSource.find("depthWrite : false"), std::string::npos);
  EXPECT_NE(debugColorSource.find("depthCulling : false"), std::string::npos);
  EXPECT_NE(renderableFactorySource.find("line.thickness"), std::string::npos);
  EXPECT_NE(
      renderableFactorySource.find("PrimitiveType::TRIANGLES"),
      std::string::npos);
  EXPECT_NE(
      renderableFactorySource.find(
          "state.baseColor = ensureReadableDisplayColor(state.baseColor)"),
      std::string::npos);
  EXPECT_NE(
      renderableFactorySource.find("isNearBlackTextureTint"),
      std::string::npos);
  EXPECT_NE(
      renderableFactorySource.find("state.textures.baseColor != nullptr"),
      std::string::npos);
  EXPECT_NE(
      renderableFactorySource.find(
          "state.baseColor = {1.0f, 1.0f, 1.0f, state.baseColor.w}"),
      std::string::npos);
  EXPECT_NE(
      renderableFactorySource.find("constexpr float kMinLuminance = 0.16f"),
      std::string::npos);
  EXPECT_NE(
      renderableFactorySource.find(
          "constexpr float3 kNeutralMeshColor{0.28f, 0.30f, 0.34f}"),
      std::string::npos);
  EXPECT_NE(
      renderableFactorySource.find("ensureReadableEmissiveColor"),
      std::string::npos);
  EXPECT_NE(
      renderableResourcesSource.find(
          "material.baseColor = ensureReadableDisplayColor(descriptorColor)"),
      std::string::npos);
  EXPECT_NE(
      renderableResourcesSource.find("constexpr float kMinLuminance = 0.16f"),
      std::string::npos);
  EXPECT_NE(
      renderableResourcesSource.find(
          "constexpr float3 kNeutralMeshColor{0.28f, 0.30f, 0.34f}"),
      std::string::npos);
  EXPECT_NE(
      selectionSource.find("const Eigen::Vector3d dragPoint"),
      std::string::npos);
  EXPECT_NE(
      selectionSource.find(
          "mSelectedDragPlanePoint = bodyDrag.targetTransform.translation()"),
      std::string::npos);
  EXPECT_NE(panelSource.find("defaultPositionIndex"), std::string::npos);
  EXPECT_NE(panelSource.find("ImGuiCond_FirstUseEver"), std::string::npos);
  EXPECT_EQ(panelSource.find("ImGuiCond_Always"), std::string::npos);
  EXPECT_NE(
      panelSource.find("context.ui.displaySize.x()) - panelWidth"),
      std::string::npos);
  EXPECT_NE(viewerHeader.find("nearScale = 0.001"), std::string::npos);
  EXPECT_NE(texturedLitSource.find("baseLuminance"), std::string::npos);
  EXPECT_NE(
      transparentTexturedLitSource.find("baseLuminance"), std::string::npos);
  EXPECT_NE(texturedLitSource.find("readableDisplayColor"), std::string::npos);
  EXPECT_NE(
      transparentTexturedLitSource.find("readableDisplayColor"),
      std::string::npos);

  EXPECT_NE(
      renderEnvironmentSource.find("{0.27f, 0.29f, 0.33f}"), std::string::npos);
  EXPECT_NE(
      renderEnvironmentSource.find("kKeyLightIntensity = 82000.0f"),
      std::string::npos);
  EXPECT_NE(
      renderEnvironmentSource.find("kFillLightIntensity = 26000.0f"),
      std::string::npos);
  EXPECT_NE(
      renderEnvironmentSource.find("kRimLightIntensity = 18000.0f"),
      std::string::npos);
  EXPECT_NE(
      renderEnvironmentSource.find(".intensity(38000.0f)"), std::string::npos);
  EXPECT_NE(
      renderEnvironmentSource.find(".color({0.30f, 0.33f, 0.38f, 1.0f})"),
      std::string::npos);
  EXPECT_NE(
      renderEnvironmentSource.find("fogOptions.enabled = true"),
      std::string::npos);
  EXPECT_NE(
      renderEnvironmentSource.find("ambientOcclusionOptions.intensity = 0.32f"),
      std::string::npos);
  EXPECT_NE(
      renderEnvironmentSource.find(
          "shadowOptions.shadowFar = headless ? 32.0f : 48.0f"),
      std::string::npos);
}

TEST(FilamentSceneExtraction, DockingPanelFallbackRequiresDockingSupport)
{
  const auto panelSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "panel.cpp");

  EXPECT_NE(
      panelSource.find(
          "const bool dockingActive = dockingEnabled && "
          "dart::gui::isDockingAvailable();"),
      std::string::npos);
  EXPECT_NE(
      panelSource.find(
          "ImGui::SetNextWindowBgAlpha(dockingActive ? 1.0f : 0.72f)"),
      std::string::npos);
  EXPECT_NE(
      panelSource.find(
          "const double defaultAlpha = dockingActive ? 1.0 : 0.72"),
      std::string::npos);
  EXPECT_NE(panelSource.find("if (!dockingActive)"), std::string::npos);
}

TEST(FilamentSceneExtraction, PromotedGuiHeadersAvoidExperimentalSurface)
{
  const auto headers
      = listPublicHeadersInDirectory(std::filesystem::path("dart") / "gui");

  const auto violations
      = scanSourceFilesForTokens(headers, kForbiddenPromotedGuiTokens);
  for (const auto& violation : violations) {
    ADD_FAILURE() << violation.source << " reaches old GUI token `"
                  << violation.token << "`";
  }
}

TEST(FilamentSceneExtraction, DiagnosticsExampleUsesPromotedGuiBoundary)
{
  const std::vector<std::filesystem::path> sources = {
      std::filesystem::path("examples") / "gui_scene_diagnostics" / "main.cpp",
      std::filesystem::path("examples") / "gui_scene_diagnostics"
          / "CMakeLists.txt"};

  const auto violations
      = scanSourceFilesForTokens(sources, kForbiddenPromotedGuiTokens);
  for (const auto& violation : violations) {
    ADD_FAILURE() << violation.source << " reaches old GUI token `"
                  << violation.token << "`";
  }

  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "gui_scene_diagnostics" / "main.cpp");
  EXPECT_NE(
      mainSource.find("#include <dart/gui/scene.hpp>"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart::gui::extractRenderables"), std::string::npos);
}

TEST(FilamentSceneExtraction, DiagnosticsExamplePreservesLegacyContractMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "gui_scene_diagnostics" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "gui_scene_diagnostics"
      / "README.md");

  EXPECT_NE(mainSource.find("options.run.maxFrames = 10"), std::string::npos);
  EXPECT_NE(mainSource.find("--frames"), std::string::npos);
  EXPECT_NE(mainSource.find("--width"), std::string::npos);
  EXPECT_NE(mainSource.find("--height"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart::gui::normalizeRunOptions"), std::string::npos);
  EXPECT_NE(mainSource.find("createDiagnosticWorld"), std::string::npos);
  EXPECT_NE(mainSource.find("gui_scene_diagnostics"), std::string::npos);
  EXPECT_NE(mainSource.find("diagnostic_box"), std::string::npos);
  EXPECT_NE(mainSource.find("box_visual"), std::string::npos);
  EXPECT_NE(mainSource.find("ground_visual"), std::string::npos);
  EXPECT_NE(mainSource.find("diagnostic_interactive_frame"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart::gui::extractRenderables"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::gui::extractDebugLines"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart::gui::makeSelectionDebugLines"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart::gui::makePerspectivePickRay"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart::gui::pickNearestRenderable"), std::string::npos);
  EXPECT_NE(mainSource.find("\"frames: \""), std::string::npos);
  EXPECT_NE(mainSource.find("\"renderables: \""), std::string::npos);
  EXPECT_NE(mainSource.find("\"debug lines: \""), std::string::npos);
  EXPECT_NE(mainSource.find("\"selection lines: \""), std::string::npos);
  EXPECT_NE(mainSource.find("\"camera eye: \""), std::string::npos);
  EXPECT_NE(mainSource.find("\"center pick: \""), std::string::npos);
  EXPECT_NE(
      readmeSource.find("GUI Scene Diagnostics Example"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("renderer-independent GUI descriptors"),
      std::string::npos);
  EXPECT_NE(readmeSource.find("`dartsim`"), std::string::npos);
}

TEST(FilamentSceneExtraction, RerunPlaceholderIsRemoved)
{
  const auto examplesCmake
      = readSourceFile(std::filesystem::path("examples") / "CMakeLists.txt");
  const auto examplesReadme
      = readSourceFile(std::filesystem::path("examples") / "README.md");

  EXPECT_EQ(examplesCmake.find("add_subdirectory(rerun)"), std::string::npos);
  EXPECT_EQ(examplesReadme.find("`rerun`"), std::string::npos);
  EXPECT_FALSE(
      std::filesystem::exists(
          std::filesystem::path(dart::config::sourcePath()) / "examples"
          / "rerun"));
}

TEST(FilamentSceneExtraction, GuiExamplesDoNotExposeSceneLauncherShim)
{
  const auto helperSource = readSourceFile(
      std::filesystem::path("examples") / "gui_scene_example.cmake");
  EXPECT_EQ(
      helperSource.find("dart_build_gui_scene_example"), std::string::npos);
  EXPECT_EQ(helperSource.find("DART_GUI_DEFAULT_SCENE"), std::string::npos);
  EXPECT_FALSE(
      std::filesystem::exists(
          std::filesystem::path(dart::config::sourcePath()) / "examples"
          / "gui_scene_launcher.cpp"));
}

TEST(FilamentSceneExtraction, GuiBuildTargetsUseOfficialCoreName)
{
  const std::vector<std::filesystem::path> cmakeFiles
      = {std::filesystem::path("dart") / "gui" / "CMakeLists.txt",
         std::filesystem::path("tests") / "unit" / "CMakeLists.txt",
         std::filesystem::path("python") / "dartpy" / "CMakeLists.txt"};

  for (const auto& cmakeFile : cmakeFiles) {
    const auto source = readSourceFile(cmakeFile);
    EXPECT_EQ(source.find("dart-gui-experimental"), std::string::npos)
        << cmakeFile;
    EXPECT_EQ(source.find("gui-experimental"), std::string::npos) << cmakeFile;
  }

  const auto coreCMake = readSourceFile(
      std::filesystem::path("dart") / "gui" / "CMakeLists.txt");
  EXPECT_NE(coreCMake.find("${PROJECT_NAME}-gui-core"), std::string::npos);
  EXPECT_NE(coreCMake.find("gui-core"), std::string::npos);
}

TEST(FilamentSceneExtraction, PanelBuilderSupportsRendererNeutralControls)
{
  bool diagnostics = false;
  double gain = 0.0;
  int clicks = 0;
  std::string selectedLabel;
  bool headlights = false;
  bool aboutOpen = false;
  Eigen::Vector4d tint = Eigen::Vector4d::Ones();
  dart::gui::RenderSettings renderSettings;
  renderSettings.shadowsEnabled = false;
  dart::gui::Panel panel;
  panel.title = "Controls";
  panel.build = [&](dart::gui::PanelBuilder& builder) {
    if (builder.beginMenuBar()) {
      if (builder.beginMenu("Menu")) {
        if (builder.menuItem("Exit")) {
          ++clicks;
        }
        builder.endMenu();
      }
      if (builder.beginMenu("Help")) {
        if (builder.menuItem("About DART")) {
          builder.openModal("About DART", aboutOpen);
        }
        builder.endMenu();
      }
      builder.endMenuBar();
    }
    if (builder.beginModal("About DART", aboutOpen)) {
      builder.text("about modal");
      if (builder.button("Close")) {
        aboutOpen = false;
      }
      builder.endModal();
    }
    builder.text("custom controls");
    builder.separator();
    if (builder.collapsingHeader("Advanced", true)) {
      builder.text("advanced controls");
    }
    if (builder.button("Trigger")) {
      ++clicks;
    }
    builder.sameLine();
    builder.checkbox("Diagnostics", diagnostics);
    builder.slider("Gain", gain, 0.0, 2.0);
    builder.colorEdit("Tint", tint);
    builder.colorSwatch("Tint swatch", tint);
    int selectedMode = 0;
    constexpr std::array<std::string_view, 2> modes{"Edit", "Simulation"};
    builder.select("Mode", selectedMode, modes);
    constexpr std::array<double, 3> trend{0.0, 1.0, 0.5};
    builder.plotLines("Trend", trend);
    constexpr std::array<std::string_view, 2> columns{"Name", "Value"};
    if (builder.beginTable("Stats", columns)) {
      builder.tableNextRow();
      builder.tableNextColumn();
      builder.text("gain");
      builder.tableNextColumn();
      builder.text(std::to_string(gain));
      builder.endTable();
    }
  };
  panel.buildWithContext = [&](dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    selectedLabel = context.selectedLabel;
    builder.text("selected:" + context.selectedLabel);
    builder.text("eye-x:" + std::to_string(context.camera.eye.x()));
    builder.text(
        "ui-display-width:" + std::to_string(context.ui.displaySize.x()));
    if (context.ui.fontTextureSize.has_value()) {
      builder.text(
          "ui-font-texture-width:"
          + std::to_string((*context.ui.fontTextureSize)[0]));
    }
    if (context.lighting.headlightsEnabled != nullptr) {
      bool enabled = *context.lighting.headlightsEnabled;
      if (builder.checkbox("Headlights On/Off", enabled)) {
        *context.lighting.headlightsEnabled = enabled;
      }
    }
    if (context.rendering.settings != nullptr) {
      bool shadows = context.rendering.settings->shadowsEnabled;
      if (builder.checkbox("Shadow On/Off", shadows)) {
        context.rendering.settings->shadowsEnabled = shadows;
      }
      bool depthMode = context.rendering.settings->outputMode
                       == dart::gui::RenderOutputMode::Depth;
      if (builder.checkbox("Depth mode", depthMode)) {
        context.rendering.settings->outputMode
            = depthMode ? dart::gui::RenderOutputMode::Depth
                        : dart::gui::RenderOutputMode::Color;
      }
    }
  };

  RecordingPanelBuilder builder;
  dart::gui::PanelContext context;
  context.selectedLabel = "box";
  context.selectedPoint = Eigen::Vector3d(1.0, 2.0, 3.0);
  context.selectedNormal = Eigen::Vector3d::UnitZ();
  context.camera.eye = Eigen::Vector3d(4.0, 5.0, 6.0);
  context.camera.target = Eigen::Vector3d(1.0, 2.0, 3.0);
  context.camera.up = Eigen::Vector3d::UnitY();
  context.ui.displaySize = Eigen::Vector2d(640.0, 480.0);
  context.ui.framebufferScale = Eigen::Vector2d(2.0, 2.0);
  context.ui.fontSize = 14.0;
  context.ui.fontGlobalScale = 1.25;
  context.ui.uiScale = 1.5;
  context.ui.fontTextureSize = std::array<int, 2>{512, 256};
  context.lighting.headlightsEnabled = &headlights;
  context.rendering.settings = &renderSettings;
  panel.build(builder);
  panel.buildWithContext(builder, context);

  EXPECT_EQ(clicks, 2);
  EXPECT_TRUE(diagnostics);
  EXPECT_DOUBLE_EQ(gain, 1.0);
  EXPECT_EQ(selectedLabel, "box");
  EXPECT_FALSE(aboutOpen);
  ASSERT_TRUE(context.selectedPoint.has_value());
  EXPECT_TRUE(context.selectedPoint->isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  ASSERT_TRUE(context.selectedNormal.has_value());
  EXPECT_TRUE(context.selectedNormal->isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(context.camera.eye.isApprox(Eigen::Vector3d(4.0, 5.0, 6.0)));
  EXPECT_TRUE(context.camera.target.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(context.camera.up.isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(context.ui.displaySize.isApprox(Eigen::Vector2d(640.0, 480.0)));
  EXPECT_TRUE(context.ui.framebufferScale.isApprox(Eigen::Vector2d(2.0, 2.0)));
  EXPECT_DOUBLE_EQ(context.ui.fontSize, 14.0);
  EXPECT_DOUBLE_EQ(context.ui.fontGlobalScale, 1.25);
  EXPECT_DOUBLE_EQ(context.ui.uiScale, 1.5);
  ASSERT_TRUE(context.ui.fontTextureSize.has_value());
  EXPECT_EQ((*context.ui.fontTextureSize)[0], 512);
  EXPECT_EQ((*context.ui.fontTextureSize)[1], 256);
  EXPECT_TRUE(headlights);
  EXPECT_TRUE(tint.isApprox(Eigen::Vector4d(0.25, 0.5, 0.75, 0.85)));
  EXPECT_TRUE(renderSettings.shadowsEnabled);
  EXPECT_EQ(renderSettings.outputMode, dart::gui::RenderOutputMode::Depth);
  bool preStepCalled = false;
  bool postStepCalled = false;
  bool preRenderCalled = false;
  bool postRenderCalled = false;
  EXPECT_EQ(
      builder.events,
      (std::vector<std::string>{
          "begin-menu-bar",
          "begin-menu:Menu",
          "menu-item:Exit",
          "end-menu",
          "begin-menu:Help",
          "menu-item:About DART",
          "open-modal:About DART",
          "end-menu",
          "end-menu-bar",
          "begin-modal:About DART:open",
          "text:about modal",
          "button:Close",
          "end-modal",
          "text:custom controls",
          "separator",
          "collapsing:Advanced:open",
          "text:advanced controls",
          "button:Trigger",
          "same-line",
          "checkbox:Diagnostics",
          "slider:Gain",
          "color:Tint",
          "color-swatch:Tint swatch",
          "plot-lines:Trend:3",
          "begin-table:Stats:2",
          "table-column:Name",
          "table-column:Value",
          "table-row",
          "table-column-next",
          "text:gain",
          "table-column-next",
          "text:1.000000",
          "end-table",
          "text:selected:box",
          "text:eye-x:4.000000",
          "text:ui-display-width:640.000000",
          "text:ui-font-texture-width:512",
          "checkbox:Headlights On/Off",
          "checkbox:Shadow On/Off",
          "checkbox:Depth mode"}));

  dart::gui::ApplicationOptions options;
  options.world = World::create("panel_test");
  options.preStep = [&preStepCalled]() {
    preStepCalled = true;
  };
  options.postStep = [&postStepCalled]() {
    postStepCalled = true;
  };
  options.preRender = [&preRenderCalled]() {
    preRenderCalled = true;
  };
  options.postRender = [&postRenderCalled]() {
    postRenderCalled = true;
  };
  options.simulateWorld = false;
  options.runDefaults = dart::gui::RunOptions{};
  options.runDefaults->width = 640;
  options.runDefaults->height = 480;
  options.camera = dart::gui::OrbitCamera{};
  options.camera->target = Eigen::Vector3d(1.0, 2.0, 3.0);
  options.renderSettings.shadowsEnabled = false;
  options.renderSettings.outputMode = dart::gui::RenderOutputMode::Depth;
  options.defaultScene = "mvp";
  options.panels.push_back(std::move(panel));
  ASSERT_NE(options.world, nullptr);
  EXPECT_EQ(options.world->getName(), "panel_test");
  options.preStep();
  EXPECT_TRUE(preStepCalled);
  options.postStep();
  EXPECT_TRUE(postStepCalled);
  options.preRender();
  EXPECT_TRUE(preRenderCalled);
  options.postRender();
  EXPECT_TRUE(postRenderCalled);
  EXPECT_FALSE(options.simulateWorld);
  ASSERT_TRUE(options.runDefaults.has_value());
  EXPECT_EQ(options.runDefaults->width, 640);
  EXPECT_EQ(options.runDefaults->height, 480);
  ASSERT_TRUE(options.camera.has_value());
  EXPECT_TRUE(options.camera->target.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_FALSE(options.renderSettings.shadowsEnabled);
  EXPECT_EQ(
      options.renderSettings.outputMode, dart::gui::RenderOutputMode::Depth);
  EXPECT_EQ(options.defaultScene, "mvp");
  ASSERT_EQ(options.panels.size(), 1u);
  EXPECT_EQ(options.panels.front().title, "Controls");

  dart::gui::detail::AppOptions appOptions;
  appOptions.world = options.world;
  appOptions.simulateWorld = options.simulateWorld;
  appOptions.renderSettings = options.renderSettings;
  const dart::gui::detail::DartScene scene
      = dart::gui::detail::createDartScene(appOptions);
  EXPECT_FALSE(scene.simulateWorld);
  EXPECT_EQ(
      scene.renderSettings.outputMode, dart::gui::RenderOutputMode::Depth);
}

TEST(FilamentSceneExtraction, ApplicationOptionsStoresIkHandles)
{
  dart::gui::ApplicationOptions options;
  dart::gui::InverseKinematicsHandle handle;
  handle.label = "left hand";
  handle.hotkey = '1';
  handle.solveMode = dart::gui::InverseKinematicsSolveMode::SkeletonHierarchy;

  options.ikHandles.push_back(handle);

  ASSERT_EQ(options.ikHandles.size(), 1u);
  EXPECT_EQ(options.ikHandles.front().label, "left hand");
  EXPECT_EQ(options.ikHandles.front().hotkey, '1');
  EXPECT_EQ(
      options.ikHandles.front().solveMode,
      dart::gui::InverseKinematicsSolveMode::SkeletonHierarchy);
}

TEST(FilamentSceneExtraction, ApplicationOptionsCopiesIkHandleSolveMode)
{
  dart::gui::detail::AppOptions appOptions;
  appOptions.world = World::create("ik_handle_solve_mode_scene");
  auto target
      = SimpleFrame::createShared(dart::dynamics::Frame::World(), "target");
  appOptions.world->addSimpleFrame(target);

  dart::gui::InverseKinematicsHandle handle;
  handle.label = "whole body target";
  handle.target = target;
  handle.solveMode = dart::gui::InverseKinematicsSolveMode::SkeletonHierarchy;
  appOptions.ikHandles.push_back(handle);

  const dart::gui::detail::DartScene scene
      = dart::gui::detail::createDartScene(appOptions);
  ASSERT_EQ(scene.ikHandles.size(), 1u);
  EXPECT_EQ(scene.ikHandles.front().label, "whole body target");
  EXPECT_EQ(
      scene.ikHandles.front().solveMode,
      dart::gui::InverseKinematicsSolveMode::SkeletonHierarchy);
}

TEST(
    FilamentSceneExtraction,
    AtlasPuppetSceneWholeBodyIkMovesEndEffectorTowardTarget)
{
  dart::gui::detail::AppOptions appOptions;
  appOptions.scene = dart::gui::detail::ExampleScene::AtlasPuppet;

  dart::gui::detail::DartScene scene
      = dart::gui::detail::createDartScene(appOptions);
  ASSERT_NE(scene.world, nullptr);
  ASSERT_GE(scene.ikHandles.size(), 4u);

  auto handle = std::find_if(
      scene.ikHandles.begin(),
      scene.ikHandles.end(),
      [](const dart::gui::detail::IkHandle& candidate) {
        return candidate.label == "1 left hand";
      });
  ASSERT_NE(handle, scene.ikHandles.end());
  ASSERT_NE(handle->ik, nullptr);
  ASSERT_NE(handle->target, nullptr);
  ASSERT_NE(handle->targetRenderableId, 0u);
  EXPECT_EQ(
      handle->solveMode,
      dart::gui::InverseKinematicsSolveMode::SkeletonHierarchy);

  auto* node = handle->ik->getNode();
  ASSERT_NE(node, nullptr);
  const auto bodyNode = node->getBodyNodePtr();
  ASSERT_NE(bodyNode, nullptr);
  const auto skeleton = bodyNode->getSkeleton();
  ASSERT_NE(skeleton, nullptr);
  const auto* rootBody = skeleton->getRootBodyNode();
  ASSERT_NE(rootBody, nullptr);
  const auto* rootJoint = rootBody->getParentJoint();
  ASSERT_NE(rootJoint, nullptr);

  const auto dofs = handle->ik->getDofs();
  const Eigen::VectorXd& componentWeights
      = handle->ik->getGradientMethod().getComponentWeights();
  ASSERT_EQ(componentWeights.size(), static_cast<Eigen::Index>(dofs.size()));
  bool sawRootDof = false;
  bool sawNonRootDof = false;
  for (std::size_t i = 0; i < dofs.size(); ++i) {
    const auto* dof = skeleton->getDof(dofs[i]);
    ASSERT_NE(dof, nullptr);
    if (dof->getJoint() == rootJoint) {
      EXPECT_DOUBLE_EQ(componentWeights[static_cast<Eigen::Index>(i)], 1e-3);
      sawRootDof = true;
    } else {
      EXPECT_DOUBLE_EQ(componentWeights[static_cast<Eigen::Index>(i)], 1.0);
      sawNonRootDof = true;
    }
  }
  EXPECT_TRUE(sawRootDof);
  EXPECT_TRUE(sawNonRootDof);

  Eigen::Vector3d linearBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d angularBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  for (auto& candidate : scene.ikHandles) {
    if (&candidate == &*handle || candidate.ik == nullptr) {
      continue;
    }
    candidate.ik->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
    candidate.ik->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);
  }
  handle->ik->getErrorMethod().setBounds(
      Eigen::Vector6d::Constant(-1e-8), Eigen::Vector6d::Constant(1e-8));

  Eigen::Isometry3d target = handle->target->getWorldTransform();
  target.translation() += Eigen::Vector3d(0.10, 0.0, 0.0);
  handle->target->setTransform(target);

  const auto distanceToTarget = [&] {
    return (node->getWorldTransform().translation()
            - handle->target->getWorldTransform().translation())
        .norm();
  };

  const double before = distanceToTarget();
  ASSERT_GT(before, 1e-3);
  const Eigen::Isometry3d rootBefore = rootBody->getWorldTransform();
  const auto wholeBodyIk = skeleton->getIK(true);
  ASSERT_NE(wholeBodyIk, nullptr);
  EXPECT_TRUE(wholeBodyIk->solveAndApply(true));
  EXPECT_LT(distanceToTarget(), 0.75 * before);
  EXPECT_LT(
      (rootBody->getWorldTransform().translation() - rootBefore.translation())
          .norm(),
      0.01);
}

TEST(FilamentSceneExtraction, ApplicationOptionsCanSkipWorldStepping)
{
  dart::gui::detail::DartScene scene;
  scene.world = World::create("kinematic_scene");
  scene.simulateWorld = false;

  int preStepCalls = 0;
  int postStepCalls = 0;
  scene.preStep = [&preStepCalls]() {
    ++preStepCalls;
  };
  scene.postStep = [&postStepCalls]() {
    ++postStepCalls;
  };

  dart::gui::ViewerLifecycleState lifecycle;
  dart::gui::ProfileAccumulator profile;
  const double initialTime = scene.world->getTime();
  EXPECT_TRUE(
      dart::gui::detail::advanceSimulationSteps(scene, 3, lifecycle, profile));

  EXPECT_EQ(preStepCalls, 3);
  EXPECT_EQ(postStepCalls, 3);
  EXPECT_DOUBLE_EQ(scene.world->getTime(), initialTime);
  EXPECT_EQ(profile.simulationSteps, 3u);
  EXPECT_DOUBLE_EQ(profile.simulatedMs, 0.0);
}

TEST(FilamentSceneExtraction, ApplicationOptionsStoresGizmos)
{
  dart::gui::ApplicationOptions options;
  auto target = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "gizmo_target");

  dart::gui::Gizmo gizmo;
  gizmo.label = "target gizmo";
  gizmo.target = target;
  gizmo.flags = dart::gui::GizmoFlags::Translate;
  gizmo.size = 0.25;

  options.gizmos.push_back(gizmo);

  ASSERT_EQ(options.gizmos.size(), 1u);
  EXPECT_EQ(options.gizmos.front().label, "target gizmo");
  EXPECT_EQ(options.gizmos.front().target, target);
  EXPECT_EQ(options.gizmos.front().flags, dart::gui::GizmoFlags::Translate);
  EXPECT_DOUBLE_EQ(options.gizmos.front().size, 0.25);
}

TEST(FilamentSceneExtraction, ApplicationOptionsStoresDebugLabels)
{
  dart::gui::ApplicationOptions options;
  options.world = World::create("debug_label_scene");
  options.debugLabels = [] {
    dart::gui::DebugLabelDescriptor label;
    label.text = "AABB 0";
    label.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    label.rgba = Eigen::Vector4d(0.1, 0.2, 0.3, 0.4);
    return std::vector<dart::gui::DebugLabelDescriptor>{label};
  };

  ASSERT_TRUE(options.debugLabels);
  const auto labels = options.debugLabels();
  ASSERT_EQ(labels.size(), 1u);
  EXPECT_EQ(labels.front().text, "AABB 0");
  EXPECT_TRUE(labels.front().position.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(
      labels.front().rgba.isApprox(Eigen::Vector4d(0.1, 0.2, 0.3, 0.4)));

  dart::gui::detail::AppOptions appOptions;
  appOptions.world = options.world;
  appOptions.debugLabels = options.debugLabels;
  const dart::gui::detail::DartScene scene
      = dart::gui::detail::createDartScene(appOptions);

  ASSERT_TRUE(scene.debugLabels);
  const auto sceneLabels = scene.debugLabels();
  ASSERT_EQ(sceneLabels.size(), 1u);
  EXPECT_EQ(sceneLabels.front().text, "AABB 0");
  EXPECT_TRUE(
      sceneLabels.front().position.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(FilamentSceneExtraction, ApplicationOptionsStoresRenderableSelectionBridge)
{
  dart::gui::ApplicationOptions options;
  options.world = World::create("renderable_selection_bridge_scene");
  options.selectedRenderableProvider = [] {
    return dart::gui::RenderableSelection{42u, "selected object"};
  };

  dart::gui::RenderableId callbackId = 0;
  options.onRenderableSelected = [&callbackId](dart::gui::RenderableId id) {
    callbackId = id;
  };

  ASSERT_TRUE(options.selectedRenderableProvider);
  const dart::gui::RenderableSelection selection
      = options.selectedRenderableProvider();
  EXPECT_EQ(selection.id, 42u);
  EXPECT_EQ(selection.label, "selected object");
  ASSERT_TRUE(options.onRenderableSelected);
  options.onRenderableSelected(24u);
  EXPECT_EQ(callbackId, 24u);

  dart::gui::detail::AppOptions appOptions;
  appOptions.world = options.world;
  appOptions.selectedRenderableProvider = options.selectedRenderableProvider;
  appOptions.onRenderableSelected = options.onRenderableSelected;
  const dart::gui::detail::DartScene scene
      = dart::gui::detail::createDartScene(appOptions);

  ASSERT_TRUE(scene.selectedRenderableProvider);
  const dart::gui::RenderableSelection sceneSelection
      = scene.selectedRenderableProvider();
  EXPECT_EQ(sceneSelection.id, 42u);
  EXPECT_EQ(sceneSelection.label, "selected object");
  ASSERT_TRUE(scene.onRenderableSelected);
  scene.onRenderableSelected(7u);
  EXPECT_EQ(callbackId, 7u);
}

TEST(FilamentSceneExtraction, ApplicationOptionsStoresBodyNodeDragHandles)
{
  auto world = World::create("body_drag_scene");
  auto skeleton = Skeleton::create("body_drag_robot");
  auto* body = skeleton->createJointAndBodyNodePair<FreeJoint>().second;
  body->setName("dragged_body");
  world->addSkeleton(skeleton);

  dart::gui::ApplicationOptions options;
  options.world = world;

  dart::gui::BodyNodeDragHandle handle;
  handle.label = "dragged body";
  handle.bodyNode = body;
  handle.useWholeBody = true;
  options.bodyNodeDragHandles.push_back(handle);

  ASSERT_EQ(options.bodyNodeDragHandles.size(), 1u);
  EXPECT_EQ(options.bodyNodeDragHandles.front().label, "dragged body");
  EXPECT_EQ(options.bodyNodeDragHandles.front().bodyNode, body);
  EXPECT_TRUE(options.bodyNodeDragHandles.front().useWholeBody);

  dart::gui::detail::AppOptions appOptions;
  appOptions.world = options.world;
  appOptions.bodyNodeDragHandles = options.bodyNodeDragHandles;
  const dart::gui::detail::DartScene scene
      = dart::gui::detail::createDartScene(appOptions);
  ASSERT_EQ(scene.bodyNodeDragHandles.size(), 1u);
  EXPECT_EQ(scene.bodyNodeDragHandles.front().bodyNode, body);
}

TEST(FilamentSceneExtraction, GizmoDebugLinesFollowTargetFrame)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  auto target = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "gizmo_target", transform);

  dart::gui::Gizmo gizmo;
  gizmo.label = "atlas_target";
  gizmo.target = target;
  gizmo.size = 0.5;

  const auto lines = dart::gui::makeGizmoDebugLines(gizmo, 2.0);
  const auto triangles = dart::gui::makeGizmoDebugTriangles(gizmo, 2.0);

  ASSERT_GE(lines.size(), 12u);
  EXPECT_FALSE(triangles.empty());
  EXPECT_EQ(lines.front().label, "atlas_target.x");
  EXPECT_TRUE(lines.front().from.isApprox(Eigen::Vector3d(1.56, 2.0, 3.0)));
  EXPECT_TRUE(lines.front().to.isApprox(Eigen::Vector3d(2.0, 2.0, 3.0)));
  EXPECT_GT(lines.front().thickness, 0.0);
  EXPECT_NE(
      std::find_if(
          lines.begin(),
          lines.end(),
          [](const dart::gui::DebugLineDescriptor& line) {
            return line.label == "atlas_target.free_x";
          }),
      lines.end());
  EXPECT_NE(
      std::find_if(
          lines.begin(),
          lines.end(),
          [](const dart::gui::DebugLineDescriptor& line) {
            return line.label == "atlas_target.rotate_x";
          }),
      lines.end());
  EXPECT_NE(
      std::find_if(
          lines.begin(),
          lines.end(),
          [](const dart::gui::DebugLineDescriptor& line) {
            return line.label == "atlas_target.rotate_x.inner";
          }),
      lines.end());
  EXPECT_NE(
      std::find_if(
          lines.begin(),
          lines.end(),
          [](const dart::gui::DebugLineDescriptor& line) {
            return line.label == "atlas_target.rotate_x.pattern";
          }),
      lines.end());
  EXPECT_NE(
      std::find_if(
          lines.begin(),
          lines.end(),
          [](const dart::gui::DebugLineDescriptor& line) {
            return line.label == "atlas_target.translate_xy";
          }),
      lines.end());

  dart::gui::Gizmo invalid;
  EXPECT_TRUE(dart::gui::makeGizmoDebugLines(invalid).empty());
  EXPECT_TRUE(dart::gui::makeGizmoDebugTriangles(invalid).empty());
}

TEST(FilamentSceneExtraction, GizmoDebugLinesHighlightSelectedHandle)
{
  auto target = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "gizmo_target");

  dart::gui::Gizmo gizmo;
  gizmo.label = "highlight_target";
  gizmo.target = target;
  gizmo.size = 1.0;

  const auto lineWithLabel
      = [](const std::vector<dart::gui::DebugLineDescriptor>& lines,
           const std::string& label) {
          const auto line = std::find_if(
              lines.begin(),
              lines.end(),
              [&](const dart::gui::DebugLineDescriptor& candidate) {
                return candidate.label == label;
              });
          return line == lines.end() ? nullptr : &*line;
        };

  const auto highlightedPlaneLines = dart::gui::makeGizmoDebugLines(
      gizmo, 1.0, dart::gui::GizmoHandleKind::TranslateXY);
  const auto* highlightedPlaneLine
      = lineWithLabel(highlightedPlaneLines, "highlight_target.translate_xy");
  ASSERT_NE(highlightedPlaneLine, nullptr);
  EXPECT_TRUE(highlightedPlaneLine->rgba.isApprox(gizmo.colors.highlight));

  const auto* xLine
      = lineWithLabel(highlightedPlaneLines, "highlight_target.x");
  ASSERT_NE(xLine, nullptr);
  EXPECT_TRUE(xLine->rgba.isApprox(gizmo.colors.x));

  const auto* freeLine
      = lineWithLabel(highlightedPlaneLines, "highlight_target.free_x");
  ASSERT_NE(freeLine, nullptr);
  EXPECT_FALSE(freeLine->rgba.isApprox(gizmo.colors.highlight));

  auto secondTarget = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "second_gizmo_target");
  dart::gui::Gizmo secondGizmo;
  secondGizmo.label = "second_target";
  secondGizmo.target = secondTarget;
  secondGizmo.size = 1.0;

  dart::gui::GizmoHandleHit activeHandle;
  activeHandle.gizmoIndex = 1u;
  activeHandle.handle = dart::gui::GizmoHandleKind::RotateZ;

  const std::vector<dart::gui::Gizmo> gizmos{gizmo, secondGizmo};
  const auto aggregateLines
      = dart::gui::makeGizmoDebugLines(gizmos, 1.0, activeHandle);

  const auto* firstRotateZ
      = lineWithLabel(aggregateLines, "highlight_target.rotate_z");
  ASSERT_NE(firstRotateZ, nullptr);
  EXPECT_TRUE(firstRotateZ->rgba.isApprox(gizmo.colors.z));

  const auto* secondRotateZ
      = lineWithLabel(aggregateLines, "second_target.rotate_z");
  ASSERT_NE(secondRotateZ, nullptr);
  EXPECT_TRUE(secondRotateZ->rgba.isApprox(secondGizmo.colors.highlight));
}

TEST(FilamentSceneExtraction, GizmoVisibilityControlsDebugLinesAndPicking)
{
  auto target = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "gizmo_target");

  dart::gui::Gizmo gizmo;
  gizmo.label = "hidden_target";
  gizmo.target = target;
  gizmo.size = 1.0;
  gizmo.isVisible = []() {
    return false;
  };

  EXPECT_TRUE(dart::gui::makeGizmoDebugLines(gizmo).empty());
  EXPECT_TRUE(dart::gui::makeGizmoDebugTriangles(gizmo).empty());

  std::vector<dart::gui::Gizmo> gizmos{gizmo};
  const dart::gui::PickRay xHandleRay{
      Eigen::Vector3d(0.75, -0.02, 0.0), Eigen::Vector3d::UnitY()};
  EXPECT_FALSE(
      dart::gui::pickNearestGizmoHandle(gizmos, xHandleRay, 1.0, 0.05)
          .has_value());

  gizmos.front().isVisible = []() {
    return true;
  };
  EXPECT_FALSE(dart::gui::makeGizmoDebugLines(gizmos).empty());
  EXPECT_FALSE(dart::gui::makeGizmoDebugTriangles(gizmos).empty());
  EXPECT_TRUE(
      dart::gui::pickNearestGizmoHandle(gizmos, xHandleRay, 1.0, 0.05)
          .has_value());
}

TEST(FilamentSceneExtraction, FilamentStartupAcceptsGizmoOnlyScenes)
{
  const auto startupSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "scene_startup.cpp");

  EXPECT_NE(
      startupSource.find("makeGizmoDebugLines(dartScene.gizmos)"),
      std::string::npos);
  const auto debugOverlaySource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "debug_overlay.cpp");
  EXPECT_NE(
      debugOverlaySource.find("makeGizmoDebugTriangles"), std::string::npos);
  EXPECT_NE(
      startupSource.find("state.sceneRenderables.empty() && !hasVisibleGizmos"),
      std::string::npos);
}

TEST(FilamentSceneExtraction, GizmoAxisHandlePickingDrivesTargetFrame)
{
  auto target = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "gizmo_target");

  dart::gui::Gizmo gizmo;
  gizmo.label = "axis_target";
  gizmo.target = target;
  gizmo.size = 1.0;

  std::vector<dart::gui::Gizmo> gizmos{gizmo};
  const dart::gui::PickRay xHandleRay{
      Eigen::Vector3d(0.75, -0.02, 0.0), Eigen::Vector3d::UnitY()};
  const auto hit
      = dart::gui::pickNearestGizmoHandle(gizmos, xHandleRay, 1.0, 0.05);

  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->gizmoIndex, 0u);
  EXPECT_EQ(hit->handle, dart::gui::GizmoHandleKind::TranslateX);
  EXPECT_TRUE(hit->axis.isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(hit->point.isApprox(Eigen::Vector3d(0.75, 0.0, 0.0)));

  const dart::gui::PickRay missRay{
      Eigen::Vector3d(0.75, -0.02, 0.20), Eigen::Vector3d::UnitY()};
  EXPECT_FALSE(
      dart::gui::pickNearestGizmoHandle(gizmos, missRay, 1.0, 0.05)
          .has_value());

  const dart::gui::PickRay previousRay{
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, -1.0)};
  const dart::gui::PickRay currentRay{
      Eigen::Vector3d(0.0, 1.0, 1.0), Eigen::Vector3d(1.0, -1.0, -1.0)};
  const auto translation = dart::gui::computeAxisDragTranslation(
      previousRay,
      currentRay,
      target->getWorldTransform().translation(),
      hit->axis);
  ASSERT_TRUE(translation.has_value());
  EXPECT_TRUE(translation->isApprox(Eigen::Vector3d::UnitX()));

  bool callbackCalled = false;
  gizmo.onChanged = [&](const Eigen::Isometry3d& transform) {
    callbackCalled = true;
    EXPECT_TRUE(transform.translation().isApprox(Eigen::Vector3d::UnitX()));
  };

  EXPECT_TRUE(dart::gui::translateGizmoTarget(gizmo, *translation));
  EXPECT_TRUE(target->getWorldTransform().translation().isApprox(
      Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(callbackCalled);

  dart::gui::Gizmo invalid;
  EXPECT_FALSE(
      dart::gui::translateGizmoTarget(invalid, Eigen::Vector3d::UnitX()));
}

TEST(FilamentSceneExtraction, GizmoPlaneHandlePickingDrivesTargetFrame)
{
  auto target = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "gizmo_target");

  dart::gui::Gizmo gizmo;
  gizmo.label = "plane_target";
  gizmo.target = target;
  gizmo.size = 1.0;

  std::vector<dart::gui::Gizmo> gizmos{gizmo};
  const dart::gui::PickRay xyHandleRay{
      Eigen::Vector3d(0.30, 0.30, 1.0), Eigen::Vector3d(0.0, 0.0, -1.0)};
  const auto hit
      = dart::gui::pickNearestGizmoHandle(gizmos, xyHandleRay, 1.0, 0.05);

  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->gizmoIndex, 0u);
  EXPECT_EQ(hit->handle, dart::gui::GizmoHandleKind::TranslateXY);
  EXPECT_TRUE(hit->axis.isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(hit->point.isApprox(Eigen::Vector3d(0.30, 0.30, 0.0)));

  const dart::gui::PickRay previousRay{
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, -1.0)};
  const dart::gui::PickRay currentRay{
      Eigen::Vector3d(1.0, 2.0, 1.0), Eigen::Vector3d(0.0, 0.0, -1.0)};
  const auto translation = dart::gui::computePlaneDragTranslation(
      previousRay, currentRay, hit->point, hit->axis);
  ASSERT_TRUE(translation.has_value());
  EXPECT_TRUE(translation->isApprox(Eigen::Vector3d(1.0, 2.0, 0.0)));

  EXPECT_TRUE(dart::gui::translateGizmoTarget(gizmo, *translation));
  EXPECT_TRUE(target->getWorldTransform().translation().isApprox(
      Eigen::Vector3d(1.0, 2.0, 0.0)));
}

TEST(FilamentSceneExtraction, GizmoRotationHandleDrivesTargetFrame)
{
  auto target = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "gizmo_target");

  dart::gui::Gizmo gizmo;
  gizmo.label = "rotation_target";
  gizmo.target = target;
  gizmo.size = 1.0;

  std::vector<dart::gui::Gizmo> gizmos{gizmo};
  const double ringPoint = 0.70 / std::sqrt(2.0);
  const dart::gui::PickRay xRingRay{
      Eigen::Vector3d(-0.02, ringPoint, ringPoint), Eigen::Vector3d::UnitX()};
  const auto hit
      = dart::gui::pickNearestGizmoHandle(gizmos, xRingRay, 1.0, 0.05);

  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->gizmoIndex, 0u);
  EXPECT_EQ(hit->handle, dart::gui::GizmoHandleKind::RotateX);
  EXPECT_TRUE(hit->axis.isApprox(Eigen::Vector3d::UnitX()));

  bool callbackCalled = false;
  gizmo.onChanged = [&](const Eigen::Isometry3d& transform) {
    callbackCalled = true;
    const Eigen::Matrix3d expected
        = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX()).toRotationMatrix();
    EXPECT_TRUE(transform.linear().isApprox(expected));
  };

  EXPECT_TRUE(dart::gui::rotateGizmoTarget(gizmo, hit->axis, 0.5));
  const Eigen::Matrix3d expected
      = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX()).toRotationMatrix();
  EXPECT_TRUE(target->getWorldTransform().linear().isApprox(expected));
  EXPECT_TRUE(callbackCalled);

  dart::gui::Gizmo invalid;
  EXPECT_FALSE(
      dart::gui::rotateGizmoTarget(invalid, Eigen::Vector3d::UnitX(), 0.5));
}

TEST(FilamentSceneExtraction, ApplicationRunDefaultsSeedParsedOptions)
{
  dart::gui::RunOptions defaults;
  defaults.width = 1280;
  defaults.height = 960;

  char executable[] = "fetch";
  char* defaultArgv[] = {executable};
  const auto defaultOptions = dart::gui::detail::parseOptions(
      1, defaultArgv, std::optional<dart::gui::RunOptions>{defaults});

  EXPECT_EQ(defaultOptions.run.width, 1280);
  EXPECT_EQ(defaultOptions.run.height, 960);

  char overrideExecutable[] = "fetch";
  char widthOption[] = "--width";
  char widthValue[] = "640";
  char heightOption[] = "--height";
  char heightValue[] = "480";
  char* overrideArgv[] = {
      overrideExecutable, widthOption, widthValue, heightOption, heightValue};
  const auto overrideOptions = dart::gui::detail::parseOptions(
      5, overrideArgv, std::optional<dart::gui::RunOptions>{defaults});

  EXPECT_EQ(overrideOptions.run.width, 640);
  EXPECT_EQ(overrideOptions.run.height, 480);

  char depthExecutable[] = "fetch";
  char renderOutputOption[] = "--render-output";
  char renderOutputValue[] = "depth";
  char* depthArgv[] = {depthExecutable, renderOutputOption, renderOutputValue};
  const auto depthOptions = dart::gui::detail::parseOptions(
      3, depthArgv, std::optional<dart::gui::RunOptions>{defaults});

  EXPECT_TRUE(depthOptions.renderOutputModeExplicit);
  EXPECT_EQ(
      depthOptions.renderSettings.outputMode,
      dart::gui::RenderOutputMode::Depth);
}

TEST(FilamentSceneExtraction, ApplicationOptionsStoresKeyboardActions)
{
  dart::gui::ApplicationOptions options;
  dart::gui::KeyboardAction action;
  action.label = "clear";
  action.shortcut
      = dart::gui::KeyboardShortcut::namedKey(dart::gui::KeyboardKey::Delete);
  action.trigger = dart::gui::KeyboardActionTrigger::Release;
  action.callback = [](dart::gui::KeyboardActionContext& context) {
    if (context.lifecycle != nullptr) {
      context.lifecycle->paused = true;
    }
    if (context.renderSettings != nullptr) {
      context.renderSettings->shadowsEnabled = false;
    }
  };

  options.keyboardActions.push_back(std::move(action));

  ASSERT_EQ(options.keyboardActions.size(), 1u);
  EXPECT_EQ(options.keyboardActions.front().label, "clear");
  EXPECT_EQ(
      options.keyboardActions.front().shortcut.key,
      dart::gui::KeyboardKey::Delete);
  EXPECT_EQ(
      options.keyboardActions.front().trigger,
      dart::gui::KeyboardActionTrigger::Release);
  dart::gui::ViewerLifecycleState lifecycle;
  dart::gui::RenderSettings renderSettings;
  bool cameraReset = false;
  dart::gui::KeyboardActionContext context;
  context.lifecycle = &lifecycle;
  context.renderSettings = &renderSettings;
  context.resetCamera = [&cameraReset]() {
    cameraReset = true;
  };
  options.keyboardActions.front().callback(context);
  EXPECT_TRUE(lifecycle.paused);
  EXPECT_FALSE(renderSettings.shadowsEnabled);
  context.resetCamera();
  EXPECT_TRUE(cameraReset);
}

TEST(FilamentSceneExtraction, RestoredExamplesUsePromotedGuiBoundary)
{
  struct ExampleExpectation
  {
    std::filesystem::path directory;
    bool usesPanel = false;
    bool ownsWorld = true;
  };

  const std::vector<ExampleExpectation> examples = {
      {std::filesystem::path("examples") / "hello_world", true},
      {std::filesystem::path("examples") / "boxes", false},
      {std::filesystem::path("examples") / "rigid_cubes", true},
      {std::filesystem::path("examples") / "box_stacking", true},
      {std::filesystem::path("examples") / "simple_frames", false},
      {std::filesystem::path("examples") / "capsule_ground_contact", true},
      {std::filesystem::path("examples") / "fetch", true},
      {std::filesystem::path("examples") / "rigid_chain", true},
      {std::filesystem::path("examples") / "rigid_loop", true},
      {std::filesystem::path("examples") / "mixed_chain", true},
      {std::filesystem::path("examples") / "coupler_constraint", true},
      {std::filesystem::path("examples") / "add_delete_skels", true},
      {std::filesystem::path("examples") / "rigid_shapes", true},
      {std::filesystem::path("examples") / "hybrid_dynamics", true},
      {std::filesystem::path("examples") / "biped_stand", true},
      {std::filesystem::path("examples") / "joint_constraints", true},
      {std::filesystem::path("examples") / "free_joint_cases", true},
      {std::filesystem::path("examples") / "human_joint_limits", true},
      {std::filesystem::path("examples") / "imgui", true},
      {std::filesystem::path("examples") / "drag_and_drop", true},
      {std::filesystem::path("examples") / "tinkertoy", true},
      {std::filesystem::path("examples") / "operational_space_control", true},
      {std::filesystem::path("examples") / "wam_ikfast", true},
      {std::filesystem::path("examples") / "atlas_simbicon", true},
      {std::filesystem::path("examples") / "atlas_puppet", true},
      {std::filesystem::path("examples") / "hubo_puppet", true},
      {std::filesystem::path("examples") / "g1_puppet", true},
      {std::filesystem::path("examples") / "hardcoded_design", true},
      {std::filesystem::path("examples") / "heightmap", true},
      {std::filesystem::path("examples") / "point_cloud", true},
      {std::filesystem::path("examples") / "polyhedron_visual", false},
      {std::filesystem::path("examples") / "lcp_physics", true},
      {std::filesystem::path("examples") / "mimic_pendulums", true},
      {std::filesystem::path("examples") / "empty", true},
      {std::filesystem::path("examples") / "simulation_event_handler", true},
      {std::filesystem::path("examples") / "soft_bodies", true},
      {std::filesystem::path("examples") / "vehicle", true}};
  std::vector<std::filesystem::path> sources;
  for (const auto& example : examples) {
    sources.push_back(example.directory / "main.cpp");
    sources.push_back(example.directory / "CMakeLists.txt");
  }

  const auto backendViolations
      = scanSourceFilesForTokens(sources, kForbiddenBackendTokens);
  for (const auto& violation : backendViolations) {
    ADD_FAILURE() << violation.source << " reaches backend token `"
                  << violation.token << "` directly";
  }

  const auto promotedViolations
      = scanSourceFilesForTokens(sources, kForbiddenPromotedGuiTokens);
  for (const auto& violation : promotedViolations) {
    ADD_FAILURE() << violation.source << " reaches old GUI token `"
                  << violation.token << "`";
  }

  for (const auto& example : examples) {
    const auto mainSource = readSourceFile(example.directory / "main.cpp");
    EXPECT_NE(
        mainSource.find("#include <dart/gui/application.hpp>"),
        std::string::npos);
    EXPECT_NE(
        mainSource.find("dart::gui::ApplicationOptions"), std::string::npos);
    EXPECT_NE(
        mainSource.find("dart::gui::runApplication(argc, argv, options)"),
        std::string::npos);
    if (example.usesPanel) {
      EXPECT_NE(
          mainSource.find("#include <dart/gui/panel.hpp>"), std::string::npos);
      EXPECT_NE(mainSource.find("dart::gui::Panel"), std::string::npos);
    }
    if (example.ownsWorld) {
      EXPECT_NE(mainSource.find("options.world"), std::string::npos);
      EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
    }
  }
}

TEST(FilamentSceneExtraction, CsvLoggerPreservesNonGuiLoggingContract)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "csv_logger" / "main.cpp");
  const auto cmakeSource = readSourceFile(
      std::filesystem::path("examples") / "csv_logger" / "CMakeLists.txt");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "csv_logger" / "README.md");

  EXPECT_NE(
      mainSource.find("dart://sample/sdf/double_pendulum.world"),
      std::string::npos);
  EXPECT_NE(mainSource.find("simulation_log.csv"), std::string::npos);
  EXPECT_NE(mainSource.find("std::size_t steps = 1000"), std::string::npos);
  EXPECT_NE(mainSource.find("double timeStep = 0.001"), std::string::npos);
  EXPECT_NE(mainSource.find("--world <uri>"), std::string::npos);
  EXPECT_NE(mainSource.find("--output <path>"), std::string::npos);
  EXPECT_NE(mainSource.find("--steps <n>"), std::string::npos);
  EXPECT_NE(mainSource.find("--dt <seconds>"), std::string::npos);
  EXPECT_NE(mainSource.find("--skeleton <name>"), std::string::npos);
  EXPECT_NE(mainSource.find("--body <name>"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::io::readWorld"), std::string::npos);
  EXPECT_NE(mainSource.find("world->setTimeStep"), std::string::npos);
  EXPECT_NE(mainSource.find("world->getSkeleton(0)"), std::string::npos);
  EXPECT_NE(mainSource.find("skeleton->getBodyNode(0)"), std::string::npos);
  EXPECT_NE(mainSource.find("time,com_x,com_y,com_z,"), std::string::npos);
  EXPECT_NE(
      mainSource.find("body_qw,body_qx,body_qy,body_qz"), std::string::npos);
  EXPECT_NE(mainSource.find("skeleton->getCOM()"), std::string::npos);
  EXPECT_NE(
      mainSource.find("bodyNode->getWorldTransform()"), std::string::npos);
  EXPECT_NE(mainSource.find("logRow();"), std::string::npos);
  EXPECT_NE(mainSource.find("world->step()"), std::string::npos);
  EXPECT_NE(mainSource.find("options.steps + 1"), std::string::npos);

  EXPECT_NE(readmeSource.find("CSV Logger Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("external tooling"), std::string::npos);
  EXPECT_NE(readmeSource.find("Pass `--help`"), std::string::npos);

  EXPECT_EQ(mainSource.find("dart/gui"), std::string::npos);
  EXPECT_EQ(mainSource.find("ApplicationOptions"), std::string::npos);
  EXPECT_EQ(cmakeSource.find("dart-gui"), std::string::npos);
}

TEST(FilamentSceneExtraction, HeadlessSimulationPreservesNonGuiContract)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "headless_simulation" / "main.cpp");
  const auto cmakeSource = readSourceFile(
      std::filesystem::path("examples") / "headless_simulation"
      / "CMakeLists.txt");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "headless_simulation" / "README.md");

  EXPECT_NE(
      mainSource.find("dart://sample/sdf/double_pendulum.world"),
      std::string::npos);
  EXPECT_NE(mainSource.find("std::size_t steps = 1000"), std::string::npos);
  EXPECT_NE(mainSource.find("double timeStep = 0.001"), std::string::npos);
  EXPECT_NE(mainSource.find("unsigned int seed = 42"), std::string::npos);
  EXPECT_NE(mainSource.find("--world <uri>"), std::string::npos);
  EXPECT_NE(mainSource.find("--steps <n>"), std::string::npos);
  EXPECT_NE(mainSource.find("--dt <seconds>"), std::string::npos);
  EXPECT_NE(mainSource.find("--seed <n>"), std::string::npos);
  EXPECT_NE(mainSource.find("parseUnsignedInt"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart::math::Random::setSeed(options.seed)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("dart::io::readWorld"), std::string::npos);
  EXPECT_NE(mainSource.find("world->setTimeStep"), std::string::npos);
  EXPECT_NE(mainSource.find("world->getNumSkeletons()"), std::string::npos);
  EXPECT_NE(mainSource.find("Time step: "), std::string::npos);
  EXPECT_NE(mainSource.find("Steps: "), std::string::npos);
  EXPECT_NE(mainSource.find("world->step()"), std::string::npos);
  EXPECT_NE(mainSource.find("Simulated time: "), std::string::npos);
  EXPECT_NE(mainSource.find("Elapsed wall time: "), std::string::npos);

  EXPECT_NE(
      readmeSource.find("Headless Simulation Example"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("deterministic, headless simulation"),
      std::string::npos);
  EXPECT_NE(readmeSource.find("Pass `--help`"), std::string::npos);

  EXPECT_EQ(mainSource.find("dart/gui"), std::string::npos);
  EXPECT_EQ(mainSource.find("ApplicationOptions"), std::string::npos);
  EXPECT_EQ(cmakeSource.find("dart-gui"), std::string::npos);
}

TEST(FilamentSceneExtraction, SpeedTestPreservesNonGuiBenchmarkContract)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "speed_test" / "main.cpp");
  const auto cmakeSource = readSourceFile(
      std::filesystem::path("examples") / "speed_test" / "CMakeLists.txt");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "speed_test" / "README.md");

  EXPECT_NE(mainSource.find("testForwardKinematicSpeed"), std::string::npos);
  EXPECT_NE(
      mainSource.find("std::size_t numTests = 100000"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::math::Random::uniform"), std::string::npos);
  EXPECT_NE(mainSource.find("getWorldTransform()"), std::string::npos);
  EXPECT_NE(mainSource.find("getSpatialVelocity()"), std::string::npos);
  EXPECT_NE(mainSource.find("getPartialAcceleration()"), std::string::npos);
  EXPECT_NE(mainSource.find("getSpatialAcceleration()"), std::string::npos);
  EXPECT_NE(mainSource.find("testDynamicsSpeed"), std::string::npos);
  EXPECT_NE(
      mainSource.find("std::size_t numIterations = 10000"), std::string::npos);
  EXPECT_NE(mainSource.find("resetPositions()"), std::string::npos);
  EXPECT_NE(mainSource.find("resetVelocities()"), std::string::npos);
  EXPECT_NE(mainSource.find("resetAccelerations()"), std::string::npos);
  EXPECT_NE(mainSource.find("world->step()"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart://sample/skel/test/chainwhipa.skel"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("serial_chain_ball_joint_40.skel"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart://sample/skel/fullbody1.skel"), std::string::npos);
  EXPECT_NE(
      mainSource.find("std::string(argv[i]) == \"-k\""), std::string::npos);
  EXPECT_NE(mainSource.find("Testing Kinematics"), std::string::npos);
  EXPECT_NE(mainSource.find("Testing Dynamics"), std::string::npos);
  EXPECT_NE(mainSource.find("Trial #"), std::string::npos);
  EXPECT_NE(mainSource.find("Final Kinematics Results"), std::string::npos);
  EXPECT_NE(mainSource.find("Final Dynamics Results"), std::string::npos);
  EXPECT_NE(mainSource.find("Average: "), std::string::npos);
  EXPECT_NE(mainSource.find("Std Dev: "), std::string::npos);

  EXPECT_NE(readmeSource.find("Speed Test Example"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("time kinematics and dynamics"), std::string::npos);
  EXPECT_NE(readmeSource.find("Controls: none"), std::string::npos);

  EXPECT_EQ(mainSource.find("dart/gui"), std::string::npos);
  EXPECT_EQ(mainSource.find("ApplicationOptions"), std::string::npos);
  EXPECT_EQ(cmakeSource.find("dart-gui"), std::string::npos);
}

TEST(FilamentSceneExtraction, UnifiedLoadingPreservesNonGuiReadOptionsContract)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "unified_loading" / "main.cpp");
  const auto cmakeSource = readSourceFile(
      std::filesystem::path("examples") / "unified_loading" / "CMakeLists.txt");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "unified_loading" / "README.md");

  EXPECT_NE(
      mainSource.find("dart://sample/sdf/double_pendulum.world"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("dart://sample/urdf/test/primitive_geometry.urdf"),
      std::string::npos);
  EXPECT_NE(mainSource.find("dart::io::ModelFormat::Auto"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart::io::RootJointType::Floating"), std::string::npos);
  EXPECT_NE(mainSource.find("--world <uri>"), std::string::npos);
  EXPECT_NE(mainSource.find("--skeleton <uri>"), std::string::npos);
  EXPECT_NE(
      mainSource.find("--format <auto|skel|sdf|urdf|mjcf>"), std::string::npos);
  EXPECT_NE(
      mainSource.find("--sdf-root-joint <floating|fixed>"), std::string::npos);
  EXPECT_NE(mainSource.find("--package <name=path>"), std::string::npos);
  EXPECT_NE(mainSource.find("--no-world"), std::string::npos);
  EXPECT_NE(mainSource.find("--no-skeleton"), std::string::npos);
  EXPECT_NE(mainSource.find("parseModelFormat"), std::string::npos);
  EXPECT_NE(mainSource.find("parseRootJointType"), std::string::npos);
  EXPECT_NE(mainSource.find("parsePackageMapping"), std::string::npos);
  EXPECT_NE(mainSource.find("Nothing to load."), std::string::npos);
  EXPECT_NE(mainSource.find("dart::io::ReadOptions"), std::string::npos);
  EXPECT_NE(
      mainSource.find("readOptions.addPackageDirectory"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::io::readWorld"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::io::readSkeleton"), std::string::npos);
  EXPECT_NE(mainSource.find("Read options: format="), std::string::npos);
  EXPECT_NE(mainSource.find("World loaded with "), std::string::npos);
  EXPECT_NE(mainSource.find("Skeleton '"), std::string::npos);

  EXPECT_NE(readmeSource.find("Unified Loading Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::io::ReadOptions"), std::string::npos);
  EXPECT_NE(readmeSource.find("format inference"), std::string::npos);
  EXPECT_NE(readmeSource.find("Pass `--help`"), std::string::npos);

  EXPECT_EQ(mainSource.find("dart/gui"), std::string::npos);
  EXPECT_EQ(mainSource.find("ApplicationOptions"), std::string::npos);
  EXPECT_EQ(cmakeSource.find("dart-gui"), std::string::npos);
}

TEST(FilamentSceneExtraction, HelloWorldExamplePreservesParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "hello_world" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "hello_world" / "README.md");

  EXPECT_NE(mainSource.find("DART_PROFILE_SCOPED_N"), std::string::npos);
  EXPECT_NE(mainSource.find("DART_PROFILE_TEXT_DUMP"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Press space to start free falling the box."),
      std::string::npos);
  EXPECT_NE(mainSource.find("Eigen::AngleAxisd(0.35"), std::string::npos);
  EXPECT_NE(mainSource.find("createHelloWorldPanel"), std::string::npos);
  EXPECT_NE(mainSource.find("makeHelloWorldRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(mainSource.find("makeHelloWorldCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d(0.0, 0.0, 0.50)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.up = Eigen::Vector3d(-0.24, -0.25, 0.94)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("options.panels"), std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_NE(readmeSource.find("Hello World Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(readmeSource.find("640x480"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
}

TEST(FilamentSceneExtraction, CapsuleGroundContactPreservesParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "capsule_ground_contact"
      / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "capsule_ground_contact"
      / "README.md");

  EXPECT_NE(mainSource.find("DART_HAVE_ODE"), std::string::npos);
  EXPECT_NE(mainSource.find("CollisionDetectorType::Ode"), std::string::npos);
  EXPECT_NE(mainSource.find("PlaneShape"), std::string::npos);
  EXPECT_NE(mainSource.find("CapsuleShape"), std::string::npos);
  EXPECT_NE(mainSource.find("capsule_joint"), std::string::npos);
  EXPECT_NE(mainSource.find("capsule_body"), std::string::npos);
  EXPECT_NE(mainSource.find("persistent manifolds"), std::string::npos);
  EXPECT_NE(mainSource.find("clearCapsuleVelocities"), std::string::npos);
  EXPECT_NE(mainSource.find("createCapsuleKeyboardActions"), std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey('h')"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey('v')"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey(' ')"),
      std::string::npos);
  EXPECT_NE(mainSource.find("makeCapsuleRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 1024"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 768"), std::string::npos);
  EXPECT_NE(mainSource.find("makeCapsuleCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d(0.0, 0.0, 0.2)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.up = Eigen::Vector3d(-0.2, -0.2, 0.95)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("Capsule Ground Contact Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(readmeSource.find("1024x768"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
}

TEST(FilamentSceneExtraction, RigidChainExamplePreservesParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "rigid_chain" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "rigid_chain" / "README.md");

  EXPECT_NE(
      mainSource.find("dart://sample/skel/chain.skel"), std::string::npos);
  EXPECT_NE(
      mainSource.find("world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0))"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("world->setTimeStep(1.0 / 2000.0)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart::math::Random::uniform(-0.5, 0.5)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("applyChainDamping"), std::string::npos);
  EXPECT_NE(mainSource.find("damping[i] *= 0.1"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("makeRigidChainRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(mainSource.find("makeRigidChainCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d::Zero()"),
      std::string::npos);
  EXPECT_NE(mainSource.find("camera.distance = 3.0"), std::string::npos);
  EXPECT_NE(readmeSource.find("Rigid Chain Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(readmeSource.find("640x480"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("RealTimeWorldNode"), std::string::npos);
  EXPECT_EQ(mainSource.find("dart::gui::Viewer"), std::string::npos);
  EXPECT_EQ(mainSource.find("::osg"), std::string::npos);
}

TEST(FilamentSceneExtraction, RigidLoopExamplePreservesParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "rigid_loop" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "rigid_loop" / "README.md");

  EXPECT_NE(
      mainSource.find("dart://sample/skel/chain.skel"), std::string::npos);
  EXPECT_NE(
      mainSource.find("world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0))"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("world->setTimeStep(1.0 / 2000.0)"), std::string::npos);
  EXPECT_NE(mainSource.find("std::array{20, 23, 26, 29}"), std::string::npos);
  EXPECT_NE(mainSource.find("0.4 * dart::math::pi"), std::string::npos);
  EXPECT_NE(mainSource.find("BallJointConstraint"), std::string::npos);
  EXPECT_NE(mainSource.find("link 6"), std::string::npos);
  EXPECT_NE(mainSource.find("link 10"), std::string::npos);
  EXPECT_NE(
      mainSource.find("link6->setColor(Eigen::Vector3d(1.0, 0.0, 0.0))"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("link10->setColor(Eigen::Vector3d(1.0, 0.0, 0.0))"),
      std::string::npos);
  EXPECT_NE(mainSource.find("printRigidLoopInstructions"), std::string::npos);
  EXPECT_NE(mainSource.find("Rigid Loop Chain Simulation"), std::string::npos);
  EXPECT_NE(mainSource.find("applyChainDamping"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("makeRigidLoopRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(readmeSource.find("Rigid Loop Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(readmeSource.find("640x480"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("WorldNode"), std::string::npos);
  EXPECT_EQ(mainSource.find("dart::gui::Viewer"), std::string::npos);
  EXPECT_EQ(mainSource.find("::osg"), std::string::npos);
}

TEST(FilamentSceneExtraction, RigidCubesExamplePreservesParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "rigid_cubes" / "main.cpp");
  const auto cmakeSource = readSourceFile(
      std::filesystem::path("examples") / "rigid_cubes" / "CMakeLists.txt");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "rigid_cubes" / "README.md");

  EXPECT_NE(
      mainSource.find("dart://sample/skel/cubes.skel"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::io::readWorld"), std::string::npos);
  EXPECT_NE(
      mainSource.find("world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0))"),
      std::string::npos);
  EXPECT_NE(mainSource.find("applyPendingForce"), std::string::npos);
  EXPECT_NE(mainSource.find("state->pendingForce /= 2.0"), std::string::npos);
  EXPECT_NE(mainSource.find("getSkeleton(1)"), std::string::npos);
  EXPECT_NE(mainSource.find("printRigidCubesInstructions"), std::string::npos);
  EXPECT_NE(mainSource.find("Rigid Cubes Example"), std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey('p')"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey('v')"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey('1')"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey('2')"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey('3')"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey('4')"),
      std::string::npos);
  EXPECT_NE(mainSource.find("makeRigidCubesRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(mainSource.find("makeRigidCubesCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d::Zero()"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.distance = 8.660254037844387"),
      std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(cmakeSource.find("dart-io"), std::string::npos);
  EXPECT_NE(readmeSource.find("Rigid Cubes Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(readmeSource.find("--out"), std::string::npos);
  EXPECT_NE(readmeSource.find("640x480"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("RealTimeWorldNode"), std::string::npos);
  EXPECT_EQ(mainSource.find("dart::gui::Viewer"), std::string::npos);
  EXPECT_EQ(mainSource.find("viewer.record"), std::string::npos);
  EXPECT_EQ(mainSource.find("::osg"), std::string::npos);
}

TEST(FilamentSceneExtraction, BoxesExamplePreservesParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "boxes" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "boxes" / "README.md");

  EXPECT_NE(mainSource.find("#if DART_HAVE_BULLET"), std::string::npos);
  EXPECT_NE(
      mainSource.find("CollisionDetectorType::Bullet"), std::string::npos);
  EXPECT_NE(mainSource.find("Skeleton::create(\"box\""), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(25.0, 25.0, 0.1)"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::Color::LightGray()"), std::string::npos);
  EXPECT_EQ(mainSource.find("[Experimental] Please note"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Press space to start free falling the box."),
      std::string::npos);
  EXPECT_NE(mainSource.find("createBoxesInstructionsPanel"), std::string::npos);
  EXPECT_NE(mainSource.find("makeBoxesRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 1360"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 768"), std::string::npos);
  EXPECT_NE(mainSource.find("makeBoxesCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d(0.0, 0.0, 3.0)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_NE(readmeSource.find("Boxes Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(readmeSource.find("1360x768"), std::string::npos);
  EXPECT_NE(readmeSource.find("Build Instructions"), std::string::npos);
  EXPECT_NE(readmeSource.find("Execute Instructions"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("::osg"), std::string::npos);
}

TEST(FilamentSceneExtraction, SimpleFramesExamplePreservesParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "simple_frames" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "simple_frames" / "README.md");

  EXPECT_NE(mainSource.find("\"F1\""), std::string::npos);
  EXPECT_NE(mainSource.find("\"F2\""), std::string::npos);
  EXPECT_NE(mainSource.find("\"F3\""), std::string::npos);
  EXPECT_NE(mainSource.find("\"A\""), std::string::npos);
  EXPECT_NE(mainSource.find("\"A1\""), std::string::npos);
  EXPECT_NE(mainSource.find("\"A2\""), std::string::npos);
  EXPECT_NE(mainSource.find("\"A3\""), std::string::npos);
  EXPECT_NE(mainSource.find("ArrowShape"), std::string::npos);
  EXPECT_NE(
      mainSource.find("ArrowShape::Properties(0.002, 1.8)"), std::string::npos);
  EXPECT_NE(mainSource.find("makeSimpleFramesRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(mainSource.find("makeSimpleFramesCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d(0.0, 0.0, 0.0)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_NE(readmeSource.find("Simple Frames Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(readmeSource.find("ArrowShape"), std::string::npos);
  EXPECT_NE(readmeSource.find("Build Instructions"), std::string::npos);
  EXPECT_NE(readmeSource.find("Execute Instructions"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("LineSegmentShape"), std::string::npos);
  EXPECT_EQ(mainSource.find("::osg"), std::string::npos);
}

TEST(FilamentSceneExtraction, DragAndDropExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "drag_and_drop" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "drag_and_drop" / "README.md");

  EXPECT_EQ(mainSource.find("createDragFrameHandleShape"), std::string::npos);
  EXPECT_EQ(mainSource.find("LineSegmentShape"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(mainSource.find("options.gizmos"), std::string::npos);
  EXPECT_NE(mainSource.find("gizmo.size = 2.0"), std::string::npos);
  EXPECT_NE(mainSource.find("interactive frame"), std::string::npos);
  EXPECT_NE(mainSource.find("draggable"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(-4.0, 4.0, 0.0)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(8.0, 0.0, 0.0)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(0.0, 8.0, 0.0)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(0.0, 0.0, 8.0)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(0.2, 0.2, 0.2)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Left-drag gizmo arrows, planes, and rings"),
      std::string::npos);
  EXPECT_EQ(mainSource.find("Ctrl-left drag"), std::string::npos);
  EXPECT_NE(mainSource.find("makeDragAndDropRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(mainSource.find("makeDragAndDropCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d(0.0, 0.0, 0.0)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.camera"), std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_NE(readmeSource.find("Drag and Drop Example"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("dart::gui::ApplicationOptions"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(readmeSource.find("rotation-ring dragging"), std::string::npos);
  EXPECT_NE(readmeSource.find("640x480"), std::string::npos);
  EXPECT_EQ(readmeSource.find("Ctrl-left drag"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("InteractiveFrame"), std::string::npos);
  EXPECT_EQ(mainSource.find("WorldNode"), std::string::npos);
  EXPECT_EQ(mainSource.find("enableDragAndDrop"), std::string::npos);
}

TEST(FilamentSceneExtraction, BoxStackingExamplePreservesParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "box_stacking" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "box_stacking" / "README.md");

  EXPECT_NE(mainSource.find("DantzigSolver"), std::string::npos);
  EXPECT_NE(mainSource.find("PgsSolver"), std::string::npos);
  EXPECT_NE(mainSource.find("--solver"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Random::uniform<Eigen::Vector3d>"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(0.0, 0.0, 0.5 + 0.25 + i * 0.5)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("floorHeight = 0.01"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::Color::LightGray()"), std::string::npos);
  EXPECT_NE(mainSource.find("setSplitImpulseEnabled"), std::string::npos);
  EXPECT_NE(mainSource.find("Box stacking demo"), std::string::npos);
  EXPECT_NE(mainSource.find("Gravity On/Off"), std::string::npos);
  EXPECT_NE(mainSource.find("Headlights On/Off"), std::string::npos);
  EXPECT_NE(
      mainSource.find("context.lighting.headlightsEnabled"), std::string::npos);
  EXPECT_NE(mainSource.find("LCP solver:"), std::string::npos);
  EXPECT_NE(
      mainSource.find("createBoxStackingKeyboardActions"), std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardActionTrigger::Release"), std::string::npos);
  EXPECT_NE(mainSource.find("Capital Q released"), std::string::npos);
  EXPECT_NE(mainSource.find("context.camera.eye"), std::string::npos);
  EXPECT_NE(mainSource.find("Eye   : "), std::string::npos);
  EXPECT_NE(mainSource.find("Center: "), std::string::npos);
  EXPECT_NE(mainSource.find("Up    : "), std::string::npos);
  EXPECT_NE(mainSource.find("User Guide:"), std::string::npos);
  EXPECT_NE(mainSource.find("makeBoxStackingRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 800"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("makeBoxStackingCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d(0.0, 0.0, 2.0)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_NE(readmeSource.find("Box Stacking Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(readmeSource.find("800x640"), std::string::npos);
  EXPECT_NE(readmeSource.find("keydown/key-release"), std::string::npos);
  EXPECT_NE(readmeSource.find("--gui-scale"), std::string::npos);
  EXPECT_NE(readmeSource.find("Build Instructions"), std::string::npos);
  EXPECT_NE(readmeSource.find("Execute Instructions"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("WorldNode"), std::string::npos);
  EXPECT_EQ(mainSource.find("ImGuiViewer"), std::string::npos);
}

TEST(FilamentSceneExtraction, FetchExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "fetch" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "fetch" / "README.md");
  const auto interactionHeader = readSourceFile(
      std::filesystem::path("dart") / "gui" / "interaction.hpp");
  const auto panelHeader
      = readSourceFile(std::filesystem::path("dart") / "gui" / "panel.hpp");
  const auto selectionSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "selection.cpp");
  const auto panelSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "panel.cpp");

  EXPECT_NE(mainSource.find("#if DART_HAVE_BULLET"), std::string::npos);
  EXPECT_NE(
      mainSource.find("CollisionDetectorType::Bullet"), std::string::npos);
  EXPECT_NE(mainSource.find("interactive frame"), std::string::npos);
  EXPECT_NE(mainSource.find("LineSegmentShape"), std::string::npos);
  EXPECT_NE(mainSource.find("MeshShape"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::math::TriMesh<double>"), std::string::npos);
  EXPECT_NE(mainSource.find("createTargetCrossShape"), std::string::npos);
  EXPECT_NE(mainSource.find("addBoxToMesh"), std::string::npos);
  EXPECT_NE(mainSource.find("two transparent green bars"), std::string::npos);
  EXPECT_NE(mainSource.find("MeshShape::SHAPE_ALPHA"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(mainSource.find("options.gizmos"), std::string::npos);
  EXPECT_NE(mainSource.find("targetGizmo.size = 0.24"), std::string::npos);
  EXPECT_NE(mainSource.find("kTargetRotationStep"), std::string::npos);
  EXPECT_NE(mainSource.find("rotateFetchTarget"), std::string::npos);
  EXPECT_NE(mainSource.find("createFetchKeyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("fetch_pick_and_place_grid"), std::string::npos);
  EXPECT_NE(mainSource.find("createFetchGridShape"), std::string::npos);
  EXPECT_NE(mainSource.find("createFetchGrid"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::dynamics::WeldJoint"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(1.3, 0.75, 0.0)"), std::string::npos);
  EXPECT_NE(mainSource.find("options.camera"), std::string::npos);
  EXPECT_NE(mainSource.find("makeFetchCamera"), std::string::npos);
  EXPECT_NE(mainSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("makeFetchRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 1280"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 960"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d(0.1, -0.3, 0.3)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("syncMocapTarget"), std::string::npos);
  EXPECT_NE(mainSource.find("Fetch robot example"), std::string::npos);
  EXPECT_NE(mainSource.find("initialPosition"), std::string::npos);
  EXPECT_NE(
      mainSource.find("std::array<double, 2>{10.0, 20.0}"), std::string::npos);
  EXPECT_NE(
      mainSource.find("std::array<double, 2>{360.0, 600.0}"),
      std::string::npos);
  EXPECT_NE(mainSource.find("backgroundAlpha = 0.5"), std::string::npos);
  EXPECT_NE(mainSource.find("horizontalScrollbar = true"), std::string::npos);
  EXPECT_NE(mainSource.find("menuBar = true"), std::string::npos);
  EXPECT_NE(mainSource.find("builder.beginMenuBar()"), std::string::npos);
  EXPECT_NE(mainSource.find("builder.beginMenu(\"Menu\")"), std::string::npos);
  EXPECT_NE(mainSource.find("builder.menuItem(\"Exit\")"), std::string::npos);
  EXPECT_NE(mainSource.find("builder.beginMenu(\"Help\")"), std::string::npos);
  EXPECT_NE(
      mainSource.find("builder.menuItem(\"About DART\")"), std::string::npos);
  EXPECT_NE(
      mainSource.find("builder.collapsingHeader(\"Help\")"), std::string::npos);
  EXPECT_NE(
      mainSource.find("builder.collapsingHeader(\"Simulation\", true)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("User Guid:"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Point cloud and voxel grid rendering example"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("whole body motion of the Fetch robot"),
      std::string::npos);
  EXPECT_NE(mainSource.find("Left drag orbits"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Left-drag target gizmo arrows/planes/rings"),
      std::string::npos);
  EXPECT_NE(mainSource.find("public target gizmo handles"), std::string::npos);
  EXPECT_EQ(mainSource.find("Ctrl-left drag"), std::string::npos);
  EXPECT_NE(mainSource.find("u/j, i/k, o/l rotate"), std::string::npos);
  EXPECT_NE(mainSource.find("builder.button(\"Play\")"), std::string::npos);
  EXPECT_NE(mainSource.find("builder.button(\"Pause\")"), std::string::npos);
  EXPECT_NE(mainSource.find("builder.button(\"Exit\")"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::gui::requestExit"), std::string::npos);
  EXPECT_NE(mainSource.find("About DART"), std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("Rotate target +X"), std::string::npos);
  EXPECT_NE(mainSource.find("Reset target pose"), std::string::npos);
  EXPECT_NE(
      interactionHeader.find("rotateSimpleFrameRenderable"), std::string::npos);
  EXPECT_NE(panelHeader.find("initialPosition"), std::string::npos);
  EXPECT_NE(panelHeader.find("initialSize"), std::string::npos);
  EXPECT_NE(panelHeader.find("backgroundAlpha"), std::string::npos);
  EXPECT_NE(panelHeader.find("collapsingHeader"), std::string::npos);
  EXPECT_NE(panelHeader.find("beginMenuBar"), std::string::npos);
  EXPECT_NE(panelHeader.find("menuItem"), std::string::npos);
  EXPECT_NE(panelHeader.find("openModal"), std::string::npos);
  EXPECT_NE(panelHeader.find("beginModal"), std::string::npos);
  EXPECT_NE(
      selectionSource.find("rotateRenderableAndApplyIk"), std::string::npos);
  EXPECT_NE(
      selectionSource.find("isRotationDragModifierDown"), std::string::npos);
  EXPECT_NE(panelSource.find("ImGui::SetNextWindowPos"), std::string::npos);
  EXPECT_NE(panelSource.find("ImGui::SetNextWindowSize"), std::string::npos);
  EXPECT_NE(
      panelSource.find("ImGuiWindowFlags_HorizontalScrollbar"),
      std::string::npos);
  EXPECT_NE(panelSource.find("ImGuiWindowFlags_MenuBar"), std::string::npos);
  EXPECT_NE(panelSource.find("ImGui::BeginMenuBar"), std::string::npos);
  EXPECT_NE(panelSource.find("ImGui::CollapsingHeader"), std::string::npos);
  EXPECT_NE(readmeSource.find("Fetch MJCF Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("two transparent green bars"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("target gizmo arrows/planes/rings"), std::string::npos);
  EXPECT_NE(readmeSource.find("public target gizmo"), std::string::npos);
  EXPECT_EQ(readmeSource.find("Ctrl-left drag"), std::string::npos);
  EXPECT_EQ(readmeSource.find("Remaining gap"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("dart::gui::ApplicationOptions"), std::string::npos);
  EXPECT_NE(readmeSource.find("1280x960"), std::string::npos);
  EXPECT_NE(readmeSource.find("--out"), std::string::npos);
  EXPECT_NE(readmeSource.find("Build Instructions"), std::string::npos);
  EXPECT_NE(readmeSource.find("Execute Instructions"), std::string::npos);
  EXPECT_NE(readmeSource.find("./fetch"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(
      mainSource.find("setGravity(Eigen::Vector3d::Zero())"),
      std::string::npos);
}

TEST(
    FilamentSceneExtraction,
    CouplerConstraintExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "coupler_constraint" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "coupler_constraint" / "README.md");

  EXPECT_NE(mainSource.find("CouplerController"), std::string::npos);
  EXPECT_NE(mainSource.find("driveReferenceJoint"), std::string::npos);
  EXPECT_NE(mainSource.find("refreshPairVisual"), std::string::npos);
  EXPECT_NE(mainSource.find("getMimicDofProperties"), std::string::npos);
  EXPECT_NE(mainSource.find("setUseCouplerConstraint"), std::string::npos);
  EXPECT_NE(mainSource.find("torqueLimit = 90.0"), std::string::npos);
  EXPECT_NE(mainSource.find("proportionalGain = 320.0"), std::string::npos);
  EXPECT_NE(mainSource.find("dampingGain = 25.0"), std::string::npos);
  EXPECT_NE(mainSource.find("coupler_constraint_grid"), std::string::npos);
  EXPECT_NE(mainSource.find("cellCount = 25"), std::string::npos);
  EXPECT_NE(mainSource.find("step = 0.05"), std::string::npos);
  EXPECT_NE(mainSource.find("createCouplerKeyboardActions"), std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey('r')"),
      std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("makeCouplerCamera"), std::string::npos);
  EXPECT_NE(mainSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 960"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 720"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d(0.4, 0.0, 0.2)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("Coupler pair (left)"), std::string::npos);
  EXPECT_NE(mainSource.find("Mimic motor pair (right)"), std::string::npos);
  EXPECT_NE(mainSource.find("MimicMotorConstraint"), std::string::npos);
  EXPECT_NE(readmeSource.find("Coupler Constraint Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("960x720"), std::string::npos);
  EXPECT_NE(readmeSource.find("headless"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("ImGuiViewer"), std::string::npos);
  EXPECT_EQ(mainSource.find("RealTimeWorldNode"), std::string::npos);
  EXPECT_EQ(mainSource.find("GUIEventHandler"), std::string::npos);
}

TEST(FilamentSceneExtraction, PanelExtensionExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "imgui" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "imgui" / "README.md");

  EXPECT_NE(mainSource.find("PanelExtensionScene"), std::string::npos);
  EXPECT_NE(mainSource.find("panel extension target"), std::string::npos);
  EXPECT_EQ(mainSource.find("LineSegmentShape"), std::string::npos);
  EXPECT_EQ(mainSource.find("createPanelTargetShape"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(mainSource.find("options.gizmos"), std::string::npos);
  EXPECT_NE(mainSource.find("createPanelExtensionControls"), std::string::npos);
  EXPECT_NE(mainSource.find("Tinkertoy Control"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Promoted panel extension example"), std::string::npos);
  EXPECT_NE(mainSource.find("Gravity On/Off"), std::string::npos);
  EXPECT_NE(mainSource.find("setPanelWorldGravity"), std::string::npos);
  EXPECT_NE(
      mainSource.find("createPanelExtensionKeyboardActions"),
      std::string::npos);
  EXPECT_NE(mainSource.find("Lowercase q pressed"), std::string::npos);
  EXPECT_NE(mainSource.find("Capital Q pressed"), std::string::npos);
  EXPECT_NE(mainSource.find("Lowercase q released"), std::string::npos);
  EXPECT_NE(mainSource.find("Capital Q released"), std::string::npos);
  EXPECT_NE(mainSource.find("Left arrow key pressed"), std::string::npos);
  EXPECT_NE(mainSource.find("Left arrow key released"), std::string::npos);
  EXPECT_NE(mainSource.find("Right arrow key pressed"), std::string::npos);
  EXPECT_NE(mainSource.find("Right arrow key released"), std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardActionTrigger::Release"), std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("Pre-step callbacks"), std::string::npos);
  EXPECT_NE(mainSource.find("options.postStep"), std::string::npos);
  EXPECT_NE(mainSource.find("Post-step callbacks"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preRender"), std::string::npos);
  EXPECT_NE(mainSource.find("Pre-render callbacks"), std::string::npos);
  EXPECT_NE(mainSource.find("options.postRender"), std::string::npos);
  EXPECT_NE(mainSource.find("Post-render callbacks"), std::string::npos);
  EXPECT_NE(
      mainSource.find("makePanelExtensionRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(mainSource.find("makePanelExtensionCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.up = Eigen::Vector3d(-0.24, -0.25, 0.94)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.yaw = 0.8848934155088675"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.pitch = 0.38410042777133657"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.distance = 4.376539729055364"),
      std::string::npos);
  EXPECT_NE(mainSource.find("User Guide"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Left-drag target gizmo arrows/planes/rings"),
      std::string::npos);
  EXPECT_NE(mainSource.find("Headlights On/Off"), std::string::npos);
  EXPECT_NE(
      mainSource.find("context.lighting.headlightsEnabled"), std::string::npos);
  EXPECT_NE(mainSource.find("context.camera.eye"), std::string::npos);
  EXPECT_NE(mainSource.find("Eye   : "), std::string::npos);
  EXPECT_NE(mainSource.find("Center: "), std::string::npos);
  EXPECT_NE(mainSource.find("Up    : "), std::string::npos);
  EXPECT_NE(readmeSource.find("Panel Extension Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(readmeSource.find("Tinkertoy Control"), std::string::npos);
  EXPECT_NE(readmeSource.find("640x480"), std::string::npos);
  EXPECT_NE(readmeSource.find("keyboard callbacks"), std::string::npos);
  EXPECT_EQ(readmeSource.find("Remaining gaps"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("ImGuiViewer"), std::string::npos);
  EXPECT_EQ(mainSource.find("WorldNode"), std::string::npos);
  EXPECT_EQ(mainSource.find("GUIEventHandler"), std::string::npos);
  EXPECT_EQ(mainSource.find("InteractiveFrame"), std::string::npos);
}

TEST(FilamentSceneExtraction, RigidShapesExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "rigid_shapes" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "rigid_shapes" / "README.md");

  EXPECT_NE(
      mainSource.find("dart://sample/skel/shapes.skel"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::io::readWorld"), std::string::npos);
  EXPECT_NE(mainSource.find("ConvexMeshShape"), std::string::npos);
  EXPECT_NE(mainSource.find("spawnConvexMesh"), std::string::npos);
  EXPECT_NE(mainSource.find("ConvexMeshShape::fromMesh"), std::string::npos);
  EXPECT_NE(mainSource.find("Random::uniform"), std::string::npos);
  EXPECT_NE(mainSource.find("printRigidShapesInstructions"), std::string::npos);
  EXPECT_NE(mainSource.find("Collision detector:"), std::string::npos);
  EXPECT_NE(mainSource.find("PointCloudShape"), std::string::npos);
  EXPECT_NE(mainSource.find("contact_points"), std::string::npos);
  EXPECT_NE(mainSource.find("toggleContactPoints"), std::string::npos);
  EXPECT_NE(mainSource.find("updateContactPoints"), std::string::npos);
  EXPECT_NE(mainSource.find("getLastCollisionResult"), std::string::npos);
  EXPECT_NE(mainSource.find("--collision-detector"), std::string::npos);
  EXPECT_NE(mainSource.find("--max-contacts"), std::string::npos);
  EXPECT_NE(mainSource.find("--ground-thickness"), std::string::npos);
  EXPECT_NE(mainSource.find("maxNumContacts"), std::string::npos);
  EXPECT_NE(mainSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("makeRigidShapesRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(mainSource.find("options.camera"), std::string::npos);
  EXPECT_NE(mainSource.find("makeRigidShapesCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d::Zero()"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.up = Eigen::Vector3d::UnitY()"),
      std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey(key)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("Keys: q box"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("GUIEventHandler"), std::string::npos);
  EXPECT_NE(readmeSource.find("Rigid Shapes Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("pixi run ex rigid_shapes"), std::string::npos);
  EXPECT_NE(readmeSource.find("--collision-detector"), std::string::npos);
  EXPECT_NE(readmeSource.find("--out"), std::string::npos);
}

TEST(FilamentSceneExtraction, BipedStandExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "biped_stand" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "biped_stand" / "README.md");

  EXPECT_NE(mainSource.find("BipedStandController"), std::string::npos);
  EXPECT_NE(mainSource.find("controller->preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("controller->perturb"), std::string::npos);
  EXPECT_NE(
      mainSource.find("createBipedStandKeyboardActions"), std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey(key)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(50.0, 0.0, 0.0)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(-50.0, 0.0, 0.0)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(0.0, 0.0, 50.0)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(0.0, 0.0, -50.0)"), std::string::npos);
  EXPECT_NE(mainSource.find("makeBipedStandRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(mainSource.find("makeBipedStandCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d::Zero()"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("Press space to start simulation"), std::string::npos);
  EXPECT_NE(
      mainSource.find("1: Push robot with +50 along x-axis N for 100 frames"),
      std::string::npos);
  EXPECT_NE(mainSource.find("Keys: 1 +X"), std::string::npos);
  EXPECT_NE(readmeSource.find("Biped Stand Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("keyboard actions"), std::string::npos);
  EXPECT_NE(readmeSource.find("Build Instructions"), std::string::npos);
  EXPECT_NE(readmeSource.find("Execute Instructions"), std::string::npos);
  EXPECT_NE(readmeSource.find("./biped_stand"), std::string::npos);
}

TEST(FilamentSceneExtraction, FreeJointCasesExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "free_joint_cases" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "free_joint_cases" / "README.md");

  EXPECT_NE(mainSource.find("computeMetrics"), std::string::npos);
  EXPECT_NE(mainSource.find("getWorldJacobian"), std::string::npos);
  EXPECT_NE(mainSource.find("getJacobianClassicDeriv"), std::string::npos);
  EXPECT_NE(mainSource.find("getRelativeJacobianTimeDeriv"), std::string::npos);
  EXPECT_NE(
      mainSource.find("GroundTruthModel::ConstantWorldTwist"),
      std::string::npos);
  EXPECT_NE(mainSource.find("setUseSphericalInertia"), std::string::npos);
  EXPECT_NE(mainSource.find("recomputeMetrics"), std::string::npos);
  EXPECT_NE(mainSource.find("--numeric-dt"), std::string::npos);
  EXPECT_NE(mainSource.find("--dt"), std::string::npos);
  EXPECT_NE(mainSource.find("--spherical-inertia"), std::string::npos);
  EXPECT_NE(mainSource.find("--ground-truth"), std::string::npos);
  EXPECT_NE(mainSource.find("--ground-truth-substeps"), std::string::npos);
  EXPECT_NE(
      mainSource.find("builder.button(\"Numeric checks\")"), std::string::npos);
  EXPECT_NE(mainSource.find("Torque-free substeps"), std::string::npos);
  EXPECT_NE(mainSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(
      mainSource.find("makeFreeJointCasesRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 1280"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 720"), std::string::npos);
  EXPECT_NE(mainSource.find("options.camera"), std::string::npos);
  EXPECT_NE(mainSource.find("makeFreeJointCasesCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d(2.5, 0.0, 0.0)"),
      std::string::npos);
  EXPECT_NE(readmeSource.find("Free Joint Cases Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui"), std::string::npos);
}

TEST(FilamentSceneExtraction, HumanJointLimitsExamplePreservesParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "human_joint_limits" / "main.cpp");
  const auto constraintSource = readSourceFile(
      std::filesystem::path("examples") / "human_joint_limits"
      / "human_joint_limit_constraints.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "human_joint_limits" / "README.md");

  EXPECT_NE(mainSource.find("kima_human_edited.skel"), std::string::npos);
  EXPECT_NE(
      mainSource.find("joint->setLimitEnforcement(true)"), std::string::npos);
  EXPECT_NE(mainSource.find("makeStaticGround"), std::string::npos);
  EXPECT_NE(mainSource.find("human->setMobile(true)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("makeHumanJointLimitsRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(
      mainSource.find("printHumanJointLimitsInstructions"), std::string::npos);
  EXPECT_NE(mainSource.find("space bar: simulation on/off"), std::string::npos);
  EXPECT_NE(
      mainSource.find("installHumanJointLimitConstraints"), std::string::npos);
  EXPECT_NE(mainSource.find("customConstraintCount"), std::string::npos);
  EXPECT_NE(mainSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_NE(
      constraintSource.find("HumanArmJointLimitConstraint"), std::string::npos);
  EXPECT_NE(
      constraintSource.find("HumanLegJointLimitConstraint"), std::string::npos);
  EXPECT_NE(
      constraintSource.find("humanJointLimits/neuralnets/net-larm"),
      std::string::npos);
  EXPECT_NE(
      constraintSource.find("humanJointLimits/neuralnets/net-lleg"),
      std::string::npos);
  EXPECT_NE(
      constraintSource.find("TinyDnnSequentialNetwork"), std::string::npos);
  EXPECT_NE(readmeSource.find("Human Joint Limits Example"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("dart::gui::ApplicationOptions"), std::string::npos);
  EXPECT_NE(readmeSource.find("640x480"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("HumanArmJointLimitConstraint"), std::string::npos);
  EXPECT_NE(readmeSource.find("TinyDNN-serialized"), std::string::npos);
  EXPECT_EQ(readmeSource.find("tracked as a follow-up"), std::string::npos);
  EXPECT_EQ(
      mainSource.find("setGravity(Eigen::Vector3d::Zero())"),
      std::string::npos);
  EXPECT_EQ(
      mainSource.find("makeVisualOnlySkeleton(human)"), std::string::npos);
  EXPECT_EQ(mainSource.find("setRequiredJointPositions"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("WorldNode"), std::string::npos);
}

TEST(FilamentSceneExtraction, AddDeleteSkelsExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "add_delete_skels" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "add_delete_skels" / "README.md");

  EXPECT_NE(mainSource.find("#if DART_HAVE_BULLET"), std::string::npos);
  EXPECT_NE(
      mainSource.find("CollisionDetectorType::Bullet"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart://sample/skel/ground.skel"), std::string::npos);
  EXPECT_NE(
      mainSource.find("world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0))"),
      std::string::npos);
  EXPECT_NE(mainSource.find("spawnRandomCube"), std::string::npos);
  EXPECT_NE(
      mainSource.find("std::uniform_real_distribution"), std::string::npos);
  EXPECT_NE(mainSource.find("body.mInertia.setMass(mass)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("BoxShape::computeInertia(size, mass)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("deleteLastCube"), std::string::npos);
  EXPECT_NE(
      mainSource.find("createAddDeleteKeyboardActions"), std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey('q')"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey('w')"),
      std::string::npos);
  EXPECT_NE(mainSource.find("'q': spawn a random cube"), std::string::npos);
  EXPECT_NE(mainSource.find("'w': delete a spawned cube"), std::string::npos);
  EXPECT_NE(mainSource.find("makeAddDeleteCamera"), std::string::npos);
  EXPECT_NE(mainSource.find("options.camera"), std::string::npos);
  EXPECT_NE(mainSource.find("makeAddDeleteRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(mainSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_EQ(mainSource.find("spawnPresetCube"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("WorldNode"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("Add/Delete Skeletons Example"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("pixi run ex add_delete_skels"), std::string::npos);
  EXPECT_NE(readmeSource.find("`q` spawns a cube"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("`w` deletes the most recent cube"), std::string::npos);
}

TEST(FilamentSceneExtraction, MixedChainExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "mixed_chain" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "mixed_chain" / "README.md");

  EXPECT_NE(
      mainSource.find("test_articulated_bodies_10bodies.skel"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("dart::math::Random::uniform(-0.5, 0.5)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("startImpulse"), std::string::npos);
  EXPECT_NE(mainSource.find("applyImpulse"), std::string::npos);
  EXPECT_NE(mainSource.find("kForceMagnitude = 500.0"), std::string::npos);
  EXPECT_NE(mainSource.find("kImpulseFrames = 100"), std::string::npos);
  EXPECT_NE(mainSource.find("getSoftBodyNode(3)"), std::string::npos);
  EXPECT_NE(mainSource.find("makeImpulseAction"), std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey(key)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("createMixedChainKeyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("Apply mixed-chain impulse -X"), std::string::npos);
  EXPECT_NE(mainSource.find("Apply mixed-chain impulse +X"), std::string::npos);
  EXPECT_NE(mainSource.find("Apply mixed-chain impulse -Y"), std::string::npos);
  EXPECT_NE(mainSource.find("Apply mixed-chain impulse +Y"), std::string::npos);
  EXPECT_NE(mainSource.find("Apply mixed-chain impulse -Z"), std::string::npos);
  EXPECT_NE(mainSource.find("Apply mixed-chain impulse +Z"), std::string::npos);
  EXPECT_NE(
      mainSource.find("'q'/'w': apply force in -X/+X direction"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("'q'/'w': Apply force in -X/+X direction"),
      std::string::npos);
  EXPECT_NE(mainSource.find("printMixedChainInstructions"), std::string::npos);
  EXPECT_NE(mainSource.find("Space: Toggle simulation"), std::string::npos);
  EXPECT_NE(mainSource.find("makeMixedChainCamera"), std::string::npos);
  EXPECT_NE(mainSource.find("options.camera"), std::string::npos);
  EXPECT_NE(mainSource.find("makeMixedChainRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(mainSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_NE(readmeSource.find("Mixed Chain Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("pixi run ex mixed_chain"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(readmeSource.find("--out"), std::string::npos);
}

TEST(FilamentSceneExtraction, HybridDynamicsExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "hybrid_dynamics" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "hybrid_dynamics" / "README.md");

  EXPECT_NE(
      mainSource.find("dart://sample/skel/fullbody1.skel"), std::string::npos);
  EXPECT_NE(
      mainSource.find("kGroundSkeletonName = \"ground skeleton\""),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("kBipedSkeletonName = \"fullbody1\""), std::string::npos);
  EXPECT_EQ(mainSource.find("setName(kGroundSkeletonName)"), std::string::npos);
  EXPECT_EQ(mainSource.find("setName(kBipedSkeletonName)"), std::string::npos);
  EXPECT_EQ(mainSource.find("colorBiped"), std::string::npos);
  EXPECT_EQ(mainSource.find("setColor("), std::string::npos);
  EXPECT_NE(mainSource.find("toggleHarness"), std::string::npos);
  EXPECT_NE(mainSource.find("The pelvis is locked."), std::string::npos);
  EXPECT_NE(mainSource.find("The pelvis is unlocked."), std::string::npos);
  EXPECT_NE(
      mainSource.find("printHybridDynamicsInstructions"), std::string::npos);
  EXPECT_NE(mainSource.find("makeHybridDynamicsCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.up = Eigen::Vector3d::UnitY()"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("makeHybridDynamicsRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(
      mainSource.find("createHybridDynamicsKeyboardActions"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey('h')"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("Toggle hybrid-dynamics harness"), std::string::npos);
  EXPECT_NE(mainSource.find("'h': toggle harness on/off"), std::string::npos);
  EXPECT_NE(mainSource.find("space bar: simulation on/off"), std::string::npos);
  EXPECT_NE(mainSource.find("options.camera"), std::string::npos);
  EXPECT_NE(mainSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_NE(readmeSource.find("Hybrid Dynamics Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("640x480"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("pixi run ex hybrid_dynamics"), std::string::npos);
  EXPECT_NE(readmeSource.find("Build Instructions"), std::string::npos);
  EXPECT_NE(readmeSource.find("Execute Instructions"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
}

TEST(
    FilamentSceneExtraction,
    JointConstraintsExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "joint_constraints" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "joint_constraints" / "README.md");

  EXPECT_NE(
      mainSource.find("dart://sample/skel/fullbody1.skel"), std::string::npos);
  EXPECT_NE(
      mainSource.find("kGroundSkeletonName = \"ground skeleton\""),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("kBipedSkeletonName = \"fullbody1\""), std::string::npos);
  EXPECT_EQ(mainSource.find("setName(kGroundSkeletonName)"), std::string::npos);
  EXPECT_EQ(mainSource.find("setName(kBipedSkeletonName)"), std::string::npos);
  EXPECT_EQ(mainSource.find("colorBiped"), std::string::npos);
  EXPECT_EQ(mainSource.find("setColor("), std::string::npos);
  EXPECT_NE(mainSource.find("makeJointConstraintsCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.up = Eigen::Vector3d::UnitY()"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("makeJointConstraintsRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(mainSource.find("makePerturbAction"), std::string::npos);
  EXPECT_NE(
      mainSource.find("printJointConstraintsInstructions"), std::string::npos);
  EXPECT_NE(
      mainSource.find("createJointConstraintsKeyboardActions"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey(key)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardShortcut::characterKey('h')"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("Apply joint-constraints forward perturbation"),
      std::string::npos);
  EXPECT_NE(mainSource.find("push forward"), std::string::npos);
  EXPECT_NE(mainSource.find("push backward"), std::string::npos);
  EXPECT_NE(mainSource.find("push right"), std::string::npos);
  EXPECT_NE(mainSource.find("push left"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Toggle joint-constraints harness"), std::string::npos);
  EXPECT_NE(mainSource.find("Harness on"), std::string::npos);
  EXPECT_NE(mainSource.find("Harness off"), std::string::npos);
  EXPECT_NE(
      mainSource.find("'1'-'4': programmed perturbations"), std::string::npos);
  EXPECT_NE(mainSource.find("'h': toggle harness on/off"), std::string::npos);
  EXPECT_NE(mainSource.find("space bar: simulation on/off"), std::string::npos);
  EXPECT_NE(mainSource.find("options.camera"), std::string::npos);
  EXPECT_NE(mainSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_NE(readmeSource.find("Joint Constraints Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("640x480"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("pixi run ex joint_constraints"), std::string::npos);
  EXPECT_NE(readmeSource.find("Build Instructions"), std::string::npos);
  EXPECT_NE(readmeSource.find("Execute Instructions"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
}

TEST(FilamentSceneExtraction, AtlasSimbiconPreservesLegacyControllerMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "atlas_simbicon" / "main.cpp");
  const auto controllerSource = readSourceFile(
      std::filesystem::path("examples") / "atlas_simbicon" / "controller.cpp");
  const auto stateSource = readSourceFile(
      std::filesystem::path("examples") / "atlas_simbicon" / "state.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "atlas_simbicon" / "README.md");

  EXPECT_NE(mainSource.find("createZUpGround"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart://sample/sdf/atlas/atlas_v5_no_head.urdf"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("scene.world->setGravity(-kDefaultGravity"),
      std::string::npos);
  EXPECT_NE(mainSource.find("Eigen::Vector3d::UnitZ()"), std::string::npos);
  EXPECT_NE(
      mainSource.find("makeAtlasMeshVisualsReadable(scene.atlas)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find(
          "mesh->setColorMode(dart::dynamics::MeshShape::MATERIAL_COLOR)"),
      std::string::npos);
  EXPECT_EQ(mainSource.find("setPosition(0, -0.5"), std::string::npos);
  EXPECT_EQ(
      mainSource.find("-kDefaultGravity * Eigen::Vector3d::UnitY()"),
      std::string::npos);
  EXPECT_EQ(
      mainSource.find("-mGravity * Eigen::Vector3d::UnitY()"),
      std::string::npos);
  EXPECT_NE(mainSource.find("std::make_unique<Controller>"), std::string::npos);
  EXPECT_NE(mainSource.find("runtime->preStep()"), std::string::npos);
  EXPECT_NE(
      mainSource.find("createAtlasSimbiconKeyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("Push Atlas forward"), std::string::npos);
  EXPECT_NE(mainSource.find("Standing controller"), std::string::npos);
  EXPECT_NE(mainSource.find("Walking-in-place controller"), std::string::npos);
  EXPECT_NE(mainSource.find("Atlas Control"), std::string::npos);
  EXPECT_NE(mainSource.find("Gravity Acc."), std::string::npos);
  EXPECT_NE(mainSource.find("Harness pelvis"), std::string::npos);
  EXPECT_NE(mainSource.find("Headlights On/Off"), std::string::npos);
  EXPECT_NE(mainSource.find("Shadow On/Off"), std::string::npos);
  EXPECT_NE(mainSource.find("Depth mode"), std::string::npos);
  EXPECT_NE(
      mainSource.find("dart::gui::RenderOutputMode::Depth"), std::string::npos);
  EXPECT_NE(mainSource.find("Normal-Stride Walking"), std::string::npos);
  EXPECT_NE(mainSource.find("Short-Stride Walking"), std::string::npos);
  EXPECT_EQ(
      mainSource.find("Depth mode needs a public render-output API"),
      std::string::npos);
  EXPECT_NE(mainSource.find("makeAtlasSimbiconCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d(0.0, 0.0, -0.25)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("options.windowTitle = \"Atlas Simbicon\""),
      std::string::npos);
  EXPECT_EQ(mainSource.find("Ctrl-left drag"), std::string::npos);
  EXPECT_NE(mainSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.camera"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(
      controllerSource.find("_createWalkingStateMachine"), std::string::npos);
  EXPECT_NE(controllerSource.find("harnessPelvis"), std::string::npos);
  EXPECT_NE(controllerSource.find("keyboard"), std::string::npos);
  EXPECT_NE(controllerSource.find("const auto z = pos[2]"), std::string::npos);
  EXPECT_NE(stateSource.find("computeControlForce"), std::string::npos);
  EXPECT_NE(
      stateSource.find(
          "const Eigen::Vector3d zAxis = Eigen::Vector3d::UnitZ()"),
      std::string::npos);
  EXPECT_NE(stateSource.find("projPelvisZ[1] = 0.0"), std::string::npos);
  EXPECT_NE(readmeSource.find("Atlas Simbicon Example"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("dart::gui::ApplicationOptions::preStep"),
      std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui::RunOptions"), std::string::npos);
  EXPECT_NE(readmeSource.find("depth output"), std::string::npos);
  EXPECT_NE(readmeSource.find("--render-output depth"), std::string::npos);
  EXPECT_EQ(
      readmeSource.find("Remaining renderer-neutral API gap"),
      std::string::npos);
  EXPECT_EQ(
      readmeSource.find("historical OSG depth camera mode"), std::string::npos);
  EXPECT_EQ(mainSource.find("ImGuiViewer"), std::string::npos);
  EXPECT_EQ(mainSource.find("WorldNode"), std::string::npos);
  EXPECT_EQ(mainSource.find("osg"), std::string::npos);
}

TEST(FilamentSceneExtraction, AtlasPuppetExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "atlas_puppet" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "atlas_puppet" / "README.md");

  EXPECT_NE(
      mainSource.find("dart://sample/sdf/atlas/atlas_v5_no_head.urdf"),
      std::string::npos);
  EXPECT_NE(mainSource.find("atlas_puppet_root_handle"), std::string::npos);
  EXPECT_EQ(mainSource.find("createIkTargetHandleShape"), std::string::npos);
  EXPECT_EQ(mainSource.find("target->setShape"), std::string::npos);
  EXPECT_NE(
      mainSource.find("atlas_puppet_ik_target_left_hand"), std::string::npos);
  EXPECT_NE(
      mainSource.find("atlas_puppet_ik_target_right_hand"), std::string::npos);
  EXPECT_NE(
      mainSource.find("atlas_puppet_ik_target_left_foot"), std::string::npos);
  EXPECT_NE(
      mainSource.find("atlas_puppet_ik_target_right_foot"), std::string::npos);
  EXPECT_NE(
      mainSource.find("makeAtlasMeshVisualsReadable(atlas)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find(
          "mesh->setColorMode(dart::dynamics::MeshShape::MATERIAL_COLOR)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("support->setActive(true)"), std::string::npos);
  EXPECT_NE(
      mainSource.find(
          "scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81))"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("atlas_puppet_support_polygon_overlay"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("atlas_puppet_support_com_overlay"), std::string::npos);
  EXPECT_NE(mainSource.find("makeAtlasSupportPolygonLines"), std::string::npos);
  EXPECT_NE(mainSource.find("makeAtlasSupportComLines"), std::string::npos);
  EXPECT_NE(mainSource.find("addDisconnectedLine"), std::string::npos);
  EXPECT_NE(
      mainSource.find("removeConnection(start - 1u, start)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("atlasSupportComColor"), std::string::npos);
  EXPECT_NE(mainSource.find("isInsideSupportPolygon"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector4d(0.0, 0.0, 1.0, 1.0)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("TargetState"), std::string::npos);
  EXPECT_NE(mainSource.find("state->toggle()"), std::string::npos);
  EXPECT_NE(mainSource.find("gizmo.isVisible"), std::string::npos);
  EXPECT_NE(mainSource.find("gizmo.onChanged"), std::string::npos);
  EXPECT_NE(mainSource.find("return state->active"), std::string::npos);
  EXPECT_NE(mainSource.find("gizmo.size = 0.20"), std::string::npos);
  EXPECT_NE(
      mainSource.find("world->addSimpleFrame(target)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("world->removeSimpleFrame(target)"), std::string::npos);
  EXPECT_NE(mainSource.find("r_arm_shx"), std::string::npos);
  EXPECT_NE(mainSource.find("l_arm_shx"), std::string::npos);
  EXPECT_NE(mainSource.find("setUnconstrainedIkBounds"), std::string::npos);
  EXPECT_NE(mainSource.find("RelaxedPosture"), std::string::npos);
  EXPECT_NE(mainSource.find("enforceIdealPosture"), std::string::npos);
  EXPECT_NE(mainSource.find("setupAtlasWholeBodySolver"), std::string::npos);
  EXPECT_NE(mainSource.find("GradientDescentSolver"), std::string::npos);
  EXPECT_NE(mainSource.find("BalanceConstraint"), std::string::npos);
  EXPECT_NE(mainSource.find("OPTIMIZE_BALANCE"), std::string::npos);
  EXPECT_NE(mainSource.find("FROM_CENTROID"), std::string::npos);
  EXPECT_NE(mainSource.find("SHIFT_SUPPORT"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Left-drag active target gizmo handles"),
      std::string::npos);
  EXPECT_NE(mainSource.find("InverseKinematicsHandle"), std::string::npos);
  EXPECT_NE(
      mainSource.find("InverseKinematicsSolveMode::SkeletonHierarchy"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("applyAtlasRootGradientWeights"), std::string::npos);
  EXPECT_NE(mainSource.find("options.ikHandles"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(mainSource.find("options.gizmos"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("applyRootTeleoperationStep"), std::string::npos);
  EXPECT_NE(mainSource.find("solveAtlasWholeBody"), std::string::npos);
  EXPECT_NE(mainSource.find("WASD moves the root"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Toggle left Atlas foot support"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Toggle right Atlas foot support"), std::string::npos);
  EXPECT_NE(mainSource.find("Print Atlas DOFs"), std::string::npos);
  EXPECT_NE(mainSource.find("Reset Atlas relaxed posture"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Optimize Atlas posture and balance"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Restore Atlas centroid balance mode"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardActionTrigger::Release"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Hold R to optimize whole-body posture and balance"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("Blue/red COM marker shows support-polygon validity"),
      std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_NE(mainSource.find("makeAtlasPuppetRunDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 1280"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 960"), std::string::npos);
  EXPECT_NE(mainSource.find("makeAtlasPuppetCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d(0.0, 0.0, 1.0)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.yaw = 0.5118558424318241"), std::string::npos);
  EXPECT_NE(mainSource.find("options.camera"), std::string::npos);
  EXPECT_NE(mainSource.find("'1'"), std::string::npos);
  EXPECT_NE(mainSource.find("'4'"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("WorldNode"), std::string::npos);
  EXPECT_EQ(mainSource.find("ImGuiViewer"), std::string::npos);
  EXPECT_NE(readmeSource.find("Atlas Puppet Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(readmeSource.find("SimpleFrame` IK targets"), std::string::npos);
  EXPECT_NE(readmeSource.find("axis-arrow dragging"), std::string::npos);
  EXPECT_NE(readmeSource.find("Target gizmos appear"), std::string::npos);
  EXPECT_NE(readmeSource.find("rotation"), std::string::npos);
  EXPECT_NE(readmeSource.find("rotation ring"), std::string::npos);
  EXPECT_NE(readmeSource.find("plane handles"), std::string::npos);
  EXPECT_NE(readmeSource.find("COM validity"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("whole-body posture and balance"), std::string::npos);
  EXPECT_NE(readmeSource.find("relaxed-posture objective"), std::string::npos);
  EXPECT_NE(readmeSource.find("balance"), std::string::npos);
  EXPECT_NE(readmeSource.find("constraint"), std::string::npos);
  EXPECT_NE(readmeSource.find("COM marker is blue"), std::string::npos);
  EXPECT_NE(readmeSource.find("pixi run ex atlas_puppet"), std::string::npos);
  EXPECT_NE(readmeSource.find("1280"), std::string::npos);
}

TEST(FilamentSceneExtraction, HuboPuppetExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "hubo_puppet" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "hubo_puppet" / "README.md");

  EXPECT_NE(mainSource.find("urdf/drchubo/drchubo.urdf"), std::string::npos);
  EXPECT_NE(
      mainSource.find("removeHuboPuppetFingerBodyNodes"), std::string::npos);
  EXPECT_NE(
      mainSource.find("kHuboSkeletonName = \"hubo_copy\""), std::string::npos);
  EXPECT_NE(
      mainSource.find("kGroundSkeletonName = \"ground\""), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(10.0, 10.0, thickness)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector4d(0.0, 0.0, 0.2, 1.0)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81))"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("kSupportVisualElevation = 0.05"), std::string::npos);
  EXPECT_NE(mainSource.find("makeSupportPolygonDebugLines"), std::string::npos);
  EXPECT_NE(mainSource.find("hubo_support_polygon_overlay"), std::string::npos);
  EXPECT_NE(mainSource.find("hubo_support_com_overlay"), std::string::npos);
  EXPECT_NE(mainSource.find("makeHuboSupportComLines"), std::string::npos);
  EXPECT_NE(mainSource.find("addDisconnectedLine"), std::string::npos);
  EXPECT_NE(
      mainSource.find("removeConnection(start - 1u, start)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("huboSupportComColor"), std::string::npos);
  EXPECT_NE(mainSource.find("isInsideSupportPolygon"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector4d(0.0, 0.0, 1.0, 1.0)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("LineSegmentShape"), std::string::npos);
  EXPECT_EQ(mainSource.find("createIkTargetHandleShape"), std::string::npos);
  EXPECT_EQ(mainSource.find("target->setShape"), std::string::npos);
  EXPECT_NE(
      mainSource.find("hubo_puppet_ik_target_left_hand"), std::string::npos);
  EXPECT_NE(
      mainSource.find("hubo_puppet_ik_target_right_peg"), std::string::npos);
  EXPECT_NE(mainSource.find("\"l_hand\""), std::string::npos);
  EXPECT_NE(mainSource.find("\"r_peg\""), std::string::npos);
  EXPECT_NE(mainSource.find("support->setActive(true)"), std::string::npos);
  EXPECT_NE(mainSource.find("setUnconstrainedIkBounds"), std::string::npos);
  EXPECT_EQ(mainSource.find("Ctrl-left drag"), std::string::npos);
  EXPECT_NE(mainSource.find("InverseKinematicsHandle"), std::string::npos);
  EXPECT_NE(
      mainSource.find("InverseKinematicsSolveMode::SkeletonHierarchy"),
      std::string::npos);
  EXPECT_NE(mainSource.find("options.ikHandles"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(mainSource.find("options.gizmos"), std::string::npos);
  EXPECT_NE(mainSource.find("gizmo.isVisible"), std::string::npos);
  EXPECT_NE(mainSource.find("gizmo.onChanged"), std::string::npos);
  EXPECT_NE(mainSource.find("return state->active"), std::string::npos);
  EXPECT_NE(mainSource.find("gizmo.size = 0.20"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Left-drag active target gizmo handles"),
      std::string::npos);
  EXPECT_NE(mainSource.find("TargetState"), std::string::npos);
  EXPECT_NE(
      mainSource.find("world->addSimpleFrame(target)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("world->removeSimpleFrame(target)"), std::string::npos);
  EXPECT_NE(mainSource.find("Activated IK target"), std::string::npos);
  EXPECT_NE(mainSource.find("Deactivated IK target"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("applyRootTeleoperationStep"), std::string::npos);
  EXPECT_NE(mainSource.find("kTeleopAmplifiedScale"), std::string::npos);
  EXPECT_NE(mainSource.find("std::string(\"Amplified \")"), std::string::npos);
  EXPECT_NE(mainSource.find("RelaxedPosture"), std::string::npos);
  EXPECT_NE(mainSource.find("setupHuboWholeBodySolver"), std::string::npos);
  EXPECT_NE(mainSource.find("GradientDescentSolver"), std::string::npos);
  EXPECT_NE(mainSource.find("BalanceConstraint"), std::string::npos);
  EXPECT_NE(mainSource.find("HuboArmIK"), std::string::npos);
  EXPECT_NE(mainSource.find("HuboLegIK"), std::string::npos);
  EXPECT_NE(mainSource.find("setGradientMethod<HuboArmIK>"), std::string::npos);
  EXPECT_NE(mainSource.find("setGradientMethod<HuboLegIK>"), std::string::npos);
  EXPECT_NE(mainSource.find("POST_ANALYTICAL"), std::string::npos);
  EXPECT_NE(mainSource.find("setExtraErrorLengthClamp"), std::string::npos);
  EXPECT_NE(mainSource.find("Body_LSP"), std::string::npos);
  EXPECT_NE(mainSource.find("Body_RHY"), std::string::npos);
  EXPECT_NE(mainSource.find("OPTIMIZE_BALANCE"), std::string::npos);
  EXPECT_NE(mainSource.find("FROM_CENTROID"), std::string::npos);
  EXPECT_NE(mainSource.find("SHIFT_SUPPORT"), std::string::npos);
  EXPECT_NE(mainSource.find("solveHuboWholeBody"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Optimize Hubo posture and balance"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Restore Hubo centroid balance mode"), std::string::npos);
  EXPECT_NE(
      mainSource.find("KeyboardActionTrigger::Release"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Toggle left Hubo foot support"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Toggle right Hubo foot support"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Blue/red COM marker shows support-polygon validity"),
      std::string::npos);
  EXPECT_NE(mainSource.find("Print Hubo DOFs"), std::string::npos);
  EXPECT_NE(mainSource.find("Reset Hubo relaxed posture"), std::string::npos);
  EXPECT_NE(mainSource.find("WASD moves the root"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Hold R to optimize whole-body posture and balance"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("Whole-body IK solves active targets and balance"),
      std::string::npos);
  EXPECT_NE(mainSource.find("'1'"), std::string::npos);
  EXPECT_NE(mainSource.find("'6'"), std::string::npos);
  EXPECT_NE(mainSource.find("'x'"), std::string::npos);
  EXPECT_NE(mainSource.find("'c'"), std::string::npos);
  EXPECT_NE(mainSource.find("'p'"), std::string::npos);
  EXPECT_NE(mainSource.find("'t'"), std::string::npos);
  EXPECT_NE(mainSource.find("'W'"), std::string::npos);
  EXPECT_NE(mainSource.find("'E'"), std::string::npos);
  EXPECT_NE(mainSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 1280"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 960"), std::string::npos);
  EXPECT_NE(mainSource.find("makeHuboPuppetCamera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d(0.0, 0.0, 0.50)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.up = Eigen::Vector3d(-0.20, -0.08, 0.98)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.yaw = 0.5118558424318241"), std::string::npos);
  EXPECT_NE(mainSource.find("options.camera"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("SupportPolygonVisual"), std::string::npos);
  EXPECT_EQ(mainSource.find("InteractiveFrame"), std::string::npos);
  EXPECT_EQ(mainSource.find("WorldNode"), std::string::npos);
  EXPECT_EQ(mainSource.find("osg"), std::string::npos);
  EXPECT_NE(readmeSource.find("Hubo Puppet Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(readmeSource.find("pixi run ex hubo_puppet"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("support-polygon and COM validity overlay"),
      std::string::npos);
  EXPECT_NE(readmeSource.find("COM marker is blue"), std::string::npos);
  EXPECT_NE(readmeSource.find("Press `1`-`6`"), std::string::npos);
  EXPECT_NE(readmeSource.find("Target gizmos appear"), std::string::npos);
  EXPECT_NE(readmeSource.find("historical 2x movement"), std::string::npos);
  EXPECT_NE(readmeSource.find("whole-body"), std::string::npos);
  EXPECT_NE(readmeSource.find("posture"), std::string::npos);
  EXPECT_NE(readmeSource.find("balance"), std::string::npos);
  EXPECT_NE(readmeSource.find("relaxed-posture objective"), std::string::npos);
  EXPECT_NE(readmeSource.find("balance constraint"), std::string::npos);
  EXPECT_NE(readmeSource.find("analytical"), std::string::npos);
  EXPECT_NE(readmeSource.find("arm and leg IK"), std::string::npos);
}

TEST(FilamentSceneExtraction, CollisionSandboxUsesMouseDrivenObjectGizmos)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "collision_sandbox" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "collision_sandbox" / "README.md");

  EXPECT_NE(
      mainSource.find("#include <dart/gui/gizmo.hpp>"), std::string::npos);
  EXPECT_NE(mainSource.find("createObjectGizmos"), std::string::npos);
  EXPECT_NE(
      mainSource.find("options.gizmos = createObjectGizmos(state)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("gizmo.onChanged"), std::string::npos);
  EXPECT_NE(mainSource.find("kContactObjectAlpha"), std::string::npos);
  EXPECT_NE(mainSource.find("showDebugLabels"), std::string::npos);
  EXPECT_NE(mainSource.find("addDebugLabel"), std::string::npos);
  EXPECT_NE(mainSource.find("BroadPhaseObjectCenters"), std::string::npos);
  EXPECT_NE(
      mainSource.find("objectCentersById = fallbackObjectCenters"),
      std::string::npos);
  EXPECT_NE(mainSource.find("options.debugLabels"), std::string::npos);
  EXPECT_NE(mainSource.find("\"Name Tags\""), std::string::npos);
  EXPECT_NE(
      mainSource.find("const bool showContactOverlay"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Left-drag object gizmo arrows, planes, and rings"),
      std::string::npos);
  EXPECT_EQ(mainSource.find("\"A Position\""), std::string::npos);
  EXPECT_EQ(mainSource.find("\"B Position\""), std::string::npos);
  EXPECT_NE(
      readmeSource.find("Left-drag object gizmo arrows, planes, and rings"),
      std::string::npos);
  EXPECT_NE(readmeSource.find("rendered translucent"), std::string::npos);
  EXPECT_NE(
      readmeSource.find("point, normal, and depth overlays"),
      std::string::npos);
  EXPECT_NE(readmeSource.find("Minimal name tags"), std::string::npos);
}

TEST(FilamentSceneExtraction, G1PuppetExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "g1_puppet" / "main.cpp");
  const auto readmeSource = readSourceFile(
      std::filesystem::path("examples") / "g1_puppet" / "README.md");

  EXPECT_NE(
      mainSource.find("package://g1_description/g1_29dof.urdf"),
      std::string::npos);
  EXPECT_NE(mainSource.find("createG1ResourceRetriever"), std::string::npos);
  EXPECT_NE(mainSource.find("--g1-package-uri"), std::string::npos);
  EXPECT_NE(mainSource.find("--package-uri"), std::string::npos);
  EXPECT_NE(mainSource.find("kG1SkeletonName = \"G1\""), std::string::npos);
  EXPECT_NE(
      mainSource.find("kGroundSkeletonName = \"ground\""), std::string::npos);
  EXPECT_NE(
      mainSource.find("kGroundBodyName = \"ground_body\""), std::string::npos);
  EXPECT_NE(
      mainSource.find("world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81))"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector3d(8.0, 8.0, thickness)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("Eigen::Vector4d(0.4, 0.4, 0.4, 1.0)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("transform.translation().z() = 0.75"), std::string::npos);
  EXPECT_NE(mainSource.find("computeVisualWorldBounds"), std::string::npos);
  EXPECT_NE(
      mainSource.find("groundClearance - bounds->first.z()"),
      std::string::npos);
  EXPECT_NE(mainSource.find("createG1GridShape"), std::string::npos);
  EXPECT_NE(mainSource.find("constexpr int cellCount = 40"), std::string::npos);
  EXPECT_NE(
      mainSource.find("constexpr double spacing = 0.1"), std::string::npos);
  EXPECT_NE(mainSource.find("kG1GridShapeName"), std::string::npos);
  EXPECT_NE(
      mainSource.find("kSupportVisualElevation = 0.02"), std::string::npos);
  EXPECT_NE(mainSource.find("makeSupportPolygonDebugLines"), std::string::npos);
  EXPECT_NE(mainSource.find("g1_support_polygon_overlay"), std::string::npos);
  EXPECT_NE(mainSource.find("addDisconnectedLine"), std::string::npos);
  EXPECT_NE(
      mainSource.find("removeConnection(start - 1u, start)"),
      std::string::npos);
  EXPECT_NE(mainSource.find("LineSegmentShape"), std::string::npos);
  EXPECT_EQ(mainSource.find("createIkTargetHandleShape"), std::string::npos);
  EXPECT_EQ(mainSource.find("target->setShape"), std::string::npos);
  EXPECT_NE(mainSource.find("ik_target_left_hand"), std::string::npos);
  EXPECT_NE(mainSource.find("ik_target_right_foot"), std::string::npos);
  EXPECT_NE(mainSource.find("left_rubber_hand_target"), std::string::npos);
  EXPECT_NE(mainSource.find("right_ankle_roll_link_target"), std::string::npos);
  EXPECT_NE(mainSource.find("support->setActive(true)"), std::string::npos);
  EXPECT_EQ(mainSource.find("Ctrl-left drag"), std::string::npos);
  EXPECT_NE(mainSource.find("InverseKinematicsHandle"), std::string::npos);
  EXPECT_NE(mainSource.find("options.ikHandles"), std::string::npos);
  EXPECT_NE(mainSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(mainSource.find("options.gizmos"), std::string::npos);
  EXPECT_NE(mainSource.find("BodyNodeDragHandle"), std::string::npos);
  EXPECT_NE(mainSource.find("options.bodyNodeDragHandles"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Alt-drag translates body nodes"), std::string::npos);
  EXPECT_NE(mainSource.find("Ctrl-drag rotates them"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Shift-drag moves a body with only its parent joint"),
      std::string::npos);
  EXPECT_NE(mainSource.find("gizmo.isVisible"), std::string::npos);
  EXPECT_NE(mainSource.find("gizmo.onChanged"), std::string::npos);
  EXPECT_NE(mainSource.find("return state->active"), std::string::npos);
  EXPECT_NE(mainSource.find("gizmo.size = 0.15"), std::string::npos);
  EXPECT_NE(
      mainSource.find("Left-drag active target gizmo handles"),
      std::string::npos);
  EXPECT_NE(mainSource.find("TargetState"), std::string::npos);
  EXPECT_NE(mainSource.find("createG1KeyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("solveActiveG1Targets"), std::string::npos);
  EXPECT_EQ(
      mainSource.find("InverseKinematicsSolveMode::SkeletonHierarchy"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("world->addSimpleFrame(target)"), std::string::npos);
  EXPECT_NE(
      mainSource.find("world->removeSimpleFrame(target)"), std::string::npos);
  EXPECT_NE(mainSource.find("Activated IK target"), std::string::npos);
  EXPECT_NE(mainSource.find("Deactivated IK target"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(mainSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(mainSource.find("options.width = 1280"), std::string::npos);
  EXPECT_NE(mainSource.find("options.height = 960"), std::string::npos);
  EXPECT_NE(mainSource.find("makeG1Camera"), std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.target = Eigen::Vector3d(0.0, 0.0, 0.75)"),
      std::string::npos);
  EXPECT_NE(
      mainSource.find("camera.yaw = 0.48995732625372834"), std::string::npos);
  EXPECT_NE(mainSource.find("options.camera"), std::string::npos);
  EXPECT_NE(mainSource.find("toggle/select targets"), std::string::npos);
  EXPECT_NE(mainSource.find("'1'"), std::string::npos);
  EXPECT_NE(mainSource.find("'4'"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(mainSource.find("GridVisual"), std::string::npos);
  EXPECT_EQ(mainSource.find("SupportPolygonVisual"), std::string::npos);
  EXPECT_EQ(mainSource.find("InteractiveFrame"), std::string::npos);
  EXPECT_EQ(mainSource.find("WorldNode"), std::string::npos);
  EXPECT_EQ(mainSource.find("osg"), std::string::npos);
  EXPECT_NE(readmeSource.find("G1 Puppet Example"), std::string::npos);
  EXPECT_NE(readmeSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(readmeSource.find("pixi run ex g1_puppet"), std::string::npos);
  EXPECT_NE(readmeSource.find("--package-uri"), std::string::npos);
  EXPECT_NE(readmeSource.find("--g1-package-uri"), std::string::npos);
  EXPECT_NE(readmeSource.find("support-polygon overlay"), std::string::npos);
  EXPECT_NE(readmeSource.find("BodyNodeDragHandle"), std::string::npos);
  EXPECT_NE(readmeSource.find("Target gizmos appear"), std::string::npos);
  EXPECT_NE(readmeSource.find("Alt preserves orientation"), std::string::npos);
  EXPECT_EQ(
      readmeSource.find("do not have a full renderer-neutral"),
      std::string::npos);
}

TEST(FilamentSceneExtraction, TargetHandleExamplesPreserveParityMarkers)
{
  const auto operationalSource = readSourceFile(
      std::filesystem::path("examples") / "operational_space_control"
      / "main.cpp");
  const auto operationalReadmeSource = readSourceFile(
      std::filesystem::path("examples") / "operational_space_control"
      / "README.md");
  const auto inputSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "input.cpp");
  const auto selectionSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "selection.cpp");
  EXPECT_NE(
      operationalSource.find("KR5/KR5 sixx R650.urdf"), std::string::npos);
  EXPECT_NE(operationalSource.find("KR5/ground.urdf"), std::string::npos);
  EXPECT_NE(
      operationalSource.find("setTransformFromParentBodyNode"),
      std::string::npos);
  EXPECT_NE(
      operationalSource.find("Eigen::Isometry3d::Identity()"),
      std::string::npos);
  EXPECT_NE(
      operationalSource.find(
          "transform.pretranslate(Eigen::Vector3d(0.0, 0.0, 0.5))"),
      std::string::npos);
  EXPECT_NE(
      operationalSource.find("Eigen::AngleAxisd(dart::math::pi / 2.0"),
      std::string::npos);
  EXPECT_NE(
      operationalSource.find("joint->setLimitEnforcement(false)"),
      std::string::npos);
  EXPECT_NE(
      operationalSource.find("joint->setDampingCoefficient(0, 0.5)"),
      std::string::npos);
  EXPECT_EQ(operationalSource.find("SphereShape"), std::string::npos);
  EXPECT_NE(operationalSource.find("getMassMatrix"), std::string::npos);
  EXPECT_NE(operationalSource.find("getLinearJacobian"), std::string::npos);
  EXPECT_NE(
      operationalSource.find("getLinearJacobianDeriv"), std::string::npos);
  EXPECT_NE(
      operationalSource.find("getCoriolisAndGravityForces"), std::string::npos);
  EXPECT_NE(operationalSource.find("mKp(i, i) = 50.0"), std::string::npos);
  EXPECT_NE(operationalSource.find("mKd(i, i) = 5.0"), std::string::npos);
  EXPECT_NE(operationalSource.find("mRobot->setForces"), std::string::npos);
  EXPECT_NE(
      operationalSource.find("makeOperationalSpaceRunDefaults"),
      std::string::npos);
  EXPECT_NE(operationalSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(operationalSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(
      operationalSource.find("makeOperationalSpaceCamera"), std::string::npos);
  EXPECT_NE(
      operationalSource.find("camera.up = Eigen::Vector3d(-0.24, -0.25, 0.94)"),
      std::string::npos);
  EXPECT_NE(
      operationalSource.find("camera.yaw = 0.8848934155088675"),
      std::string::npos);
  EXPECT_NE(
      operationalSource.find("camera.distance = 4.376539729055364"),
      std::string::npos);
  EXPECT_NE(operationalSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(operationalSource.find("options.ikHandles"), std::string::npos);
  EXPECT_NE(operationalSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(operationalSource.find("options.gizmos"), std::string::npos);
  EXPECT_NE(operationalSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(operationalSource.find("options.camera"), std::string::npos);
  EXPECT_NE(
      operationalSource.find("Left-drag the target gizmo"), std::string::npos);
  EXPECT_NE(operationalSource.find("Press 1 to select"), std::string::npos);
  EXPECT_NE(
      operationalSource.find("createOperationalSpaceKeyboardActions"),
      std::string::npos);
  EXPECT_NE(operationalSource.find("Toggle shadows"), std::string::npos);
  EXPECT_NE(
      operationalSource.find("makeToggleShadowsAction('S')"),
      std::string::npos);
  EXPECT_NE(operationalSource.find("Shadow On/Off"), std::string::npos);
  EXPECT_EQ(operationalSource.find("Ctrl-left drag"), std::string::npos);
  EXPECT_EQ(
      operationalSource.find("Hold key 1 or X while Ctrl-dragging"),
      std::string::npos);
  EXPECT_EQ(operationalSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(operationalSource.find("wam.urdf"), std::string::npos);
  EXPECT_NE(inputSource.find("GLFW_KEY_1"), std::string::npos);
  EXPECT_NE(inputSource.find("GLFW_KEY_2"), std::string::npos);
  EXPECT_NE(inputSource.find("GLFW_KEY_3"), std::string::npos);
  EXPECT_NE(selectionSource.find("GLFW_KEY_1"), std::string::npos);
  EXPECT_NE(selectionSource.find("GLFW_KEY_2"), std::string::npos);
  EXPECT_NE(selectionSource.find("GLFW_KEY_3"), std::string::npos);
  EXPECT_NE(
      operationalReadmeSource.find("Operational Space Control Example"),
      std::string::npos);
  EXPECT_NE(
      operationalReadmeSource.find("pixi run ex operational_space_control"),
      std::string::npos);
  EXPECT_NE(
      operationalReadmeSource.find("public `dart::gui::Gizmo`"),
      std::string::npos);
  EXPECT_NE(
      operationalReadmeSource.find("Press `1` to select"), std::string::npos);
  EXPECT_NE(
      operationalReadmeSource.find("Press `s` or `S` to toggle shadows"),
      std::string::npos);
  EXPECT_NE(operationalReadmeSource.find("--out"), std::string::npos);
  EXPECT_NE(
      operationalReadmeSource.find("shadow toggle are restored"),
      std::string::npos);
  EXPECT_EQ(
      operationalReadmeSource.find("shadow-control API gap"),
      std::string::npos);

  const auto wamSource = readSourceFile(
      std::filesystem::path("examples") / "wam_ikfast" / "main.cpp");
  const auto wamReadmeSource = readSourceFile(
      std::filesystem::path("examples") / "wam_ikfast" / "README.md");
  const auto wamCmakeSource = readSourceFile(
      std::filesystem::path("examples") / "wam_ikfast" / "CMakeLists.txt");
  const auto wamIkfastCmakeSource = readSourceFile(
      std::filesystem::path("examples") / "wam_ikfast" / "ikfast"
      / "CMakeLists.txt");
  EXPECT_EQ(wamSource.find("createTargetHandleShape"), std::string::npos);
  EXPECT_EQ(wamSource.find("LineSegmentShape"), std::string::npos);
  EXPECT_NE(wamSource.find("SharedLibraryIkFast"), std::string::npos);
  EXPECT_NE(wamSource.find("DART_WAM_IKFAST_LIB_PATH"), std::string::npos);
  EXPECT_NE(
      wamSource.find("std::vector<std::size_t>{0, 1, 3, 4, 5, 6}"),
      std::string::npos);
  EXPECT_NE(wamSource.find("std::vector<std::size_t>{2}"), std::string::npos);
  EXPECT_NE(wamSource.find("lh_target"), std::string::npos);
  EXPECT_NE(
      wamSource.find("createWamIkFastKeyboardActions"), std::string::npos);
  EXPECT_NE(wamSource.find("Toggle WAM IKFast target"), std::string::npos);
  EXPECT_NE(wamSource.find("Print WAM joint values"), std::string::npos);
  EXPECT_NE(wamSource.find("Reset WAM relaxed posture"), std::string::npos);
  EXPECT_NE(wamSource.find("targetState->solve"), std::string::npos);
  EXPECT_NE(wamSource.find("makeWamIkFastRunDefaults"), std::string::npos);
  EXPECT_NE(wamSource.find("options.width = 1280"), std::string::npos);
  EXPECT_NE(wamSource.find("options.height = 960"), std::string::npos);
  EXPECT_NE(wamSource.find("makeWamIkFastCamera"), std::string::npos);
  EXPECT_NE(
      wamSource.find("camera.distance = 6.285196894290584"), std::string::npos);
  EXPECT_NE(wamSource.find("options.ikHandles"), std::string::npos);
  EXPECT_NE(wamSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(wamSource.find("options.gizmos"), std::string::npos);
  EXPECT_NE(wamSource.find("BodyNodeDragHandle"), std::string::npos);
  EXPECT_NE(wamSource.find("options.bodyNodeDragHandles"), std::string::npos);
  EXPECT_NE(wamSource.find("gizmo.isVisible"), std::string::npos);
  EXPECT_NE(wamSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(wamSource.find("options.simulateWorld = false"), std::string::npos);
  EXPECT_NE(wamSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(wamSource.find("Left-drag active target gizmo"), std::string::npos);
  EXPECT_NE(wamSource.find("Alt + left-drag body"), std::string::npos);
  EXPECT_NE(wamSource.find("Ctrl + left-drag body"), std::string::npos);
  EXPECT_NE(wamSource.find("Shift + left-drag body"), std::string::npos);
  EXPECT_EQ(wamSource.find("Ctrl-left drag"), std::string::npos);
  EXPECT_NE(
      wamSource.find("Note that this is purely kinematic"), std::string::npos);
  EXPECT_EQ(wamSource.find("options.defaultScene"), std::string::npos);
  EXPECT_NE(wamCmakeSource.find("add_subdirectory(ikfast)"), std::string::npos);
  EXPECT_NE(wamCmakeSource.find("DART_WAM_IKFAST_LIB_PATH"), std::string::npos);
  EXPECT_NE(wamCmakeSource.find("wamIk"), std::string::npos);
  EXPECT_NE(wamIkfastCmakeSource.find("add_library(wamIk"), std::string::npos);
  EXPECT_NE(wamReadmeSource.find("WAM IKFast Example"), std::string::npos);
  EXPECT_NE(wamReadmeSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(wamReadmeSource.find("pixi run ex wam_ikfast"), std::string::npos);
  EXPECT_NE(wamReadmeSource.find("--screenshot"), std::string::npos);
  EXPECT_NE(wamReadmeSource.find("--out"), std::string::npos);
  EXPECT_NE(wamReadmeSource.find("simulateWorld = false"), std::string::npos);
  EXPECT_NE(wamReadmeSource.find("BodyNodeDragHandle"), std::string::npos);
  EXPECT_NE(wamReadmeSource.find("Alt/Ctrl/Shift"), std::string::npos);
  EXPECT_NE(wamReadmeSource.find("Shift-left-drag a body"), std::string::npos);
  EXPECT_EQ(
      wamReadmeSource.find("remains a public manipulation API gap"),
      std::string::npos);

  const auto tinkertoySource = readSourceFile(
      std::filesystem::path("examples") / "tinkertoy" / "main.cpp");
  const auto tinkertoyReadmeSource = readSourceFile(
      std::filesystem::path("examples") / "tinkertoy" / "README.md");
  const auto panelHeader
      = readSourceFile(std::filesystem::path("dart") / "gui" / "panel.hpp");
  EXPECT_EQ(tinkertoySource.find("createTargetHandleShape"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("LineSegmentShape"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("options.gizmos"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("tinkertoy_target"), std::string::npos);
  EXPECT_NE(
      tinkertoySource.find("syncPickFromSelectionContext"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("context.selectedPoint"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("context.selectedNormal"), std::string::npos);
  EXPECT_NE(
      tinkertoySource.find("kDefaultBlockWidth / 2.0"), std::string::npos);
  EXPECT_NE(panelHeader.find("selectedPoint"), std::string::npos);
  EXPECT_NE(panelHeader.find("selectedNormal"), std::string::npos);
  EXPECT_NE(panelHeader.find("LightingState"), std::string::npos);
  EXPECT_NE(panelHeader.find("headlightsEnabled"), std::string::npos);
  EXPECT_NE(
      tinkertoySource.find("Left-drag target gizmo arrows/planes/rings"),
      std::string::npos);
  EXPECT_EQ(tinkertoySource.find("Ctrl-left drag"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("class TinkertoyState"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("addWeldJointBlock"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("deletePick"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("setGravityEnabled"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("setForceCoeff"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("Tinkertoy Control"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("initialPosition"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("initialSize"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("backgroundAlpha = 0.5"), std::string::npos);
  EXPECT_NE(
      tinkertoySource.find("horizontalScrollbar = true"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("menuBar = true"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("Gravity On/Off"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("Headlights On/Off"), std::string::npos);
  EXPECT_NE(
      tinkertoySource.find("context.lighting.headlightsEnabled"),
      std::string::npos);
  EXPECT_NE(tinkertoySource.find("Force Coeff"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("Add a Weld-Joint Block"), std::string::npos);
  EXPECT_NE(
      tinkertoySource.find("Add a Revolute-Joint Block"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("Add a Ball-Joint Block"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("options.preStep"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("options.width = 1280"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("options.height = 720"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("KeyboardShortcut"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("KeyboardAction"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("KeyboardKey::Tab"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("KeyboardKey::Enter"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("context.resetCamera"), std::string::npos);
  EXPECT_NE(
      tinkertoySource.find("toggleTinkertoyRecording"), std::string::npos);
  EXPECT_NE(
      tinkertoySource.find("toggleFrameOutputCapture"), std::string::npos);
  EXPECT_NE(
      tinkertoySource.find("kTinkertoyRecordingDirectory"), std::string::npos);
  EXPECT_NE(
      tinkertoySource.find("openModal(\"About DART\""), std::string::npos);
  EXPECT_NE(
      tinkertoySource.find("beginModal(\"About DART\""), std::string::npos);
  EXPECT_NE(tinkertoySource.find("frameOutputEnabled"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("Start Recording"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("Stop Recording"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("CollisionAspect"), std::string::npos);
  EXPECT_NE(tinkertoySource.find("DynamicsAspect"), std::string::npos);
  EXPECT_NE(
      tinkertoyReadmeSource.find("Tinkertoy Builder Example"),
      std::string::npos);
  EXPECT_NE(
      tinkertoyReadmeSource.find("pixi run ex tinkertoy"), std::string::npos);
  EXPECT_NE(tinkertoyReadmeSource.find("dart::gui::Gizmo"), std::string::npos);
  EXPECT_NE(tinkertoyReadmeSource.find("--gui-scale"), std::string::npos);
  EXPECT_NE(tinkertoyReadmeSource.find("--out"), std::string::npos);
  EXPECT_NE(
      tinkertoyReadmeSource.find("Enter-key recording"), std::string::npos);
  EXPECT_EQ(
      tinkertoyReadmeSource.find("Runtime Enter-key recording remains"),
      std::string::npos);
  EXPECT_EQ(tinkertoyReadmeSource.find("viewer headlight"), std::string::npos);
  EXPECT_EQ(tinkertoySource.find("options.defaultScene"), std::string::npos);
}

TEST(FilamentSceneExtraction, StaticGeometryExamplesPreserveParityMarkers)
{
  const auto hardcodedSource = readSourceFile(
      std::filesystem::path("examples") / "hardcoded_design" / "main.cpp");
  const auto hardcodedReadmeSource = readSourceFile(
      std::filesystem::path("examples") / "hardcoded_design" / "README.md");
  EXPECT_NE(hardcodedSource.find("visual_hardcoded_design"), std::string::npos);
  EXPECT_NE(hardcodedSource.find("RevoluteJoint"), std::string::npos);
  EXPECT_NE(hardcodedSource.find("LHY"), std::string::npos);
  EXPECT_NE(hardcodedSource.find("LineSegmentShape"), std::string::npos);
  EXPECT_NE(hardcodedSource.find("createBoxWireframeShape"), std::string::npos);
  EXPECT_NE(hardcodedSource.find("addBoxWireframe"), std::string::npos);
  EXPECT_NE(hardcodedSource.find("kJointStep = 0.1"), std::string::npos);
  EXPECT_NE(hardcodedSource.find("moveDof"), std::string::npos);
  EXPECT_NE(hardcodedSource.find("toggleDirection"), std::string::npos);
  EXPECT_NE(
      hardcodedSource.find("createHardcodedDesignKeyboardActions"),
      std::string::npos);
  EXPECT_NE(
      hardcodedSource.find("makeMoveJointAction('1', 0, controls)"),
      std::string::npos);
  EXPECT_NE(
      hardcodedSource.find("makeMoveJointAction('2', 1, controls)"),
      std::string::npos);
  EXPECT_NE(
      hardcodedSource.find("makeMoveJointAction('3', 2, controls)"),
      std::string::npos);
  EXPECT_NE(
      hardcodedSource.find("KeyboardShortcut::characterKey(key)"),
      std::string::npos);
  EXPECT_NE(
      hardcodedSource.find("KeyboardShortcut::characterKey('-')"),
      std::string::npos);
  EXPECT_NE(
      hardcodedSource.find("makeHardcodedDesignCamera"), std::string::npos);
  EXPECT_NE(
      hardcodedSource.find("camera.target = Eigen::Vector3d(0.0, 0.0, 0.0)"),
      std::string::npos);
  EXPECT_NE(hardcodedSource.find("options.camera"), std::string::npos);
  EXPECT_NE(hardcodedSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(hardcodedSource.find("Hardcoded Design"), std::string::npos);
  EXPECT_NE(
      hardcodedReadmeSource.find("Hardcoded Design Example"),
      std::string::npos);
  EXPECT_NE(hardcodedReadmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(hardcodedReadmeSource.find("wireframe link"), std::string::npos);
  EXPECT_NE(
      hardcodedReadmeSource.find("DART line geometry"), std::string::npos);
  EXPECT_EQ(
      hardcodedReadmeSource.find("render-style or debug-mode API"),
      std::string::npos);
  EXPECT_EQ(hardcodedSource.find("options.defaultScene"), std::string::npos);

  const auto sourceGridHeader = readSourceFile(
      std::filesystem::path("examples") / "gui_source_grid.hpp");
  EXPECT_NE(sourceGridHeader.find("SourceOwnedGridState"), std::string::npos);
  EXPECT_NE(sourceGridHeader.find("SourceOwnedGridPlane"), std::string::npos);
  EXPECT_NE(sourceGridHeader.find("LineSegmentShape"), std::string::npos);
  EXPECT_NE(
      sourceGridHeader.find("addSourceOwnedGridPanelControls"),
      std::string::npos);
  EXPECT_NE(sourceGridHeader.find("Show Source Grid"), std::string::npos);
  EXPECT_NE(sourceGridHeader.find("Major Line Color"), std::string::npos);
  EXPECT_NE(sourceGridHeader.find("Minor Line Color"), std::string::npos);
  EXPECT_EQ(sourceGridHeader.find("GridVisual"), std::string::npos);

  const auto heightmapSource = readSourceFile(
      std::filesystem::path("examples") / "heightmap" / "main.cpp");
  const auto heightmapReadmeSource = readSourceFile(
      std::filesystem::path("examples") / "heightmap" / "README.md");
  EXPECT_NE(heightmapSource.find("visual_heightmap"), std::string::npos);
  EXPECT_NE(heightmapSource.find("HeightmapShaped"), std::string::npos);
  EXPECT_NE(
      heightmapSource.find("visual_heightmap_sample_ball_"), std::string::npos);
  EXPECT_NE(heightmapSource.find("--demo"), std::string::npos);
  EXPECT_NE(heightmapSource.find("HeightmapDemo"), std::string::npos);
  EXPECT_NE(heightmapSource.find("createHeightmapPanel"), std::string::npos);
  EXPECT_NE(
      heightmapSource.find("makeHeightmapRunDefaults"), std::string::npos);
  EXPECT_NE(heightmapSource.find("options.width = 1280"), std::string::npos);
  EXPECT_NE(heightmapSource.find("options.height = 720"), std::string::npos);
  EXPECT_NE(heightmapSource.find("makeHeightmapCamera"), std::string::npos);
  EXPECT_NE(heightmapSource.find("Heightmap Demo"), std::string::npos);
  EXPECT_NE(
      heightmapSource.find("Heightmap rendering example"), std::string::npos);
  EXPECT_NE(heightmapSource.find("builder.slider"), std::string::npos);
  EXPECT_NE(heightmapSource.find("Show Terrain"), std::string::npos);
  EXPECT_NE(heightmapSource.find("Terrain Color"), std::string::npos);
  EXPECT_NE(heightmapSource.find("SourceOwnedGridState"), std::string::npos);
  EXPECT_NE(
      heightmapSource.find("attachSourceOwnedGridFrames"), std::string::npos);
  EXPECT_NE(
      heightmapSource.find("addSourceOwnedGridPanelControls"),
      std::string::npos);
  EXPECT_NE(heightmapSource.find("Regenerate"), std::string::npos);
  EXPECT_NE(heightmapSource.find("requestSingleStep"), std::string::npos);
  EXPECT_NE(heightmapSource.find("requestExit"), std::string::npos);
  EXPECT_NE(heightmapSource.find("setupAlignmentDemo"), std::string::npos);
  EXPECT_NE(heightmapSource.find("DART_HAVE_ODE"), std::string::npos);
  EXPECT_NE(heightmapSource.find("heightmap_ball_"), std::string::npos);
  EXPECT_NE(heightmapSource.find("box_ball_"), std::string::npos);
  EXPECT_NE(heightmapSource.find("reference_box"), std::string::npos);
  EXPECT_EQ(heightmapSource.find("debug-grid API"), std::string::npos);
  EXPECT_NE(heightmapReadmeSource.find("Heightmap Example"), std::string::npos);
  EXPECT_NE(heightmapReadmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(
      heightmapReadmeSource.find("edit the terrain color"), std::string::npos);
  EXPECT_NE(heightmapReadmeSource.find("line count"), std::string::npos);
  EXPECT_NE(heightmapReadmeSource.find("major/minor/axis"), std::string::npos);
  EXPECT_NE(heightmapReadmeSource.find("--demo alignment"), std::string::npos);
  EXPECT_EQ(heightmapSource.find("options.defaultScene"), std::string::npos);

  const auto pointCloudSource = readSourceFile(
      std::filesystem::path("examples") / "point_cloud" / "main.cpp");
  const auto pointCloudReadmeSource = readSourceFile(
      std::filesystem::path("examples") / "point_cloud" / "README.md");
  EXPECT_NE(pointCloudSource.find("visual_point_cloud"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("PointCloudShape"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("point_cloud_sensor"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("KR5 sixx R650.urdf"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("ground.urdf"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("PointSamplingMode"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("SampleOnRobot"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("SampleInBox"), std::string::npos);
  EXPECT_NE(
      pointCloudSource.find("generatePointCloudOnRobot"), std::string::npos);
  EXPECT_NE(
      pointCloudSource.find("generatePointCloudInBox"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("updateRobotPose"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("updateSensor"), std::string::npos);
  EXPECT_EQ(pointCloudSource.find("DART_HAVE_OCTOMAP"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("VoxelGridShape"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("visual_voxel_grid"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("point_cloud_grid"), std::string::npos);
  EXPECT_NE(
      pointCloudSource.find("state->grid.visible = false"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("SourceOwnedGridState"), std::string::npos);
  EXPECT_NE(
      pointCloudSource.find("attachSourceOwnedGridFrames"), std::string::npos);
  EXPECT_NE(
      pointCloudSource.find("Point Cloud & Voxel Grid Demo"),
      std::string::npos);
  EXPECT_NE(
      pointCloudSource.find("Point cloud and voxel grid rendering example"),
      std::string::npos);
  EXPECT_NE(pointCloudSource.find("Run Robot Updating"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("Sample on robot"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("Sample in box"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("Cycle Color Mode"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("Point Cloud Color"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("Voxel Grid Color"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("Cycle Point Shape Type"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("Visual Size"), std::string::npos);
  EXPECT_EQ(pointCloudSource.find("debug-grid API"), std::string::npos);
  EXPECT_EQ(
      pointCloudSource.find("Color editors and detailed debug-grid"),
      std::string::npos);
  EXPECT_NE(
      pointCloudSource.find("makePointCloudRunDefaults"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("options.width = 1280"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("options.height = 720"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("makePointCloudCamera"), std::string::npos);
  EXPECT_NE(pointCloudSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(
      pointCloudReadmeSource.find("Point Cloud Example"), std::string::npos);
  EXPECT_NE(pointCloudReadmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(pointCloudReadmeSource.find("1280x720"), std::string::npos);
  EXPECT_NE(
      pointCloudReadmeSource.find("Point Cloud Color"), std::string::npos);
  EXPECT_NE(
      pointCloudReadmeSource.find("Scene-grid controls"), std::string::npos);
  EXPECT_NE(pointCloudReadmeSource.find("line count"), std::string::npos);
  EXPECT_EQ(pointCloudSource.find("options.defaultScene"), std::string::npos);

  const auto polyhedronSource = readSourceFile(
      std::filesystem::path("examples") / "polyhedron_visual" / "main.cpp");
  const auto polyhedronReadmeSource = readSourceFile(
      std::filesystem::path("examples") / "polyhedron_visual" / "README.md");
  EXPECT_NE(
      polyhedronSource.find("visual_polyhedron_surface"), std::string::npos);
  EXPECT_NE(
      polyhedronSource.find("visual_polyhedron_wireframe"), std::string::npos);
  EXPECT_NE(polyhedronSource.find("visual_polyhedron_grid"), std::string::npos);
  EXPECT_NE(polyhedronSource.find("ConvexMeshShape"), std::string::npos);
  EXPECT_NE(polyhedronSource.find("LineSegmentShape"), std::string::npos);
  EXPECT_NE(
      polyhedronSource.find("createPolyhedronGridShape"), std::string::npos);
  EXPECT_NE(
      polyhedronSource.find("makePolyhedronRunDefaults"), std::string::npos);
  EXPECT_NE(polyhedronSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(polyhedronSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(polyhedronSource.find("makePolyhedronCamera"), std::string::npos);
  EXPECT_NE(
      polyhedronSource.find("camera.target = Eigen::Vector3d(0.0, 0.0, 0.4)"),
      std::string::npos);
  EXPECT_NE(polyhedronSource.find("options.camera"), std::string::npos);
  EXPECT_NE(polyhedronSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(
      polyhedronReadmeSource.find("Polyhedron Visual Example"),
      std::string::npos);
  EXPECT_NE(polyhedronReadmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(polyhedronReadmeSource.find("640x480"), std::string::npos);
  EXPECT_EQ(polyhedronSource.find("options.defaultScene"), std::string::npos);
}

TEST(FilamentSceneExtraction, LcpAndMimicExamplesPreserveParityMarkers)
{
  const auto lcpSource = readSourceFile(
      std::filesystem::path("examples") / "lcp_physics" / "main.cpp");
  const auto lcpReadmeSource = readSourceFile(
      std::filesystem::path("examples") / "lcp_physics" / "README.md");
  EXPECT_NE(
      lcpSource.find("constexpr int kBallDropSphereCount = 75"),
      std::string::npos);
  EXPECT_NE(
      lcpSource.find("constexpr int kBoxStackLayers = 5"), std::string::npos);
  EXPECT_NE(
      lcpSource.find("constexpr int kDominoCount = 20"), std::string::npos);
  EXPECT_NE(lcpSource.find("\"light_box\""), std::string::npos);
  EXPECT_NE(lcpSource.find("\"heavy_box\""), std::string::npos);
  EXPECT_NE(lcpSource.find("\"box_\""), std::string::npos);
  EXPECT_NE(lcpSource.find("\"ball_\""), std::string::npos);
  EXPECT_NE(lcpSource.find("\"domino_\""), std::string::npos);
  EXPECT_NE(lcpSource.find("DantzigSolver"), std::string::npos);
  EXPECT_NE(lcpSource.find("PgsSolver"), std::string::npos);
  EXPECT_NE(lcpSource.find("--scenario"), std::string::npos);
  EXPECT_NE(lcpSource.find("--solver"), std::string::npos);
  EXPECT_NE(lcpSource.find("--list"), std::string::npos);
  EXPECT_NE(lcpSource.find("resetLcpWorld"), std::string::npos);
  EXPECT_NE(lcpSource.find("builder.button(label)"), std::string::npos);
  EXPECT_NE(
      lcpSource.find("builder.slider(\"Timestep (Hz)\""), std::string::npos);
  EXPECT_NE(
      lcpSource.find("builder.slider(\"Gravity (m/s^2)\""), std::string::npos);
  EXPECT_NE(
      lcpSource.find("std::deque<double> stepTimeHistory"), std::string::npos);
  EXPECT_NE(lcpSource.find("updateRenderFps"), std::string::npos);
  EXPECT_NE(lcpSource.find("render fps:"), std::string::npos);
  EXPECT_NE(
      lcpSource.find("builder.beginTable(\"lcp_performance\""),
      std::string::npos);
  EXPECT_NE(
      lcpSource.find("builder.plotLines(\"Step time (ms)\""),
      std::string::npos);
  EXPECT_NE(
      lcpSource.find("builder.collapsingHeader(\"UI Debug\""),
      std::string::npos);
  EXPECT_NE(lcpSource.find("DisplaySize:"), std::string::npos);
  EXPECT_NE(lcpSource.find("FramebufferScale:"), std::string::npos);
  EXPECT_NE(lcpSource.find("FontSize:"), std::string::npos);
  EXPECT_NE(lcpSource.find("FontGlobalScale:"), std::string::npos);
  EXPECT_NE(lcpSource.find("UiScale:"), std::string::npos);
  EXPECT_NE(lcpSource.find("FontTex:"), std::string::npos);
  EXPECT_EQ(
      lcpSource.find("Display/font metrics need backend debug access"),
      std::string::npos);
  EXPECT_EQ(
      lcpSource.find("Step-time line plot needs a public panel plotting API"),
      std::string::npos);
  EXPECT_EQ(
      lcpSource.find("step-time plots need a public panel plotting"),
      std::string::npos);
  EXPECT_EQ(lcpSource.find("ImGui"), std::string::npos);
  EXPECT_NE(lcpSource.find("createInclinedPlaneScenario"), std::string::npos);
  EXPECT_NE(lcpSource.find("makeLcpRunDefaults"), std::string::npos);
  EXPECT_NE(lcpSource.find("options.width = 1280"), std::string::npos);
  EXPECT_NE(lcpSource.find("options.height = 720"), std::string::npos);
  EXPECT_NE(lcpSource.find("makeLcpCamera"), std::string::npos);
  EXPECT_NE(
      lcpSource.find("camera.target = Eigen::Vector3d(0.0, 0.3, 0.0)"),
      std::string::npos);
  EXPECT_NE(lcpSource.find("options.world"), std::string::npos);
  EXPECT_NE(lcpReadmeSource.find("LCP Physics Example"), std::string::npos);
  EXPECT_NE(lcpReadmeSource.find("mass_ratio"), std::string::npos);
  EXPECT_NE(lcpReadmeSource.find("75 balls dropping"), std::string::npos);
  EXPECT_NE(lcpReadmeSource.find("Command-Line Options"), std::string::npos);
  EXPECT_NE(lcpReadmeSource.find("--out <dir>"), std::string::npos);
  EXPECT_EQ(lcpSource.find("options.defaultScene"), std::string::npos);

  const auto mimicSource = readSourceFile(
      std::filesystem::path("examples") / "mimic_pendulums" / "main.cpp");
  const auto mimicReadmeSource = readSourceFile(
      std::filesystem::path("examples") / "mimic_pendulums" / "README.md");
  EXPECT_NE(
      mimicSource.find(
          "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf"),
      std::string::npos);
  EXPECT_NE(mimicSource.find("mimic_pendulums_xy_grid"), std::string::npos);
  EXPECT_NE(mimicSource.find("LineSegmentShape"), std::string::npos);
  EXPECT_NE(mimicSource.find("pendulum_with_base\""), std::string::npos);
  EXPECT_NE(
      mimicSource.find("pendulum_with_base_mimic_slow_follows_fast"),
      std::string::npos);
  EXPECT_NE(
      mimicSource.find("pendulum_with_base_mimic_fast_follows_slow"),
      std::string::npos);
  EXPECT_NE(mimicSource.find("retargetMimicsToBaseline"), std::string::npos);
  EXPECT_NE(mimicSource.find("collectMimicPairs"), std::string::npos);
  EXPECT_NE(mimicSource.find("tintBases"), std::string::npos);
  EXPECT_NE(mimicSource.find("base drift"), std::string::npos);
  EXPECT_NE(mimicSource.find("colorSwatch"), std::string::npos);
  EXPECT_NE(mimicSource.find("beginTable(\"mimic_table\""), std::string::npos);
  EXPECT_NE(mimicSource.find("Reference (rad)"), std::string::npos);
  EXPECT_NE(mimicSource.find("Follower (rad)"), std::string::npos);
  EXPECT_NE(mimicSource.find("toDegree"), std::string::npos);
  EXPECT_NE(mimicSource.find("Reset world"), std::string::npos);
  EXPECT_NE(mimicSource.find("Collision / solver"), std::string::npos);
  EXPECT_NE(mimicSource.find("Use ODE collision"), std::string::npos);
  EXPECT_NE(mimicSource.find("Force PGS solver"), std::string::npos);
  EXPECT_NE(mimicSource.find("Contacts last step"), std::string::npos);
  EXPECT_NE(mimicSource.find("--solver"), std::string::npos);
  EXPECT_NE(mimicSource.find("--collision"), std::string::npos);
  EXPECT_NE(mimicSource.find("PgsSolver"), std::string::npos);
  EXPECT_NE(
      mimicSource.find("makeMimicPendulumsRunDefaults"), std::string::npos);
  EXPECT_NE(mimicSource.find("options.width = 1280"), std::string::npos);
  EXPECT_NE(mimicSource.find("options.height = 720"), std::string::npos);
  EXPECT_NE(mimicSource.find("makeMimicPendulumsCamera"), std::string::npos);
  EXPECT_NE(
      mimicSource.find("camera.target = Eigen::Vector3d(0.5, 0.0, 1.5)"),
      std::string::npos);
  EXPECT_NE(mimicSource.find("options.world"), std::string::npos);
  EXPECT_NE(
      mimicReadmeSource.find("Mimic Pendulums Example"), std::string::npos);
  EXPECT_NE(mimicReadmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(mimicReadmeSource.find("color legend"), std::string::npos);
  EXPECT_NE(mimicReadmeSource.find("Command-Line Options"), std::string::npos);
  EXPECT_NE(mimicReadmeSource.find("--out <dir>"), std::string::npos);
  EXPECT_EQ(mimicSource.find("options.defaultScene"), std::string::npos);
}

TEST(FilamentSceneExtraction, InteractionEventExamplesPreserveParityMarkers)
{
  const auto emptySource = readSourceFile(
      std::filesystem::path("examples") / "empty" / "main.cpp");
  const auto emptyReadmeSource = readSourceFile(
      std::filesystem::path("examples") / "empty" / "README.md");
  EXPECT_NE(emptySource.find("interactive frame"), std::string::npos);
  EXPECT_NE(emptySource.find("draggable"), std::string::npos);
  EXPECT_NE(emptySource.find("createEmptyWorld"), std::string::npos);
  EXPECT_NE(emptySource.find("emptyPreStepScaffold"), std::string::npos);
  EXPECT_NE(emptySource.find("emptyPostStepScaffold"), std::string::npos);
  EXPECT_NE(emptySource.find("emptyPreRenderScaffold"), std::string::npos);
  EXPECT_NE(emptySource.find("emptyPostRenderScaffold"), std::string::npos);
  EXPECT_NE(emptySource.find("createEmptyKeyboardActions"), std::string::npos);
  EXPECT_NE(
      emptySource.find("KeyboardShortcut::characterKey('q')"),
      std::string::npos);
  EXPECT_NE(
      emptySource.find("KeyboardShortcut::characterKey('Q')"),
      std::string::npos);
  EXPECT_NE(emptySource.find("KeyboardKey::Left"), std::string::npos);
  EXPECT_NE(emptySource.find("KeyboardKey::Right"), std::string::npos);
  EXPECT_NE(emptySource.find("Lowercase q pressed"), std::string::npos);
  EXPECT_NE(emptySource.find("Capital Q pressed"), std::string::npos);
  EXPECT_NE(emptySource.find("Lowercase q released"), std::string::npos);
  EXPECT_NE(emptySource.find("Capital Q released"), std::string::npos);
  EXPECT_NE(
      emptySource.find("KeyboardActionTrigger::Release"), std::string::npos);
  EXPECT_NE(emptySource.find("Left arrow key pressed"), std::string::npos);
  EXPECT_NE(emptySource.find("Left arrow key released"), std::string::npos);
  EXPECT_NE(emptySource.find("Right arrow key pressed"), std::string::npos);
  EXPECT_NE(emptySource.find("Right arrow key released"), std::string::npos);
  EXPECT_NE(emptySource.find("Pre-step callbacks"), std::string::npos);
  EXPECT_NE(emptySource.find("Post-step callbacks"), std::string::npos);
  EXPECT_NE(emptySource.find("Pre-render callbacks"), std::string::npos);
  EXPECT_NE(emptySource.find("Post-render callbacks"), std::string::npos);
  EXPECT_NE(emptySource.find("makeEmptyRunDefaults"), std::string::npos);
  EXPECT_NE(emptySource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(emptySource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(emptySource.find("makeEmptyCamera"), std::string::npos);
  EXPECT_NE(
      emptySource.find("camera.target = Eigen::Vector3d::Zero()"),
      std::string::npos);
  EXPECT_NE(
      emptySource.find("camera.up = Eigen::Vector3d(-0.24, -0.25, 0.94)"),
      std::string::npos);
  EXPECT_NE(emptySource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(emptySource.find("options.preStep"), std::string::npos);
  EXPECT_NE(emptySource.find("options.postStep"), std::string::npos);
  EXPECT_NE(emptySource.find("options.preRender"), std::string::npos);
  EXPECT_NE(emptySource.find("options.postRender"), std::string::npos);
  EXPECT_NE(emptySource.find("options.world"), std::string::npos);
  EXPECT_NE(emptyReadmeSource.find("Empty Viewer Example"), std::string::npos);
  EXPECT_NE(emptyReadmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(emptyReadmeSource.find("640x480"), std::string::npos);
  EXPECT_NE(
      emptyReadmeSource.find("keydown and key-release"), std::string::npos);
  EXPECT_NE(emptyReadmeSource.find("render callbacks"), std::string::npos);
  EXPECT_EQ(emptyReadmeSource.find("public API follow-up"), std::string::npos);
  EXPECT_EQ(emptySource.find("options.defaultScene"), std::string::npos);

  const auto eventSource = readSourceFile(
      std::filesystem::path("examples") / "simulation_event_handler"
      / "main.cpp");
  const auto eventReadmeSource = readSourceFile(
      std::filesystem::path("examples") / "simulation_event_handler"
      / "README.md");
  EXPECT_NE(
      eventSource.find("class BlinkingMarkerSensor final"), std::string::npos);
  EXPECT_NE(eventSource.find("class SimulationEventState"), std::string::npos);
  EXPECT_NE(eventSource.find("fast_sensor"), std::string::npos);
  EXPECT_NE(eventSource.find("slow_sensor"), std::string::npos);
  EXPECT_NE(
      eventSource.find("simulation_event_handler_fast_sensor"),
      std::string::npos);
  EXPECT_NE(
      eventSource.find("simulation_event_handler_force_arrow"),
      std::string::npos);
  EXPECT_NE(eventSource.find("LineSegmentShape"), std::string::npos);
  EXPECT_NE(
      eventSource.find("createSimulationEventKeyboardActions"),
      std::string::npos);
  EXPECT_NE(eventSource.find("KeyboardKey::Tab"), std::string::npos);
  EXPECT_NE(eventSource.find("KeyboardKey::Backspace"), std::string::npos);
  EXPECT_NE(eventSource.find("KeyboardKey::Up"), std::string::npos);
  EXPECT_NE(eventSource.find("KeyboardKey::Down"), std::string::npos);
  EXPECT_NE(eventSource.find("KeyboardKey::Left"), std::string::npos);
  EXPECT_NE(eventSource.find("KeyboardKey::Right"), std::string::npos);
  EXPECT_NE(
      eventSource.find("KeyboardShortcut::characterKey"), std::string::npos);
  EXPECT_NE(eventSource.find("character('?')"), std::string::npos);
  EXPECT_NE(eventSource.find("addExtForce"), std::string::npos);
  EXPECT_NE(eventSource.find("addExtTorque"), std::string::npos);
  EXPECT_NE(
      eventSource.find("makeSimulationEventRunDefaults"), std::string::npos);
  EXPECT_NE(eventSource.find("options.width = 1280"), std::string::npos);
  EXPECT_NE(eventSource.find("options.height = 960"), std::string::npos);
  EXPECT_NE(eventSource.find("makeSimulationEventCamera"), std::string::npos);
  EXPECT_NE(
      eventSource.find("camera.target = Eigen::Vector3d(0.0, 0.0, 1.0)"),
      std::string::npos);
  EXPECT_NE(eventSource.find("world->addSensor"), std::string::npos);
  EXPECT_NE(eventSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(eventSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(eventSource.find("options.world"), std::string::npos);
  EXPECT_NE(
      eventReadmeSource.find("Simulation Event Handler Example"),
      std::string::npos);
  EXPECT_NE(eventReadmeSource.find("dart::gui"), std::string::npos);
  EXPECT_NE(eventReadmeSource.find("Tab / Backspace"), std::string::npos);
  EXPECT_NE(eventReadmeSource.find("`h` / `?`"), std::string::npos);
  EXPECT_EQ(eventSource.find("options.defaultScene"), std::string::npos);
}

TEST(FilamentSceneExtraction, SoftBodiesAndVehicleExamplesPreserveParityMarkers)
{
  const auto softBodiesSource = readSourceFile(
      std::filesystem::path("examples") / "soft_bodies" / "main.cpp");
  const auto softBodiesReadmeSource = readSourceFile(
      std::filesystem::path("examples") / "soft_bodies" / "README.md");
  EXPECT_NE(
      softBodiesSource.find("dart://sample/skel/softBodies.skel"),
      std::string::npos);
  EXPECT_NE(softBodiesSource.find("class SoftBodyHistory"), std::string::npos);
  EXPECT_NE(softBodiesSource.find("captureStepStart"), std::string::npos);
  EXPECT_NE(softBodiesSource.find("moveBackward"), std::string::npos);
  EXPECT_NE(softBodiesSource.find("makePlaybackAction"), std::string::npos);
  EXPECT_NE(
      softBodiesSource.find("KeyboardShortcut::characterKey(key)"),
      std::string::npos);
  EXPECT_NE(
      softBodiesSource.find("createSoftBodiesKeyboardActions"),
      std::string::npos);
  EXPECT_NE(
      softBodiesSource.find("Move soft-body playback backward one frame"),
      std::string::npos);
  EXPECT_NE(
      softBodiesSource.find("Move soft-body playback forward one frame"),
      std::string::npos);
  EXPECT_NE(
      softBodiesSource.find("Move soft-body playback backward ten frames"),
      std::string::npos);
  EXPECT_NE(
      softBodiesSource.find("Move soft-body playback forward ten frames"),
      std::string::npos);
  EXPECT_NE(
      softBodiesSource.find("Restart soft-body playback"), std::string::npos);
  EXPECT_NE(
      softBodiesSource.find("Jump soft-body playback to latest frame"),
      std::string::npos);
  EXPECT_NE(
      softBodiesSource.find("'['/']': move backward/forward one frame"),
      std::string::npos);
  EXPECT_NE(
      softBodiesSource.find("'{'/'}': move backward/forward ten frames"),
      std::string::npos);
  EXPECT_NE(
      softBodiesSource.find("printSoftBodiesInstructions"), std::string::npos);
  EXPECT_NE(softBodiesSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(softBodiesSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(softBodiesSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(softBodiesSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(
      softBodiesSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(softBodiesSource.find("options.world"), std::string::npos);
  EXPECT_EQ(softBodiesSource.find("options.defaultScene"), std::string::npos);
  EXPECT_NE(
      softBodiesReadmeSource.find("Soft Bodies Example"), std::string::npos);
  EXPECT_NE(
      softBodiesReadmeSource.find("pixi run ex soft_bodies"),
      std::string::npos);
  EXPECT_NE(softBodiesReadmeSource.find("--out"), std::string::npos);

  const auto inputSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "input.cpp");
  EXPECT_NE(inputSource.find("GLFW_KEY_LEFT_BRACKET"), std::string::npos);
  EXPECT_NE(inputSource.find("GLFW_KEY_RIGHT_BRACKET"), std::string::npos);
  EXPECT_NE(inputSource.find("GLFW_KEY_SLASH"), std::string::npos);
  EXPECT_NE(inputSource.find("GLFW_KEY_BACKSLASH"), std::string::npos);
  EXPECT_NE(inputSource.find("case '?'"), std::string::npos);
  EXPECT_NE(inputSource.find("isShiftDown"), std::string::npos);

  const auto vehicleSource = readSourceFile(
      std::filesystem::path("examples") / "vehicle" / "main.cpp");
  const auto vehicleReadmeSource = readSourceFile(
      std::filesystem::path("examples") / "vehicle" / "README.md");
  EXPECT_NE(
      vehicleSource.find("dart://sample/skel/vehicle.skel"), std::string::npos);
  EXPECT_NE(vehicleSource.find("visual_vehicle_car"), std::string::npos);
  EXPECT_NE(vehicleSource.find("wheel_front_left"), std::string::npos);
  EXPECT_NE(vehicleSource.find("kWheelSpeedCommand"), std::string::npos);
  EXPECT_NE(vehicleSource.find("mBackWheelVelocity"), std::string::npos);
  EXPECT_NE(vehicleSource.find("forces[6]"), std::string::npos);
  EXPECT_NE(vehicleSource.find("makeVehicleCamera"), std::string::npos);
  EXPECT_NE(vehicleSource.find("makeVehicleAction"), std::string::npos);
  EXPECT_NE(
      vehicleSource.find("KeyboardShortcut::characterKey(key)"),
      std::string::npos);
  EXPECT_NE(
      vehicleSource.find("createVehicleKeyboardActions"), std::string::npos);
  EXPECT_NE(vehicleSource.find("Move vehicle forward"), std::string::npos);
  EXPECT_NE(vehicleSource.find("Stop vehicle"), std::string::npos);
  EXPECT_NE(vehicleSource.find("Move vehicle backward"), std::string::npos);
  EXPECT_NE(vehicleSource.find("Steer vehicle left"), std::string::npos);
  EXPECT_NE(vehicleSource.find("Steer vehicle right"), std::string::npos);
  EXPECT_NE(vehicleSource.find("printVehicleInstructions"), std::string::npos);
  EXPECT_NE(vehicleSource.find("'w': move forward"), std::string::npos);
  EXPECT_NE(
      vehicleSource.find("'a': rotate steering wheels to left"),
      std::string::npos);
  EXPECT_NE(vehicleSource.find("options.runDefaults"), std::string::npos);
  EXPECT_NE(vehicleSource.find("options.width = 640"), std::string::npos);
  EXPECT_NE(vehicleSource.find("options.height = 480"), std::string::npos);
  EXPECT_NE(vehicleSource.find("options.camera"), std::string::npos);
  EXPECT_NE(vehicleSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(vehicleSource.find("options.keyboardActions"), std::string::npos);
  EXPECT_NE(vehicleSource.find("options.world"), std::string::npos);
  EXPECT_EQ(vehicleSource.find("options.defaultScene"), std::string::npos);
  EXPECT_NE(vehicleReadmeSource.find("Vehicle Example"), std::string::npos);
  EXPECT_NE(vehicleReadmeSource.find("pixi run ex vehicle"), std::string::npos);
  EXPECT_NE(vehicleReadmeSource.find("--screenshot"), std::string::npos);
  EXPECT_NE(vehicleReadmeSource.find("--out"), std::string::npos);
}

TEST(
    FilamentSceneExtraction,
    DartsimApplicationHeadersAvoidDirectFilamentIncludes)
{
  const auto headers
      = listPublicHeadersInDirectory(kDartsimApplicationDirectory);

  const auto violations
      = scanSourceFilesForTokens(headers, kForbiddenFilamentIncludeTokens);
  for (const auto& violation : violations) {
    ADD_FAILURE() << violation.source << " includes Filament header token `"
                  << violation.token << "` directly";
  }
}

TEST(
    FilamentSceneExtraction,
    DartsimApplicationEntryPointAvoidsDirectFilamentIncludes)
{
  const std::vector<std::filesystem::path> sources
      = {kDartsimApplicationDirectory / "main.cpp"};

  const auto violations
      = scanSourceFilesForTokens(sources, kForbiddenFilamentIncludeTokens);
  for (const auto& violation : violations) {
    ADD_FAILURE() << violation.source << " includes Filament header token `"
                  << violation.token << "` directly";
  }
}

TEST(FilamentSceneExtraction, DartsimApplicationEntryPointAvoidsBackendTokens)
{
  const std::vector<std::filesystem::path> sources
      = {kDartsimApplicationDirectory / "main.cpp"};

  const auto violations
      = scanSourceFilesForTokens(sources, kForbiddenBackendTokens);
  for (const auto& violation : violations) {
    ADD_FAILURE() << violation.source << " reaches backend token `"
                  << violation.token << "` directly";
  }
}

TEST(FilamentSceneExtraction, DartsimApplicationEntryPointUsesPublicGuiBoundary)
{
  const std::vector<std::filesystem::path> sources
      = {kDartsimApplicationDirectory / "main.cpp"};

  const auto violations
      = scanSourceFilesForTokens(sources, kForbiddenDetailTokens);
  for (const auto& violation : violations) {
    ADD_FAILURE() << violation.source << " reaches backend detail token `"
                  << violation.token << "` directly";
  }
}

TEST(FilamentSceneExtraction, DartsimApplicationEntryPointStaysMinimal)
{
  const auto mainSource
      = readSourceFile(kDartsimApplicationDirectory / "main.cpp");

  EXPECT_EQ(countOccurrences(mainSource, "#include "), 1u);
  EXPECT_NE(
      mainSource.find("#include <dartsim_ui/editor.hpp>"), std::string::npos);
  EXPECT_EQ(countOccurrences(mainSource, "int main("), 1u);
  EXPECT_EQ(
      countOccurrences(
          mainSource, "return dartsim::ui::runEditor(argc, argv);"),
      1u);
}

TEST(FilamentSceneExtraction, DartsimSceneFixtureModeSkipsEditorPanels)
{
  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");

  EXPECT_NE(
      editorSource.find("std::string_view(argv[i]) == \"--scene\""),
      std::string::npos);

  const auto fixtureModeGuard
      = editorSource.find("if (hasSceneOption(argc, argv))");
  ASSERT_NE(fixtureModeGuard, std::string::npos);
  const auto fixtureModeReturn = editorSource.find(
      "return dart::gui::runApplication(argc, argv);", fixtureModeGuard);
  ASSERT_NE(fixtureModeReturn, std::string::npos);

  const auto appConstruction
      = editorSource.find("auto app = std::make_shared<EditorApp>();");
  const auto demoSeeding = editorSource.find("seedDemoScene(*app);");
  const auto panelRegistration = editorSource.find("options.panels.push_back");
  ASSERT_NE(appConstruction, std::string::npos);
  ASSERT_NE(demoSeeding, std::string::npos);
  ASSERT_NE(panelRegistration, std::string::npos);

  EXPECT_LT(fixtureModeReturn, appConstruction);
  EXPECT_LT(fixtureModeReturn, demoSeeding);
  EXPECT_LT(fixtureModeReturn, panelRegistration);
}

TEST(FilamentSceneExtraction, DartsimSceneTreeSelectionUsesEngineFacade)
{
  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");

  EXPECT_EQ(
      editorSource.find("app.engine.selection().select(id)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("selectOutlinerObject(app.engine, row.id);"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("toggleOutlinerObjectSelection(app.engine, row.id)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("clearOutlinerSelection(app.engine)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("selectViewportRenderable(app->engine, renderableId)"),
      std::string::npos);
}

TEST(FilamentSceneExtraction, DartsimSceneTreeUsesOutlinerStateAndActions)
{
  const auto panelHeader = readSourceFile(kDartGuiDirectory / "panel.hpp");
  const auto panelSource
      = readSourceFile(kDartGuiDirectory / "detail/panel.cpp");
  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");

  EXPECT_NE(
      panelHeader.find("textInput(std::string_view label"), std::string::npos);
  EXPECT_NE(panelSource.find("ImGui::InputText("), std::string::npos);
  EXPECT_NE(editorSource.find("OutlinerState outliner"), std::string::npos);
  EXPECT_NE(
      editorSource.find("buildOutlinerRows(app.engine, app.outliner)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("toggleOutlinerExpanded(app.outliner"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("commitOutlinerRename(app.engine, app.outliner)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("buildOutlinerContextActions(app.engine, row.id)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("applyOutlinerContextAction("), std::string::npos);
}

TEST(FilamentSceneExtraction, DartsimViewportTransformUsesActionSeam)
{
  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");

  EXPECT_NE(
      editorSource.find("#include <dartsim_ui/viewport_actions.hpp>"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("options.gizmos.push_back(app->transformGizmo.gizmo)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("applyViewportTransformGizmo("), std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "options.keyboardActions = makeViewportMoveActions(app)"),
      std::string::npos);
}

TEST(FilamentSceneExtraction, DartsimSimulationPanelUsesActionSeam)
{
  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");

  EXPECT_NE(
      editorSource.find("#include <dartsim_ui/simulation_actions.hpp>"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "const SimulationStatus status = buildSimulationStatus"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("buildSimulationModeActions(app.engine)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("applySimulationModeAction(app.engine, action.kind)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("ui.text(status.modeDescription)"), std::string::npos);
  EXPECT_NE(
      editorSource.find("ui.text(status.editStateLabel)"), std::string::npos);
  EXPECT_NE(
      editorSource.find("setSimulationRecording(app.engine, record)"),
      std::string::npos);
  EXPECT_EQ(
      editorSource.find("app.engine.simulation().play()"), std::string::npos);
  EXPECT_EQ(
      editorSource.find("app.engine.simulation().pause()"), std::string::npos);
  EXPECT_EQ(
      editorSource.find("app.engine.simulation().step(1)"), std::string::npos);
  EXPECT_EQ(
      editorSource.find("app.engine.simulation().reset()"), std::string::npos);
}

TEST(FilamentSceneExtraction, DartsimRelationshipMenuUsesActionSeam)
{
  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");
  const auto cmakeSource
      = readSourceFile(kDartsimUiDirectory / "CMakeLists.txt");

  EXPECT_NE(
      editorSource.find("#include <dartsim_ui/relationship_actions.hpp>"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("buildRelationshipActions(app.engine)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("applyRelationshipAction(app.engine, action.kind)"),
      std::string::npos);
  EXPECT_NE(
      cmakeSource.find("src/relationship_actions.cpp"), std::string::npos);
}

TEST(FilamentSceneExtraction, DartsimProjectMenuUsesNativeFileDialogSeam)
{
  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");
  const auto cmakeSource
      = readSourceFile(kDartsimUiDirectory / "CMakeLists.txt");
  const auto projectActionsHeader = readSourceFile(
      kDartsimUiDirectory / "include" / "dartsim_ui" / "project_actions.hpp");
  const auto projectDialogSource
      = readSourceFile(kDartsimUiDirectory / "src" / "project_file_dialog.cpp");
  const auto panelHeader = readSourceFile(kDartGuiDirectory / "panel.hpp");
  const auto uiFrameSource
      = readSourceFile(kDartGuiDirectory / "detail" / "ui_frame.cpp");

  EXPECT_NE(
      editorSource.find("#include <dartsim_ui/project_file_dialog.hpp>"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("ui.menuItem(\"Open Project...\")"), std::string::npos);
  EXPECT_NE(
      editorSource.find("openProjectFromNativeDialog(app)"), std::string::npos);
  EXPECT_NE(
      editorSource.find("saveProjectFromNativeDialog(app)"), std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "requestProjectPathModal(app, ProjectFileDialogKind::Open"),
      std::string::npos);
  EXPECT_NE(editorSource.find("ui.button(\"Browse...\")"), std::string::npos);
  EXPECT_NE(editorSource.find("projectBrowserEntries"), std::string::npos);
  EXPECT_NE(
      editorSource.find("ui.button(\"Up##project-browser\")"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("nativeProjectFileDialog(request)"), std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "request.parentNativeWindow = app.projectDialogParentWindow"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "app->projectDialogParentWindow = context.nativeWindow"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("ui.menuItem(\"Save Project As...\")"),
      std::string::npos);
  EXPECT_EQ(
      editorSource.find("app.engine.loadProject(\"scene.dartsim\")"),
      std::string::npos);
  EXPECT_EQ(
      editorSource.find("app.engine.saveProject(\"scene.dartsim\")"),
      std::string::npos);
  EXPECT_NE(
      cmakeSource.find("find_package(nfd CONFIG REQUIRED)"), std::string::npos);
  EXPECT_NE(cmakeSource.find("src/project_file_dialog.cpp"), std::string::npos);
  EXPECT_NE(cmakeSource.find("nfd::nfd"), std::string::npos);
  EXPECT_NE(
      projectActionsHeader.find("void* parentNativeWindow = nullptr"),
      std::string::npos);
  EXPECT_NE(
      projectDialogSource.find(
          "args.parentWindow = nativeParentWindow(request.parentNativeWindow)"),
      std::string::npos);
  EXPECT_NE(
      panelHeader.find("void* nativeWindow = nullptr"), std::string::npos);
  EXPECT_NE(
      uiFrameSource.find(
          "window == nullptr ? nullptr : getNativeWindow(window)"),
      std::string::npos);
}

TEST(FilamentSceneExtraction, DartsimSimulationModeUsesElapsedFrameSeconds)
{
  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");

  EXPECT_EQ(editorSource.find("sim.advance(1.0 / 60.0)"), std::string::npos);
  EXPECT_NE(
      editorSource.find("std::chrono::steady_clock::now()"), std::string::npos);
  EXPECT_NE(
      editorSource.find("std::chrono::duration<double>"), std::string::npos);
  EXPECT_NE(
      editorSource.find("sim.advance(app.consumeRunFrameSeconds());"),
      std::string::npos);
  const auto preRender = editorSource.find("options.preRender = [app]()");
  ASSERT_NE(preRender, std::string::npos);
  EXPECT_NE(
      editorSource.find("advanceRunningSimulation(*app);", preRender),
      std::string::npos);
}

TEST(FilamentSceneExtraction, DartsimInspectorLocksEditsDuringSimulationMode)
{
  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");

  const auto inspectorStart = editorSource.find("void buildInspector");
  ASSERT_NE(inspectorStart, std::string::npos);
  const auto canEditCheck = editorSource.find(
      "const InspectorStatus status = buildInspectorStatus(app.engine);");
  ASSERT_NE(canEditCheck, std::string::npos);
  const auto lockGuard = editorSource.find("if (status.locked)", canEditCheck);
  ASSERT_NE(lockGuard, std::string::npos);
  const auto lockMessage
      = editorSource.find("Inspector locked during Simulation Mode", lockGuard);
  ASSERT_NE(lockMessage, std::string::npos);
  const auto firstSlider
      = editorSource.find("ui.slider(property.label", inspectorStart);
  const auto deleteButton
      = editorSource.find("ui.button(\"Delete##inspector\")", inspectorStart);
  ASSERT_NE(firstSlider, std::string::npos);
  ASSERT_NE(deleteButton, std::string::npos);

  EXPECT_LT(lockGuard, firstSlider);
  EXPECT_LT(lockGuard, deleteButton);
}

TEST(FilamentSceneExtraction, DartsimInspectorUsesEnumChoiceControls)
{
  const auto panelHeader = readSourceFile(kDartGuiDirectory / "panel.hpp");
  const auto panelSource
      = readSourceFile(kDartGuiDirectory / "detail" / "panel.cpp");
  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");

  EXPECT_NE(panelHeader.find("select("), std::string::npos);
  EXPECT_NE(panelSource.find("ImGui::BeginCombo("), std::string::npos);
  EXPECT_NE(editorSource.find("status.enumProperties"), std::string::npos);
  EXPECT_NE(editorSource.find("ui.select(property.label"), std::string::npos);
  EXPECT_NE(editorSource.find("setInspectorEnumProperty("), std::string::npos);
}

TEST(FilamentSceneExtraction, DartsimApplicationKeepsOnlyMinimalCppEntryPoint)
{
  const auto sources
      = listCppSourceFilesRecursively(kDartsimApplicationDirectory);

  const std::vector<std::filesystem::path> expectedSources
      = {kDartsimApplicationDirectory / "main.cpp"};
  EXPECT_EQ(sources, expectedSources);
}

TEST(
    FilamentSceneExtraction,
    DartsimApplicationKeepsOnlyEntryPointAndWrapperFiles)
{
  const auto files = listRegularFilesRecursively(kDartsimApplicationDirectory);

  const std::vector<std::filesystem::path> expectedFiles
      = {kDartsimApplicationDirectory / "CMakeLists.txt",
         kDartsimApplicationDirectory / "main.cpp"};
  EXPECT_EQ(files, expectedFiles);
}

TEST(FilamentSceneExtraction, MeshGeometryBuildersProduceBoundedMeshes)
{
  const auto ellipsoid
      = dart::gui::makeEllipsoidMeshGeometry(Eigen::Vector3d(1.0, 2.0, 3.0));
  expectMeshGeometryIsWellFormed(ellipsoid);
  EXPECT_EQ(ellipsoid.vertices.size(), 17u * 33u);
  EXPECT_EQ(ellipsoid.triangles.size(), 16u * 32u * 2u);
  EXPECT_TRUE(
      ellipsoid.boundsMin.isApprox(Eigen::Vector3f(-1.0f, -2.0f, -3.0f)));
  EXPECT_TRUE(ellipsoid.boundsMax.isApprox(Eigen::Vector3f(1.0f, 2.0f, 3.0f)));

  const auto cylinder = dart::gui::makeCylinderMeshGeometry(0.5, 1.4);
  expectMeshGeometryIsWellFormed(cylinder);
  EXPECT_EQ(cylinder.vertices.size(), 166u);
  EXPECT_EQ(cylinder.triangles.size(), 160u);
  EXPECT_TRUE(
      cylinder.boundsMin.isApprox(Eigen::Vector3f(-0.5f, -0.5f, -0.7f)));
  EXPECT_TRUE(cylinder.boundsMax.isApprox(Eigen::Vector3f(0.5f, 0.5f, 0.7f)));

  const auto pyramid
      = dart::gui::makePyramidMeshGeometry(Eigen::Vector3d(2.0, 4.0, 6.0));
  expectMeshGeometryIsWellFormed(pyramid);
  EXPECT_EQ(pyramid.vertices.size(), 18u);
  EXPECT_EQ(pyramid.triangles.size(), 6u);
  EXPECT_TRUE(pyramid.boundsMin.isApprox(Eigen::Vector3f(-1.0f, -2.0f, -3.0f)));
  EXPECT_TRUE(pyramid.boundsMax.isApprox(Eigen::Vector3f(1.0f, 2.0f, 3.0f)));
}

TEST(FilamentSceneExtraction, BoxMeshAppenderReportsIndexRanges)
{
  MeshGeometry mesh;
  const auto first = dart::gui::appendBoxMeshGeometry(
      mesh, Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d::Ones());
  const auto second = dart::gui::appendBoxMeshGeometry(
      mesh, Eigen::Vector3d(-1.0, -2.0, -3.0), Eigen::Vector3d::Ones() * 2.0);

  expectMeshGeometryIsWellFormed(mesh);
  EXPECT_EQ(first.indexOffset, 0u);
  EXPECT_EQ(first.indexCount, 36u);
  EXPECT_EQ(second.indexOffset, 36u);
  EXPECT_EQ(second.indexCount, 36u);
  EXPECT_EQ(mesh.vertices.size(), 48u);
  EXPECT_EQ(mesh.triangles.size(), 24u);
  EXPECT_TRUE(mesh.boundsMin.isApprox(Eigen::Vector3f(-2.0f, -3.0f, -4.0f)));
  EXPECT_TRUE(mesh.boundsMax.isApprox(Eigen::Vector3f(1.5f, 2.5f, 3.5f)));
}

TEST(
    FilamentSceneExtraction,
    ExtractRenderables_VisibleShape_ContainsStableDescriptor)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 2.0, 3.0));
  auto* shapeNode
      = body->createShapeNodeWith<VisualAspect>(shape, "box_visual");
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.2, 0.3, 0.4, 0.5));
  shapeNode->getVisualAspect()->setShadowed(false);

  Eigen::Isometry3d bodyTransform = Eigen::Isometry3d::Identity();
  bodyTransform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  joint->setTransformFromParentBodyNode(bodyTransform);

  Eigen::Isometry3d shapeTransform = Eigen::Isometry3d::Identity();
  shapeTransform.translation() = Eigen::Vector3d(0.25, 0.0, 0.5);
  shapeNode->setRelativeTransform(shapeTransform);

  world->addSkeleton(skeleton);

  const auto firstExtract = dart::gui::extractRenderables(*world);
  const auto secondExtract = dart::gui::extractRenderables(*world);

  ASSERT_EQ(firstExtract.size(), 1u);
  ASSERT_EQ(secondExtract.size(), 1u);
  const auto& descriptor = firstExtract.front();
  EXPECT_EQ(descriptor.id, secondExtract.front().id);
  EXPECT_EQ(descriptor.skeletonName, "robot");
  EXPECT_EQ(descriptor.shapeNodeName, "box_visual");
  EXPECT_EQ(descriptor.geometry.kind, ShapeKind::Box);
  EXPECT_TRUE(
      descriptor.geometry.size.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(
      descriptor.material.rgba.isApprox(Eigen::Vector4d(0.2, 0.3, 0.4, 0.5)));
  EXPECT_TRUE(descriptor.material.visible);
  EXPECT_FALSE(descriptor.material.castsShadows);
  EXPECT_FALSE(descriptor.material.receivesShadows);
  EXPECT_TRUE(
      descriptor.worldTransform.isApprox(bodyTransform * shapeTransform));
}

TEST(
    FilamentSceneExtraction,
    ExtractRenderables_PropertyChanges_UpdateDescriptors)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  (void)joint;
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5));
  auto* shapeNode
      = body->createShapeNodeWith<VisualAspect>(shape, "box_visual");
  world->addSkeleton(skeleton);

  const auto before = dart::gui::extractRenderables(*world);
  ASSERT_EQ(before.size(), 1u);

  shape->setSize(Eigen::Vector3d(2.0, 0.75, 1.25));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.7, 0.6, 0.5, 0.4));
  shapeNode->getVisualAspect()->hide();

  const auto after = dart::gui::extractRenderables(*world);
  ASSERT_EQ(after.size(), 1u);

  EXPECT_EQ(before.front().id, after.front().id);
  EXPECT_TRUE(
      after.front().geometry.size.isApprox(Eigen::Vector3d(2.0, 0.75, 1.25)));
  EXPECT_TRUE(after.front().material.rgba.isApprox(
      Eigen::Vector4d(0.7, 0.6, 0.5, 0.4)));
  EXPECT_FALSE(after.front().material.visible);
  EXPECT_NE(before.front().shapeVersion, after.front().shapeVersion);
  EXPECT_NE(
      before.front().renderResourceVersion,
      after.front().renderResourceVersion);
  EXPECT_NE(before.front().shapeNodeVersion, after.front().shapeNodeVersion);
}

TEST(
    FilamentSceneExtraction,
    ExtractRenderables_VisualAlphaChange_UpdatesRenderResourceVersion)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  (void)joint;
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5));
  auto* shapeNode
      = body->createShapeNodeWith<VisualAspect>(shape, "box_visual");
  world->addSkeleton(skeleton);

  const auto before = dart::gui::extractRenderables(*world);
  ASSERT_EQ(before.size(), 1u);
  const auto beforeShapeVersion = before.front().shapeVersion;

  shapeNode->getVisualAspect()->setAlpha(0.25);

  const auto after = dart::gui::extractRenderables(*world);
  ASSERT_EQ(after.size(), 1u);
  EXPECT_EQ(before.front().id, after.front().id);
  EXPECT_EQ(beforeShapeVersion, after.front().shapeVersion);
  EXPECT_NE(
      before.front().renderResourceVersion,
      after.front().renderResourceVersion);
}

TEST(
    FilamentSceneExtraction,
    ExtractRenderables_PointCloudColorModeChange_UpdatesRenderResourceVersion)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  (void)joint;
  auto pointCloud = std::make_shared<PointCloudShape>(0.2);
  pointCloud->addPoint(Eigen::Vector3d(-0.4, 0.0, 0.1));
  pointCloud->addPoint(Eigen::Vector3d(0.2, -0.3, 0.5));
  const std::array<Eigen::Vector4d, 2> pointColors{
      Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), Eigen::Vector4d(0.0, 1.0, 0.0, 0.8)};
  pointCloud->setColors(pointColors);
  pointCloud->setColorMode(PointCloudShape::BIND_OVERALL);
  body->createShapeNodeWith<VisualAspect>(pointCloud, "points_visual");
  world->addSkeleton(skeleton);

  const auto before = dart::gui::extractRenderables(*world);
  ASSERT_EQ(before.size(), 1u);
  const auto beforeShapeVersion = before.front().shapeVersion;
  ASSERT_EQ(before.front().geometry.pointCloudColors.size(), 1u);

  pointCloud->setColorMode(PointCloudShape::BIND_PER_POINT);

  const auto after = dart::gui::extractRenderables(*world);
  ASSERT_EQ(after.size(), 1u);
  ASSERT_EQ(after.front().geometry.pointCloudColors.size(), 2u);
  EXPECT_EQ(before.front().id, after.front().id);
  EXPECT_EQ(beforeShapeVersion, after.front().shapeVersion);
  EXPECT_NE(
      before.front().renderResourceVersion,
      after.front().renderResourceVersion);
}

TEST(
    FilamentSceneExtraction,
    ExtractRenderables_SoftMeshMotion_UpdatesRenderResourceVersion)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("soft_robot");
  auto [joint, softBody]
      = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  (void)joint;
  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d(0.4, 0.3, 0.2),
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3i(3, 3, 3),
      1.0,
      10.0,
      10.0,
      0.1);
  for (std::size_t i = 0; i < softBody->getNumShapeNodes(); ++i) {
    auto* shapeNode = softBody->getShapeNode(i);
    ASSERT_NE(shapeNode, nullptr);
    shapeNode->createVisualAspect();
  }
  world->addSkeleton(skeleton);

  const auto findSoftMeshDescriptor =
      [](const std::vector<dart::gui::RenderableDescriptor>& descriptors) {
        return std::find_if(
            descriptors.begin(), descriptors.end(), [](const auto& descriptor) {
              return descriptor.geometry.kind == ShapeKind::SoftMesh;
            });
      };

  const auto before = dart::gui::extractRenderables(*world);
  const auto beforeSoftMesh = findSoftMeshDescriptor(before);
  ASSERT_NE(beforeSoftMesh, before.end());
  ASSERT_FALSE(beforeSoftMesh->geometry.triangleVertices.empty());

  ASSERT_GT(softBody->getNumPointMasses(), 0u);
  softBody->getPointMass(0u)->setPositions(Eigen::Vector3d(0.0, 0.0, 0.2));

  const auto after = dart::gui::extractRenderables(*world);
  const auto afterSoftMesh = findSoftMeshDescriptor(after);
  ASSERT_NE(afterSoftMesh, after.end());
  ASSERT_FALSE(afterSoftMesh->geometry.triangleVertices.empty());

  EXPECT_EQ(beforeSoftMesh->id, afterSoftMesh->id);
  EXPECT_NE(
      beforeSoftMesh->renderResourceVersion,
      afterSoftMesh->renderResourceVersion);
  EXPECT_FALSE(beforeSoftMesh->geometry.triangleVertices.front().isApprox(
      afterSoftMesh->geometry.triangleVertices.front()));
}

TEST(
    FilamentSceneExtraction,
    ExtractRenderables_SimpleFrameVisual_ContainsStableDescriptor)
{
  auto world = World::create("world");

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, -2.0, 0.5);
  auto frame = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "interactive_target", transform);
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(0.4, 0.5, 0.6)));
  frame->getVisualAspect(true)->setRGBA(Eigen::Vector4d(0.9, 0.2, 0.1, 0.8));
  frame->getVisualAspect()->setShadowed(false);
  world->addSimpleFrame(frame);

  const auto firstExtract = dart::gui::extractRenderables(*world);
  const auto secondExtract = dart::gui::extractRenderables(*world);

  ASSERT_EQ(firstExtract.size(), 1u);
  ASSERT_EQ(secondExtract.size(), 1u);
  const auto& descriptor = firstExtract.front();
  EXPECT_EQ(descriptor.id, secondExtract.front().id);
  EXPECT_EQ(descriptor.shapeFrame, frame.get());
  EXPECT_EQ(descriptor.shapeNode, nullptr);
  EXPECT_EQ(descriptor.skeletonName, "");
  EXPECT_EQ(descriptor.bodyName, "");
  EXPECT_EQ(descriptor.shapeFrameName, "interactive_target");
  EXPECT_EQ(descriptor.shapeNodeName, "");
  EXPECT_EQ(descriptor.geometry.kind, ShapeKind::Box);
  EXPECT_TRUE(
      descriptor.geometry.size.isApprox(Eigen::Vector3d(0.4, 0.5, 0.6)));
  EXPECT_TRUE(
      descriptor.material.rgba.isApprox(Eigen::Vector4d(0.9, 0.2, 0.1, 0.8)));
  EXPECT_TRUE(descriptor.material.visible);
  EXPECT_FALSE(descriptor.material.castsShadows);
  EXPECT_FALSE(descriptor.material.receivesShadows);
  EXPECT_TRUE(descriptor.worldTransform.isApprox(transform));
  EXPECT_EQ(descriptor.shapeNodeVersion, 0u);
  EXPECT_EQ(descriptor.shapeFrameVersion, frame->getVersion());
  EXPECT_EQ(descriptor.shapeVersion, frame->getShape()->getVersion());
  EXPECT_EQ(
      descriptor.renderResourceVersion,
      secondExtract.front().renderResourceVersion);
  EXPECT_NE(descriptor.renderResourceVersion, 0u);
}

TEST(FilamentSceneExtraction, ApplyDebugVisualStyle_SimpleFrame_DisablesShadows)
{
  auto world = World::create("world");
  auto frame = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "debug_marker");
  frame->setShape(std::make_shared<SphereShape>(0.12));
  const Eigen::Vector4d color(0.95, 0.16, 0.12, 0.75);
  dart::gui::applyDebugVisualStyle(*frame, color);
  world->addSimpleFrame(frame);

  ASSERT_NE(frame->getVisualAspect(), nullptr);
  EXPECT_TRUE(frame->getVisualAspect()->getRGBA().isApprox(color));
  EXPECT_FALSE(frame->getVisualAspect()->getShadowed());

  const auto descriptors = dart::gui::extractRenderables(*world);
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_TRUE(descriptors.front().material.rgba.isApprox(color));
  EXPECT_FALSE(descriptors.front().material.castsShadows);
  EXPECT_FALSE(descriptors.front().material.receivesShadows);
}

TEST(
    FilamentSceneExtraction,
    DescribeShape_CommonPrimitiveShapes_ReturnsRendererDescriptors)
{
  using dart::gui::describeShape;

  const auto sphere = describeShape(SphereShape(0.25));
  ASSERT_TRUE(sphere.has_value());
  EXPECT_EQ(sphere->kind, ShapeKind::Sphere);
  EXPECT_DOUBLE_EQ(sphere->radius, 0.25);
  EXPECT_TRUE(sphere->size.isApprox(Eigen::Vector3d::Constant(0.5)));

  const auto ellipsoid
      = describeShape(EllipsoidShape(Eigen::Vector3d(1.0, 2.0, 3.0)));
  ASSERT_TRUE(ellipsoid.has_value());
  EXPECT_EQ(ellipsoid->kind, ShapeKind::Ellipsoid);
  EXPECT_TRUE(ellipsoid->size.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  const auto cylinder = describeShape(CylinderShape(0.4, 1.5));
  ASSERT_TRUE(cylinder.has_value());
  EXPECT_EQ(cylinder->kind, ShapeKind::Cylinder);
  EXPECT_DOUBLE_EQ(cylinder->radius, 0.4);
  EXPECT_DOUBLE_EQ(cylinder->height, 1.5);

  const auto capsule = describeShape(CapsuleShape(0.2, 0.9));
  ASSERT_TRUE(capsule.has_value());
  EXPECT_EQ(capsule->kind, ShapeKind::Capsule);
  EXPECT_TRUE(capsule->size.isApprox(Eigen::Vector3d(0.4, 0.4, 1.3)));

  const auto cone = describeShape(ConeShape(0.3, 0.8));
  ASSERT_TRUE(cone.has_value());
  EXPECT_EQ(cone->kind, ShapeKind::Cone);
  EXPECT_TRUE(cone->size.isApprox(Eigen::Vector3d(0.6, 0.6, 0.8)));

  const auto pyramid = describeShape(PyramidShape(0.7, 0.5, 0.9));
  ASSERT_TRUE(pyramid.has_value());
  EXPECT_EQ(pyramid->kind, ShapeKind::Pyramid);
  EXPECT_DOUBLE_EQ(pyramid->height, 0.9);
  EXPECT_TRUE(pyramid->size.isApprox(Eigen::Vector3d(0.7, 0.5, 0.9)));
  ASSERT_TRUE(pyramid->hasLocalBounds);
  EXPECT_TRUE(
      pyramid->localBoundsMin.isApprox(Eigen::Vector3d(-0.35, -0.25, -0.45)));
  EXPECT_TRUE(
      pyramid->localBoundsMax.isApprox(Eigen::Vector3d(0.35, 0.25, 0.45)));

  const MultiSphereConvexHullShape::Spheres spheres{
      {0.2, Eigen::Vector3d(-0.5, 0.0, 0.1)},
      {0.35, Eigen::Vector3d(0.25, 0.1, -0.05)}};
  const auto multiSphere = describeShape(MultiSphereConvexHullShape(spheres));
  ASSERT_TRUE(multiSphere.has_value());
  EXPECT_EQ(multiSphere->kind, ShapeKind::MultiSphere);
  EXPECT_DOUBLE_EQ(multiSphere->radius, 0.35);
  ASSERT_EQ(multiSphere->sphereCenters.size(), 2u);
  ASSERT_EQ(multiSphere->sphereRadii.size(), 2u);
  EXPECT_TRUE(
      multiSphere->sphereCenters[0].isApprox(Eigen::Vector3d(-0.5, 0.0, 0.1)));
  EXPECT_DOUBLE_EQ(multiSphere->sphereRadii[0], 0.2);
  EXPECT_TRUE(multiSphere->sphereCenters[1].isApprox(
      Eigen::Vector3d(0.25, 0.1, -0.05)));
  EXPECT_DOUBLE_EQ(multiSphere->sphereRadii[1], 0.35);
  ASSERT_TRUE(multiSphere->hasLocalBounds);
  EXPECT_TRUE(
      multiSphere->localBoundsMin.isApprox(Eigen::Vector3d(-0.7, -0.25, -0.4)));
  EXPECT_TRUE(
      multiSphere->localBoundsMax.isApprox(Eigen::Vector3d(0.6, 0.45, 0.3)));
  EXPECT_TRUE(multiSphere->size.isApprox(Eigen::Vector3d(1.3, 0.7, 0.7)));

  LineSegmentShape lineSegments(
      Eigen::Vector3d(-0.1, 0.0, 0.0), Eigen::Vector3d(0.4, 0.2, 0.3), 2.5f);
  lineSegments.addVertex(Eigen::Vector3d(0.2, -0.3, 0.1), 0);
  const auto lines = describeShape(lineSegments);
  ASSERT_TRUE(lines.has_value());
  EXPECT_EQ(lines->kind, ShapeKind::LineSegments);
  EXPECT_DOUBLE_EQ(lines->lineThickness, 2.5);
  ASSERT_EQ(lines->lineVertices.size(), 3u);
  ASSERT_EQ(lines->lineConnections.size(), 2u);
  EXPECT_TRUE(lines->lineVertices[0].isApprox(Eigen::Vector3d(-0.1, 0.0, 0.0)));
  EXPECT_TRUE(lines->lineVertices[1].isApprox(Eigen::Vector3d(0.4, 0.2, 0.3)));
  EXPECT_TRUE(lines->lineVertices[2].isApprox(Eigen::Vector3d(0.2, -0.3, 0.1)));
  EXPECT_EQ(lines->lineConnections[0].x(), 0);
  EXPECT_EQ(lines->lineConnections[0].y(), 1);
  EXPECT_EQ(lines->lineConnections[1].x(), 0);
  EXPECT_EQ(lines->lineConnections[1].y(), 2);
  ASSERT_TRUE(lines->hasLocalBounds);
  EXPECT_TRUE(lines->localBoundsMin.isApprox(Eigen::Vector3d(-0.1, -0.3, 0.0)));
  EXPECT_TRUE(lines->localBoundsMax.isApprox(Eigen::Vector3d(0.4, 0.2, 0.3)));
  EXPECT_TRUE(lines->size.isApprox(Eigen::Vector3d(0.5, 0.5, 0.3)));

  auto convexTriMesh = std::make_shared<dart::math::TriMesh<double>>();
  convexTriMesh->addVertex(-0.25, -0.2, -0.1);
  convexTriMesh->addVertex(0.45, -0.2, -0.1);
  convexTriMesh->addVertex(0.0, 0.35, -0.1);
  convexTriMesh->addVertex(0.0, 0.0, 0.55);
  convexTriMesh->addTriangle(0, 2, 1);
  convexTriMesh->addTriangle(0, 1, 3);
  convexTriMesh->addTriangle(1, 2, 3);
  convexTriMesh->addTriangle(2, 0, 3);
  const auto convexMesh = describeShape(ConvexMeshShape(convexTriMesh));
  ASSERT_TRUE(convexMesh.has_value());
  EXPECT_EQ(convexMesh->kind, ShapeKind::ConvexMesh);
  ASSERT_TRUE(convexMesh->hasLocalBounds);
  EXPECT_TRUE(
      convexMesh->localBoundsMin.isApprox(Eigen::Vector3d(-0.25, -0.2, -0.1)));
  EXPECT_TRUE(
      convexMesh->localBoundsMax.isApprox(Eigen::Vector3d(0.45, 0.35, 0.55)));
  EXPECT_TRUE(convexMesh->size.isApprox(Eigen::Vector3d(0.7, 0.55, 0.65)));
  ASSERT_EQ(convexMesh->triangleVertices.size(), 4u);
  ASSERT_EQ(convexMesh->triangleIndices.size(), 4u);
  EXPECT_TRUE(convexMesh->triangleVertices[0].isApprox(
      Eigen::Vector3d(-0.25, -0.2, -0.1)));
  EXPECT_EQ(convexMesh->triangleIndices[0].x(), 0);
  EXPECT_EQ(convexMesh->triangleIndices[0].y(), 2);
  EXPECT_EQ(convexMesh->triangleIndices[0].z(), 1);

  PointCloudShape pointCloud(0.2);
  pointCloud.addPoint(Eigen::Vector3d(-0.4, 0.0, 0.1));
  pointCloud.addPoint(Eigen::Vector3d(0.2, -0.3, 0.5));
  pointCloud.setColorMode(PointCloudShape::BIND_PER_POINT);
  const std::array<Eigen::Vector4d, 2> pointColors{
      Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), Eigen::Vector4d(0.0, 1.0, 0.0, 0.8)};
  pointCloud.setColors(pointColors);
  const auto points = describeShape(pointCloud);
  ASSERT_TRUE(points.has_value());
  EXPECT_EQ(points->kind, ShapeKind::PointCloud);
  EXPECT_DOUBLE_EQ(points->pointSize, 0.2);
  ASSERT_EQ(points->pointCloudPoints.size(), 2u);
  ASSERT_EQ(points->pointCloudColors.size(), 2u);
  EXPECT_TRUE(
      points->pointCloudPoints[0].isApprox(Eigen::Vector3d(-0.4, 0.0, 0.1)));
  EXPECT_TRUE(points->pointCloudColors[1].isApprox(
      Eigen::Vector4d(0.0, 1.0, 0.0, 0.8)));
  ASSERT_TRUE(points->hasLocalBounds);
  EXPECT_TRUE(
      points->localBoundsMin.isApprox(Eigen::Vector3d(-0.5, -0.4, 0.0)));
  EXPECT_TRUE(points->localBoundsMax.isApprox(Eigen::Vector3d(0.3, 0.1, 0.6)));
  EXPECT_TRUE(points->size.isApprox(Eigen::Vector3d(0.8, 0.5, 0.6)));

  PointCloudShape emptyPointCloud(0.2);
  const auto emptyPoints = describeShape(emptyPointCloud);
  ASSERT_TRUE(emptyPoints.has_value());
  EXPECT_EQ(emptyPoints->kind, ShapeKind::PointCloud);
  EXPECT_NE(
      emptyPoints->unsupportedReason.find("no points"), std::string::npos);

  HeightmapShaped heightmap;
  const std::array<double, 6> heights{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  heightmap.setHeightField(3u, 2u, heights);
  heightmap.setScale(Eigen::Vector3d(0.5, 0.25, 1.5));
  const auto terrain = describeShape(heightmap);
  ASSERT_TRUE(terrain.has_value());
  EXPECT_EQ(terrain->kind, ShapeKind::Heightmap);
  ASSERT_TRUE(terrain->hasLocalBounds);
  EXPECT_TRUE(
      terrain->localBoundsMin.isApprox(Eigen::Vector3d(-0.75, -0.25, 1.5)));
  EXPECT_TRUE(
      terrain->localBoundsMax.isApprox(Eigen::Vector3d(0.75, 0.25, 9.0)));
  EXPECT_TRUE(terrain->size.isApprox(Eigen::Vector3d(1.5, 0.5, 7.5)));
  ASSERT_EQ(terrain->triangleVertices.size(), 6u);
  ASSERT_EQ(terrain->triangleIndices.size(), 4u);
  EXPECT_TRUE(
      terrain->triangleVertices[0].isApprox(Eigen::Vector3d(-0.5, 0.125, 1.5)));
  EXPECT_EQ(terrain->triangleIndices[0].x(), 0);
  EXPECT_EQ(terrain->triangleIndices[0].y(), 3);
  EXPECT_EQ(terrain->triangleIndices[0].z(), 1);

  auto softSkeleton = Skeleton::create("soft_skel");
  auto* softBody
      = softSkeleton->createJointAndBodyNodePair<WeldJoint, SoftBodyNode>()
            .second;
  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d(0.4, 0.6, 0.8),
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3i(3, 3, 3),
      1.0,
      10.0,
      10.0,
      0.1);
  ASSERT_GT(softBody->getNumPointMasses(), 0u);
  softBody->getPointMass(0u)->setPositions(Eigen::Vector3d(0.0, 0.0, 0.2));
  auto softMeshShape = findSoftMeshShape(*softBody);
  ASSERT_NE(softMeshShape, nullptr);
  const auto softMesh = describeShape(*softMeshShape);
  ASSERT_TRUE(softMesh.has_value());
  EXPECT_EQ(softMesh->kind, ShapeKind::SoftMesh);
  EXPECT_EQ(softMesh->shapeType, "SoftMeshShape");
  ASSERT_TRUE(softMesh->hasLocalBounds);
  EXPECT_GT(softMesh->size.x(), 0.0);
  EXPECT_GT(softMesh->size.y(), 0.0);
  EXPECT_GT(softMesh->size.z(), 0.0);
  EXPECT_FALSE(softMesh->triangleVertices.empty());
  EXPECT_FALSE(softMesh->triangleIndices.empty());

  VoxelGridShape voxelGrid(0.1);
  voxelGrid.updateOccupancy(Eigen::Vector3d(0.05, 0.05, 0.05));
  voxelGrid.updateOccupancy(Eigen::Vector3d(0.25, -0.05, 0.15));
  const auto voxels = describeShape(voxelGrid);
  ASSERT_TRUE(voxels.has_value());
  EXPECT_EQ(voxels->kind, ShapeKind::VoxelGrid);
  EXPECT_EQ(voxels->shapeType, "VoxelGridShape");
  EXPECT_DOUBLE_EQ(voxels->voxelSize, 0.1);
  ASSERT_GE(voxels->voxelCenters.size(), 2u);
  ASSERT_TRUE(voxels->hasLocalBounds);
  EXPECT_GT(voxels->size.x(), 0.0);
  EXPECT_GT(voxels->size.y(), 0.0);
  EXPECT_GT(voxels->size.z(), 0.0);

  const auto unsupported = describeShape(UnsupportedTestShape());
  ASSERT_TRUE(unsupported.has_value());
  EXPECT_EQ(unsupported->kind, ShapeKind::Unsupported);
  EXPECT_EQ(unsupported->shapeType, "UnsupportedTestShape");
  EXPECT_FALSE(unsupported->unsupportedReason.empty());
  EXPECT_NE(
      unsupported->unsupportedReason.find("UnsupportedTestShape"),
      std::string::npos);
  EXPECT_FALSE(unsupported->hasLocalBounds);

  auto triMesh = std::make_shared<dart::math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addVertex(1.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);
  triMesh->addTriangle(1, 3, 2);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);
  MeshDescriptorHarness meshShape(
      Eigen::Vector3d(2.0, 3.0, 4.0), triMesh, dart::common::Uri{});
  meshShape.setColorMode(MeshShape::MATERIAL_COLOR);
  meshShape.setAlphaMode(MeshShape::SHAPE_ALPHA);

  std::vector<MeshMaterial> materials(2);
  materials[0].diffuse = Eigen::Vector4f(0.2f, 0.3f, 0.4f, 0.75f);
  materials[0].metallicFactor = 0.25f;
  materials[0].roughnessFactor = 0.65f;
  materials[0].baseColorTexturePath = "textures/base_color.png";
  materials[0].metallicRoughnessTexturePath = "textures/metal_rough.png";
  materials[0].textureImagePaths = {"textures/legacy_base.png"};
  materials[1].diffuse = Eigen::Vector4f(0.7f, 0.6f, 0.5f, 1.0f);
  materials[1].emissive = Eigen::Vector4f(0.1f, 0.2f, 0.3f, 1.0f);
  materials[1].normalTexturePath = "textures/normal.png";
  meshShape.setMaterials(std::move(materials));

  std::vector<MeshDescriptorHarness::SubMeshRange> ranges(2);
  ranges[0].vertexOffset = 0u;
  ranges[0].vertexCount = 3u;
  ranges[0].triangleOffset = 0u;
  ranges[0].triangleCount = 1u;
  ranges[0].materialIndex = 0u;
  ranges[1].vertexOffset = 1u;
  ranges[1].vertexCount = 3u;
  ranges[1].triangleOffset = 1u;
  ranges[1].triangleCount = 1u;
  ranges[1].materialIndex = 1u;
  meshShape.setSubMeshRanges(std::move(ranges));
  meshShape.setTextureCoords(
      {Eigen::Vector3d(0.0, 0.0, 0.0),
       Eigen::Vector3d(1.0, 0.0, 0.0),
       Eigen::Vector3d(0.0, 1.0, 0.0),
       Eigen::Vector3d(1.0, 1.0, 0.0)},
      2);

  const auto mesh = describeShape(meshShape);
  ASSERT_TRUE(mesh.has_value());
  EXPECT_EQ(mesh->kind, ShapeKind::Mesh);
  EXPECT_TRUE(mesh->scale.isApprox(Eigen::Vector3d(2.0, 3.0, 4.0)));
  EXPECT_TRUE(mesh->meshUri.empty());
  EXPECT_TRUE(mesh->meshUsesMaterialColors);
  EXPECT_EQ(mesh->meshAlphaMode, MeshAlphaMode::ShapeAlpha);
  EXPECT_EQ(mesh->meshTextureCoordComponents, 2);
  ASSERT_EQ(mesh->meshMaterials.size(), 2u);
  EXPECT_TRUE(mesh->meshMaterials[0].diffuse.isApprox(
      Eigen::Vector4d(0.2, 0.3, 0.4, 0.75), 1e-6));
  EXPECT_NEAR(mesh->meshMaterials[0].metallicFactor, 0.25, 1e-6);
  EXPECT_NEAR(mesh->meshMaterials[0].roughnessFactor, 0.65, 1e-6);
  EXPECT_EQ(
      mesh->meshMaterials[0].baseColorTexturePath, "textures/base_color.png");
  EXPECT_EQ(
      mesh->meshMaterials[0].metallicRoughnessTexturePath,
      "textures/metal_rough.png");
  ASSERT_EQ(mesh->meshMaterials[0].textureImagePaths.size(), 1u);
  EXPECT_EQ(
      mesh->meshMaterials[0].textureImagePaths[0], "textures/legacy_base.png");
  EXPECT_TRUE(mesh->meshMaterials[1].emissive.isApprox(
      Eigen::Vector4d(0.1, 0.2, 0.3, 1.0), 1e-6));
  EXPECT_EQ(mesh->meshMaterials[1].normalTexturePath, "textures/normal.png");
  ASSERT_EQ(mesh->meshParts.size(), 2u);
  EXPECT_EQ(mesh->meshParts[0].triangleOffset, 0u);
  EXPECT_EQ(mesh->meshParts[0].triangleCount, 1u);
  EXPECT_EQ(mesh->meshParts[0].materialIndex, 0u);
  EXPECT_EQ(mesh->meshParts[1].triangleOffset, 1u);
  EXPECT_EQ(mesh->meshParts[1].triangleCount, 1u);
  EXPECT_EQ(mesh->meshParts[1].materialIndex, 1u);
  ASSERT_EQ(mesh->triangleVertices.size(), 4u);
  ASSERT_EQ(mesh->triangleIndices.size(), 2u);
  ASSERT_EQ(mesh->triangleNormals.size(), 4u);
  ASSERT_EQ(mesh->meshTextureCoordinates.size(), 4u);
  EXPECT_TRUE(
      mesh->triangleVertices[1].isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_TRUE(mesh->triangleNormals[1].isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(
      mesh->meshTextureCoordinates[3].isApprox(Eigen::Vector3d(1.0, 1.0, 0.0)));
  EXPECT_EQ(mesh->triangleIndices[1].x(), 1);
  EXPECT_EQ(mesh->triangleIndices[1].y(), 3);
  EXPECT_EQ(mesh->triangleIndices[1].z(), 2);
  ASSERT_TRUE(mesh->hasLocalBounds);
  EXPECT_TRUE(mesh->localBoundsMin.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(mesh->localBoundsMax.isApprox(Eigen::Vector3d(2.0, 3.0, 0.0)));

  std::shared_ptr<dart::math::TriMesh<double>> emptyTriMesh;
  const MeshShape emptyMeshShape(
      Eigen::Vector3d::Ones(), emptyTriMesh, dart::common::Uri{});
  const auto emptyMesh = describeShape(emptyMeshShape);
  ASSERT_TRUE(emptyMesh.has_value());
  EXPECT_EQ(emptyMesh->kind, ShapeKind::Mesh);
  EXPECT_NE(
      emptyMesh->unsupportedReason.find("no triangle mesh data"),
      std::string::npos);

  const auto plane = describeShape(PlaneShape(Eigen::Vector3d::UnitZ(), 0.25));
  ASSERT_TRUE(plane.has_value());
  EXPECT_EQ(plane->kind, ShapeKind::Plane);
  EXPECT_TRUE(plane->normal.isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_DOUBLE_EQ(plane->offset, 0.25);
  ASSERT_TRUE(plane->hasLocalBounds);
  EXPECT_LT(plane->localBoundsMin.z(), 0.25);
  EXPECT_GT(plane->localBoundsMax.z(), 0.25);
}

TEST(
    FilamentSceneExtraction,
    ExtractRenderables_UnsupportedShape_ContainsDiagnosticDescriptor)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<WeldJoint>();
  (void)joint;
  auto shape = std::make_shared<UnsupportedTestShape>();
  auto* shapeNode
      = body->createShapeNodeWith<VisualAspect>(shape, "unsupported_visual");
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.8, 0.2, 0.1, 1.0));
  world->addSkeleton(skeleton);

  const auto descriptors = dart::gui::extractRenderables(*world);
  ASSERT_EQ(descriptors.size(), 1u);
  const auto& descriptor = descriptors.front();
  EXPECT_EQ(descriptor.geometry.kind, ShapeKind::Unsupported);
  EXPECT_EQ(descriptor.geometry.shapeType, "UnsupportedTestShape");
  EXPECT_NE(
      descriptor.geometry.unsupportedReason.find("UnsupportedTestShape"),
      std::string::npos);
  EXPECT_EQ(descriptor.shapeNodeName, "unsupported_visual");
  EXPECT_FALSE(descriptor.geometry.hasLocalBounds);
}

TEST(
    FilamentSceneExtraction,
    PlanRenderableSetUpdate_AddsVisibleAndRemovesStaleResources)
{
  using dart::gui::RenderableDescriptor;
  using dart::gui::RenderableId;

  RenderableDescriptor visibleA;
  visibleA.id = 1u;
  visibleA.material.visible = true;

  RenderableDescriptor hidden;
  hidden.id = 2u;
  hidden.material.visible = false;

  RenderableDescriptor visibleB;
  visibleB.id = 3u;
  visibleB.material.visible = true;

  RenderableDescriptor duplicateVisibleA;
  duplicateVisibleA.id = visibleA.id;
  duplicateVisibleA.material.visible = true;

  RenderableDescriptor invalidId;
  invalidId.id = 0u;
  invalidId.material.visible = true;

  const std::vector<RenderableDescriptor> descriptors{
      visibleA, hidden, visibleB, duplicateVisibleA, invalidId};
  const std::vector<RenderableId> activeIds{
      visibleA.id, hidden.id, 4u, visibleA.id, 0u};

  const auto plan = dart::gui::planRenderableSetUpdate(descriptors, activeIds);

  ASSERT_EQ(plan.descriptorIndicesToAdd.size(), 1u);
  EXPECT_EQ(plan.descriptorIndicesToAdd[0], 2u);

  const std::vector<std::size_t> expectedRemovals{1u, 2u, 3u, 4u};
  EXPECT_EQ(plan.activeRenderableIndicesToRemove, expectedRemovals);

  visibleA.shapeVersion = 2u;
  visibleA.renderResourceVersion = 20u;
  visibleB.shapeVersion = 4u;
  visibleB.renderResourceVersion = 40u;
  const std::vector<RenderableDescriptor> versionedDescriptors{
      visibleA, visibleB};
  ActiveRenderableState staleA;
  staleA.id = visibleA.id;
  staleA.shapeVersion = visibleA.shapeVersion;
  staleA.renderResourceVersion = 10u;
  ActiveRenderableState currentB;
  currentB.id = visibleB.id;
  currentB.shapeVersion = visibleB.shapeVersion;
  currentB.renderResourceVersion = visibleB.renderResourceVersion;
  const std::vector<ActiveRenderableState> activeStates{staleA, currentB};

  const auto versionedPlan
      = dart::gui::planRenderableSetUpdate(versionedDescriptors, activeStates);

  const std::vector<std::size_t> expectedVersionRemovals{0u};
  const std::vector<std::size_t> expectedVersionAdds{0u};
  EXPECT_EQ(
      versionedPlan.activeRenderableIndicesToRemove, expectedVersionRemovals);
  EXPECT_EQ(versionedPlan.descriptorIndicesToAdd, expectedVersionAdds);

  ActiveRenderableState staleShapeOnlyA;
  staleShapeOnlyA.id = visibleA.id;
  staleShapeOnlyA.shapeVersion = 1u;

  const auto shapeOnlyPlan = dart::gui::planRenderableSetUpdate(
      versionedDescriptors,
      std::vector<ActiveRenderableState>{staleShapeOnlyA, currentB});

  EXPECT_EQ(
      shapeOnlyPlan.activeRenderableIndicesToRemove, expectedVersionRemovals);
  EXPECT_EQ(shapeOnlyPlan.descriptorIndicesToAdd, expectedVersionAdds);
}

TEST(
    FilamentSceneExtraction,
    PickNearestRenderable_RayThroughBoxes_ReturnsClosestVisibleShape)
{
  auto world = World::create("world");

  auto makeBox = [&](const std::string& name, double x, bool hidden = false) {
    auto skeleton = Skeleton::create(name);
    auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(x, 0.0, 0.0);
    joint->setTransformFromParentBodyNode(transform);
    auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
    auto* shapeNode = body->createShapeNodeWith<VisualAspect>(shape, name);
    if (hidden) {
      shapeNode->getVisualAspect()->hide();
    }
    world->addSkeleton(skeleton);
  };

  makeBox("near_box", 2.0);
  makeBox("far_box", 4.0);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 2u);

  const dart::gui::PickRay ray{
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX()};
  const auto hit = dart::gui::pickNearestRenderable(renderables, ray);

  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->id, renderables.front().id);
  EXPECT_EQ(hit->renderableIndex, 0u);
  EXPECT_NEAR(hit->distance, 1.5, 1e-12);
  EXPECT_TRUE(hit->point.isApprox(Eigen::Vector3d(1.5, 0.0, 0.0)));
  EXPECT_TRUE(hit->normal.isApprox(Eigen::Vector3d(-1.0, 0.0, 0.0)));
}

TEST(
    FilamentSceneExtraction,
    PickNearestRenderable_SphereUsesPrimitiveSurfaceNormal)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("sphere_robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  joint->setTransformFromParentBodyNode(transform);
  body->createShapeNodeWith<VisualAspect>(std::make_shared<SphereShape>(1.0));
  world->addSkeleton(skeleton);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  const dart::gui::PickRay ray{
      Eigen::Vector3d(0.0, 0.5, 0.0), Eigen::Vector3d::UnitX()};
  const auto hit = dart::gui::pickNearestRenderable(renderables, ray);

  const double expectedX = 2.0 - std::sqrt(0.75);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, expectedX, 1e-12);
  EXPECT_TRUE(hit->point.isApprox(Eigen::Vector3d(expectedX, 0.5, 0.0)));
  EXPECT_TRUE(hit->normal.isApprox(
      Eigen::Vector3d(-std::sqrt(0.75), 0.5, 0.0).normalized()));
}

TEST(
    FilamentSceneExtraction,
    PickNearestRenderable_MultiSphereUsesPrimitiveSurfaceNormal)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("multi_sphere_robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  const MultiSphereConvexHullShape::Spheres spheres{
      {0.5, Eigen::Vector3d(-1.0, 0.0, 0.0)},
      {0.25, Eigen::Vector3d(1.0, 0.0, 0.0)}};
  body->createShapeNodeWith<VisualAspect>(
      std::make_shared<MultiSphereConvexHullShape>(spheres));
  world->addSkeleton(skeleton);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  const dart::gui::PickRay ray{
      Eigen::Vector3d(-3.0, 0.25, 0.0), Eigen::Vector3d::UnitX()};
  const auto hit = dart::gui::pickNearestRenderable(renderables, ray);

  const double expectedX = -1.0 - std::sqrt(0.1875);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, 3.0 + expectedX, 1e-12);
  EXPECT_TRUE(hit->point.isApprox(Eigen::Vector3d(expectedX, 0.25, 0.0)));
  EXPECT_TRUE(hit->normal.isApprox(
      Eigen::Vector3d(expectedX + 1.0, 0.25, 0.0).normalized()));
}

TEST(
    FilamentSceneExtraction,
    PickNearestRenderable_CylinderUsesPrimitiveSurfaceNormal)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("cylinder_robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  joint->setTransformFromParentBodyNode(transform);
  body->createShapeNodeWith<VisualAspect>(
      std::make_shared<CylinderShape>(1.0, 2.0));
  world->addSkeleton(skeleton);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  const dart::gui::PickRay ray{
      Eigen::Vector3d(0.0, 0.5, 0.0), Eigen::Vector3d::UnitX()};
  const auto hit = dart::gui::pickNearestRenderable(renderables, ray);

  const double expectedX = 2.0 - std::sqrt(0.75);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, expectedX, 1e-12);
  EXPECT_TRUE(hit->point.isApprox(Eigen::Vector3d(expectedX, 0.5, 0.0)));
  EXPECT_TRUE(hit->normal.isApprox(
      Eigen::Vector3d(-std::sqrt(0.75), 0.5, 0.0).normalized()));
}

TEST(
    FilamentSceneExtraction,
    PickNearestRenderable_CapsuleUsesPrimitiveSurfaceNormal)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("capsule_robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  body->createShapeNodeWith<VisualAspect>(
      std::make_shared<CapsuleShape>(1.0, 2.0));
  world->addSkeleton(skeleton);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  const dart::gui::PickRay sideRay{
      Eigen::Vector3d(-2.0, 0.5, 0.0), Eigen::Vector3d::UnitX()};
  const auto sideHit = dart::gui::pickNearestRenderable(renderables, sideRay);

  const double sideX = -std::sqrt(0.75);
  ASSERT_TRUE(sideHit.has_value());
  EXPECT_NEAR(sideHit->distance, 2.0 + sideX, 1e-12);
  EXPECT_TRUE(sideHit->point.isApprox(Eigen::Vector3d(sideX, 0.5, 0.0)));
  EXPECT_TRUE(
      sideHit->normal.isApprox(Eigen::Vector3d(sideX, 0.5, 0.0).normalized()));

  const dart::gui::PickRay capRay{
      Eigen::Vector3d(0.5, 0.0, 4.0), -Eigen::Vector3d::UnitZ()};
  const auto capHit = dart::gui::pickNearestRenderable(renderables, capRay);

  const double capZ = 1.0 + std::sqrt(0.75);
  ASSERT_TRUE(capHit.has_value());
  EXPECT_NEAR(capHit->distance, 4.0 - capZ, 1e-12);
  EXPECT_TRUE(capHit->point.isApprox(Eigen::Vector3d(0.5, 0.0, capZ)));
  EXPECT_TRUE(capHit->normal.isApprox(
      Eigen::Vector3d(0.5, 0.0, std::sqrt(0.75)).normalized()));
}

TEST(
    FilamentSceneExtraction,
    PickNearestRenderable_ConeUsesPrimitiveSurfaceNormal)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("cone_robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  body->createShapeNodeWith<VisualAspect>(
      std::make_shared<ConeShape>(1.0, 2.0));
  world->addSkeleton(skeleton);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  const dart::gui::PickRay sideRay{
      Eigen::Vector3d(-2.0, 0.25, 0.0), Eigen::Vector3d::UnitX()};
  const auto sideHit = dart::gui::pickNearestRenderable(renderables, sideRay);

  const double sideX = -std::sqrt(0.1875);
  ASSERT_TRUE(sideHit.has_value());
  EXPECT_NEAR(sideHit->distance, 2.0 + sideX, 1e-12);
  EXPECT_TRUE(sideHit->point.isApprox(Eigen::Vector3d(sideX, 0.25, 0.0)));
  EXPECT_TRUE(sideHit->normal.isApprox(
      Eigen::Vector3d(sideX, 0.25, 0.25).normalized()));

  const dart::gui::PickRay baseRay{
      Eigen::Vector3d(0.5, 0.0, -4.0), Eigen::Vector3d::UnitZ()};
  const auto baseHit = dart::gui::pickNearestRenderable(renderables, baseRay);

  ASSERT_TRUE(baseHit.has_value());
  EXPECT_NEAR(baseHit->distance, 3.0, 1e-12);
  EXPECT_TRUE(baseHit->point.isApprox(Eigen::Vector3d(0.5, 0.0, -1.0)));
  EXPECT_TRUE(baseHit->normal.isApprox(Eigen::Vector3d(0.0, 0.0, -1.0)));
}

TEST(
    FilamentSceneExtraction,
    PickNearestRenderable_PyramidUsesPrimitiveSurfaceNormal)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("pyramid_robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  body->createShapeNodeWith<VisualAspect>(
      std::make_shared<PyramidShape>(2.0, 2.0, 2.0));
  world->addSkeleton(skeleton);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  const dart::gui::PickRay sideRay{
      Eigen::Vector3d(0.0, -3.0, 0.0), Eigen::Vector3d::UnitY()};
  const auto sideHit = dart::gui::pickNearestRenderable(renderables, sideRay);

  ASSERT_TRUE(sideHit.has_value());
  EXPECT_NEAR(sideHit->distance, 2.5, 1e-12);
  EXPECT_TRUE(sideHit->point.isApprox(Eigen::Vector3d(0.0, -0.5, 0.0)));
  EXPECT_TRUE(
      sideHit->normal.isApprox(Eigen::Vector3d(0.0, -2.0, 1.0).normalized()));

  const dart::gui::PickRay baseRay{
      Eigen::Vector3d(0.5, 0.25, -4.0), Eigen::Vector3d::UnitZ()};
  const auto baseHit = dart::gui::pickNearestRenderable(renderables, baseRay);

  ASSERT_TRUE(baseHit.has_value());
  EXPECT_NEAR(baseHit->distance, 3.0, 1e-12);
  EXPECT_TRUE(baseHit->point.isApprox(Eigen::Vector3d(0.5, 0.25, -1.0)));
  EXPECT_TRUE(baseHit->normal.isApprox(Eigen::Vector3d(0.0, 0.0, -1.0)));
}

TEST(FilamentSceneExtraction, PickNearestRenderable_PlaneUsesFiniteProxySurface)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("plane_robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  body->createShapeNodeWith<VisualAspect>(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.25));
  world->addSkeleton(skeleton);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  const dart::gui::PickRay hitRay{
      Eigen::Vector3d(0.25, 0.5, 2.0), -Eigen::Vector3d::UnitZ()};
  const auto hit = dart::gui::pickNearestRenderable(renderables, hitRay);

  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, 1.75, 1e-12);
  EXPECT_TRUE(hit->point.isApprox(Eigen::Vector3d(0.25, 0.5, 0.25)));
  EXPECT_TRUE(hit->normal.isApprox(Eigen::Vector3d::UnitZ()));

  const dart::gui::PickRay missRay{
      Eigen::Vector3d(1.5, 0.0, 2.0), -Eigen::Vector3d::UnitZ()};
  EXPECT_FALSE(
      dart::gui::pickNearestRenderable(renderables, missRay).has_value());
}

TEST(
    FilamentSceneExtraction,
    PickNearestRenderable_TriangleMeshUsesSurfaceTriangles)
{
  dart::gui::RenderableDescriptor renderable;
  renderable.id = 1u;
  renderable.geometry.kind = ShapeKind::Mesh;
  renderable.geometry.hasLocalBounds = true;
  renderable.geometry.localBoundsMin = Eigen::Vector3d(0.0, 0.0, -0.1);
  renderable.geometry.localBoundsMax = Eigen::Vector3d(1.0, 1.0, 0.1);
  renderable.geometry.triangleVertices
      = {Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0)};
  renderable.geometry.triangleIndices = {Eigen::Vector3i(0, 1, 2)};

  const std::vector<dart::gui::RenderableDescriptor> renderables{renderable};

  const dart::gui::PickRay hitRay{
      Eigen::Vector3d(0.25, 0.25, 1.0), -Eigen::Vector3d::UnitZ()};
  const auto hit = dart::gui::pickNearestRenderable(renderables, hitRay);

  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, 1.0, 1e-12);
  EXPECT_TRUE(hit->point.isApprox(Eigen::Vector3d(0.25, 0.25, 0.0)));
  EXPECT_TRUE(hit->normal.isApprox(Eigen::Vector3d::UnitZ()));

  const dart::gui::PickRay missRay{
      Eigen::Vector3d(0.75, 0.75, 1.0), -Eigen::Vector3d::UnitZ()};
  EXPECT_FALSE(
      dart::gui::pickNearestRenderable(renderables, missRay).has_value());
}

TEST(
    FilamentSceneExtraction,
    PickNearestRenderable_PointCloudUsesPerPointBoxSurface)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("point_cloud_robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto pointCloud = std::make_shared<PointCloudShape>(0.2);
  pointCloud->addPoint(Eigen::Vector3d(0.0, 0.0, 0.0));
  pointCloud->addPoint(Eigen::Vector3d(2.0, 0.0, 0.0));
  body->createShapeNodeWith<VisualAspect>(pointCloud);
  world->addSkeleton(skeleton);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  const dart::gui::PickRay hitRay{
      Eigen::Vector3d(-2.0, 0.05, 0.0), Eigen::Vector3d::UnitX()};
  const auto hit = dart::gui::pickNearestRenderable(renderables, hitRay);

  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, 1.9, 1e-12);
  EXPECT_TRUE(hit->point.isApprox(Eigen::Vector3d(-0.1, 0.05, 0.0)));
  EXPECT_TRUE(hit->normal.isApprox(-Eigen::Vector3d::UnitX()));

  const dart::gui::PickRay gapRay{
      Eigen::Vector3d(1.0, 0.0, -1.0), Eigen::Vector3d::UnitZ()};
  EXPECT_FALSE(
      dart::gui::pickNearestRenderable(renderables, gapRay).has_value());
}

TEST(
    FilamentSceneExtraction,
    PickNearestRenderable_VoxelGridUsesPerVoxelBoxSurface)
{
  dart::gui::RenderableDescriptor renderable;
  renderable.id = 1;
  renderable.geometry.kind = ShapeKind::VoxelGrid;
  renderable.geometry.voxelSize = 0.2;
  renderable.geometry.voxelCenters
      = {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0)};
  renderable.geometry.hasLocalBounds = true;
  renderable.geometry.localBoundsMin = Eigen::Vector3d(-0.1, -0.1, -0.1);
  renderable.geometry.localBoundsMax = Eigen::Vector3d(2.1, 0.1, 0.1);

  const std::vector<dart::gui::RenderableDescriptor> renderables{renderable};

  const dart::gui::PickRay hitRay{
      Eigen::Vector3d(-2.0, 0.05, 0.0), Eigen::Vector3d::UnitX()};
  const auto hit = dart::gui::pickNearestRenderable(renderables, hitRay);

  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, 1.9, 1e-12);
  EXPECT_TRUE(hit->point.isApprox(Eigen::Vector3d(-0.1, 0.05, 0.0)));
  EXPECT_TRUE(hit->normal.isApprox(-Eigen::Vector3d::UnitX()));

  const dart::gui::PickRay gapRay{
      Eigen::Vector3d(1.0, 0.0, -1.0), Eigen::Vector3d::UnitZ()};
  EXPECT_FALSE(
      dart::gui::pickNearestRenderable(renderables, gapRay).has_value());
}

TEST(
    FilamentSceneExtraction, PickNearestRenderable_IgnoresHiddenAndMissedShapes)
{
  auto world = World::create("world");

  auto hiddenSkeleton = Skeleton::create("hidden_box");
  auto [hiddenJoint, hiddenBody]
      = hiddenSkeleton->createJointAndBodyNodePair<FreeJoint>();
  Eigen::Isometry3d hiddenTransform = Eigen::Isometry3d::Identity();
  hiddenTransform.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  hiddenJoint->setTransformFromParentBodyNode(hiddenTransform);
  auto hiddenShape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  auto* hiddenShapeNode
      = hiddenBody->createShapeNodeWith<VisualAspect>(hiddenShape);
  hiddenShapeNode->getVisualAspect()->hide();
  world->addSkeleton(hiddenSkeleton);

  auto visibleSkeleton = Skeleton::create("visible_box");
  auto [visibleJoint, visibleBody]
      = visibleSkeleton->createJointAndBodyNodePair<FreeJoint>();
  Eigen::Isometry3d visibleTransform = Eigen::Isometry3d::Identity();
  visibleTransform.translation() = Eigen::Vector3d(4.0, 0.0, 0.0);
  visibleJoint->setTransformFromParentBodyNode(visibleTransform);
  auto visibleShape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  visibleBody->createShapeNodeWith<VisualAspect>(visibleShape);
  world->addSkeleton(visibleSkeleton);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 2u);

  const dart::gui::PickRay hitRay{
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX()};
  const auto hit = dart::gui::pickNearestRenderable(renderables, hitRay);
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->id, renderables.back().id);
  EXPECT_NEAR(hit->distance, 3.5, 1e-12);
  EXPECT_TRUE(hit->normal.isApprox(Eigen::Vector3d(-1.0, 0.0, 0.0)));

  const dart::gui::PickRay missRay{
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY()};
  EXPECT_FALSE(
      dart::gui::pickNearestRenderable(renderables, missRay).has_value());
}

TEST(
    FilamentSceneExtraction,
    TranslateFreeJointRenderable_SelectedBody_UpdatesBodyTransform)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  Eigen::Isometry3d initialTransform = Eigen::Isometry3d::Identity();
  initialTransform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  joint->setTransformFromParentBodyNode(initialTransform);
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  body->createShapeNodeWith<VisualAspect>(shape, "box_visual");
  world->addSkeleton(skeleton);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  const Eigen::Vector3d offset(0.25, -0.5, 0.75);
  EXPECT_TRUE(
      dart::gui::translateFreeJointRenderable(renderables.front(), offset));
  EXPECT_TRUE(body->getWorldTransform().translation().isApprox(
      initialTransform.translation() + offset));

  EXPECT_FALSE(
      dart::gui::translateFreeJointRenderable(
          renderables.front(),
          Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)));
}

TEST(
    FilamentSceneExtraction,
    TranslateFreeJointRenderable_FixedBody_ReturnsFalse)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("ground");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<WeldJoint>();
  (void)joint;
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  body->createShapeNodeWith<VisualAspect>(shape, "box_visual");
  world->addSkeleton(skeleton);

  const auto before = body->getWorldTransform();
  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  EXPECT_FALSE(
      dart::gui::translateFreeJointRenderable(
          renderables.front(), Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_TRUE(body->getWorldTransform().isApprox(before));
}

TEST(
    FilamentSceneExtraction,
    TranslateFreeJointRenderable_RemovedShapeNode_ReturnsFalse)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  (void)joint;
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  body->createShapeNodeWith<VisualAspect>(shape, "box_visual");
  world->addSkeleton(skeleton);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  body->removeAllShapeNodes();
  EXPECT_FALSE(
      dart::gui::translateFreeJointRenderable(
          renderables.front(), Eigen::Vector3d(1.0, 0.0, 0.0)));
}

TEST(
    FilamentSceneExtraction,
    TranslateSimpleFrameRenderable_SelectedFrame_UpdatesFrameTransform)
{
  auto world = World::create("world");

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(-1.0, 2.0, 0.5);
  auto frame = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "interactive_target", transform);
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->getVisualAspect(true);
  world->addSimpleFrame(frame);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  const Eigen::Vector3d offset(0.2, 0.3, -0.1);
  EXPECT_TRUE(
      dart::gui::translateSimpleFrameRenderable(renderables.front(), offset));
  EXPECT_TRUE(frame->getWorldTransform().translation().isApprox(
      transform.translation() + offset));

  EXPECT_TRUE(
      dart::gui::translateFrameRenderable(renderables.front(), -offset));
  EXPECT_TRUE(frame->getWorldTransform().translation().isApprox(
      transform.translation()));

  EXPECT_FALSE(
      dart::gui::translateSimpleFrameRenderable(
          renderables.front(),
          Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)));
}

TEST(
    FilamentSceneExtraction,
    TranslateSimpleFrameRenderable_RemovedFrame_ReturnsFalse)
{
  auto world = World::create("world");

  auto frame = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "interactive_target");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->getVisualAspect(true);
  world->addSimpleFrame(frame);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  world->removeSimpleFrame(frame);
  frame.reset();
  EXPECT_FALSE(
      dart::gui::translateSimpleFrameRenderable(
          renderables.front(), Eigen::Vector3d(1.0, 0.0, 0.0)));
}

TEST(
    FilamentSceneExtraction,
    RotateSimpleFrameRenderable_SelectedFrame_UpdatesFrameOrientation)
{
  auto world = World::create("world");

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(-1.0, 2.0, 0.5);
  auto frame = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "interactive_target", transform);
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->getVisualAspect(true);
  world->addSimpleFrame(frame);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  const double angle = 0.5;
  EXPECT_TRUE(
      dart::gui::rotateSimpleFrameRenderable(
          renderables.front(), Eigen::Vector3d::UnitZ(), angle));

  Eigen::Isometry3d expected = transform;
  expected.linear()
      = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix()
        * expected.linear();
  EXPECT_TRUE(frame->getWorldTransform().isApprox(expected));

  EXPECT_TRUE(
      dart::gui::rotateFrameRenderable(
          renderables.front(), Eigen::Vector3d::UnitZ(), -angle));
  EXPECT_TRUE(frame->getWorldTransform().isApprox(transform));

  EXPECT_FALSE(
      dart::gui::rotateSimpleFrameRenderable(
          renderables.front(), Eigen::Vector3d::Zero(), angle));
  EXPECT_FALSE(
      dart::gui::rotateSimpleFrameRenderable(
          renderables.front(),
          Eigen::Vector3d::UnitZ(),
          std::numeric_limits<double>::quiet_NaN()));
}

TEST(FilamentSceneExtraction, PlaneDragHelpers_ReturnExpectedTranslation)
{
  const Eigen::Vector3d planePoint = Eigen::Vector3d::Zero();
  const Eigen::Vector3d planeNormal = Eigen::Vector3d::UnitZ();
  const dart::gui::PickRay previousRay{
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, -1.0)};
  const dart::gui::PickRay currentRay{
      Eigen::Vector3d(1.0, 2.0, 1.0), Eigen::Vector3d(0.0, 0.0, -1.0)};

  const auto intersection
      = dart::gui::intersectPlane(previousRay, planePoint, planeNormal);
  ASSERT_TRUE(intersection.has_value());
  EXPECT_TRUE(intersection->isApprox(Eigen::Vector3d::Zero()));

  const auto translation = dart::gui::computePlaneDragTranslation(
      previousRay, currentRay, planePoint, planeNormal);
  ASSERT_TRUE(translation.has_value());
  EXPECT_TRUE(translation->isApprox(Eigen::Vector3d(1.0, 2.0, 0.0)));

  const dart::gui::PickRay parallelRay{
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::UnitX()};
  EXPECT_FALSE(
      dart::gui::intersectPlane(parallelRay, planePoint, planeNormal)
          .has_value());
  EXPECT_FALSE(
      dart::gui::computePlaneDragTranslation(
          parallelRay, currentRay, planePoint, planeNormal)
          .has_value());
}

TEST(FilamentSceneExtraction, AxisDragHelpers_ReturnExpectedTranslation)
{
  const Eigen::Vector3d axisPoint = Eigen::Vector3d::Zero();
  const Eigen::Vector3d axisDirection = Eigen::Vector3d::UnitX();
  const dart::gui::PickRay previousRay{
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, -1.0)};
  const dart::gui::PickRay currentRay{
      Eigen::Vector3d(0.0, 1.0, 1.0), Eigen::Vector3d(1.0, -1.0, -1.0)};

  const auto translation = dart::gui::computeAxisDragTranslation(
      previousRay, currentRay, axisPoint, axisDirection);
  ASSERT_TRUE(translation.has_value());
  EXPECT_TRUE(translation->isApprox(Eigen::Vector3d::UnitX()));

  const dart::gui::PickRay parallelRay{
      Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d::UnitX()};
  EXPECT_FALSE(
      dart::gui::computeAxisDragTranslation(
          parallelRay, currentRay, axisPoint, axisDirection)
          .has_value());
  EXPECT_FALSE(
      dart::gui::computeAxisDragTranslation(
          previousRay, currentRay, axisPoint, Eigen::Vector3d::Zero())
          .has_value());
}

TEST(FilamentSceneExtraction, MakeSelectionDebugLines_ReturnsWorldSpaceBounds)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 4.0, 6.0));
  auto* shapeNode
      = body->createShapeNodeWith<VisualAspect>(shape, "box_visual");

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  joint->setTransformFromParentBodyNode(transform);
  world->addSkeleton(skeleton);

  const auto renderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(renderables.size(), 1u);

  const auto lines = dart::gui::makeSelectionDebugLines(renderables.front());

  ASSERT_EQ(lines.size(), 12u);
  EXPECT_EQ(lines.front().label, "selection.bounds");
  EXPECT_TRUE(lines.front().from.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(lines.front().to.isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_TRUE(lines.back().from.isApprox(Eigen::Vector3d(0.0, 4.0, 0.0)));
  EXPECT_TRUE(lines.back().to.isApprox(Eigen::Vector3d(0.0, 4.0, 6.0)));

  shapeNode->getVisualAspect()->hide();
  const auto hiddenRenderables = dart::gui::extractRenderables(*world);
  ASSERT_EQ(hiddenRenderables.size(), 1u);
  EXPECT_TRUE(
      dart::gui::makeSelectionDebugLines(hiddenRenderables.front()).empty());
}

TEST(FilamentSceneExtraction, MakeCollisionShapeDebugLines_ReturnsBounds)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  body->setName("body");
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 4.0, 6.0)));
  shapeNode->setName("collision_box");

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  joint->setTransformFromParentBodyNode(transform);
  world->addSkeleton(skeleton);

  EXPECT_TRUE(dart::gui::extractRenderables(*world).empty());

  dart::gui::DebugDrawOptions options;
  options.drawGrid = false;
  options.drawWorldFrame = false;
  options.drawContacts = false;
  options.drawCollisionShapeBounds = true;
  options.collisionBoundsPadding = 0.1;

  const auto lines = dart::gui::makeCollisionShapeDebugLines(
      *shapeNode, options, "robot/body/collision_box");

  ASSERT_EQ(lines.size(), 12u);
  EXPECT_EQ(lines.front().label, "robot/body/collision_box.collision_bounds");

  Eigen::Vector3d min
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d max
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (const auto& line : lines) {
    min = min.cwiseMin(line.from).cwiseMin(line.to);
    max = max.cwiseMax(line.from).cwiseMax(line.to);
  }
  EXPECT_TRUE(min.isApprox(Eigen::Vector3d(-0.1, -0.1, -0.1)));
  EXPECT_TRUE(max.isApprox(Eigen::Vector3d(2.1, 4.1, 6.1)));

  const auto extractedLines = dart::gui::extractDebugLines(*world, options);
  ASSERT_EQ(extractedLines.size(), lines.size());
  EXPECT_EQ(
      extractedLines.front().label,
      "robot/body/collision_box.collision_bounds");
}

TEST(FilamentSceneExtraction, ExtractDebugLines_ReturnsGridAndFrameAxes)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  body->setName("body");

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  joint->setTransformFromParentBodyNode(transform);
  world->addSkeleton(skeleton);

  dart::gui::DebugDrawOptions options;
  options.drawContacts = false;
  options.drawBodyFrames = true;
  options.gridHalfExtent = 1.0;
  options.gridSpacing = 1.0;
  options.worldFrameAxisLength = 0.5;
  options.bodyFrameAxisLength = 0.25;

  const auto lines = dart::gui::extractDebugLines(*world, options);

  ASSERT_EQ(lines.size(), 12u);
  EXPECT_EQ(lines[0].label, "grid");
  EXPECT_TRUE(lines[6].from.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(lines[6].to.isApprox(Eigen::Vector3d(0.5, 0.0, 0.0)));
  EXPECT_EQ(lines[9].label, "robot/body.x");
  EXPECT_TRUE(lines[9].from.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(lines[9].to.isApprox(Eigen::Vector3d(1.25, 2.0, 3.0)));
}

TEST(FilamentSceneExtraction, ExtractDebugLines_ReturnsCenterOfMassMarker)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  body->setMass(2.0);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  joint->setTransformFromParentBodyNode(transform);
  world->addSkeleton(skeleton);

  dart::gui::DebugDrawOptions options;
  options.drawGrid = false;
  options.drawWorldFrame = false;
  options.drawContacts = false;
  options.drawCentersOfMass = true;
  options.centerOfMassMarkerRadius = 0.2;

  const auto lines = dart::gui::extractDebugLines(*world, options);

  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[0].label, "robot.com.x");
  EXPECT_TRUE(lines[0].from.isApprox(Eigen::Vector3d(0.8, 2.0, 3.0)));
  EXPECT_TRUE(lines[0].to.isApprox(Eigen::Vector3d(1.2, 2.0, 3.0)));
  EXPECT_EQ(lines[2].label, "robot.com.z");
  EXPECT_TRUE(lines[2].to.isApprox(Eigen::Vector3d(1.0, 2.0, 3.2)));
}

TEST(FilamentSceneExtraction, MakeInertiaDebugLines_ReturnsEquivalentBox)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  body->setName("body");
  body->setMass(12.0);
  body->setMomentOfInertia(52.0, 40.0, 20.0);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  joint->setTransformFromParentBodyNode(transform);
  world->addSkeleton(skeleton);

  dart::gui::DebugDrawOptions options;
  options.drawGrid = false;
  options.drawWorldFrame = false;
  options.drawContacts = false;
  options.drawInertiaBoxes = true;

  const auto lines
      = dart::gui::makeInertiaDebugLines(*body, options, "robot/body");

  ASSERT_EQ(lines.size(), 12u);
  EXPECT_EQ(lines.front().label, "robot/body.inertia");

  Eigen::Vector3d min
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d max
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (const auto& line : lines) {
    min = min.cwiseMin(line.from).cwiseMin(line.to);
    max = max.cwiseMax(line.from).cwiseMax(line.to);
  }
  EXPECT_TRUE(min.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(max.isApprox(Eigen::Vector3d(2.0, 4.0, 6.0)));

  const auto extractedLines = dart::gui::extractDebugLines(*world, options);
  ASSERT_EQ(extractedLines.size(), lines.size());
  EXPECT_EQ(extractedLines.front().label, "robot/body.inertia");
}

TEST(
    FilamentSceneExtraction,
    MakeSupportPolygonDebugLines_ReturnsOutlineAndCentroid)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("support_bot");
  skeleton->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  auto* body = skeleton->createJointAndBodyNodePair<FreeJoint>().second;
  body->setName("base");
  body->setMass(1.0);

  const SupportGeometry supportGeometry = makeSupportFootGeometry();
  addSupportEndEffector(
      body, "left_foot", Eigen::Vector3d(-0.35, 0.18, 0.0), supportGeometry);
  addSupportEndEffector(
      body, "right_foot", Eigen::Vector3d(0.35, -0.18, 0.0), supportGeometry);

  dart::gui::DebugDrawOptions options;
  options.drawGrid = false;
  options.drawWorldFrame = false;
  options.drawContacts = false;
  options.drawSupportPolygons = true;
  options.drawSupportCentroids = true;
  options.supportPolygonElevation = 0.04;
  options.supportCentroidMarkerRadius = 0.05;

  const dart::math::SupportPolygon& polygon = skeleton->getSupportPolygon();
  ASSERT_GE(polygon.size(), 3u);

  const auto lines = dart::gui::makeSupportPolygonDebugLines(
      *skeleton, options, skeleton->getName());

  ASSERT_EQ(lines.size(), polygon.size() + 3u);
  EXPECT_EQ(lines.front().label, "support_bot.support_polygon");
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    EXPECT_NEAR(lines[i].from.z(), options.supportPolygonElevation, 1e-12);
    EXPECT_NEAR(lines[i].to.z(), options.supportPolygonElevation, 1e-12);
  }

  const auto& axes = skeleton->getSupportAxes();
  const Eigen::Vector3d up = axes.first.cross(axes.second).normalized();
  const Eigen::Vector2d& centroid = skeleton->getSupportCentroid();
  const Eigen::Vector3d expectedCentroid
      = axes.first * centroid.x() + axes.second * centroid.y()
        + up * options.supportPolygonElevation;
  const std::size_t centroidStart = polygon.size();
  EXPECT_EQ(lines[centroidStart].label, "support_bot.support_centroid.x");
  EXPECT_TRUE(
      lines[centroidStart].from.isApprox(
          expectedCentroid
          - Eigen::Vector3d::UnitX() * options.supportCentroidMarkerRadius));
  EXPECT_TRUE(
      lines[centroidStart].to.isApprox(
          expectedCentroid
          + Eigen::Vector3d::UnitX() * options.supportCentroidMarkerRadius));

  world->addSkeleton(skeleton);
  const auto extractedLines = dart::gui::extractDebugLines(*world, options);
  ASSERT_EQ(extractedLines.size(), lines.size());
  EXPECT_EQ(extractedLines.front().label, "support_bot.support_polygon");
}

TEST(FilamentSceneExtraction, ExtractContactDebugLines_ReturnsMarkersAndVectors)
{
  auto detector = dart::collision::DartCollisionDetector::create();
  auto skeletonA = Skeleton::create("contact_a");
  auto skeletonB = Skeleton::create("contact_b");
  auto* bodyA = skeletonA->createJointAndBodyNodePair<FreeJoint>().second;
  auto* bodyB = skeletonB->createJointAndBodyNodePair<FreeJoint>().second;
  auto* shapeA = bodyA->createShapeNodeWith<dart::dynamics::CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.5)));
  auto* shapeB = bodyB->createShapeNodeWith<dart::dynamics::CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.5)));

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(shapeA);
  group->addShapeFrame(shapeB);

  dart::collision::CollisionResult result;
  dart::collision::CollisionOption collisionOptions;
  collisionOptions.maxNumContacts = 1u;
  ASSERT_TRUE(group->collide(collisionOptions, &result));
  ASSERT_LT(0u, result.getNumContacts());

  auto& contact = result.getContact(0);
  contact.point = Eigen::Vector3d(1.0, 2.0, 3.0);
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.force = Eigen::Vector3d(10.0, 0.0, 0.0);

  dart::gui::DebugDrawOptions options;
  options.contactMarkerHalfExtent = 0.1;
  options.contactNormalLength = 0.3;
  options.contactForceScale = 0.02;
  options.contactForceMinLength = 0.0;
  options.contactForceMaxLength = 1.0;

  const auto lines = dart::gui::extractContactDebugLines(result, options);

  ASSERT_EQ(lines.size(), 8u);
  EXPECT_EQ(lines[0].label, "contact.point");
  EXPECT_TRUE(lines[0].from.isApprox(Eigen::Vector3d(0.9, 2.0, 3.0)));
  EXPECT_TRUE(lines[0].to.isApprox(Eigen::Vector3d(1.1, 2.0, 3.0)));
  EXPECT_EQ(lines[2].label, "contact.normal");
  EXPECT_TRUE(lines[2].to.isApprox(Eigen::Vector3d(1.0, 2.0, 3.3)));
  EXPECT_EQ(lines[3].label, "contact.normal");
  EXPECT_TRUE(lines[3].from.isApprox(Eigen::Vector3d(1.0, 2.0, 3.3)));
  EXPECT_EQ(lines[5].label, "contact.force");
  EXPECT_TRUE(lines[5].to.isApprox(Eigen::Vector3d(1.2, 2.0, 3.0)));
  EXPECT_EQ(lines[6].label, "contact.force");
  EXPECT_TRUE(lines[6].from.isApprox(Eigen::Vector3d(1.2, 2.0, 3.0)));
}

TEST(FilamentSceneExtraction, RunOptions_NormalizeAndGateBoundedCapture)
{
  const auto nativeWindowSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "native_window.cpp");

  dart::gui::RunOptions options;
  EXPECT_EQ(options.windowTitle, "dartsim");
  options.windowTitle.clear();
  options.width = 0;
  options.height = -8;
  options.guiScale = std::numeric_limits<double>::quiet_NaN();
  options.headless = true;
  options.screenshotPath = "capture.ppm";

  dart::gui::normalizeRunOptions(options);

  EXPECT_EQ(options.windowTitle, "dartsim");
  EXPECT_EQ(options.width, 1);
  EXPECT_EQ(options.height, 1);
  EXPECT_NEAR(options.guiScale, 1.0, 1e-12);
  EXPECT_EQ(options.maxFrames, 1);
  EXPECT_TRUE(options.headless);
  EXPECT_TRUE(dart::gui::shouldRequestScreenshot(options, 0, false));
  EXPECT_FALSE(dart::gui::shouldCaptureFrameOutput(options));
  EXPECT_FALSE(dart::gui::shouldRequestScreenshot(options, 0, true));
  EXPECT_FALSE(dart::gui::shouldStopAfterFrame(options, 0));
  EXPECT_TRUE(dart::gui::shouldStopAfterFrame(options, 1));

  dart::gui::ViewerLifecycleState state;
  EXPECT_TRUE(dart::gui::shouldAdvanceSimulation(state));
  dart::gui::togglePaused(state);
  EXPECT_TRUE(state.paused);
  EXPECT_FALSE(dart::gui::shouldAdvanceSimulation(state));
  dart::gui::requestSingleStep(state);
  EXPECT_TRUE(state.paused);
  EXPECT_TRUE(dart::gui::shouldAdvanceSimulation(state));
  dart::gui::markSimulationAdvanced(state);
  EXPECT_FALSE(state.stepOnce);
  EXPECT_FALSE(dart::gui::shouldAdvanceSimulation(state));
  EXPECT_TRUE(dart::gui::shouldRequestScreenshot(options, state));
  dart::gui::markScreenshotRequested(state);
  EXPECT_FALSE(dart::gui::shouldRequestScreenshot(options, state));
  dart::gui::markFrameSkipped(state);
  EXPECT_EQ(state.skippedFrames, 1);
  dart::gui::markFrameRendered(state);
  EXPECT_EQ(state.renderedFrames, 1);
  EXPECT_EQ(state.skippedFrames, 0);
  EXPECT_TRUE(dart::gui::shouldStopAfterFrame(options, state));
  EXPECT_FALSE(state.exitRequested);
  dart::gui::requestExit(state);
  EXPECT_TRUE(state.exitRequested);

  dart::gui::RunOptions windowOnly;
  windowOnly.guiScale = 10.0;
  dart::gui::normalizeRunOptions(windowOnly);
  EXPECT_NEAR(windowOnly.guiScale, 4.0, 1e-12);
  EXPECT_FALSE(dart::gui::shouldRequestScreenshot(windowOnly, 0, false));

  dart::gui::RunOptions sequenceOutput;
  sequenceOutput.frameOutputDirectory = "frames";
  dart::gui::normalizeRunOptions(sequenceOutput);
  EXPECT_EQ(sequenceOutput.maxFrames, 1);
  EXPECT_TRUE(dart::gui::shouldCaptureFrameOutput(sequenceOutput));
  EXPECT_EQ(
      std::filesystem::path(dart::gui::makeFrameOutputPath(sequenceOutput, 7))
          .filename()
          .string(),
      "frame_000007.ppm");
  dart::gui::ViewerLifecycleState captureState;
  EXPECT_FALSE(dart::gui::shouldCaptureFrameOutput(windowOnly, captureState));
  dart::gui::setFrameOutputCapture(captureState, "runtime_frames", true);
  EXPECT_TRUE(captureState.frameOutputEnabled);
  EXPECT_TRUE(dart::gui::shouldCaptureFrameOutput(windowOnly, captureState));
  const std::filesystem::path runtimeFramePath
      = dart::gui::makeFrameOutputPath(windowOnly, captureState, 8);
  EXPECT_EQ(runtimeFramePath.filename().string(), "frame_000008.ppm");
  EXPECT_EQ(
      runtimeFramePath.parent_path().filename().string(), "runtime_frames");
  dart::gui::toggleFrameOutputCapture(captureState, "runtime_frames");
  EXPECT_FALSE(captureState.frameOutputEnabled);
  EXPECT_NE(
      nativeWindowSource.find("options.windowTitle.c_str()"),
      std::string::npos);
}

TEST(FilamentSceneExtraction, WriteRgbaPpm_DropsAlphaAndHandlesBottomLeftOrigin)
{
  const auto path
      = std::filesystem::temp_directory_path() / "dart_gui_rgba.ppm";
  std::filesystem::remove(path);

  const std::vector<std::uint8_t> rgbaPixels = {
      255,
      0,
      0,
      11, // bottom-left red
      0,
      255,
      0,
      12, // bottom-right green
      0,
      0,
      255,
      13, // top-left blue
      255,
      255,
      255,
      14, // top-right white
  };

  std::string error;
  ASSERT_TRUE(
      dart::gui::writeRgbaPpm(path.string(), 2, 2, rgbaPixels, true, &error))
      << error;

  std::ifstream in(path, std::ios::binary);
  ASSERT_TRUE(in);
  const std::vector<char> bytes(
      (std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  in.close();
  const std::vector<char> expected = {
      'P',
      '6',
      '\n',
      '2',
      ' ',
      '2',
      '\n',
      '2',
      '5',
      '5',
      '\n',
      static_cast<char>(0),
      static_cast<char>(0),
      static_cast<char>(255),
      static_cast<char>(255),
      static_cast<char>(255),
      static_cast<char>(255),
      static_cast<char>(255),
      static_cast<char>(0),
      static_cast<char>(0),
      static_cast<char>(0),
      static_cast<char>(255),
      static_cast<char>(0),
  };
  EXPECT_EQ(bytes, expected);

  error.clear();
  EXPECT_FALSE(
      dart::gui::writeRgbaPpm(
          path.string(), 2, 2, {255, 0, 0, 255}, false, &error));
  EXPECT_FALSE(error.empty());

  std::error_code cleanupError;
  std::filesystem::remove(path, cleanupError);
}

TEST(FilamentSceneExtraction, OrbitCamera_UpdateBasisAndPickingAreStable)
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.yaw = 0.0;
  camera.pitch = 0.0;
  camera.distance = 2.0;

  const auto basis = dart::gui::makeOrbitCameraBasis(camera);
  EXPECT_TRUE(basis.eye.isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_TRUE(basis.forward.isApprox(-Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(basis.right.isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(basis.up.isApprox(Eigen::Vector3d::UnitZ()));

  dart::gui::OrbitCamera yUpCamera = camera;
  yUpCamera.up = Eigen::Vector3d::UnitY();
  const auto yUpBasis = dart::gui::makeOrbitCameraBasis(yUpCamera);
  EXPECT_TRUE(yUpBasis.eye.isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_TRUE(yUpBasis.forward.isApprox(-Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(yUpBasis.right.isApprox(-Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(yUpBasis.up.isApprox(Eigen::Vector3d::UnitY()));

  dart::gui::DirectionalNudgeInput nudgeInput;
  nudgeInput.right = true;
  nudgeInput.forward = true;
  nudgeInput.up = true;
  nudgeInput.fast = true;
  nudgeInput.stepSize = 0.25;
  nudgeInput.fastMultiplier = 2.0;
  const auto nudge = dart::gui::computeCameraRelativeNudge(camera, nudgeInput);
  EXPECT_TRUE(nudge.isApprox(Eigen::Vector3d(-0.5, 0.5, 0.5)));

  nudgeInput.stepSize = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(
      dart::gui::computeCameraRelativeNudge(camera, nudgeInput).isZero());

  dart::gui::OrbitCameraController controller;
  controller.camera = camera;
  dart::gui::OrbitCameraControllerInput controllerInput;
  controllerInput.cursorX = 100.0;
  controllerInput.cursorY = 50.0;
  dart::gui::updateOrbitCameraController(controller, controllerInput);
  EXPECT_TRUE(controller.hasLastCursor);
  EXPECT_NEAR(controller.lastCursorX, 100.0, 1e-12);
  EXPECT_NEAR(controller.lastCursorY, 50.0, 1e-12);
  EXPECT_TRUE(controller.camera.target.isApprox(Eigen::Vector3d::Zero()));

  dart::gui::addOrbitCameraScroll(controller, 1.0);
  controllerInput.cursorX = 110.0;
  controllerInput.cursorY = 70.0;
  controllerInput.pan = true;
  dart::gui::updateOrbitCameraController(controller, controllerInput);
  EXPECT_NEAR(controller.scrollDelta, 0.0, 1e-12);
  EXPECT_TRUE(
      controller.camera.target.isApprox(Eigen::Vector3d(0.0, -0.03, 0.06)));
  EXPECT_LT(controller.camera.distance, 2.0);

  controllerInput.hasCursor = false;
  dart::gui::updateOrbitCameraController(controller, controllerInput);
  EXPECT_FALSE(controller.hasLastCursor);

  const auto ray
      = dart::gui::makePerspectivePickRay(camera, 320, 240, 640, 480);
  EXPECT_TRUE(ray.origin.isApprox(basis.eye));
  EXPECT_TRUE(ray.direction.isApprox(basis.forward));

  dart::gui::OrbitCameraUpdate panUpdate;
  panUpdate.pan = true;
  panUpdate.deltaX = 10.0;
  panUpdate.deltaY = 20.0;
  panUpdate.panScale = 0.1;
  dart::gui::updateOrbitCamera(camera, panUpdate);
  EXPECT_TRUE(camera.target.isApprox(Eigen::Vector3d(0.0, -2.0, 4.0)));

  dart::gui::OrbitCameraUpdate orbitUpdate;
  orbitUpdate.orbit = true;
  orbitUpdate.deltaX = 10.0;
  orbitUpdate.deltaY = 1000.0;
  dart::gui::updateOrbitCamera(camera, orbitUpdate);
  EXPECT_NEAR(camera.yaw, -0.06, 1e-12);
  EXPECT_NEAR(camera.pitch, orbitUpdate.maxPitch, 1e-12);

  dart::gui::OrbitCameraUpdate scrollUpdate;
  scrollUpdate.scrollDelta = 100.0;
  dart::gui::updateOrbitCamera(camera, scrollUpdate);
  EXPECT_NEAR(camera.distance, scrollUpdate.minDistance, 1e-12);

  camera.distance = 2.0;
  const auto projection
      = dart::gui::makePerspectiveProjection(camera, 640, 480);
  EXPECT_NEAR(projection.verticalFovDegrees, 45.0, 1e-12);
  EXPECT_NEAR(projection.aspectRatio, 4.0 / 3.0, 1e-12);
  EXPECT_NEAR(projection.nearPlane, 0.002, 1e-12);
  EXPECT_NEAR(projection.farPlane, 37.0, 1e-12);

  camera.distance = 0.1;
  const auto closeProjection
      = dart::gui::makePerspectiveProjection(camera, 640, 480);
  EXPECT_NEAR(closeProjection.nearPlane, 0.0005, 1e-12);
  camera.distance = 2.0;

  dart::gui::ProjectionOptions projectionOptions;
  projectionOptions.verticalFovDegrees
      = std::numeric_limits<double>::quiet_NaN();
  projectionOptions.nearPlane = 0.1;
  projectionOptions.farPlane = 0.05;
  projectionOptions.minFarPlane = 10.0;
  projectionOptions.farPadding = 5.0;
  const auto overrideProjection
      = dart::gui::makePerspectiveProjection(camera, 0, -20, projectionOptions);
  EXPECT_NEAR(overrideProjection.verticalFovDegrees, 45.0, 1e-12);
  EXPECT_NEAR(overrideProjection.aspectRatio, 1.0, 1e-12);
  EXPECT_NEAR(overrideProjection.nearPlane, 0.1, 1e-12);
  EXPECT_NEAR(overrideProjection.farPlane, 10.0, 1e-12);
}

} // namespace
