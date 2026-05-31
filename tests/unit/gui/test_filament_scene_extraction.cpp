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
#include <dart/gui/detail/application.hpp>
#include <dart/gui/detail/frame_viewport.hpp>
#include <dart/gui/detail/gui_scale.hpp>
#include <dart/gui/detail/input.hpp>
#include <dart/gui/detail/scenes.hpp>
#include <dart/gui/detail/simulation_stepper.hpp>
#include <dart/gui/geometry.hpp>
#include <dart/gui/gizmo.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/scene.hpp>

#define private public
#include <dart/gui/detail/selection.hpp>
#undef private

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
  EXPECT_NE(
      inputSource.find("controls.mouseMode == OrbitCameraMouseMode::Orbit"),
      std::string::npos);
  EXPECT_NE(
      inputSource.find("controls.mouseMode == OrbitCameraMouseMode::Pan"),
      std::string::npos);
  EXPECT_NE(
      inputSource.find("controls.mouseMode == OrbitCameraMouseMode::Zoom"),
      std::string::npos);
  EXPECT_NE(sceneFrameSource.find("uiCapturesMouse"), std::string::npos);
  EXPECT_NE(
      sceneFrameSource.find("const bool mouseSelectionCommitted"),
      std::string::npos);
  EXPECT_NE(
      sceneFrameSource.find("= mSelectionController.updateMouseSelection"),
      std::string::npos);
  EXPECT_NE(
      sceneFrameSource.find("mouseSelectionCommitted && after == 0"),
      std::string::npos);
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
      applicationSource.find(
          "appOptions.cameraUpdater(cameraController.camera)"),
      std::string::npos);
  EXPECT_EQ(
      applicationSource.find("resetOrbitCameraTracking(cameraController)"),
      std::string::npos);
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
  EXPECT_NE(panelSource.find("\"Debug overlays\""), std::string::npos);
  EXPECT_NE(panelSource.find("\"Viewer help\""), std::string::npos);
}

TEST(FilamentSceneExtraction, DockLayoutBuildsBeforeDockspaceSubmission)
{
  const auto uiFrameSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "ui_frame.cpp");

  const auto dockIdPos = uiFrameSource.find("ImHashStr(\"DARTMainDockSpace\")");
  const auto buildLayoutPos
      = uiFrameSource.find("buildDefaultDockLayout(dockId, panels)");
  const auto submitDockspacePos
      = uiFrameSource.find("ImGui::DockSpaceOverViewport(");
  EXPECT_NE(dockIdPos, std::string::npos);
  EXPECT_NE(buildLayoutPos, std::string::npos);
  EXPECT_NE(submitDockspacePos, std::string::npos);
  EXPECT_LT(buildLayoutPos, submitDockspacePos);
  EXPECT_NE(
      uiFrameSource.find("DockBuilderDockWindow(panel.title.c_str(), target)"),
      std::string::npos);
}

TEST(FilamentSceneExtraction, DemoCatalogSearchMatchesSceneMetadata)
{
  const auto factory = [] {
    return dart::gui::ApplicationOptions{};
  };
  const dart::gui::DemoSceneEntry scene{
      "ipc_deformable_fem_buckle",
      "IPC FEM Buckle",
      "IPC Deformable (sx)",
      "Self-contact buckling demo",
      factory};

  EXPECT_EQ(dart::gui::detail::normalizedDemoSearchText("  FEM  "), "  fem  ");
  EXPECT_TRUE(
      dart::gui::detail::demoSceneMatchesSearch(
          scene, dart::gui::detail::normalizedDemoSearchText("FEM")));
  EXPECT_TRUE(
      dart::gui::detail::demoSceneMatchesSearch(
          scene, dart::gui::detail::normalizedDemoSearchText("deformable")));
  EXPECT_TRUE(
      dart::gui::detail::demoSceneMatchesSearch(
          scene, dart::gui::detail::normalizedDemoSearchText("SELF-CONTACT")));
  EXPECT_TRUE(dart::gui::detail::demoSceneMatchesSearch(scene, ""));
  EXPECT_TRUE(dart::gui::detail::demoSceneMatchesExperimentalFocus(scene));
  EXPECT_TRUE(
      dart::gui::detail::demoSceneVisibleInNavigator(
          scene, dart::gui::detail::normalizedDemoSearchText("FEM"), true));
  EXPECT_FALSE(
      dart::gui::detail::demoSceneMatchesSearch(
          scene, dart::gui::detail::normalizedDemoSearchText("cartpole")));

  const dart::gui::DemoSceneEntry legacyScene{
      "boxes", "Boxes", "Rigid Body", "stacked boxes", factory};
  EXPECT_FALSE(
      dart::gui::detail::demoSceneMatchesExperimentalFocus(legacyScene));
  EXPECT_TRUE(
      dart::gui::detail::demoSceneVisibleInNavigator(
          legacyScene,
          dart::gui::detail::normalizedDemoSearchText("boxes"),
          false));
  EXPECT_FALSE(
      dart::gui::detail::demoSceneVisibleInNavigator(
          legacyScene,
          dart::gui::detail::normalizedDemoSearchText("boxes"),
          true));
}

TEST(FilamentSceneExtraction, DemoCatalogGroupsNonContiguousCategories)
{
  const auto factory = [] {
    return dart::gui::ApplicationOptions{};
  };
  const std::vector<dart::gui::DemoSceneEntry> scenes{
      {"hello_world", "Hello World", "Getting Started", "intro", factory},
      {"sx_contact", "Contact sx", "Experimental", "contact", factory},
      {"boxes", "Boxes", "Rigid Body", "rigid", factory},
      {"empty", "Empty", "Getting Started", "empty", factory},
      {"sx_articulated", "Articulated sx", "Experimental", "joints", factory},
  };

  const auto groups = dart::gui::detail::groupDemoScenesByCategory(scenes);

  ASSERT_EQ(groups.size(), 3u);
  EXPECT_EQ(groups[0].category, "Getting Started");
  ASSERT_EQ(groups[0].sceneIndices.size(), 2u);
  EXPECT_EQ(groups[0].sceneIndices[0], 0u);
  EXPECT_EQ(groups[0].sceneIndices[1], 3u);
  EXPECT_EQ(groups[1].category, "Experimental");
  ASSERT_EQ(groups[1].sceneIndices.size(), 2u);
  EXPECT_EQ(groups[1].sceneIndices[0], 1u);
  EXPECT_EQ(groups[1].sceneIndices[1], 4u);
  EXPECT_EQ(groups[2].category, "Rigid Body");
  ASSERT_EQ(groups[2].sceneIndices.size(), 1u);
  EXPECT_EQ(groups[2].sceneIndices[0], 2u);
}

TEST(FilamentSceneExtraction, DemosWorkspaceUsesDockedNavigationAndControls)
{
  const auto applicationSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "application.cpp");
  const auto uiFrameSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "ui_frame.cpp");
  const auto panelSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "panel.cpp");
  const auto runnerSource = readSourceFile(
      std::filesystem::path("python") / "examples" / "demos" / "runner.py");
  const auto viewerBindingSource = readSourceFile(
      std::filesystem::path("python") / "dartpy" / "gui" / "viewer.cpp");
  const auto panelBindingSource = readSourceFile(
      std::filesystem::path("python") / "dartpy" / "gui" / "panel.cpp");
  const auto panelBindingHeaderSource = readSourceFile(
      std::filesystem::path("python") / "dartpy" / "gui" / "panel.hpp");
  const auto triMeshBindingSource = readSourceFile(
      std::filesystem::path("python") / "dartpy" / "math" / "tri_mesh.cpp");
  const auto endEffectorBindingHeaderSource = readSourceFile(
      std::filesystem::path("python") / "dartpy" / "dynamics"
      / "end_effector.hpp");
  const auto hierarchicalIkBindingHeaderSource = readSourceFile(
      std::filesystem::path("python") / "dartpy" / "dynamics"
      / "hierarchical_ik.hpp");
  const auto jacobianNodeBindingHeaderSource = readSourceFile(
      std::filesystem::path("python") / "dartpy" / "dynamics"
      / "jacobian_node.hpp");
  const auto selectionSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "selection.cpp");

  EXPECT_NE(
      applicationSource.find("makeDemoSimulationPanel"), std::string::npos);
  EXPECT_NE(
      applicationSource.find("panel.title = \"Simulation\""),
      std::string::npos);
  EXPECT_NE(
      applicationSource.find("panel.dockSide = dart::gui::DockSide::Top"),
      std::string::npos);
  EXPECT_NE(
      applicationSource.find("builder.textInput(\"##demo_search\", search)"),
      std::string::npos);
  EXPECT_NE(
      applicationSource.find("groupDemoScenesByCategory"), std::string::npos);
  EXPECT_NE(applicationSource.find("Experimental focus"), std::string::npos);
  EXPECT_NE(
      applicationSource.find("demoSidebarExperimentalFocus"),
      std::string::npos);
  EXPECT_NE(applicationSource.find("visibleSceneCount"), std::string::npos);
  EXPECT_EQ(applicationSource.find("categoryEnd"), std::string::npos);
  EXPECT_NE(applicationSource.find("categoryHasActive"), std::string::npos);
  EXPECT_NE(
      applicationSource.find("toggleFrameOutputCapture"), std::string::npos);
  EXPECT_NE(
      applicationSource.find(
          "recordedFramePaths(lifecycle->frameOutputDirectory)"),
      std::string::npos);
  EXPECT_NE(
      applicationSource.find(
          "advanceRecordedFramePlayback(*lifecycle, recordedFrameCount)"),
      std::string::npos);
  EXPECT_NE(
      applicationSource.find(">##play_recorded_frames"), std::string::npos);
  EXPECT_NE(
      applicationSource.find("requestSceneSwitch(*lifecycle, active->id)"),
      std::string::npos);
  EXPECT_NE(
      applicationSource.find("pendingDemoFallbackIndex"), std::string::npos);
  EXPECT_EQ(
      applicationSource.find("&& !lifecycle->sceneSwitchRequested"),
      std::string::npos);
  EXPECT_NE(
      applicationSource.find("sceneActivationPendingScene != entry.id"),
      std::string::npos);
  EXPECT_NE(
      applicationSource.find("kDemoSceneStartupTimeoutMs"), std::string::npos);
  EXPECT_NE(
      applicationSource.find("restoring previous demo"), std::string::npos);
  EXPECT_NE(applicationSource.find("sceneActivationStatus"), std::string::npos);
  EXPECT_NE(
      applicationSource.find("Restored previous demo"), std::string::npos);
  EXPECT_NE(
      applicationSource.find("DART_DEMO_SCENE_STARTUP_TIMEOUT_MS"),
      std::string::npos);
  EXPECT_NE(
      viewerBindingSource.find("py-demos factory error"), std::string::npos);
  EXPECT_NE(viewerBindingSource.find("throw;"), std::string::npos);
  EXPECT_NE(
      applicationSource.find("requestSceneReplay(*lifecycle, active->id)"),
      std::string::npos);
  EXPECT_NE(
      applicationSource.find("requestDockLayoutReset(*lifecycle)"),
      std::string::npos);
  EXPECT_NE(
      uiFrameSource.find("consumeDockLayoutResetRequest(lifecycle)"),
      std::string::npos);
  EXPECT_NE(
      uiFrameSource.find("buildDefaultDockLayout(dockId, panels)"),
      std::string::npos);
  EXPECT_NE(
      uiFrameSource.find("lifecycle.dockLayoutInitialized"), std::string::npos);
  EXPECT_EQ(
      uiFrameSource.find("dartScene.dockLayoutInitialized"), std::string::npos);
  EXPECT_NE(
      uiFrameSource.find("clearDockNodeResizeLocks(dockId)"),
      std::string::npos);
  EXPECT_NE(
      uiFrameSource.find("ImGuiDockNodeFlags_NoResizeX"), std::string::npos);
  EXPECT_EQ(
      uiFrameSource.find("node == nullptr || node->IsLeafNode()"),
      std::string::npos);
  EXPECT_NE(runnerSource.find("dock_side: str = \"right\""), std::string::npos);
  EXPECT_NE(runnerSource.find("auto_resize: bool = False"), std::string::npos);
  EXPECT_NE(
      panelSource.find("panel.autoResize && !dockingActive"),
      std::string::npos);
  EXPECT_EQ(
      applicationSource.find(
          "scenePanel.dockSide = dart::gui::DockSide::Right"),
      std::string::npos);
  EXPECT_NE(
      uiFrameSource.find("center, ImGuiDir_Down, 0.12f"), std::string::npos);
  EXPECT_NE(
      uiFrameSource.find("center, ImGuiDir_Left, 0.24f"), std::string::npos);
  EXPECT_NE(
      uiFrameSource.find("center, ImGuiDir_Right, 0.34f"), std::string::npos);
  EXPECT_NE(
      viewerBindingSource.find("makeGuiPanelFromPython(panel)"),
      std::string::npos);
  EXPECT_NE(
      panelBindingHeaderSource.find("#include <dart/gui/fwd.hpp>"),
      std::string::npos);
  EXPECT_NE(
      panelBindingHeaderSource.find("#include <nanobind/nanobind.h>"),
      std::string::npos);
  EXPECT_EQ(
      panelBindingHeaderSource.find("#include <dart/gui/panel.hpp>"),
      std::string::npos);
  const auto bindingHeaderInclude
      = panelBindingSource.find("#include \"gui/panel.hpp\"");
  const auto dartPanelInclude
      = panelBindingSource.find("#include <dart/gui/panel.hpp>");
  EXPECT_NE(bindingHeaderInclude, std::string::npos);
  EXPECT_NE(dartPanelInclude, std::string::npos);
  EXPECT_LT(bindingHeaderInclude, dartPanelInclude);
  const auto triMeshNanobindInclude
      = triMeshBindingSource.find("#include <nanobind/nanobind.h>");
  const auto triMeshDartInclude
      = triMeshBindingSource.find("#include \"dart/math/tri_mesh.hpp\"");
  EXPECT_NE(triMeshNanobindInclude, std::string::npos);
  EXPECT_NE(triMeshDartInclude, std::string::npos);
  EXPECT_LT(triMeshNanobindInclude, triMeshDartInclude);
  EXPECT_NE(
      endEffectorBindingHeaderSource.find("#include <nanobind/nanobind.h>"),
      std::string::npos);
  EXPECT_NE(
      hierarchicalIkBindingHeaderSource.find("#include <nanobind/nanobind.h>"),
      std::string::npos);
  EXPECT_NE(
      jacobianNodeBindingHeaderSource.find("#include <nanobind/nanobind.h>"),
      std::string::npos);
  EXPECT_EQ(
      selectionSource.find(
          "mSelectedDragMode = DragMode::Force;\n"
          "  mSelectionBoundsVisible = false;\n"
          "  lifecycle.paused = true;"),
      std::string::npos);
  EXPECT_EQ(
      selectionSource.find(
          "if (mSelectedDragMode == DragMode::Force) {\n"
          "      const PickRay ray = makePanePickRay(inputPane, cursorX, "
          "cursorY);\n"
          "      updateForceDrag(scene, descriptors, ray);\n"
          "      lifecycle.paused = true;"),
      std::string::npos);
}

TEST(FilamentSceneExtraction, PanelControlsUseDockFriendlyLabels)
{
  const auto panelSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "panel.cpp");

  EXPECT_NE(panelSource.find("visiblePanelLabel"), std::string::npos);
  EXPECT_NE(
      panelSource.find("ImGui::TextUnformatted(displayLabel.c_str())"),
      std::string::npos);
  EXPECT_NE(panelSource.find("ImGui::SliderScalar("), std::string::npos);
  EXPECT_NE(panelSource.find("\"##value\""), std::string::npos);
  EXPECT_NE(panelSource.find("ImGui::PlotLines("), std::string::npos);
  EXPECT_NE(panelSource.find("\"##lines\""), std::string::npos);
  EXPECT_NE(
      panelSource.find(
          "ImGui::SetNextItemWidth(-std::numeric_limits<float>::min())"),
      std::string::npos);
}

TEST(FilamentSceneExtraction, DemoWorkspaceSupportsScriptedSwitchCaptureEvents)
{
  const auto applicationSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "application.cpp");
  const auto sceneFrameSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "scene_frame.cpp");
  const auto captureSource
      = readSourceFile(std::filesystem::path("scripts") / "capture_py_demo.py");

  EXPECT_NE(applicationSource.find("ScriptedDemoSwitch"), std::string::npos);
  EXPECT_NE(
      applicationSource.find("\"--scripted-demo-switch\""), std::string::npos);
  EXPECT_NE(
      applicationSource.find("\"--scripted-demo-event-log\""),
      std::string::npos);
  EXPECT_NE(
      applicationSource.find("requestSceneSwitch(lifecycle"),
      std::string::npos);
  EXPECT_NE(
      applicationSource.find("\"observed_target_demo\""), std::string::npos);
  EXPECT_NE(
      applicationSource.find("\"restored_previous_demo\""), std::string::npos);
  EXPECT_NE(
      applicationSource.find("\"--scripted-force-drag\""), std::string::npos);
  EXPECT_NE(
      applicationSource.find("\"--scripted-pointer-force-drag\""),
      std::string::npos);
  EXPECT_NE(sceneFrameSource.find("\"force_drag_started\""), std::string::npos);
  EXPECT_NE(captureSource.find("\"--switch-scene\""), std::string::npos);
  EXPECT_NE(
      captureSource.find("\"--scripted-demo-switch\""), std::string::npos);
  EXPECT_NE(captureSource.find("\"--force-drag-target\""), std::string::npos);
  EXPECT_NE(captureSource.find("\"--force-drag-pixel\""), std::string::npos);
  EXPECT_NE(captureSource.find("\"events.jsonl\""), std::string::npos);
  EXPECT_NE(captureSource.find("\"manifest.json\""), std::string::npos);
}

TEST(FilamentSceneExtraction, ForceDragRoutesExternalRenderablesThroughCallback)
{
  dart::gui::detail::SelectionController selection;
  dart::gui::detail::DartScene scene;
  std::vector<dart::gui::ForceDragEvent> events;
  scene.onForceDrag = [&](const dart::gui::ForceDragEvent& event) {
    events.push_back(event);
  };

  dart::gui::RenderableDescriptor descriptor;
  descriptor.id = 42;
  descriptor.shapeFrameName = "sx_box";
  descriptor.worldTransform = Eigen::Isometry3d::Identity();
  descriptor.worldTransform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  dart::gui::PickRay pressRay;
  pressRay.origin = Eigen::Vector3d(1.0, 2.0, 1.0);
  pressRay.direction = Eigen::Vector3d(0.0, 0.0, 1.0);

  dart::gui::ViewerLifecycleState lifecycle;
  lifecycle.paused = false;
  ASSERT_TRUE(selection.beginForceDrag(
      scene,
      descriptor,
      pressRay,
      descriptor.worldTransform.translation(),
      lifecycle));
  EXPECT_FALSE(lifecycle.paused);

  std::vector<dart::gui::RenderableDescriptor> descriptors{descriptor};
  dart::gui::PickRay dragRay;
  dragRay.origin = Eigen::Vector3d(2.0, 2.0, 1.0);
  dragRay.direction = Eigen::Vector3d(0.0, 0.0, 1.0);
  selection.updateForceDrag(scene, descriptors, dragRay);

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events.front().renderableId, descriptor.id);
  EXPECT_EQ(events.front().renderableName, "sx_box");
  EXPECT_TRUE(events.front().active);
  EXPECT_TRUE(events.front().applicationPoint.isApprox(
      descriptor.worldTransform.translation()));
  EXPECT_TRUE(events.front().force.isApprox(Eigen::Vector3d(60.0, 0.0, 0.0)));
  EXPECT_NE(
      selection.interactionStatus().find("external force: sx_box"),
      std::string::npos);
  EXPECT_NE(selection.interactionStatus().find("60.0 N"), std::string::npos);

  const auto debugLines = selection.forceDragDebugLines();
  ASSERT_GE(debugLines.size(), 4u);
  EXPECT_TRUE(
      std::any_of(debugLines.begin(), debugLines.end(), [](const auto& line) {
        return line.label == "force_drag.spring"
               && line.from.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0))
               && line.to.isApprox(Eigen::Vector3d(2.0, 2.0, 3.0));
      }));
  EXPECT_TRUE(
      std::any_of(debugLines.begin(), debugLines.end(), [](const auto& line) {
        return line.label == "force_drag.force"
               && line.from.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0))
               && line.to.x() > line.from.x();
      }));

  selection.endForceDrag(scene);
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events.back().renderableId, descriptor.id);
  EXPECT_EQ(events.back().renderableName, "sx_box");
  EXPECT_FALSE(events.back().active);
  EXPECT_TRUE(selection.forceDragDebugLines().empty());
}

TEST(FilamentSceneExtraction, ForceDragAppliesBodyNodeForceAtPickedShapePoint)
{
  auto world = World::create("world");
  auto skeleton = Skeleton::create("robot");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  auto* shapeNode
      = body->createShapeNodeWith<VisualAspect>(shape, "offset_box_visual");

  Eigen::Isometry3d shapeTransform = Eigen::Isometry3d::Identity();
  shapeTransform.translation() = Eigen::Vector3d(0.0, 0.5, 0.0);
  shapeNode->setRelativeTransform(shapeTransform);

  world->addSkeleton(skeleton);
  std::vector<dart::gui::RenderableDescriptor> descriptors
      = dart::gui::extractRenderables(*world);
  ASSERT_EQ(descriptors.size(), 1u);

  dart::gui::detail::SelectionController selection;
  dart::gui::detail::DartScene scene;
  scene.world = world;
  scene.onForceDrag = [](const dart::gui::ForceDragEvent&) {
    ADD_FAILURE() << "BodyNode-backed force drag should not call onForceDrag";
  };

  dart::gui::PickRay pressRay;
  pressRay.origin = Eigen::Vector3d(0.0, 0.5, -2.0);
  pressRay.direction = Eigen::Vector3d(0.0, 0.0, 1.0);
  const Eigen::Vector3d pickedPoint
      = descriptors.front().worldTransform.translation();

  dart::gui::ViewerLifecycleState lifecycle;
  lifecycle.paused = false;
  ASSERT_TRUE(selection.beginForceDrag(
      scene, descriptors.front(), pressRay, pickedPoint, lifecycle));
  EXPECT_FALSE(lifecycle.paused);

  body->clearExternalForces();

  dart::gui::PickRay dragRay;
  dragRay.origin = Eigen::Vector3d(1.0, 0.5, -2.0);
  dragRay.direction = Eigen::Vector3d(0.0, 0.0, 1.0);
  selection.updateForceDrag(scene, descriptors, dragRay);

  const Eigen::Vector6d externalForce = body->getExternalForceLocal();
  EXPECT_NEAR(externalForce[0], 0.0, 1e-10);
  EXPECT_NEAR(externalForce[1], 0.0, 1e-10);
  EXPECT_NEAR(externalForce[2], -30.0, 1e-10);
  EXPECT_NEAR(externalForce[3], 60.0, 1e-10);
  EXPECT_NEAR(externalForce[4], 0.0, 1e-10);
  EXPECT_NEAR(externalForce[5], 0.0, 1e-10);
  EXPECT_NE(
      selection.interactionStatus().find("external force: offset_box_visual"),
      std::string::npos);
  EXPECT_FALSE(selection.forceDragDebugLines().empty());
}

TEST(FilamentSceneExtraction, ScriptedForceDragTargetsRenderableAndCancels)
{
  dart::gui::detail::SelectionController selection;
  dart::gui::detail::DartScene scene;
  std::vector<dart::gui::ForceDragEvent> events;
  scene.onForceDrag = [&](const dart::gui::ForceDragEvent& event) {
    events.push_back(event);
  };

  dart::gui::RenderableDescriptor descriptor;
  descriptor.id = 91;
  descriptor.shapeFrameName = "script_shape";
  descriptor.shapeNodeName = "script_node";
  descriptor.bodyName = "script_body";
  descriptor.geometry.kind = dart::gui::ShapeKind::Box;
  descriptor.geometry.hasLocalBounds = true;
  descriptor.geometry.localBoundsMin = Eigen::Vector3d::Constant(-0.5);
  descriptor.geometry.localBoundsMax = Eigen::Vector3d::Constant(0.5);
  descriptor.geometry.shapeType = "BoxShape";
  descriptor.worldTransform = Eigen::Isometry3d::Identity();
  std::vector<dart::gui::RenderableDescriptor> descriptors{descriptor};

  dart::gui::detail::FrameViewport viewport;
  viewport.width = 640;
  viewport.height = 480;
  viewport.paneCount = 1u;
  viewport.panes[0].x = 0;
  viewport.panes[0].y = 0;
  viewport.panes[0].width = 640;
  viewport.panes[0].height = 480;
  viewport.panes[0].active = true;
  viewport.panes[0].camera.target = Eigen::Vector3d::Zero();
  viewport.panes[0].camera.yaw = 0.0;
  viewport.panes[0].camera.pitch = 0.0;
  viewport.panes[0].camera.distance = 4.0;

  dart::gui::ViewerLifecycleState lifecycle;
  Eigen::Vector3d startPoint = Eigen::Vector3d::Zero();
  ASSERT_TRUE(selection.beginScriptedForceDrag(
      viewport, scene, descriptors, "script_shape", startPoint, lifecycle));
  EXPECT_TRUE(startPoint.isApprox(Eigen::Vector3d(0.5, 0.0, 0.0), 1e-12));

  EXPECT_TRUE(selection.updateScriptedForceDragToTarget(
      viewport,
      scene,
      descriptors,
      startPoint + Eigen::Vector3d(0.0, 0.35, 0.25)));

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events.back().renderableId, descriptor.id);
  EXPECT_EQ(events.back().renderableName, "script_shape");
  EXPECT_TRUE(events.back().active);
  EXPECT_TRUE(events.back().applicationPoint.isApprox(startPoint, 1e-12));
  EXPECT_GT(events.back().force.norm(), 1.0);
  EXPECT_NE(
      selection.interactionStatus().find("external force: script_shape"),
      std::string::npos);
  EXPECT_FALSE(selection.forceDragDebugLines().empty());

  selection.cancelActiveDrag(scene);
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events.back().renderableId, descriptor.id);
  EXPECT_EQ(events.back().renderableName, "script_shape");
  EXPECT_FALSE(events.back().active);
  EXPECT_TRUE(selection.forceDragDebugLines().empty());

  ASSERT_TRUE(selection.beginScriptedForceDrag(
      viewport, scene, descriptors, "id:91", startPoint, lifecycle));
  selection.cancelActiveDrag(scene);
  const std::size_t eventsBeforePointerDrag = events.size();
  ASSERT_TRUE(selection.beginScriptedForceDragAtPointer(
      viewport,
      scene,
      descriptors,
      Eigen::Vector2d(320.0, 240.0),
      startPoint,
      lifecycle));
  EXPECT_TRUE(startPoint.isApprox(Eigen::Vector3d(0.5, 0.0, 0.0), 1e-12));
  EXPECT_TRUE(selection.updateScriptedForceDragAtPointer(
      viewport, scene, descriptors, Eigen::Vector2d(380.0, 200.0)));
  ASSERT_EQ(events.size(), eventsBeforePointerDrag + 1u);
  EXPECT_EQ(events.back().renderableId, descriptor.id);
  EXPECT_EQ(events.back().renderableName, "script_shape");
  EXPECT_TRUE(events.back().active);
  selection.cancelActiveDrag(scene);
  ASSERT_EQ(events.size(), eventsBeforePointerDrag + 2u);
  EXPECT_FALSE(events.back().active);
  EXPECT_FALSE(selection.beginScriptedForceDrag(
      viewport, scene, descriptors, "missing_target", startPoint, lifecycle));
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
  context.ui.userScale = 1.2;
  context.ui.dpiScale = 1.25;
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
  EXPECT_DOUBLE_EQ(context.ui.userScale, 1.2);
  EXPECT_DOUBLE_EQ(context.ui.dpiScale, 1.25);
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
  char implicitExecutable[] = "dartsim";
  char* implicitArgv[] = {implicitExecutable};
  const auto implicitOptions = dart::gui::detail::parseOptions(
      1, implicitArgv, std::optional<dart::gui::RunOptions>{});

  EXPECT_EQ(implicitOptions.run.width, 1280);
  EXPECT_EQ(implicitOptions.run.height, 720);
  EXPECT_FALSE(implicitOptions.windowWidthExplicit);
  EXPECT_FALSE(implicitOptions.windowHeightExplicit);

  char headlessExecutable[] = "dartsim";
  char headlessOption[] = "--headless";
  char* headlessArgv[] = {headlessExecutable, headlessOption};
  const auto headlessOptions = dart::gui::detail::parseOptions(
      2, headlessArgv, std::optional<dart::gui::RunOptions>{});

  EXPECT_EQ(headlessOptions.run.width, 1280);
  EXPECT_EQ(headlessOptions.run.height, 720);
  EXPECT_FALSE(headlessOptions.windowWidthExplicit);
  EXPECT_FALSE(headlessOptions.windowHeightExplicit);

  dart::gui::RunOptions unrelatedDefaults;
  unrelatedDefaults.maxFrames = 4;
  unrelatedDefaults.renderBackend = "noop";
  char unrelatedExecutable[] = "dartsim";
  char* unrelatedArgv[] = {unrelatedExecutable};
  const auto unrelatedOptions = dart::gui::detail::parseOptions(
      1,
      unrelatedArgv,
      std::optional<dart::gui::RunOptions>{unrelatedDefaults});

  EXPECT_EQ(unrelatedOptions.run.width, 1280);
  EXPECT_EQ(unrelatedOptions.run.height, 720);
  EXPECT_EQ(unrelatedOptions.run.maxFrames, 4);
  EXPECT_EQ(unrelatedOptions.run.renderBackend, "noop");
  EXPECT_FALSE(unrelatedOptions.windowWidthExplicit);
  EXPECT_FALSE(unrelatedOptions.windowHeightExplicit);

  dart::gui::RunOptions defaults;
  defaults.width = 1280;
  defaults.height = 960;

  char executable[] = "fetch";
  char* defaultArgv[] = {executable};
  const auto defaultOptions = dart::gui::detail::parseOptions(
      1, defaultArgv, std::optional<dart::gui::RunOptions>{defaults});

  EXPECT_EQ(defaultOptions.run.width, 1280);
  EXPECT_EQ(defaultOptions.run.height, 960);
  EXPECT_TRUE(defaultOptions.windowWidthExplicit);
  EXPECT_TRUE(defaultOptions.windowHeightExplicit);

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
  EXPECT_TRUE(overrideOptions.windowWidthExplicit);
  EXPECT_TRUE(overrideOptions.windowHeightExplicit);

  char widthOnlyExecutable[] = "fetch";
  char widthOnlyOption[] = "--width";
  char widthOnlyValue[] = "1000";
  char* widthOnlyArgv[]
      = {widthOnlyExecutable, widthOnlyOption, widthOnlyValue};
  const auto widthOnlyOptions = dart::gui::detail::parseOptions(
      3, widthOnlyArgv, std::optional<dart::gui::RunOptions>{defaults});

  EXPECT_EQ(widthOnlyOptions.run.width, 1000);
  EXPECT_EQ(widthOnlyOptions.run.height, 960);
  EXPECT_TRUE(widthOnlyOptions.windowWidthExplicit);
  EXPECT_TRUE(widthOnlyOptions.windowHeightExplicit);

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

TEST(FilamentSceneExtraction, ApplicationOptionsStoresCameraControls)
{
  dart::gui::ApplicationOptions options;
  options.cameraControlsProvider = [] {
    dart::gui::OrbitCameraControlOptions controls;
    controls.mouseMode = dart::gui::OrbitCameraMouseMode::Pan;
    controls.locked = true;
    return controls;
  };
  options.cameraUpdater = [](dart::gui::OrbitCamera& camera) {
    camera.target = Eigen::Vector3d(1.0, 2.0, 3.0);
    return true;
  };

  ASSERT_TRUE(static_cast<bool>(options.cameraControlsProvider));
  EXPECT_EQ(
      options.cameraControlsProvider().mouseMode,
      dart::gui::OrbitCameraMouseMode::Pan);
  EXPECT_TRUE(options.cameraControlsProvider().locked);

  const dart::gui::OrbitCameraControllerInput lockedInput
      = dart::gui::detail::makeOrbitCameraControllerInput(
          10.0,
          20.0,
          true,
          false,
          false,
          false,
          false,
          options.cameraControlsProvider());
  EXPECT_TRUE(lockedInput.locked);
  EXPECT_TRUE(lockedInput.pan);
  EXPECT_FALSE(lockedInput.orbit);
  EXPECT_FALSE(lockedInput.zoom);

  dart::gui::OrbitCamera camera;
  ASSERT_TRUE(options.cameraUpdater(camera));
  EXPECT_TRUE(camera.target.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(FilamentSceneExtraction, ApplicationOptionsStoresViewportLayoutProvider)
{
  dart::gui::ApplicationOptions options;
  options.viewportLayoutProvider = [](const dart::gui::OrbitCamera& camera) {
    dart::gui::ViewportLayoutOptions layout;
    layout.mode = dart::gui::ViewportLayoutMode::Quad;
    layout.paneCount = dart::gui::kMaxViewportPanes;
    layout.panes[0].kind = dart::gui::ViewportPaneKind::Perspective;
    layout.panes[0].camera = camera;
    layout.panes[0].active = true;
    layout.panes[1].kind = dart::gui::ViewportPaneKind::Top;
    return layout;
  };
  dart::gui::ViewportPaneKind activatedPane
      = dart::gui::ViewportPaneKind::Perspective;
  options.onViewportPaneActivated = [&](dart::gui::ViewportPaneKind pane) {
    activatedPane = pane;
  };

  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(4.0, 5.0, 6.0);
  camera.distance = 7.0;

  ASSERT_TRUE(static_cast<bool>(options.viewportLayoutProvider));
  const auto layout = options.viewportLayoutProvider(camera);
  EXPECT_EQ(layout.mode, dart::gui::ViewportLayoutMode::Quad);
  ASSERT_EQ(layout.paneCount, dart::gui::kMaxViewportPanes);
  EXPECT_EQ(layout.panes[0].kind, dart::gui::ViewportPaneKind::Perspective);
  EXPECT_TRUE(layout.panes[0].active);
  EXPECT_TRUE(layout.panes[0].camera.target.isApprox(camera.target));
  EXPECT_EQ(layout.panes[0].camera.distance, camera.distance);
  EXPECT_EQ(layout.panes[1].kind, dart::gui::ViewportPaneKind::Top);

  ASSERT_TRUE(static_cast<bool>(options.onViewportPaneActivated));
  options.onViewportPaneActivated(dart::gui::ViewportPaneKind::Right);
  EXPECT_EQ(activatedPane, dart::gui::ViewportPaneKind::Right);
}

TEST(FilamentSceneExtraction, FrameViewportLayoutSplitsQuadPanes)
{
  dart::gui::ViewportLayoutOptions layout;
  layout.mode = dart::gui::ViewportLayoutMode::Quad;
  layout.paneCount = dart::gui::kMaxViewportPanes;
  layout.panes[0].kind = dart::gui::ViewportPaneKind::Perspective;
  layout.panes[1].kind = dart::gui::ViewportPaneKind::Top;
  layout.panes[2].kind = dart::gui::ViewportPaneKind::Front;
  layout.panes[2].active = true;
  layout.panes[3].kind = dart::gui::ViewportPaneKind::Right;
  layout.panes[1].camera = dart::gui::OrbitCamera{};
  layout.panes[1].camera.target = Eigen::Vector3d(1.0, 2.0, 3.0);
  layout.panes[1].camera.yaw = 0.75;
  layout.panes[1].camera.pitch = 0.25;
  layout.panes[1].camera.distance = 9.0;

  const auto viewport = dart::gui::detail::makeFrameViewport(layout, 640, 480);
  EXPECT_EQ(viewport.width, 640);
  EXPECT_EQ(viewport.height, 480);
  ASSERT_EQ(viewport.paneCount, dart::gui::kMaxViewportPanes);

  EXPECT_EQ(viewport.panes[0].kind, dart::gui::ViewportPaneKind::Perspective);
  EXPECT_EQ(viewport.panes[0].x, 320);
  EXPECT_EQ(viewport.panes[0].y, 0);
  EXPECT_EQ(viewport.panes[0].width, 320);
  EXPECT_EQ(viewport.panes[0].height, 240);

  EXPECT_EQ(viewport.panes[1].kind, dart::gui::ViewportPaneKind::Top);
  EXPECT_EQ(viewport.panes[1].x, 0);
  EXPECT_EQ(viewport.panes[1].y, 0);
  EXPECT_EQ(viewport.panes[1].width, 320);
  EXPECT_EQ(viewport.panes[1].height, 240);

  EXPECT_EQ(viewport.panes[2].kind, dart::gui::ViewportPaneKind::Front);
  EXPECT_EQ(viewport.panes[2].x, 0);
  EXPECT_EQ(viewport.panes[2].y, 240);
  EXPECT_EQ(viewport.panes[2].width, 320);
  EXPECT_EQ(viewport.panes[2].height, 240);
  EXPECT_TRUE(viewport.panes[2].active);

  EXPECT_EQ(viewport.panes[3].kind, dart::gui::ViewportPaneKind::Right);
  EXPECT_EQ(viewport.panes[3].x, 320);
  EXPECT_EQ(viewport.panes[3].y, 240);
  EXPECT_EQ(viewport.panes[3].width, 320);
  EXPECT_EQ(viewport.panes[3].height, 240);

  EXPECT_EQ(
      dart::gui::detail::activeViewportPane(viewport).kind,
      dart::gui::ViewportPaneKind::Front);
  ASSERT_TRUE(
      dart::gui::detail::viewportPaneIndexAtCursor(viewport, 10.0, 10.0));
  EXPECT_EQ(
      dart::gui::detail::viewportPaneAtCursor(viewport, 10.0, 10.0)->kind,
      dart::gui::ViewportPaneKind::Top);
  EXPECT_EQ(
      dart::gui::detail::viewportPaneAtCursor(viewport, 330.0, 10.0)->kind,
      dart::gui::ViewportPaneKind::Perspective);
  EXPECT_EQ(
      dart::gui::detail::viewportPaneAtCursor(viewport, 10.0, 250.0)->kind,
      dart::gui::ViewportPaneKind::Front);
  EXPECT_EQ(
      dart::gui::detail::viewportPaneAtCursor(viewport, 330.0, 250.0)->kind,
      dart::gui::ViewportPaneKind::Right);
}

TEST(FilamentSceneExtraction, FrameViewportLayoutFallsBackWhenTooSmall)
{
  dart::gui::ViewportLayoutOptions layout;
  layout.mode = dart::gui::ViewportLayoutMode::Quad;
  layout.paneCount = dart::gui::kMaxViewportPanes;
  layout.panes[0].kind = dart::gui::ViewportPaneKind::Perspective;
  layout.panes[1].kind = dart::gui::ViewportPaneKind::Top;
  layout.panes[2].kind = dart::gui::ViewportPaneKind::Front;
  layout.panes[2].active = true;
  layout.panes[3].kind = dart::gui::ViewportPaneKind::Right;

  const auto narrow = dart::gui::detail::makeFrameViewport(layout, 1, 480);
  ASSERT_EQ(narrow.paneCount, 1u);
  EXPECT_EQ(narrow.width, 1);
  EXPECT_EQ(narrow.height, 480);
  EXPECT_EQ(narrow.panes[0].kind, dart::gui::ViewportPaneKind::Front);
  EXPECT_EQ(narrow.panes[0].x, 0);
  EXPECT_EQ(narrow.panes[0].y, 0);
  EXPECT_EQ(narrow.panes[0].width, 1);
  EXPECT_EQ(narrow.panes[0].height, 480);
  EXPECT_TRUE(narrow.panes[0].active);

  const auto shortViewport
      = dart::gui::detail::makeFrameViewport(layout, 640, 1);
  ASSERT_EQ(shortViewport.paneCount, 1u);
  EXPECT_EQ(shortViewport.width, 640);
  EXPECT_EQ(shortViewport.height, 1);
  EXPECT_EQ(shortViewport.panes[0].kind, dart::gui::ViewportPaneKind::Front);
}

TEST(FilamentSceneExtraction, FrameViewportActivationFindsInactiveCursorPane)
{
  dart::gui::ViewportLayoutOptions layout;
  layout.mode = dart::gui::ViewportLayoutMode::Quad;
  layout.paneCount = dart::gui::kMaxViewportPanes;
  layout.panes[0].kind = dart::gui::ViewportPaneKind::Perspective;
  layout.panes[1].kind = dart::gui::ViewportPaneKind::Top;
  layout.panes[2].kind = dart::gui::ViewportPaneKind::Front;
  layout.panes[2].active = true;
  layout.panes[3].kind = dart::gui::ViewportPaneKind::Right;
  layout.panes[1].camera.target = Eigen::Vector3d(1.0, 2.0, 3.0);
  layout.panes[1].camera.yaw = 0.75;
  layout.panes[1].camera.pitch = 0.25;
  layout.panes[1].camera.distance = 9.0;

  const auto viewport = dart::gui::detail::makeFrameViewport(layout, 640, 480);
  const auto topActivation
      = dart::gui::detail::viewportPaneActivationIndexAtCursor(
          viewport, 10.0, 10.0);
  ASSERT_TRUE(topActivation.has_value());
  EXPECT_EQ(*topActivation, 1u);
  ASSERT_NE(
      dart::gui::detail::viewportPaneActivationAtCursor(viewport, 10.0, 10.0),
      nullptr);
  EXPECT_EQ(
      dart::gui::detail::viewportPaneActivationAtCursor(viewport, 10.0, 10.0)
          ->kind,
      dart::gui::ViewportPaneKind::Top);

  EXPECT_FALSE(
      dart::gui::detail::viewportPaneActivationIndexAtCursor(
          viewport, 10.0, 250.0)
          .has_value());
  EXPECT_FALSE(
      dart::gui::detail::viewportPaneActivationIndexAtCursor(
          viewport, 700.0, 10.0)
          .has_value());

  dart::gui::OrbitCameraController controller;
  controller.camera.yaw = -0.5;
  dart::gui::ViewportPaneKind activatedPane
      = dart::gui::ViewportPaneKind::Perspective;
  bool callbackCalled = false;
  dart::gui::detail::ViewportPaneActivationState activationState;
  EXPECT_TRUE(
      dart::gui::detail::applyViewportPaneActivationAtCursor(
          activationState,
          viewport,
          10.0,
          10.0,
          true,
          false,
          controller,
          [&](dart::gui::ViewportPaneKind pane) {
            callbackCalled = true;
            activatedPane = pane;
          }));
  EXPECT_TRUE(callbackCalled);
  EXPECT_EQ(activatedPane, dart::gui::ViewportPaneKind::Top);
  EXPECT_TRUE(
      controller.camera.target.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_EQ(controller.camera.yaw, 0.75);
  EXPECT_EQ(controller.camera.pitch, 0.25);
  EXPECT_EQ(controller.camera.distance, 9.0);
  callbackCalled = false;
  EXPECT_FALSE(
      dart::gui::detail::applyViewportPaneActivationAtCursor(
          activationState,
          viewport,
          10.0,
          10.0,
          true,
          false,
          controller,
          [&](dart::gui::ViewportPaneKind) { callbackCalled = true; }));
  EXPECT_FALSE(callbackCalled);

  dart::gui::OrbitCameraController noCallbackController;
  noCallbackController.camera.yaw = -0.5;
  dart::gui::detail::ViewportPaneActivationState noCallbackState;
  EXPECT_FALSE(
      dart::gui::detail::applyViewportPaneActivationAtCursor(
          noCallbackState,
          viewport,
          10.0,
          10.0,
          true,
          false,
          noCallbackController,
          {}));
  EXPECT_EQ(noCallbackController.camera.yaw, -0.5);

  dart::gui::detail::ViewportPaneActivationState capturedState;
  EXPECT_FALSE(
      dart::gui::detail::applyViewportPaneActivationAtCursor(
          capturedState,
          viewport,
          10.0,
          10.0,
          true,
          true,
          controller,
          [&](dart::gui::ViewportPaneKind) { callbackCalled = true; }));
  EXPECT_FALSE(callbackCalled);

  layout.mode = dart::gui::ViewportLayoutMode::Single;
  const auto single = dart::gui::detail::makeFrameViewport(layout, 640, 480);
  EXPECT_FALSE(
      dart::gui::detail::viewportPaneActivationIndexAtCursor(single, 10.0, 10.0)
          .has_value());
}

TEST(FilamentSceneExtraction, ViewportPaneDisplayNamesAreStable)
{
  EXPECT_EQ(
      dart::gui::detail::viewportPaneDisplayName(
          dart::gui::ViewportPaneKind::Perspective),
      "Perspective");
  EXPECT_EQ(
      dart::gui::detail::viewportPaneDisplayName(
          dart::gui::ViewportPaneKind::Top),
      "Top");
  EXPECT_EQ(
      dart::gui::detail::viewportPaneDisplayName(
          dart::gui::ViewportPaneKind::Front),
      "Front");
  EXPECT_EQ(
      dart::gui::detail::viewportPaneDisplayName(
          dart::gui::ViewportPaneKind::Right),
      "Right");
}

TEST(FilamentSceneExtraction, ViewportPaneLabelStateUsesActivePaneFallback)
{
  dart::gui::ViewportLayoutOptions layout;
  layout.mode = dart::gui::ViewportLayoutMode::Quad;
  layout.paneCount = dart::gui::kMaxViewportPanes;
  layout.panes[0].kind = dart::gui::ViewportPaneKind::Perspective;
  layout.panes[1].kind = dart::gui::ViewportPaneKind::Top;
  layout.panes[2].kind = dart::gui::ViewportPaneKind::Front;
  layout.panes[3].kind = dart::gui::ViewportPaneKind::Right;

  const auto noExplicitActive
      = dart::gui::detail::makeFrameViewport(layout, 640, 480);
  EXPECT_EQ(dart::gui::detail::activeViewportPaneIndex(noExplicitActive), 0u);
  EXPECT_TRUE(
      dart::gui::detail::viewportPaneLabelState(noExplicitActive, 0u).active);
  EXPECT_FALSE(
      dart::gui::detail::viewportPaneLabelState(noExplicitActive, 1u).active);

  layout.panes[1].active = true;
  layout.panes[2].active = true;
  const auto multipleActive
      = dart::gui::detail::makeFrameViewport(layout, 640, 480);
  EXPECT_EQ(dart::gui::detail::activeViewportPaneIndex(multipleActive), 1u);
  EXPECT_EQ(
      dart::gui::detail::viewportPaneLabelState(multipleActive, 1u).text,
      "Top");
  EXPECT_TRUE(
      dart::gui::detail::viewportPaneLabelState(multipleActive, 1u).active);
  EXPECT_FALSE(
      dart::gui::detail::viewportPaneLabelState(multipleActive, 2u).active);
}

TEST(FilamentSceneExtraction, DartsimEditorViewportLayoutUsesRendererSeam)
{
  const auto applicationHeader
      = readSourceFile(kDartGuiDirectory / "application.hpp");
  EXPECT_NE(
      applicationHeader.find("viewportLayoutProvider"), std::string::npos);
  EXPECT_NE(
      applicationHeader.find("struct ViewportLayoutOptions"),
      std::string::npos);
  EXPECT_NE(
      applicationHeader.find("onViewportPaneActivated"), std::string::npos);

  const auto applicationSource
      = readSourceFile(kDartGuiDirectory / "detail" / "application.cpp");
  EXPECT_NE(
      applicationSource.find("appOptions.viewportLayoutProvider"),
      std::string::npos);
  EXPECT_NE(
      applicationSource.find("appOptions.onViewportPaneActivated"),
      std::string::npos);
  EXPECT_NE(applicationSource.find("renderContext.views"), std::string::npos);
  EXPECT_NE(applicationSource.find("renderContext.cameras"), std::string::npos);
  EXPECT_NE(
      applicationSource.find(
          "renderContext.activeViewCount = viewport.paneCount"),
      std::string::npos);

  const auto renderContextSource
      = readSourceFile(kDartGuiDirectory / "detail" / "render_context.cpp");
  EXPECT_NE(
      renderContextSource.find("for (std::size_t i = 0; i < activeViewCount"),
      std::string::npos);
  EXPECT_NE(
      renderContextSource.find("context.renderer->render(context.views[i])"),
      std::string::npos);

  const auto frameViewportSource
      = readSourceFile(kDartGuiDirectory / "detail" / "frame_viewport.cpp");
  EXPECT_NE(frameViewportSource.find("makeFrameViewport"), std::string::npos);
  EXPECT_NE(
      frameViewportSource.find("viewportPaneIndexAtCursor"), std::string::npos);
  EXPECT_NE(
      frameViewportSource.find("activeViewportPaneIndex"), std::string::npos);
  EXPECT_NE(
      frameViewportSource.find("viewport.height - pane.y - pane.height"),
      std::string::npos);
  EXPECT_NE(
      frameViewportSource.find("viewportPaneDisplayName"), std::string::npos);
  EXPECT_NE(
      frameViewportSource.find("applyViewportPaneActivationAtCursor"),
      std::string::npos);

  const auto uiFrameSource
      = readSourceFile(kDartGuiDirectory / "detail" / "ui_frame.cpp");
  EXPECT_NE(uiFrameSource.find("renderViewportPaneLabels"), std::string::npos);
  EXPECT_NE(
      uiFrameSource.find(
          "renderViewportPaneLabels(viewport, guiScale.effectiveScale)"),
      std::string::npos);
  EXPECT_NE(
      uiFrameSource.find("viewportPaneLabelState(viewport, i)"),
      std::string::npos);
  EXPECT_NE(
      uiFrameSource.find("updateViewportPaneActivation"), std::string::npos);

  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");
  EXPECT_NE(
      editorSource.find("buildViewportLayoutMenu(ui, app, context)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("options.viewportLayoutProvider"), std::string::npos);
  EXPECT_NE(
      editorSource.find("options.onViewportPaneActivated"), std::string::npos);
  EXPECT_NE(
      editorSource.find("applyViewportPaneActivation"), std::string::npos);
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

TEST(FilamentSceneExtraction, DartsimDefaultsToEmptySceneUnlessDemoRequested)
{
  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");

  // A --demo flag opt-in guards the demo scene; default launch starts empty.
  EXPECT_NE(
      editorSource.find("std::string_view(argv[i]) == \"--demo\""),
      std::string::npos);

  const auto appConstruction
      = editorSource.find("auto app = std::make_shared<EditorApp>();");
  ASSERT_NE(appConstruction, std::string::npos);

  const auto demoGuard
      = editorSource.find("if (hasDemoOption(argc, argv))", appConstruction);
  const auto demoSeeding = editorSource.find("seedDemoScene(*app);", demoGuard);
  const auto emptyStart
      = editorSource.find("startEmptyScene(*app);", demoGuard);
  ASSERT_NE(demoGuard, std::string::npos);
  ASSERT_NE(demoSeeding, std::string::npos);
  ASSERT_NE(emptyStart, std::string::npos);

  // Demo seeding is the guarded opt-in branch, not an unconditional call.
  EXPECT_LT(appConstruction, demoGuard);
  EXPECT_LT(demoGuard, demoSeeding);
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
  EXPECT_NE(editorSource.find("selectViewportRenderable("), std::string::npos);
  EXPECT_NE(
      editorSource.find("app->engine, app->viewportLayers, renderableId"),
      std::string::npos);
  EXPECT_EQ(
      editorSource.find("app->engine.select(kNoObject)"), std::string::npos);
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
          "app->transformGizmo, app->engine, app->viewportLayers"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "options.keyboardActions = makeViewportMoveActions(app)"),
      std::string::npos);
}

TEST(FilamentSceneExtraction, DartsimViewportCameraUsesActionSeam)
{
  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");
  const auto panelHeader = readSourceFile(kDartGuiDirectory / "panel.hpp");
  const auto uiFrameSource
      = readSourceFile(kDartGuiDirectory / "detail" / "ui_frame.cpp");

  EXPECT_NE(editorSource.find("ui.beginMenu(\"View\")"), std::string::npos);
  EXPECT_NE(
      editorSource.find("ViewportLayerFilterState viewportLayers"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("filteredViewportRenderItems("), std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "selectedViewportRenderable(app->engine, app->viewportLayers)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "selectedViewportLabel(app->engine, app->viewportLayers)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("buildViewportLayerFilterActions(app.viewportLayers)"),
      std::string::npos);
  EXPECT_NE(editorSource.find("buildViewportStatus("), std::string::npos);
  EXPECT_NE(editorSource.find("buildViewportPanel("), std::string::npos);
  EXPECT_NE(
      editorSource.find("ui.text(status.layoutLabel)"), std::string::npos);
  EXPECT_NE(
      editorSource.find("ui.text(status.activePaneLabel)"), std::string::npos);
  EXPECT_NE(
      editorSource.find("ui.text(status.cameraModeLabel)"), std::string::npos);
  EXPECT_NE(
      editorSource.find("ui.text(status.cameraLockLabel)"), std::string::npos);
  EXPECT_NE(
      editorSource.find("ui.text(status.trackingLabel)"), std::string::npos);
  EXPECT_NE(
      editorSource.find("ui.text(status.visibleLayerLabel)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("ui.text(status.selectionLabel)"), std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "if (!action.enabled) {\n      "
          "ui.text(cameraControlMenuLabel(action));"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("if (ui.button(cameraControlMenuLabel(action)))"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "options.panels.push_back(makePanel(\n      \"Viewport\""),
      std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "[app](dart::gui::PanelBuilder& ui) { buildViewportPanel(ui, *app); "
          "}"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "setViewportLayerVisible(app.viewportLayers, action.kind, checked)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "buildViewportCameraActions(app.engine, app.viewportLayers)"),
      std::string::npos);
  EXPECT_NE(editorSource.find("applyViewportCameraAction("), std::string::npos);
  EXPECT_NE(editorSource.find("app.viewportLayers"), std::string::npos);
  EXPECT_NE(
      editorSource.find("context.camera.setOrbitCamera"), std::string::npos);
  EXPECT_NE(
      panelHeader.find(
          "std::function<void(const OrbitCamera&)> setOrbitCamera"),
      std::string::npos);
  EXPECT_NE(
      uiFrameSource.find("cameraController.camera = camera"),
      std::string::npos);
  EXPECT_NE(
      uiFrameSource.find("resetOrbitCameraTracking(cameraController)"),
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

TEST(FilamentSceneExtraction, DartsimWatchPanelUsesActionSeam)
{
  const auto editorSource
      = readSourceFile(kDartsimUiDirectory / "src" / "editor.cpp");

  EXPECT_NE(
      editorSource.find("#include <dartsim_ui/watch_actions.hpp>"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("std::string watchPresetName"), std::string::npos);
  EXPECT_NE(
      editorSource.find("WatchStatus status = buildWatchStatus"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("saveWatchPreset(app.watch, app.engine"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("applyWatchPreset(app.watch, app.engine"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("deleteWatchPreset(app.engine"), std::string::npos);
  EXPECT_NE(editorSource.find("status.presetOptions"), std::string::npos);
  EXPECT_NE(editorSource.find("canEditWatchPresets"), std::string::npos);
  EXPECT_NE(editorSource.find("missingTargetCount"), std::string::npos);
  EXPECT_NE(editorSource.find("ignoredSignalCount"), std::string::npos);
  EXPECT_EQ(
      editorSource.find("app.engine.objects().model().workspace"),
      std::string::npos);
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

TEST(FilamentSceneExtraction, DartsimProjectMenuUsesBrowserAndNativeDialogSeam)
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
  EXPECT_NE(editorSource.find("openProjectFromMenu(app)"), std::string::npos);
  EXPECT_NE(
      editorSource.find("requestOpenProjectReplacementWithDialog("),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("requestProjectPathModalAfterDialogFailure("),
      std::string::npos);
  EXPECT_NE(
      editorSource.find(
          "makeProjectFileDialogRequest(app, app.projectPathKind)"),
      std::string::npos);
  EXPECT_NE(editorSource.find("requestProjectPathModal("), std::string::npos);
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
  EXPECT_NE(
      editorSource.find("ui.menuItem(\"Close Project\")"), std::string::npos);
  EXPECT_NE(
      editorSource.find("requestCloseProjectReplacement(app.engine)"),
      std::string::npos);
  EXPECT_NE(
      editorSource.find("saveProjectFromMenu(app, true)"), std::string::npos);
  EXPECT_NE(editorSource.find("saveProjectWithDialog("), std::string::npos);
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
          "showNativeProjectFileDialog(request, filters, parent, &rawPath)"),
      std::string::npos);
  EXPECT_NE(projectDialogSource.find("NFD_ClearError()"), std::string::npos);
  EXPECT_NE(
      projectDialogSource.find(
          "showNativeProjectFileDialog(request, filters, {}, &rawPath)"),
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

  const auto inspectorStart = editorSource.find(
      "void buildInspector(dart::gui::PanelBuilder& ui, EditorApp& app)");
  ASSERT_NE(inspectorStart, std::string::npos);
  const auto canEditCheck = editorSource.find(
      "const InspectorStatus status = buildInspectorStatus(app.engine);");
  ASSERT_NE(canEditCheck, std::string::npos);
  const auto lockGuard = editorSource.find("if (status.locked)", canEditCheck);
  ASSERT_NE(lockGuard, std::string::npos);
  const auto lockMessage
      = editorSource.find("Inspector locked during Simulation Mode", lockGuard);
  ASSERT_NE(lockMessage, std::string::npos);
  const auto sectionBuild = editorSource.find(
      "buildInspectorSection(ui, app, status, section);", inspectorStart);
  const auto deleteButton
      = editorSource.find("ui.button(\"Delete##inspector\")", inspectorStart);
  ASSERT_NE(sectionBuild, std::string::npos);
  ASSERT_NE(deleteButton, std::string::npos);

  EXPECT_LT(lockGuard, sectionBuild);
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
  EXPECT_NE(editorSource.find("inspectorSections(status)"), std::string::npos);
  EXPECT_NE(
      editorSource.find("ui.collapsingHeader(section"), std::string::npos);
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

TEST(
    FilamentSceneExtraction,
    MakeDeformableSurfaceRenderable_GridSurface_ReturnsDynamicMeshDescriptor)
{
  const auto triangles = dart::gui::makeGridSurfaceTriangles(2u, 2u);
  ASSERT_EQ(triangles.size(), 2u);
  EXPECT_TRUE(triangles[0].isApprox(Eigen::Vector3i(0, 1, 2)));
  EXPECT_TRUE(triangles[1].isApprox(Eigen::Vector3i(1, 3, 2)));

  const std::vector<Eigen::Vector3d> positions{
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 0.0)};
  dart::gui::DeformableSurfaceRenderOptions options;
  options.version = 7u;
  options.surfaceColor = Eigen::Vector4d(0.4, 0.7, 0.9, 0.85);
  options.boundsPadding = 0.05;

  const auto descriptor = dart::gui::makeDeformableSurfaceRenderable(
      42u, positions, triangles, options);
  ASSERT_TRUE(descriptor.has_value());
  EXPECT_EQ(descriptor->id, 42u);
  EXPECT_EQ(descriptor->geometry.kind, ShapeKind::Mesh);
  EXPECT_TRUE(descriptor->material.rgba.isApprox(options.surfaceColor));
  EXPECT_TRUE(descriptor->material.visible);
  EXPECT_TRUE(descriptor->material.castsShadows);
  EXPECT_TRUE(descriptor->material.receivesShadows);
  EXPECT_EQ(descriptor->shapeVersion, options.version);
  EXPECT_NE(descriptor->renderResourceVersion, 0u);
  EXPECT_EQ(descriptor->geometry.triangleVertices.size(), positions.size());
  EXPECT_EQ(descriptor->geometry.triangleIndices.size(), triangles.size());
  EXPECT_EQ(descriptor->geometry.triangleNormals.size(), positions.size());
  EXPECT_TRUE(descriptor->geometry.meshUsesMaterialColors);
  EXPECT_EQ(descriptor->geometry.meshAlphaMode, MeshAlphaMode::ShapeAlpha);
  ASSERT_EQ(descriptor->geometry.meshMaterials.size(), 1u);
  EXPECT_TRUE(descriptor->geometry.meshMaterials.front().diffuse.isApprox(
      options.surfaceColor));
  ASSERT_EQ(descriptor->geometry.meshParts.size(), 1u);
  EXPECT_EQ(
      descriptor->geometry.meshParts.front().vertexCount, positions.size());
  EXPECT_EQ(
      descriptor->geometry.meshParts.front().triangleCount, triangles.size());
  EXPECT_TRUE(descriptor->geometry.hasLocalBounds);
  EXPECT_TRUE(descriptor->geometry.localBoundsMin.isApprox(
      Eigen::Vector3d(-0.05, -0.05, -0.05)));
  EXPECT_TRUE(descriptor->geometry.localBoundsMax.isApprox(
      Eigen::Vector3d(1.05, 1.05, 0.05)));
  for (const auto& normal : descriptor->geometry.triangleNormals) {
    EXPECT_TRUE(normal.isApprox(Eigen::Vector3d::UnitZ()));
  }

  auto movedPositions = positions;
  movedPositions[3].z() = 0.2;
  const auto movedDescriptor = dart::gui::makeDeformableSurfaceRenderable(
      42u, movedPositions, triangles, options);
  ASSERT_TRUE(movedDescriptor.has_value());
  EXPECT_EQ(movedDescriptor->id, descriptor->id);
  EXPECT_NE(
      movedDescriptor->renderResourceVersion,
      descriptor->renderResourceVersion);
}

TEST(
    FilamentSceneExtraction,
    RenderableExtractor_LineSegmentVertexMutation_RebuildsDynamicGeometry)
{
  auto world = World::create("world");
  auto line = std::make_shared<LineSegmentShape>(2.0f);
  const std::size_t first = line->addVertex(Eigen::Vector3d(0.0, 0.0, 0.0));
  const std::size_t second = line->addVertex(Eigen::Vector3d(1.0, 0.0, 0.0));
  line->addConnection(first, second);

  auto frame = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "dynamic_line");
  frame->setShape(line);
  frame->getVisualAspect(true);
  world->addSimpleFrame(frame);

  dart::gui::RenderableExtractor extractor;
  const auto before = extractor.extract(*world);
  ASSERT_EQ(before.size(), 1u);
  ASSERT_EQ(before.front().geometry.lineVertices.size(), 2u);

  line->setVertex(second, Eigen::Vector3d(1.0, 0.5, 0.0));

  const auto after = extractor.extract(*world);
  ASSERT_EQ(after.size(), 1u);
  ASSERT_EQ(after.front().geometry.lineVertices.size(), 2u);
  EXPECT_EQ(after.front().id, before.front().id);
  EXPECT_NE(
      after.front().renderResourceVersion,
      before.front().renderResourceVersion);
  EXPECT_TRUE(after.front().geometry.lineVertices[1].isApprox(
      Eigen::Vector3d(1.0, 0.5, 0.0)));
}

TEST(
    FilamentSceneExtraction,
    MakeDeformableSurfaceRenderable_InvalidTopology_ReturnsNullopt)
{
  const std::vector<Eigen::Vector3d> positions{
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0)};
  const std::vector<Eigen::Vector3i> validTriangle{Eigen::Vector3i(0, 1, 2)};
  const std::vector<Eigen::Vector3i> outOfRangeTriangle{
      Eigen::Vector3i(0, 1, 3)};
  const std::vector<Eigen::Vector3i> negativeTriangle{
      Eigen::Vector3i(0, -1, 2)};

  EXPECT_FALSE(
      dart::gui::makeDeformableSurfaceRenderable(0u, positions, validTriangle)
          .has_value());
  EXPECT_FALSE(
      dart::gui::makeDeformableSurfaceRenderable(
          5u, positions, outOfRangeTriangle)
          .has_value());
  EXPECT_FALSE(
      dart::gui::makeDeformableSurfaceRenderable(
          5u, positions, negativeTriangle)
          .has_value());
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
  const auto applicationSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "application.cpp");
  const auto scenesSource = readSourceFile(
      std::filesystem::path("dart") / "gui" / "detail" / "scenes.cpp");

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
  state.paused = true;
  state.stepOnce = true;
  dart::gui::requestSceneSwitch(state, "other_demo");
  EXPECT_TRUE(state.sceneSwitchRequested);
  EXPECT_EQ(state.requestedScene, "other_demo");
  EXPECT_EQ(state.sceneActivationPendingScene, "other_demo");
  EXPECT_NE(
      state.sceneActivationStatus.find("Starting demo 'other_demo'"),
      std::string::npos);
  dart::gui::requestSceneSwitch(state, "third_demo");
  EXPECT_TRUE(state.sceneSwitchRequested);
  EXPECT_EQ(state.requestedScene, "third_demo");
  EXPECT_EQ(state.sceneActivationPendingScene, "third_demo");
  EXPECT_NE(
      state.sceneActivationStatus.find("Starting demo 'third_demo'"),
      std::string::npos);
  dart::gui::requestSceneReplay(state, "demo_scene");
  EXPECT_TRUE(state.sceneSwitchRequested);
  EXPECT_EQ(state.requestedScene, "demo_scene");
  EXPECT_EQ(state.sceneActivationPendingScene, "demo_scene");
  EXPECT_NE(
      state.sceneActivationStatus.find("Starting demo 'demo_scene'"),
      std::string::npos);
  EXPECT_FALSE(state.paused);
  EXPECT_FALSE(state.stepOnce);
  EXPECT_FALSE(dart::gui::consumeDockLayoutResetRequest(state));
  dart::gui::requestDockLayoutReset(state);
  EXPECT_TRUE(state.dockLayoutResetRequested);
  EXPECT_TRUE(dart::gui::consumeDockLayoutResetRequest(state));
  EXPECT_FALSE(state.dockLayoutResetRequested);

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
  dart::gui::ViewerLifecycleState sequenceState;
  sequenceState.frameOutputDirectory = sequenceOutput.frameOutputDirectory;
  sequenceState.frameOutputEnabled
      = !sequenceOutput.frameOutputDirectory.empty();
  EXPECT_TRUE(
      dart::gui::shouldCaptureFrameOutput(sequenceOutput, sequenceState));
  dart::gui::toggleFrameOutputCapture(sequenceState, "frames");
  EXPECT_FALSE(sequenceState.frameOutputEnabled);
  EXPECT_FALSE(
      dart::gui::shouldCaptureFrameOutput(sequenceOutput, sequenceState));
  EXPECT_EQ(sequenceState.recordedFramePlaybackIndex, 0);
  EXPECT_FALSE(sequenceState.recordedFramePlaybackPlaying);
  dart::gui::toggleRecordedFramePlayback(sequenceState, 0);
  EXPECT_FALSE(sequenceState.recordedFramePlaybackPlaying);
  dart::gui::toggleRecordedFramePlayback(sequenceState, 3);
  EXPECT_TRUE(sequenceState.recordedFramePlaybackPlaying);
  EXPECT_EQ(sequenceState.recordedFramePlaybackIndex, 0);
  dart::gui::advanceRecordedFramePlayback(sequenceState, 3);
  EXPECT_TRUE(sequenceState.recordedFramePlaybackPlaying);
  EXPECT_EQ(sequenceState.recordedFramePlaybackIndex, 1);
  dart::gui::stepRecordedFramePlayback(sequenceState, 3, 10);
  EXPECT_EQ(sequenceState.recordedFramePlaybackIndex, 2);
  dart::gui::advanceRecordedFramePlayback(sequenceState, 3);
  EXPECT_FALSE(sequenceState.recordedFramePlaybackPlaying);
  EXPECT_EQ(sequenceState.recordedFramePlaybackIndex, 2);
  dart::gui::setRecordedFramePlaybackIndex(sequenceState, 3, -4);
  EXPECT_EQ(sequenceState.recordedFramePlaybackIndex, 0);
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
  EXPECT_NE(
      nativeWindowSource.find("glfwGetWindowContentScale"), std::string::npos);
  EXPECT_NE(nativeWindowSource.find("glfwSetWindowSize"), std::string::npos);
  EXPECT_NE(
      nativeWindowSource.find("guiScale.effectiveScale"), std::string::npos);
  EXPECT_NE(
      nativeWindowSource.find("(std::max)(xScale, yScale)"), std::string::npos);
  EXPECT_NE(
      applicationSource.find("!automaticWindowSizeResolved"),
      std::string::npos);
  EXPECT_EQ(
      scenesSource.find("options.run.width) * options.run.guiScale"),
      std::string::npos);
}

TEST(FilamentSceneExtraction, GuiScale_SeparatesUserScaleFromDpiScale)
{
  EXPECT_NEAR(
      dart::gui::detail::normalizeGuiUserScale(
          std::numeric_limits<double>::quiet_NaN()),
      1.0,
      1e-12);
  EXPECT_NEAR(dart::gui::detail::normalizeGuiUserScale(0.25), 0.5, 1e-12);
  EXPECT_NEAR(dart::gui::detail::normalizeGuiUserScale(10.0), 4.0, 1e-12);

  EXPECT_EQ(dart::gui::detail::parseGuiDpiScaleOverride(""), std::nullopt);
  EXPECT_EQ(dart::gui::detail::parseGuiDpiScaleOverride("2x"), std::nullopt);

  const auto overrideScale = dart::gui::detail::parseGuiDpiScaleOverride("2.5");
  ASSERT_TRUE(overrideScale.has_value());
  EXPECT_NEAR(*overrideScale, 2.5, 1e-12);
  EXPECT_NEAR(*dart::gui::detail::parseGuiDpiScaleOverride("12.0"), 4.0, 1e-12);

  const dart::gui::detail::GuiScaleState scale
      = dart::gui::detail::makeGuiScaleState(1.5, 2.0);
  EXPECT_NEAR(scale.userScale, 1.5, 1e-12);
  EXPECT_NEAR(scale.dpiScale, 2.0, 1e-12);
  EXPECT_NEAR(scale.effectiveScale, 3.0, 1e-12);

  const dart::gui::detail::GuiWindowSize explicitSize
      = dart::gui::detail::resolveAutomaticGuiWindowSize(
          1280, 720, 2.0, 0, 0, false, false);
  EXPECT_EQ(explicitSize.width, 1280);
  EXPECT_EQ(explicitSize.height, 720);

  const dart::gui::detail::GuiWindowSize automaticDefaultSize
      = dart::gui::detail::resolveAutomaticGuiWindowSize(
          1280, 720, 1.0, 0, 0, true, true);
  EXPECT_EQ(automaticDefaultSize.width, 1600);
  EXPECT_EQ(automaticDefaultSize.height, 900);

  const dart::gui::detail::GuiWindowSize scaledSize
      = dart::gui::detail::resolveAutomaticGuiWindowSize(
          1280, 720, 2.0, 0, 0, true, true);
  EXPECT_EQ(scaledSize.width, 3200);
  EXPECT_EQ(scaledSize.height, 1800);

  const dart::gui::detail::GuiWindowSize oneExplicitDimension
      = dart::gui::detail::resolveAutomaticGuiWindowSize(
          1000, 720, 2.0, 0, 0, false, true);
  EXPECT_EQ(oneExplicitDimension.width, 1000);
  EXPECT_EQ(oneExplicitDimension.height, 1800);

  const dart::gui::detail::GuiWindowSize largerBaseSize
      = dart::gui::detail::resolveAutomaticGuiWindowSize(
          1920, 1080, 1.0, 0, 0, true, true);
  EXPECT_EQ(largerBaseSize.width, 1920);
  EXPECT_EQ(largerBaseSize.height, 1080);

  const dart::gui::detail::GuiWindowSize clampedSize
      = dart::gui::detail::resolveAutomaticGuiWindowSize(
          1280, 720, 2.0, 2000, 1200, true, true);
  EXPECT_EQ(clampedSize.width, 1700);
  EXPECT_EQ(clampedSize.height, 1020);

  const dart::gui::detail::GuiWindowSize clampedSmallMonitorSize
      = dart::gui::detail::resolveAutomaticGuiWindowSize(
          1280, 720, 1.0, 1200, 800, true, true);
  EXPECT_EQ(clampedSmallMonitorSize.width, 1020);
  EXPECT_EQ(clampedSmallMonitorSize.height, 680);

  const dart::gui::detail::GuiWindowSize largeMonitorSize
      = dart::gui::detail::resolveAutomaticGuiWindowSize(
          1280, 720, 1.0, 6144, 3840, true, true);
  EXPECT_EQ(largeMonitorSize.width, 4301);
  EXPECT_EQ(largeMonitorSize.height, 2688);

  const dart::gui::detail::GuiWindowSize largeMonitorHiDpiSize
      = dart::gui::detail::resolveAutomaticGuiWindowSize(
          1280, 720, 2.0, 6144, 3840, true, true);
  EXPECT_EQ(largeMonitorHiDpiSize.width, 5222);
  EXPECT_EQ(largeMonitorHiDpiSize.height, 3264);
}

TEST(FilamentSceneExtraction, GuiScale_AutomaticWindowSizingCoversScaleRange)
{
  using dart::gui::detail::resolveAutomaticGuiWindowSize;

  const std::array<double, 15> scales = {
      0.25,
      0.5,
      0.75,
      1.0,
      1.1,
      1.25,
      1.333333333333,
      1.5,
      1.75,
      2.0,
      2.5,
      3.0,
      4.0,
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN(),
  };
  const std::array<std::array<int, 2>, 7> monitorSizes = {{
      {0, 0},
      {1024, 768},
      {1600, 900},
      {1920, 1080},
      {2560, 1440},
      {3840, 2160},
      {6144, 3840},
  }};

  for (const auto& monitorSize : monitorSizes) {
    const int maxWidth = monitorSize[0];
    const int maxHeight = monitorSize[1];
    int previousWidth = 0;
    int previousHeight = 0;
    for (const double scale : scales) {
      const auto size = resolveAutomaticGuiWindowSize(
          1280, 720, scale, maxWidth, maxHeight, true, true);
      EXPECT_GT(size.width, 0);
      EXPECT_GT(size.height, 0);

      if (maxWidth > 0) {
        EXPECT_LE(
            size.width,
            static_cast<int>(std::lround(static_cast<double>(maxWidth) * 0.85)))
            << "scale=" << scale << " maxWidth=" << maxWidth;
      }
      if (maxHeight > 0) {
        EXPECT_LE(
            size.height,
            static_cast<int>(
                std::lround(static_cast<double>(maxHeight) * 0.85)))
            << "scale=" << scale << " maxHeight=" << maxHeight;
      }

      if (std::isfinite(scale) && scale >= 1.0) {
        EXPECT_GE(size.width, previousWidth)
            << "scale=" << scale << " maxWidth=" << maxWidth;
        EXPECT_GE(size.height, previousHeight)
            << "scale=" << scale << " maxHeight=" << maxHeight;
        previousWidth = size.width;
        previousHeight = size.height;
      }
    }
  }

  const auto scaleOneSize
      = resolveAutomaticGuiWindowSize(1280, 720, 1.0, 3840, 2160, true, true);
  for (const double scale : {0.0, 0.25, 0.5, 0.75}) {
    const auto subUnitySize = resolveAutomaticGuiWindowSize(
        1280, 720, scale, 3840, 2160, true, true);
    EXPECT_EQ(subUnitySize.width, scaleOneSize.width);
    EXPECT_EQ(subUnitySize.height, scaleOneSize.height);
  }

  const auto explicitSize
      = resolveAutomaticGuiWindowSize(1280, 720, 8.0, 3840, 2160, false, false);
  EXPECT_EQ(explicitSize.width, 1280);
  EXPECT_EQ(explicitSize.height, 720);

  for (const double scale : {1.0, 1.25, 1.5, 2.0, 3.0, 4.0}) {
    const auto oneAutomaticDimension = resolveAutomaticGuiWindowSize(
        1000, 720, scale, 3840, 2160, false, true);
    EXPECT_EQ(oneAutomaticDimension.width, 1000);
    EXPECT_GT(oneAutomaticDimension.height, 0);
    EXPECT_LE(oneAutomaticDimension.height, 1836);
  }
}

TEST(FilamentSceneExtraction, GuiScale_NormalizesUserAndDpiScaleRange)
{
  const auto normalizedScale = [](double scale) {
    if (!std::isfinite(scale) || scale <= 0.0) {
      return 1.0;
    }
    return std::clamp(scale, 0.5, 4.0);
  };
  const std::array<double, 11> values = {
      -2.0,
      0.0,
      0.25,
      0.5,
      0.75,
      1.0,
      1.5,
      2.0,
      4.0,
      8.0,
      std::numeric_limits<double>::quiet_NaN(),
  };

  for (const double userScale : values) {
    for (const double dpiScale : values) {
      const dart::gui::detail::GuiScaleState scale
          = dart::gui::detail::makeGuiScaleState(userScale, dpiScale);
      EXPECT_NEAR(scale.userScale, normalizedScale(userScale), 1e-12);
      EXPECT_NEAR(scale.dpiScale, normalizedScale(dpiScale), 1e-12);
      EXPECT_NEAR(
          scale.effectiveScale, scale.userScale * scale.dpiScale, 1e-12);
      EXPECT_GE(scale.userScale, 0.5);
      EXPECT_LE(scale.userScale, 4.0);
      EXPECT_GE(scale.dpiScale, 0.5);
      EXPECT_LE(scale.dpiScale, 4.0);
      EXPECT_GE(scale.effectiveScale, 0.25);
      EXPECT_LE(scale.effectiveScale, 16.0);
    }
  }
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

  const double distanceAfterScroll = controller.camera.distance;
  controllerInput.pan = false;
  controllerInput.zoom = true;
  controllerInput.cursorY = 60.0;
  dart::gui::updateOrbitCameraController(controller, controllerInput);
  EXPECT_LT(controller.camera.distance, distanceAfterScroll);

  const double yawBeforeExternalTargetUpdate = controller.camera.yaw;
  controller.camera.target = Eigen::Vector3d(5.0, 0.0, 0.0);
  controllerInput.orbit = true;
  controllerInput.zoom = false;
  controllerInput.cursorX = 120.0;
  dart::gui::updateOrbitCameraController(controller, controllerInput);
  EXPECT_NE(controller.camera.yaw, yawBeforeExternalTargetUpdate);
  EXPECT_TRUE(
      controller.camera.target.isApprox(Eigen::Vector3d(5.0, 0.0, 0.0)));

  controllerInput.hasCursor = false;
  dart::gui::updateOrbitCameraController(controller, controllerInput);
  EXPECT_FALSE(controller.hasLastCursor);

  const dart::gui::OrbitCamera cameraBeforeLock = controller.camera;
  controller.hasLastCursor = true;
  controller.lastCursorX = 50.0;
  controller.lastCursorY = 60.0;
  dart::gui::addOrbitCameraScroll(controller, 1.0);
  controllerInput.hasCursor = true;
  controllerInput.locked = true;
  controllerInput.orbit = true;
  controllerInput.cursorX = 140.0;
  controllerInput.cursorY = 160.0;
  dart::gui::updateOrbitCameraController(controller, controllerInput);
  EXPECT_FALSE(controller.hasLastCursor);
  EXPECT_NEAR(controller.scrollDelta, 0.0, 1e-12);
  EXPECT_TRUE(controller.camera.target.isApprox(cameraBeforeLock.target));
  EXPECT_TRUE(controller.camera.up.isApprox(cameraBeforeLock.up));
  EXPECT_EQ(controller.camera.yaw, cameraBeforeLock.yaw);
  EXPECT_EQ(controller.camera.pitch, cameraBeforeLock.pitch);
  EXPECT_EQ(controller.camera.distance, cameraBeforeLock.distance);

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
