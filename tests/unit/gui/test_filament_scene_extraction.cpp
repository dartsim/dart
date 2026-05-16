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
#include <dart/gui/geometry.hpp>
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
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/heightmap_shape.hpp>
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
#include <dart/dynamics/weld_joint.hpp>

#if DART_HAVE_OCTOMAP
  #include <dart/dynamics/voxel_grid_shape.hpp>
#endif

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
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

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

#if DART_HAVE_OCTOMAP
using dart::dynamics::VoxelGridShape;
#endif

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

  bool buttonPressed = true;
  bool checkboxValue = true;
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

constexpr std::array<std::string_view, 2> kForbiddenFilamentDetailTokens
    = {"detail/filament", "experimental::filament"};

constexpr std::array<std::string_view, 3> kForbiddenPromotedGuiTokens
    = {"dart/gui/experimental",
       "dart::gui::experimental",
       "dart-gui-experimental"};

const std::filesystem::path kDartsimApplicationDirectory
    = std::filesystem::path("apps") / "dartsim";

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
      = {std::filesystem::path("dart") / "gui" / "experimental",
         std::filesystem::path("dart") / "gui"};
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

TEST(FilamentSceneExtraction, ExperimentalPublicHeadersStayBackendHidden)
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
  EXPECT_NE(aggregate.find("#include <dart/gui/panel.hpp>"), std::string::npos);
  EXPECT_NE(aggregate.find("#include <dart/gui/scene.hpp>"), std::string::npos);
  EXPECT_NE(
      aggregate.find("#include <dart/gui/viewer.hpp>"), std::string::npos);
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

TEST(FilamentSceneExtraction, PanelBuilderSupportsRendererNeutralControls)
{
  bool diagnostics = false;
  double gain = 0.0;
  int clicks = 0;
  std::string selectedLabel;
  dart::gui::Panel panel;
  panel.title = "Controls";
  panel.build = [&](dart::gui::PanelBuilder& builder) {
    builder.text("custom controls");
    builder.separator();
    if (builder.button("Trigger")) {
      ++clicks;
    }
    builder.sameLine();
    builder.checkbox("Diagnostics", diagnostics);
    builder.slider("Gain", gain, 0.0, 2.0);
  };
  panel.buildWithContext = [&](dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    selectedLabel = context.selectedLabel;
    builder.text("selected:" + context.selectedLabel);
  };

  RecordingPanelBuilder builder;
  dart::gui::PanelContext context;
  context.selectedLabel = "box";
  panel.build(builder);
  panel.buildWithContext(builder, context);

  EXPECT_EQ(clicks, 1);
  EXPECT_TRUE(diagnostics);
  EXPECT_DOUBLE_EQ(gain, 1.0);
  EXPECT_EQ(selectedLabel, "box");
  bool preStepCalled = false;
  EXPECT_EQ(
      builder.events,
      (std::vector<std::string>{
          "text:custom controls",
          "separator",
          "button:Trigger",
          "same-line",
          "checkbox:Diagnostics",
          "slider:Gain",
          "text:selected:box"}));

  dart::gui::ApplicationOptions options;
  options.world = World::create("panel_test");
  options.preStep = [&preStepCalled]() {
    preStepCalled = true;
  };
  options.defaultScene = "mvp";
  options.panels.push_back(std::move(panel));
  ASSERT_NE(options.world, nullptr);
  EXPECT_EQ(options.world->getName(), "panel_test");
  options.preStep();
  EXPECT_TRUE(preStepCalled);
  EXPECT_EQ(options.defaultScene, "mvp");
  ASSERT_EQ(options.panels.size(), 1u);
  EXPECT_EQ(options.panels.front().title, "Controls");
}

TEST(FilamentSceneExtraction, RestoredExamplesUsePromotedGuiBoundary)
{
  struct ExampleExpectation
  {
    std::filesystem::path directory;
    bool usesPanel = false;
    bool ownsWorld = true;
  };

  const std::vector<ExampleExpectation> examples
      = {{std::filesystem::path("examples") / "hello_world", false},
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
         {std::filesystem::path("examples") / "tinkertoy", true}};
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

TEST(FilamentSceneExtraction, FetchExamplePreservesLegacyParityMarkers)
{
  const auto mainSource = readSourceFile(
      std::filesystem::path("examples") / "fetch" / "main.cpp");

  EXPECT_NE(mainSource.find("#if DART_HAVE_BULLET"), std::string::npos);
  EXPECT_NE(
      mainSource.find("CollisionDetectorType::Bullet"), std::string::npos);
  EXPECT_NE(mainSource.find("interactive frame"), std::string::npos);
  EXPECT_NE(mainSource.find("fetch_target_x_bar"), std::string::npos);
  EXPECT_NE(mainSource.find("fetch_target_y_bar"), std::string::npos);
  EXPECT_NE(mainSource.find("options.world"), std::string::npos);
  EXPECT_NE(mainSource.find("options.preStep"), std::string::npos);
  EXPECT_NE(mainSource.find("syncMocapTarget"), std::string::npos);
  EXPECT_EQ(mainSource.find("options.defaultScene"), std::string::npos);
  EXPECT_EQ(
      mainSource.find("setGravity(Eigen::Vector3d::Zero())"),
      std::string::npos);
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
      = scanSourceFilesForTokens(sources, kForbiddenFilamentDetailTokens);
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
      mainSource.find("#include <dart/gui/application.hpp>"),
      std::string::npos);
  EXPECT_EQ(countOccurrences(mainSource, "int main("), 1u);
  EXPECT_EQ(
      countOccurrences(
          mainSource, "return dart::gui::runApplication(argc, argv);"),
      1u);
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
         kDartsimApplicationDirectory / "README.md",
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

#if DART_HAVE_OCTOMAP
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
#endif

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
  auto detector = dart::collision::DARTCollisionDetector::create();
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
  dart::gui::RunOptions options;
  options.width = 0;
  options.height = -8;
  options.guiScale = std::numeric_limits<double>::quiet_NaN();
  options.headless = true;
  options.screenshotPath = "capture.ppm";

  dart::gui::normalizeRunOptions(options);

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
}

TEST(FilamentSceneExtraction, WriteRgbaPpm_DropsAlphaAndHandlesBottomLeftOrigin)
{
  const auto path = std::filesystem::temp_directory_path()
                    / "dart_gui_experimental_rgba.ppm";
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
  EXPECT_NEAR(projection.nearPlane, 0.008, 1e-12);
  EXPECT_NEAR(projection.farPlane, 37.0, 1e-12);

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
