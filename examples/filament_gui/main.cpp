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

#include "imgui_overlay.hpp"
#include "input.hpp"
#include "native_window.hpp"
#include "profile.hpp"
#include "render_environment.hpp"
#include "renderable_factory.hpp"
#include "renderable_resources.hpp"
#include "renderable_sync.hpp"
#include "scenes.hpp"
#include "screenshot.hpp"
#include "selection.hpp"
#include "textures.hpp"

#include <dart/all.hpp>
#include <dart/common/local_resource_retriever.hpp>
#include <dart/common/profile.hpp>
#include <dart/config.hpp>
#include <dart/gui/experimental/scene.hpp>
#include <dart/io/read.hpp>
#include <dart/utils/composite_resource_retriever.hpp>
#include <dart/utils/dart_resource_retriever.hpp>
#include <dart/utils/http_resource_retriever.hpp>
#include <dart/utils/mesh_loader.hpp>
#include <dart/utils/package_resource_retriever.hpp>
#include <dart/utils/urdf/All.hpp>

#include <GLFW/glfw3.h>
#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/SwapChain.h>
#include <filament/View.h>
#include <utils/EntityManager.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace {

using dart::dynamics::BoxShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::ConvexMeshShape;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::FreeJoint;
using dart::dynamics::HeightmapShaped;
using dart::dynamics::InverseKinematics;
using dart::dynamics::InverseKinematicsPtr;
using dart::dynamics::MeshShape;
using dart::dynamics::PlaneShape;
using dart::dynamics::PointCloudShape;
using dart::dynamics::Shape;
using dart::dynamics::ShapePtr;
using dart::dynamics::ShapeNode;
using dart::dynamics::SimpleFrame;
using dart::dynamics::Skeleton;
using dart::dynamics::SoftBodyNode;
using dart::dynamics::SoftBodyNodeHelper;
using dart::dynamics::SphereShape;
using dart::dynamics::VisualAspect;
using dart::dynamics::WeldJoint;

#if DART_HAVE_OCTOMAP
using dart::dynamics::VoxelGridShape;
#endif

using dart::gui::experimental::DebugDrawOptions;
using dart::gui::experimental::OrbitCamera;
using dart::gui::experimental::OrbitCameraController;
using dart::gui::experimental::PickRay;
using dart::gui::experimental::RenderableDescriptor;
using dart::gui::experimental::RenderableId;
using dart::gui::experimental::RunOptions;
using dart::gui::experimental::ShapeKind;
using dart::gui::experimental::ViewerLifecycleState;
using dart::gui::experimental::computePlaneDragTranslation;
using dart::gui::experimental::extractContactDebugLines;
using dart::gui::experimental::extractDebugLines;
using dart::gui::experimental::extractRenderables;
using dart::gui::experimental::intersectPlane;
using dart::gui::experimental::markFrameRendered;
using dart::gui::experimental::markFrameSkipped;
using dart::gui::experimental::markScreenshotRequested;
using dart::gui::experimental::markSimulationAdvanced;
using dart::gui::experimental::makeOrbitCameraBasis;
using dart::gui::experimental::makePerspectivePickRay;
using dart::gui::experimental::makeRenderableId;
using dart::gui::experimental::makeSelectionDebugLines;
using dart::gui::experimental::normalizeRunOptions;
using dart::gui::experimental::pickNearestRenderable;
using dart::gui::experimental::requestSingleStep;
using dart::gui::experimental::shouldAdvanceSimulation;
using dart::gui::experimental::shouldRequestScreenshot;
using dart::gui::experimental::shouldStopAfterFrame;
using dart::gui::experimental::togglePaused;
using dart::simulation::World;
using dart::examples::filament_gui::AppOptions;
using dart::examples::filament_gui::DartScene;
using dart::examples::filament_gui::ExampleScene;
using dart::examples::filament_gui::ImGuiOverlay;
using dart::examples::filament_gui::MaterialResources;
using dart::examples::filament_gui::MaterialSet;
using dart::examples::filament_gui::ProfileAccumulator;
using dart::examples::filament_gui::Renderable;
using dart::examples::filament_gui::ScreenshotCapture;
using dart::examples::filament_gui::SceneRenderable;
using dart::examples::filament_gui::SceneLights;
using dart::examples::filament_gui::configureWindowedViewQuality;
using dart::examples::filament_gui::configureViewportCamera;
using dart::examples::filament_gui::createDartScene;
using dart::examples::filament_gui::createDebugColorGrading;
using dart::examples::filament_gui::createDebugLineRenderable;
using dart::examples::filament_gui::createImGuiOverlay;
using dart::examples::filament_gui::createMaterialResources;
using dart::examples::filament_gui::createNeutralIndirectLight;
using dart::examples::filament_gui::createNeutralSkybox;
using dart::examples::filament_gui::createRenderableFromDescriptor;
using dart::examples::filament_gui::createSceneLights;
using dart::examples::filament_gui::destroyImGuiOverlay;
using dart::examples::filament_gui::destroyMaterialResources;
using dart::examples::filament_gui::destroyRenderable;
using dart::examples::filament_gui::elapsedMs;
using dart::examples::filament_gui::handleScroll;
using dart::examples::filament_gui::initialCameraForScene;
using dart::examples::filament_gui::isDragModifierDown;
using dart::examples::filament_gui::isInsideStatusPanel;
using dart::examples::filament_gui::kAtlasFixtureSkeletonName;
using dart::examples::filament_gui::kAtlasRobotFixtureSkeletonName;
using dart::examples::filament_gui::kConvexMeshFixtureSkeletonName;
using dart::examples::filament_gui::kG1FixtureSkeletonName;
using dart::examples::filament_gui::kHeightmapFixtureSkeletonName;
using dart::examples::filament_gui::kLineSegmentFixtureSkeletonName;
using dart::examples::filament_gui::kMinDragAndDropFrameRenderableCount;
using dart::examples::filament_gui::kMinG1RenderableCount;
using dart::examples::filament_gui::kMinPbrEnvironmentRenderableCount;
using dart::examples::filament_gui::kMultiSphereFixtureSkeletonName;
using dart::examples::filament_gui::kPbrEnvironmentFixtureSkeletonName;
using dart::examples::filament_gui::kPointCloudFixtureSkeletonName;
using dart::examples::filament_gui::kPyramidFixtureSkeletonName;
using dart::examples::filament_gui::kSoftMeshFixtureSkeletonName;
using dart::examples::filament_gui::kVoxelGridFixtureSkeletonName;
using dart::examples::filament_gui::kWamFixtureSkeletonName;
using dart::examples::filament_gui::getNativeWindow;
using dart::examples::filament_gui::loadImGuiFont;
using dart::examples::filament_gui::logUnsupportedRenderableDescriptorOnce;
using dart::examples::filament_gui::parseOptions;
using dart::examples::filament_gui::printProfile;
using dart::examples::filament_gui::requestScreenshot;
using dart::examples::filament_gui::saveScreenshot;
using dart::examples::filament_gui::sceneName;
using dart::examples::filament_gui::selectedNudgeFromKeyboard;
using dart::examples::filament_gui::selectionLabelForRenderable;
using dart::examples::filament_gui::setRenderableTransform;
using dart::examples::filament_gui::synchronizeSceneRenderables;
using dart::examples::filament_gui::translateRenderableAndApplyIk;
using dart::examples::filament_gui::updateCameraController;
using dart::examples::filament_gui::updateImGuiOverlay;
using dart::examples::filament_gui::updateImGuiMouseInput;
using dart::examples::filament_gui::updateSceneRenderableFromDescriptor;
using dart::examples::filament_gui::updateOrbitingKeyLight;
using dart::examples::filament_gui::waitForScreenshot;
using utils::EntityManager;


} // namespace

int main(int argc, char* argv[])
{
  const AppOptions appOptions = parseOptions(argc, argv);
  const RunOptions& options = appOptions.run;

  GLFWwindow* window = nullptr;
#if defined(__linux__)
  if (!options.headless) {
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
  }
#endif
  if (!options.headless) {
    if (!glfwInit()) {
      std::cerr << "Failed to initialize GLFW\n";
      return 1;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    window = glfwCreateWindow(
        options.width,
        options.height,
        "DART + Filament (experimental)",
        nullptr,
        nullptr);
    if (window == nullptr) {
      std::cerr << "Failed to create GLFW window\n";
      glfwTerminate();
      return 1;
    }
  }

  OrbitCameraController cameraController;
  cameraController.camera = initialCameraForScene(appOptions.scene);
  if (window != nullptr) {
    glfwSetWindowUserPointer(window, &cameraController);
    glfwSetScrollCallback(window, handleScroll);
  }

  auto* engine = filament::Engine::create(filament::Engine::Backend::OPENGL);
  auto* renderer = engine->createRenderer();
  if (options.headless) {
    filament::Renderer::DisplayInfo displayInfo;
    displayInfo.refreshRate = 0.0f;
    renderer->setDisplayInfo(displayInfo);
  }
  auto* swapChain = options.headless
                        ? engine->createSwapChain(
                              static_cast<std::uint32_t>(options.width),
                              static_cast<std::uint32_t>(options.height))
                        : engine->createSwapChain(getNativeWindow(window));
  auto* view = engine->createView();
  auto* scene = engine->createScene();
  auto cameraEntity = EntityManager::get().create();
  auto* camera = engine->createCamera(cameraEntity);
  view->setScene(scene);
  view->setCamera(camera);
  auto* colorGrading = createDebugColorGrading(*engine);
  view->setColorGrading(colorGrading);
  view->setShadowingEnabled(true);
  view->setShadowType(filament::ShadowType::PCF);
  auto* indirectLight = createNeutralIndirectLight(*engine);
  auto* skybox = createNeutralSkybox(*engine);
  scene->setIndirectLight(indirectLight);
  scene->setSkybox(skybox);
  if (!options.headless) {
    configureWindowedViewQuality(*view);
  }

  MaterialResources materialResources = createMaterialResources(*engine);
  const MaterialSet materials = materialResources.materialSet();

  DartScene dartScene = createDartScene(appOptions);
  const auto initialDescriptors = extractRenderables(*dartScene.world);
  const std::size_t wamDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kWamFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Mesh;
          }));
  const std::size_t atlasDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kAtlasFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Mesh;
          }));
  const std::size_t atlasRobotDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kAtlasRobotFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Mesh;
          }));
  const std::size_t pyramidDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kPyramidFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Pyramid;
          }));
  const std::size_t multiSphereDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kMultiSphereFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::MultiSphere;
          }));
  const std::size_t lineSegmentDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kLineSegmentFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::LineSegments;
          }));
  const std::size_t convexMeshDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kConvexMeshFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::ConvexMesh;
          }));
  const std::size_t pointCloudDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kPointCloudFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::PointCloud;
          }));
  const std::size_t heightmapDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kHeightmapFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Heightmap;
          }));
  const std::size_t softMeshDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kSoftMeshFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::SoftMesh;
          }));
#if DART_HAVE_OCTOMAP
  const std::size_t voxelGridDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kVoxelGridFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::VoxelGrid;
          }));
#endif
  const std::size_t pbrEnvironmentDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kPbrEnvironmentFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Mesh;
          }));
  const std::size_t g1DescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kG1FixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Mesh;
          }));
  const std::size_t dragAndDropFrameDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return !descriptor.shapeFrameName.empty()
                   && descriptor.material.visible;
          }));
  if (appOptions.scene == ExampleScene::Mvp) {
    if (wamDescriptorCount < 5) {
      std::cerr << "Expected the WAM robot fixture to provide at least five "
                   "visible mesh renderables, but extracted "
                << wamDescriptorCount << "\n";
      return 1;
    }
    if (atlasDescriptorCount != 1) {
      std::cerr << "Expected the Atlas mesh fixture to provide one visible mesh "
                   "renderable, but extracted "
                << atlasDescriptorCount << "\n";
      return 1;
    }
    if (atlasRobotDescriptorCount < 20) {
      std::cerr << "Expected the Atlas robot fixture to provide at least twenty "
                   "visible mesh renderables, but extracted "
                << atlasRobotDescriptorCount << "\n";
      return 1;
    }
    if (pyramidDescriptorCount != 1) {
      std::cerr << "Expected the pyramid fixture to provide one visible pyramid "
                   "renderable descriptor, but extracted "
                << pyramidDescriptorCount << "\n";
      return 1;
    }
    if (multiSphereDescriptorCount != 1) {
      std::cerr
          << "Expected the multi-sphere fixture to provide one visible "
             "multi-sphere renderable descriptor, but extracted "
          << multiSphereDescriptorCount << "\n";
      return 1;
    }
    if (lineSegmentDescriptorCount != 1) {
      std::cerr
          << "Expected the line segment fixture to provide one visible line "
             "renderable descriptor, but extracted "
          << lineSegmentDescriptorCount << "\n";
      return 1;
    }
    if (convexMeshDescriptorCount != 1) {
      std::cerr
          << "Expected the convex mesh fixture to provide one visible convex "
             "mesh renderable descriptor, but extracted "
          << convexMeshDescriptorCount << "\n";
      return 1;
    }
    if (pointCloudDescriptorCount != 1) {
      std::cerr
          << "Expected the point cloud fixture to provide one visible point "
             "cloud renderable descriptor, but extracted "
          << pointCloudDescriptorCount << "\n";
      return 1;
    }
    if (heightmapDescriptorCount != 1) {
      std::cerr
          << "Expected the heightmap fixture to provide one visible heightmap "
             "renderable descriptor, but extracted "
          << heightmapDescriptorCount << "\n";
      return 1;
    }
    if (softMeshDescriptorCount != 1) {
      std::cerr
          << "Expected the soft mesh fixture to provide one visible soft mesh "
             "renderable descriptor, but extracted "
          << softMeshDescriptorCount << "\n";
      return 1;
    }
#if DART_HAVE_OCTOMAP
    if (voxelGridDescriptorCount != 1) {
      std::cerr
          << "Expected the voxel grid fixture to provide one visible voxel "
             "grid renderable descriptor, but extracted "
          << voxelGridDescriptorCount << "\n";
      return 1;
    }
#endif
    if (pbrEnvironmentDescriptorCount < kMinPbrEnvironmentRenderableCount) {
      std::cerr << "Expected the PBR environment fixture to provide at least "
                << kMinPbrEnvironmentRenderableCount
                << " visible mesh renderables, but extracted "
                << pbrEnvironmentDescriptorCount << "\n";
      return 1;
    }
  } else if (appOptions.scene == ExampleScene::G1) {
    if (g1DescriptorCount < kMinG1RenderableCount) {
      std::cerr << "Expected the G1 robot fixture to provide at least "
                << kMinG1RenderableCount
                << " visible mesh renderables, but extracted "
                << g1DescriptorCount << "\n";
      return 1;
    }
  } else if (appOptions.scene == ExampleScene::DragAndDrop
             && dragAndDropFrameDescriptorCount
                    < kMinDragAndDropFrameRenderableCount) {
    std::cerr << "Expected the drag-and-drop scene to provide at least "
              << kMinDragAndDropFrameRenderableCount
              << " visible frame renderables, but extracted "
              << dragAndDropFrameDescriptorCount << "\n";
    return 1;
  }

  std::vector<SceneRenderable> sceneRenderables;
  std::vector<RenderableId> loggedUnsupportedRenderableIds;
  std::size_t createdWamRenderableCount = 0;
  std::size_t createdAtlasRenderableCount = 0;
  std::size_t createdAtlasRobotRenderableCount = 0;
  std::size_t createdPyramidRenderableCount = 0;
  std::size_t createdMultiSphereRenderableCount = 0;
  std::size_t createdLineSegmentRenderableCount = 0;
  std::size_t createdConvexMeshRenderableCount = 0;
  std::size_t createdPointCloudRenderableCount = 0;
  std::size_t createdHeightmapRenderableCount = 0;
  std::size_t createdSoftMeshRenderableCount = 0;
#if DART_HAVE_OCTOMAP
  std::size_t createdVoxelGridRenderableCount = 0;
#endif
  std::size_t createdPbrEnvironmentRenderableCount = 0;
  std::size_t createdG1RenderableCount = 0;
  std::size_t createdDragAndDropFrameRenderableCount = 0;
  for (const RenderableDescriptor& descriptor : initialDescriptors) {
    if (!descriptor.material.visible) {
      continue;
    }

    auto renderable = createRenderableFromDescriptor(
        *engine, materials, materialResources.textureCache, descriptor);
    if (!renderable) {
      if (descriptor.geometry.kind == ShapeKind::Unsupported) {
        logUnsupportedRenderableDescriptorOnce(
            loggedUnsupportedRenderableIds, descriptor);
      }
      continue;
    }
    if (descriptor.skeletonName == kWamFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Mesh) {
      ++createdWamRenderableCount;
    }
    if (descriptor.skeletonName == kAtlasFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Mesh) {
      ++createdAtlasRenderableCount;
    }
    if (descriptor.skeletonName == kAtlasRobotFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Mesh) {
      ++createdAtlasRobotRenderableCount;
    }
    if (descriptor.skeletonName == kPyramidFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Pyramid) {
      ++createdPyramidRenderableCount;
    }
    if (descriptor.skeletonName == kMultiSphereFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::MultiSphere) {
      ++createdMultiSphereRenderableCount;
    }
    if (descriptor.skeletonName == kLineSegmentFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::LineSegments) {
      ++createdLineSegmentRenderableCount;
    }
    if (descriptor.skeletonName == kConvexMeshFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::ConvexMesh) {
      ++createdConvexMeshRenderableCount;
    }
    if (descriptor.skeletonName == kPointCloudFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::PointCloud) {
      ++createdPointCloudRenderableCount;
    }
    if (descriptor.skeletonName == kHeightmapFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Heightmap) {
      ++createdHeightmapRenderableCount;
    }
    if (descriptor.skeletonName == kSoftMeshFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::SoftMesh) {
      ++createdSoftMeshRenderableCount;
    }
#if DART_HAVE_OCTOMAP
    if (descriptor.skeletonName == kVoxelGridFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::VoxelGrid) {
      ++createdVoxelGridRenderableCount;
    }
#endif
    if (descriptor.skeletonName == kPbrEnvironmentFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Mesh) {
      ++createdPbrEnvironmentRenderableCount;
    }
    if (descriptor.skeletonName == kG1FixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Mesh) {
      ++createdG1RenderableCount;
    }
    if (!descriptor.shapeFrameName.empty()) {
      ++createdDragAndDropFrameRenderableCount;
    }

    SceneRenderable sceneRenderable;
    sceneRenderable.id = descriptor.id;
    sceneRenderable.renderable = *renderable;
    scene->addEntity(sceneRenderable.renderable.entity);
    setRenderableTransform(
        *engine, sceneRenderable.renderable, descriptor.worldTransform);
    sceneRenderables.push_back(sceneRenderable);
  }
  if (sceneRenderables.empty()) {
    std::cerr << "No supported visible DART renderables were extracted\n";
    return 1;
  }
  if (appOptions.scene == ExampleScene::Mvp) {
    if (createdWamRenderableCount < wamDescriptorCount) {
      std::cerr << "Only " << createdWamRenderableCount << " of "
                << wamDescriptorCount
                << " WAM robot mesh renderables were created\n";
      return 1;
    }
    if (createdAtlasRenderableCount != atlasDescriptorCount) {
      std::cerr << "Only " << createdAtlasRenderableCount << " of "
                << atlasDescriptorCount
                << " Atlas mesh renderables were created\n";
      return 1;
    }
    if (createdAtlasRobotRenderableCount < atlasRobotDescriptorCount) {
      std::cerr << "Only " << createdAtlasRobotRenderableCount << " of "
                << atlasRobotDescriptorCount
                << " Atlas robot mesh renderables were created\n";
      return 1;
    }
    if (createdPyramidRenderableCount != pyramidDescriptorCount) {
      std::cerr << "Only " << createdPyramidRenderableCount << " of "
                << pyramidDescriptorCount
                << " pyramid renderables were created\n";
      return 1;
    }
    if (createdMultiSphereRenderableCount != multiSphereDescriptorCount) {
      std::cerr << "Only " << createdMultiSphereRenderableCount << " of "
                << multiSphereDescriptorCount
                << " multi-sphere renderables were created\n";
      return 1;
    }
    if (createdLineSegmentRenderableCount != lineSegmentDescriptorCount) {
      std::cerr << "Only " << createdLineSegmentRenderableCount << " of "
                << lineSegmentDescriptorCount
                << " line segment renderables were created\n";
      return 1;
    }
    if (createdConvexMeshRenderableCount != convexMeshDescriptorCount) {
      std::cerr << "Only " << createdConvexMeshRenderableCount << " of "
                << convexMeshDescriptorCount
                << " convex mesh renderables were created\n";
      return 1;
    }
    if (createdPointCloudRenderableCount != pointCloudDescriptorCount) {
      std::cerr << "Only " << createdPointCloudRenderableCount << " of "
                << pointCloudDescriptorCount
                << " point cloud renderables were created\n";
      return 1;
    }
    if (createdHeightmapRenderableCount != heightmapDescriptorCount) {
      std::cerr << "Only " << createdHeightmapRenderableCount << " of "
                << heightmapDescriptorCount
                << " heightmap renderables were created\n";
      return 1;
    }
    if (createdSoftMeshRenderableCount != softMeshDescriptorCount) {
      std::cerr << "Only " << createdSoftMeshRenderableCount << " of "
                << softMeshDescriptorCount
                << " soft mesh renderables were created\n";
      return 1;
    }
#if DART_HAVE_OCTOMAP
    if (createdVoxelGridRenderableCount != voxelGridDescriptorCount) {
      std::cerr << "Only " << createdVoxelGridRenderableCount << " of "
                << voxelGridDescriptorCount
                << " voxel grid renderables were created\n";
      return 1;
    }
#endif
    if (createdPbrEnvironmentRenderableCount < pbrEnvironmentDescriptorCount) {
      std::cerr << "Only " << createdPbrEnvironmentRenderableCount << " of "
                << pbrEnvironmentDescriptorCount
                << " PBR environment mesh renderables were created\n";
      return 1;
    }
  } else if (appOptions.scene == ExampleScene::G1) {
    if (createdG1RenderableCount < g1DescriptorCount) {
      std::cerr << "Only " << createdG1RenderableCount << " of "
                << g1DescriptorCount
                << " G1 robot mesh renderables were created\n";
      return 1;
    }
  } else if (appOptions.scene == ExampleScene::DragAndDrop
             && createdDragAndDropFrameRenderableCount
                    < dragAndDropFrameDescriptorCount) {
    std::cerr << "Only " << createdDragAndDropFrameRenderableCount << " of "
              << dragAndDropFrameDescriptorCount
              << " drag-and-drop frame renderables were created\n";
    return 1;
  }

  DebugDrawOptions staticDebugOptions;
  staticDebugOptions.drawBodyFrames = true;
  staticDebugOptions.drawCentersOfMass = true;
  staticDebugOptions.drawInertiaBoxes = false;
  staticDebugOptions.drawCollisionShapeBounds = false;
  staticDebugOptions.drawSupportPolygons
      = appOptions.scene == ExampleScene::G1;
  staticDebugOptions.drawContacts = false;
  std::optional<Renderable> debugOverlay;
  auto refreshDebugOverlay = [&]() {
    if (debugOverlay) {
      scene->remove(debugOverlay->entity);
      destroyRenderable(*engine, *debugOverlay);
      debugOverlay.reset();
    }

    debugOverlay = createDebugLineRenderable(
        *engine,
        materials.debugColor,
        extractDebugLines(*dartScene.world, staticDebugOptions));
    if (debugOverlay) {
      scene->addEntity(debugOverlay->entity);
    }
  };
  refreshDebugOverlay();
  if (!debugOverlay) {
    std::cerr << "No debug overlay lines were extracted\n";
    return 1;
  }

  DebugDrawOptions contactDebugOptions;
  contactDebugOptions.drawGrid = false;
  contactDebugOptions.drawWorldFrame = false;
  contactDebugOptions.drawBodyFrames = false;
  contactDebugOptions.drawCentersOfMass = false;
  std::optional<Renderable> contactDebugOverlay;
  auto refreshContactDebugOverlay = [&]() {
    if (contactDebugOverlay) {
      scene->remove(contactDebugOverlay->entity);
      destroyRenderable(*engine, *contactDebugOverlay);
      contactDebugOverlay.reset();
    }

    contactDebugOverlay = createDebugLineRenderable(
        *engine,
        materials.debugColor,
        extractContactDebugLines(
            dartScene.world->getLastCollisionResult(), contactDebugOptions));
    if (contactDebugOverlay) {
      scene->addEntity(contactDebugOverlay->entity);
    }
  };

  std::optional<Renderable> selectionDebugOverlay;
  auto refreshSelectionDebugOverlay
      = [&](const std::vector<RenderableDescriptor>& descriptors,
            RenderableId selectedRenderableId) {
          if (selectionDebugOverlay) {
            scene->remove(selectionDebugOverlay->entity);
            destroyRenderable(*engine, *selectionDebugOverlay);
            selectionDebugOverlay.reset();
          }

          if (selectedRenderableId == 0) {
            return;
          }

          const auto selectedDescriptor = std::find_if(
              descriptors.begin(),
              descriptors.end(),
              [&](const RenderableDescriptor& candidate) {
                return candidate.id == selectedRenderableId;
              });
          if (selectedDescriptor == descriptors.end()
              || !selectedDescriptor->material.visible) {
            return;
          }

          selectionDebugOverlay = createDebugLineRenderable(
              *engine,
              materials.debugColor,
              makeSelectionDebugLines(*selectedDescriptor));
          if (selectionDebugOverlay) {
            scene->addEntity(selectionDebugOverlay->entity);
          }
        };

  bool orbitLight = appOptions.orbitLight;
  SceneLights lights = createSceneLights(
      *engine, options.headless, orbitLight, appOptions.orbitLightPeriodSeconds);
  scene->addEntity(lights.key);
  scene->addEntity(lights.fill);
  scene->addEntity(lights.rim);

  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  auto& imguiStyle = ImGui::GetStyle();
  const float guiScale = static_cast<float>(options.guiScale);
  imguiStyle.ScaleAllSizes(guiScale);
  imguiStyle.WindowRounding = 4.0f * guiScale;
  imguiStyle.Colors[ImGuiCol_WindowBg].w = 0.72f;
  auto& imguiIo = ImGui::GetIO();
  loadImGuiFont(imguiIo, guiScale);
  imguiIo.Fonts->Build();
  ImGuiOverlay imguiOverlay = createImGuiOverlay(*engine);

  ViewerLifecycleState lifecycle;
  bool wasSpacePressed = false;
  bool wasStepPressed = false;
  bool wasLeftMousePressed = false;
  bool leftMouseStartedOnPanel = false;
  bool leftMouseStartedDrag = false;
  double leftMousePressX = 0.0;
  double leftMousePressY = 0.0;
  PickRay selectedDragLastRay;
  Eigen::Vector3d selectedDragPlanePoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d selectedDragPlaneNormal = Eigen::Vector3d::UnitX();
  RenderableId selectedRenderableId = 0;
  std::string selectedLabel = "none";
  bool screenshotSucceeded = options.screenshotPath.empty();
  ScreenshotCapture screenshotCapture;
  ProfileAccumulator profile;
  auto lastSimulationClock = ProfileAccumulator::Clock::now();
  const auto orbitStartClock = ProfileAccumulator::Clock::now();
  double simulationAccumulator = 0.0;
  constexpr std::size_t kMaxSimulationStepsPerRenderedFrame = 64;

  while (options.headless || !glfwWindowShouldClose(window)) {
    const auto frameStart = ProfileAccumulator::Clock::now();
    auto phaseStart = ProfileAccumulator::Clock::now();
    if (window != nullptr) {
      glfwPollEvents();
      if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
      }
      const bool isSpacePressed
          = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
      if (isSpacePressed && !wasSpacePressed) {
        togglePaused(lifecycle);
      }
      wasSpacePressed = isSpacePressed;

      const bool isStepPressed = glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS;
      if (isStepPressed && !wasStepPressed) {
        requestSingleStep(lifecycle, false);
      }
      wasStepPressed = isStepPressed;

      for (const auto& handle : dartScene.ikHandles) {
        if (glfwGetKey(window, handle.hotkey) == GLFW_PRESS) {
          selectedRenderableId = handle.targetRenderableId;
          selectedLabel = handle.label + " IK target";
          lifecycle.paused = true;
        }
      }
    }
    profile.inputMs += elapsedMs(phaseStart);

    phaseStart = ProfileAccumulator::Clock::now();
    int width = options.width;
    int height = options.height;
    if (window != nullptr) {
      glfwGetFramebufferSize(window, &width, &height);
    }
    width = std::max(1, width);
    height = std::max(1, height);
    imguiIo.DisplaySize
        = ImVec2(static_cast<float>(width), static_cast<float>(height));
    imguiIo.DeltaTime = static_cast<float>(dartScene.world->getTimeStep());
    configureViewportCamera(*view, *camera, cameraController.camera, width, height);
    if (window != nullptr) {
      double cursorX = 0.0;
      double cursorY = 0.0;
      glfwGetCursorPos(window, &cursorX, &cursorY);
      const bool suppressCameraOrbit
          = leftMouseStartedDrag
            || (appOptions.showUi
                && isInsideStatusPanel(cursorX, cursorY, options.guiScale));
      updateCameraController(window, cameraController, suppressCameraOrbit);
    }
    profile.viewportCameraMs += elapsedMs(phaseStart);

    std::size_t simulationStepsToRun = 0;
    if (shouldAdvanceSimulation(lifecycle)) {
      if (options.headless || lifecycle.stepOnce) {
        simulationStepsToRun = 1;
      } else {
        const auto now = ProfileAccumulator::Clock::now();
        simulationAccumulator += std::chrono::duration<double>(
                                     now - lastSimulationClock)
                                     .count();
        lastSimulationClock = now;
        const double timeStep = dartScene.world->getTimeStep();
        if (timeStep > 0.0) {
          simulationAccumulator = std::min(
              simulationAccumulator,
              timeStep
                  * static_cast<double>(kMaxSimulationStepsPerRenderedFrame));
          while (simulationStepsToRun < kMaxSimulationStepsPerRenderedFrame
                 && simulationAccumulator + 1e-12 >= timeStep) {
            ++simulationStepsToRun;
            simulationAccumulator -= timeStep;
          }
        }
      }
    } else {
      lastSimulationClock = ProfileAccumulator::Clock::now();
      simulationAccumulator = 0.0;
    }

    if (simulationStepsToRun > 0) {
      phaseStart = ProfileAccumulator::Clock::now();
      for (std::size_t i = 0; i < simulationStepsToRun; ++i) {
        const double timeStep = dartScene.world->getTimeStep();
        dartScene.world->step();
        profile.simulatedMs += timeStep * 1000.0;
      }
      markSimulationAdvanced(lifecycle);
      profile.simulationSteps += simulationStepsToRun;
      profile.simulationMs += elapsedMs(phaseStart);

      phaseStart = ProfileAccumulator::Clock::now();
      refreshContactDebugOverlay();
      profile.contactDebugMs += elapsedMs(phaseStart);
    }

    phaseStart = ProfileAccumulator::Clock::now();
    auto descriptors = extractRenderables(*dartScene.world);
    profile.extractionMs += elapsedMs(phaseStart);

    phaseStart = ProfileAccumulator::Clock::now();
    if (window != nullptr && selectedRenderableId != 0) {
      const Eigen::Vector3d nudge = selectedNudgeFromKeyboard(
          window, cameraController.camera, 0.035);
      if (nudge.squaredNorm() > 0.0) {
        const auto selectedDescriptor = std::find_if(
            descriptors.begin(),
            descriptors.end(),
            [&](const RenderableDescriptor& candidate) {
              return candidate.id == selectedRenderableId;
            });
        if (selectedDescriptor != descriptors.end()
            && translateRenderableAndApplyIk(
                dartScene, *selectedDescriptor, nudge)) {
          lifecycle.paused = true;
          descriptors = extractRenderables(*dartScene.world);
        }
      }
    }
    profile.interactionMs += elapsedMs(phaseStart);

    phaseStart = ProfileAccumulator::Clock::now();
    synchronizeSceneRenderables(
        *engine,
        *scene,
        descriptors,
        sceneRenderables,
        loggedUnsupportedRenderableIds,
        [&](const RenderableDescriptor& descriptor) {
          return createRenderableFromDescriptor(
              *engine, materials, materialResources.textureCache, descriptor);
        });
    bool selectedRenderableStillVisible = selectedRenderableId == 0;
    for (SceneRenderable& sceneRenderable : sceneRenderables) {
      const auto descriptor = std::find_if(
          descriptors.begin(),
          descriptors.end(),
          [&](const RenderableDescriptor& candidate) {
            return candidate.id == sceneRenderable.id;
          });
      if (descriptor == descriptors.end() || !descriptor->material.visible) {
        continue;
      }

      const bool isSelected = descriptor->id == selectedRenderableId;
      if (isSelected) {
        selectedRenderableStillVisible = true;
      }
      updateSceneRenderableFromDescriptor(
          *engine, sceneRenderable, *descriptor, isSelected);
    }
    if (!selectedRenderableStillVisible) {
      selectedRenderableId = 0;
      selectedLabel = "none";
    }
    profile.syncMs += elapsedMs(phaseStart);

    if (orbitLight) {
      const double orbitElapsedSeconds = std::chrono::duration<double>(
                                             ProfileAccumulator::Clock::now()
                                             - orbitStartClock)
                                             .count();
      updateOrbitingKeyLight(
          engine->getLightManager(),
          lights,
          orbitElapsedSeconds,
          appOptions.orbitLightPeriodSeconds);
    }

    phaseStart = ProfileAccumulator::Clock::now();
    if (window != nullptr) {
      double cursorX = 0.0;
      double cursorY = 0.0;
      glfwGetCursorPos(window, &cursorX, &cursorY);
      const bool isLeftMousePressed
          = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
      if (isLeftMousePressed && !wasLeftMousePressed) {
        leftMousePressX = cursorX;
        leftMousePressY = cursorY;
        leftMouseStartedDrag = false;
        leftMouseStartedOnPanel
            = appOptions.showUi
              && isInsideStatusPanel(cursorX, cursorY, options.guiScale);

        if (!leftMouseStartedOnPanel && selectedRenderableId != 0
            && isDragModifierDown(window)) {
          const auto selectedDescriptor = std::find_if(
              descriptors.begin(),
              descriptors.end(),
              [&](const RenderableDescriptor& candidate) {
                return candidate.id == selectedRenderableId;
              });
          if (selectedDescriptor != descriptors.end()) {
            const auto basis = makeOrbitCameraBasis(cameraController.camera);
            const PickRay ray = makePerspectivePickRay(
                cameraController.camera, cursorX, cursorY, width, height);
            const Eigen::Vector3d planePoint
                = selectedDescriptor->worldTransform.translation();
            const Eigen::Vector3d planeNormal = basis.forward;
            if (intersectPlane(ray, planePoint, planeNormal)) {
              leftMouseStartedDrag = true;
              selectedDragLastRay = ray;
              selectedDragPlanePoint = planePoint;
              selectedDragPlaneNormal = planeNormal;
              lifecycle.paused = true;
            }
          }
        }
      }
      if (isLeftMousePressed && leftMouseStartedDrag) {
        const PickRay ray = makePerspectivePickRay(
            cameraController.camera, cursorX, cursorY, width, height);
        const auto translation = computePlaneDragTranslation(
            selectedDragLastRay,
            ray,
            selectedDragPlanePoint,
            selectedDragPlaneNormal);
        if (translation && translation->squaredNorm() > 1e-12) {
          const auto selectedDescriptor = std::find_if(
              descriptors.begin(),
              descriptors.end(),
              [&](const RenderableDescriptor& candidate) {
                return candidate.id == selectedRenderableId;
              });
          if (selectedDescriptor != descriptors.end()
              && translateRenderableAndApplyIk(
                  dartScene, *selectedDescriptor, *translation)) {
            selectedDragLastRay = ray;
            lifecycle.paused = true;
            descriptors = extractRenderables(*dartScene.world);
          }
        }
      }
      if (!isLeftMousePressed && wasLeftMousePressed) {
        const double dragDistance = std::hypot(
            cursorX - leftMousePressX, cursorY - leftMousePressY);
        if (!leftMouseStartedOnPanel && !leftMouseStartedDrag
            && dragDistance < 4.0) {
          const auto hit = pickNearestRenderable(
              descriptors,
              makePerspectivePickRay(
                  cameraController.camera, cursorX, cursorY, width, height));
          if (hit) {
            selectedRenderableId = hit->id;
            const RenderableDescriptor& descriptor
                = descriptors[hit->renderableIndex];
            selectedLabel = selectionLabelForRenderable(dartScene, descriptor);
          } else {
            selectedRenderableId = 0;
            selectedLabel = "none";
          }
        }
        leftMouseStartedDrag = false;
      }
      wasLeftMousePressed = isLeftMousePressed;
    }
    profile.interactionMs += elapsedMs(phaseStart);

    phaseStart = ProfileAccumulator::Clock::now();
    refreshSelectionDebugOverlay(descriptors, selectedRenderableId);
    profile.selectionDebugMs += elapsedMs(phaseStart);

    if (appOptions.showUi) {
      phaseStart = ProfileAccumulator::Clock::now();
      updateImGuiMouseInput(window, imguiIo, width, height);
      ImGui::NewFrame();
      ImGui::SetNextWindowPos(
          {20.0f * guiScale, 20.0f * guiScale},
          ImGuiCond_Always);
      ImGui::SetNextWindowBgAlpha(0.72f);
      ImGui::Begin(
          "DART",
          nullptr,
          ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings);
      ImGui::PushTextWrapPos(
          ImGui::GetCursorPosX() + 300.0f * guiScale);
      ImGui::TextWrapped(
          "DART scene viewer: inspect renderables, shadows, and debug overlays.");
      ImGui::TextWrapped(
          "Mouse: left orbit, right/middle pan, wheel zoom, click select.");
      ImGui::TextWrapped(
          "Keys: Space pause, N step, arrows/Pg or Ctrl-left drag selected, "
          "Esc exit.");
      if (!dartScene.ikHandles.empty()) {
        ImGui::TextWrapped(
            "G1 IK: press 1-4 or click a colored target, then move it.");
      }
      ImGui::PopTextWrapPos();
      ImGui::Separator();
      ImGui::Text("scene: %s", sceneName(appOptions.scene));
      ImGui::Text("time: %.3f", dartScene.world->getTime());
      ImGui::Text(
          "contacts: %zu",
          dartScene.world->getLastCollisionResult().getNumContacts());
      ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 300.0f * guiScale);
      ImGui::Text("selected: %s", selectedLabel.c_str());
      ImGui::PopTextWrapPos();
      if (ImGui::Button(lifecycle.paused ? "Resume" : "Pause")) {
        togglePaused(lifecycle);
      }
      ImGui::SameLine();
      if (ImGui::Button("Step")) {
        requestSingleStep(lifecycle);
      }
      ImGui::SameLine();
      ImGui::Checkbox("Orbit light", &orbitLight);
      bool debugOptionsChanged = false;
      debugOptionsChanged
          |= ImGui::Checkbox("Grid", &staticDebugOptions.drawGrid);
      ImGui::SameLine();
      debugOptionsChanged
          |= ImGui::Checkbox("World", &staticDebugOptions.drawWorldFrame);
      ImGui::SameLine();
      debugOptionsChanged
          |= ImGui::Checkbox("Body", &staticDebugOptions.drawBodyFrames);
      debugOptionsChanged
          |= ImGui::Checkbox("COM", &staticDebugOptions.drawCentersOfMass);
      ImGui::SameLine();
      debugOptionsChanged |= ImGui::Checkbox(
          "Inertia", &staticDebugOptions.drawInertiaBoxes);
      ImGui::SameLine();
      debugOptionsChanged |= ImGui::Checkbox(
          "Collision", &staticDebugOptions.drawCollisionShapeBounds);
      ImGui::SameLine();
      debugOptionsChanged
          |= ImGui::Checkbox("Contacts", &contactDebugOptions.drawContacts);
      ImGui::SameLine();
      debugOptionsChanged |= ImGui::Checkbox(
          "Support", &staticDebugOptions.drawSupportPolygons);
      debugOptionsChanged |= ImGui::Checkbox(
          "Normals", &contactDebugOptions.drawContactNormals);
      ImGui::SameLine();
      debugOptionsChanged
          |= ImGui::Checkbox("Forces", &contactDebugOptions.drawContactForces);
      if (debugOptionsChanged) {
        refreshDebugOverlay();
        refreshContactDebugOverlay();
      }
      ImGui::End();
      ImGui::Render();
      updateImGuiOverlay(
          *engine,
          imguiOverlay,
          ImGui::GetDrawData(),
          static_cast<std::uint32_t>(width),
          static_cast<std::uint32_t>(height));
      profile.uiMs += elapsedMs(phaseStart);
    }

    const bool shouldCaptureScreenshot
        = shouldRequestScreenshot(options, lifecycle);

    const auto renderFrameStart = ProfileAccumulator::Clock::now();
    const bool shouldRenderFrame = renderer->beginFrame(swapChain);
    profile.beginFrameMs += elapsedMs(renderFrameStart);
    if (!shouldRenderFrame) {
      markFrameSkipped(lifecycle);
      ++profile.skippedFrames;
      if (!options.headless || !renderer->shouldRenderFrame()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        const double frameMs = elapsedMs(frameStart);
        profile.frameMs += frameMs;
        profile.maxFrameMs = std::max(profile.maxFrameMs, frameMs);
        ++profile.frames;
        continue;
      }
      // Filament allows callers to ignore a pacing-only false return. Headless
      // software GL can report skips almost every frame, so keep deterministic
      // offscreen captures while still skipping backend failures above.
    }

    phaseStart = ProfileAccumulator::Clock::now();
    renderer->render(view);
    if (appOptions.showUi) {
      renderer->render(imguiOverlay.view);
    }
    if (shouldCaptureScreenshot) {
      requestScreenshot(
          *renderer,
          screenshotCapture,
          static_cast<std::uint32_t>(width),
          static_cast<std::uint32_t>(height));
      markScreenshotRequested(lifecycle);
    }
    renderer->endFrame();
    ++profile.renderedFrames;
    const double renderMs = elapsedMs(phaseStart);
    profile.renderMs += renderMs;
    profile.maxRenderMs = std::max(profile.maxRenderMs, renderMs);

    const double frameMs = elapsedMs(frameStart);
    profile.frameMs += frameMs;
    profile.maxFrameMs = std::max(profile.maxFrameMs, frameMs);
    ++profile.frames;
    markFrameRendered(lifecycle);
    if (shouldStopAfterFrame(options, lifecycle)) {
      break;
    }
  }

  if (!options.screenshotPath.empty() && !lifecycle.screenshotRequested) {
    std::cerr << "No rendered frame was available for screenshot capture\n";
  }
  if (lifecycle.screenshotRequested) {
    const auto screenshotWaitStart = ProfileAccumulator::Clock::now();
    screenshotSucceeded = waitForScreenshot(*engine, screenshotCapture);
    profile.screenshotWaitMs += elapsedMs(screenshotWaitStart);
    if (screenshotSucceeded) {
      const auto screenshotSaveStart = ProfileAccumulator::Clock::now();
      saveScreenshot(screenshotCapture, options.screenshotPath);
      profile.screenshotSaveMs += elapsedMs(screenshotSaveStart);
    } else {
      std::cerr << "Timed out waiting for Filament screenshot readback\n";
    }
  }
  if (options.maxFrames >= 0) {
    std::cout << "Final contacts: "
              << dartScene.world->getLastCollisionResult().getNumContacts()
              << "\n";
  }
  if (appOptions.profile) {
    printProfile(profile);
    DART_PROFILE_TEXT_DUMP();
  }

  destroyImGuiOverlay(*engine, imguiOverlay);
  ImGui::DestroyContext();

  scene->remove(lights.key);
  scene->remove(lights.fill);
  scene->remove(lights.rim);
  scene->setIndirectLight(nullptr);
  scene->setSkybox(nullptr);
  view->setColorGrading(nullptr);
  for (const SceneRenderable& sceneRenderable : sceneRenderables) {
    scene->remove(sceneRenderable.renderable.entity);
  }
  if (debugOverlay) {
    scene->remove(debugOverlay->entity);
  }
  if (contactDebugOverlay) {
    scene->remove(contactDebugOverlay->entity);
  }
  if (selectionDebugOverlay) {
    scene->remove(selectionDebugOverlay->entity);
  }
  engine->destroy(lights.key);
  engine->destroy(lights.fill);
  engine->destroy(lights.rim);
  EntityManager::get().destroy(lights.key);
  EntityManager::get().destroy(lights.fill);
  EntityManager::get().destroy(lights.rim);
  engine->destroy(indirectLight);
  engine->destroy(skybox);
  engine->destroy(colorGrading);
  for (SceneRenderable& sceneRenderable : sceneRenderables) {
    destroyRenderable(*engine, sceneRenderable.renderable);
  }
  if (contactDebugOverlay) {
    destroyRenderable(*engine, *contactDebugOverlay);
  }
  if (selectionDebugOverlay) {
    destroyRenderable(*engine, *selectionDebugOverlay);
  }
  if (debugOverlay) {
    destroyRenderable(*engine, *debugOverlay);
  }
  destroyMaterialResources(*engine, materialResources);
  engine->destroyCameraComponent(cameraEntity);
  EntityManager::get().destroy(cameraEntity);
  engine->destroy(view);
  engine->destroy(scene);
  engine->destroy(renderer);
  engine->destroy(swapChain);
  filament::Engine::destroy(&engine);

  if (window != nullptr) {
    glfwDestroyWindow(window);
    glfwTerminate();
  }
  return screenshotSucceeded ? 0 : 1;
}
