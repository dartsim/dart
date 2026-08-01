/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "../memory_diagnostics.hpp"

#include <dart/gui/osg/IncludeImGui.hpp>

#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/constraint/WeldJointConstraint.hpp>

#include <dart/collision/CollisionGroup.hpp>

#include <dart/dynamics/dynamics.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>

namespace dart::examples::demos {
namespace {

//==============================================================================
ProcessMemoryReading makeSyntheticProcessReading()
{
  ProcessMemoryReading reading;
  reading.residentBytes = 12u * 1024u * 1024u;
  reading.peakResidentBytes = 18u * 1024u * 1024u;
  reading.platform = "test";
  reading.source = "synthetic process probe";
  reading.limitation = "synthetic test reading";
  return reading;
}

//==============================================================================
TEST(MemoryDiagnostics, SyntheticWorldReportsExactGraphAndScratchCounts)
{
  using dynamics::BodyNode;
  using dynamics::BoxShape;
  using dynamics::CollisionAspect;
  using dynamics::FreeJoint;
  using dynamics::GenericJoint;
  using dynamics::RevoluteJoint;
  using dynamics::ShapeNode;
  using dynamics::Skeleton;
  using dynamics::SoftBodyNode;
  using dynamics::SoftBodyNodeHelper;
  using dynamics::SphereShape;
  using dynamics::VisualAspect;
  using dynamics::WeldJoint;

  auto world = simulation::World::create("memory_diagnostics_test");

  auto rigidSkeleton = Skeleton::create("rigid");
  auto rootPair = rigidSkeleton->createJointAndBodyNodePair<WeldJoint>(nullptr);
  auto* rootBody = rootPair.second;
  auto childPair
      = rigidSkeleton->createJointAndBodyNodePair<RevoluteJoint>(rootBody);
  auto* childBody = childPair.second;

  const auto sharedBox
      = std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.3, 0.4));
  const auto uniqueSphere = std::make_shared<SphereShape>(0.1);
  ASSERT_NE(
      (rootBody->createShapeNodeWith<VisualAspect, CollisionAspect>(sharedBox)),
      nullptr);
  ASSERT_NE(rootBody->createShapeNodeWith<VisualAspect>(uniqueSphere), nullptr);
  ASSERT_NE(
      (childBody->createShapeNodeWith<VisualAspect, CollisionAspect>(
          sharedBox)),
      nullptr);

  auto softSkeleton = Skeleton::create("soft");
  GenericJoint<math::SE3Space>::Properties softJointProperties(
      std::string("soft_joint"));
  BodyNode::Properties softBodyProperties(
      BodyNode::AspectProperties("soft_body"));
  softBodyProperties.mInertia.setMass(1.0);
  SoftBodyNode::Properties softProperties(
      softBodyProperties,
      SoftBodyNodeHelper::makeSinglePointMassProperties(1.0, 0.0, 0.0, 0.0));
  auto softPair
      = softSkeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
          nullptr, softJointProperties, softProperties);
  auto* softBody = softPair.second;
  ASSERT_NE(softBody, nullptr);
  ASSERT_NE(softBody->createShapeNodeWith<VisualAspect>(sharedBox), nullptr);

  world->addSkeleton(rigidSkeleton);
  world->addSkeleton(softSkeleton);
  auto manualConstraint
      = std::make_shared<constraint::WeldJointConstraint>(rootBody);
  world->getConstraintSolver()->addConstraint(manualConstraint);

  auto& memoryManager = world->getMemoryManager();
  auto& frameAllocator = memoryManager.getFrameAllocator();
  ASSERT_NE(frameAllocator.allocate(96u), nullptr);
  const std::size_t expectedFrameUsed = frameAllocator.used();
  auto& poolAllocator = memoryManager.getPoolAllocator();
  void* poolAllocation = poolAllocator.allocate(64u);
  ASSERT_NE(poolAllocation, nullptr);
  const int expectedPoolBlocks = poolAllocator.getNumAllocatedMemoryBlocks();

  DiagnosticSnapshot snapshot
      = collectMemoryDiagnostics(world, 17u, makeSyntheticProcessReading());
  poolAllocator.deallocate(poolAllocation, 64u);

  const auto requireValue = [&snapshot](const char* key) -> double {
    const DiagnosticMetric* metric = findMetric(snapshot, key);
    EXPECT_NE(metric, nullptr) << key;
    if (metric == nullptr) {
      return 0.0;
    }
    EXPECT_TRUE(metric->value.has_value()) << key;
    return metric->value.value_or(0.0);
  };

  EXPECT_EQ(snapshot.engine, "DART 6");
  EXPECT_EQ(snapshot.platform, "test");
  EXPECT_EQ(snapshot.generation, 17u);
  EXPECT_DOUBLE_EQ(requireValue("world.count"), 1.0);
  EXPECT_DOUBLE_EQ(requireValue("world.skeleton_count"), 2.0);
  EXPECT_DOUBLE_EQ(requireValue("world.body_node_count"), 3.0);
  EXPECT_DOUBLE_EQ(requireValue("world.rigid_body_node_count"), 2.0);
  EXPECT_DOUBLE_EQ(requireValue("world.soft_body_node_count"), 1.0);
  EXPECT_DOUBLE_EQ(requireValue("world.point_mass_count"), 1.0);
  EXPECT_DOUBLE_EQ(requireValue("world.joint_count"), 3.0);
  EXPECT_DOUBLE_EQ(requireValue("world.dof_count"), 7.0);
  // SoftBodyNode creates its own SoftMeshShape node in addition to the four
  // ShapeNodes explicitly attached above.
  EXPECT_DOUBLE_EQ(requireValue("world.shape_node_count"), 5.0);
  EXPECT_DOUBLE_EQ(requireValue("world.simple_frame_count"), 0.0);
  EXPECT_DOUBLE_EQ(requireValue("world.unique_shape_count"), 3.0);
  EXPECT_DOUBLE_EQ(requireValue("world.last_contact_count"), 0.0);
  EXPECT_DOUBLE_EQ(requireValue("world.manual_constraint_count"), 1.0);
  EXPECT_DOUBLE_EQ(
      requireValue("world.collision_shape_frame_count"),
      static_cast<double>(world->getConstraintSolver()
                              ->getCollisionGroup()
                              ->getNumShapeFrames()));
  EXPECT_DOUBLE_EQ(
      requireValue("world.scratch.frame_used_bytes"),
      static_cast<double>(expectedFrameUsed));
  EXPECT_DOUBLE_EQ(
      requireValue("world.scratch.pool_block_count"),
      static_cast<double>(expectedPoolBlocks));
  EXPECT_DOUBLE_EQ(
      requireValue(kProcessResidentBytesKey), 12.0 * 1024.0 * 1024.0);

  const DiagnosticMetric* shallowFloor
      = findMetric(snapshot, "world.object_shallow_floor_bytes");
  ASSERT_NE(shallowFloor, nullptr);
  ASSERT_TRUE(shallowFloor->value);
  EXPECT_GT(*shallowFloor->value, 0.0);
  EXPECT_EQ(shallowFloor->quality, MetricQuality::Estimate);

  const DiagnosticMetric* locality
      = findMetric(snapshot, "world.locality.address_page_count");
  ASSERT_NE(locality, nullptr);
  ASSERT_TRUE(locality->value);
  EXPECT_GT(*locality->value, 0.0);
  EXPECT_EQ(locality->quality, MetricQuality::Proxy);
  EXPECT_NE(locality->limitation.find("cache misses"), std::string::npos);

  const DiagnosticMetric* activeAllocations
      = findMetric(snapshot, "allocation.active_count");
  ASSERT_NE(activeAllocations, nullptr);
  EXPECT_FALSE(activeAllocations->value);

  const DiagnosticMetric* manualConstraints
      = findMetric(snapshot, "world.manual_constraint_count");
  ASSERT_NE(manualConstraints, nullptr);
  EXPECT_NE(manualConstraints->label.find("Manually"), std::string::npos);

  ASSERT_FALSE(snapshot.allocatorMemoryMap.empty());
  const auto findAllocatorRegion = [&snapshot](const std::string& prefix) {
    return std::find_if(
        snapshot.allocatorMemoryMap.begin(),
        snapshot.allocatorMemoryMap.end(),
        [&prefix](const MemoryMapRegion& region) {
          return region.id.find(prefix) == 0u;
        });
  };
  EXPECT_NE(
      findAllocatorRegion("free-list-"), snapshot.allocatorMemoryMap.end());
  const auto frameRegion = findAllocatorRegion("frame-primary-");
  ASSERT_NE(frameRegion, snapshot.allocatorMemoryMap.end());
  const auto frameSpanWithLabel = [&frameRegion](const std::string& label) {
    return std::find_if(
        frameRegion->spans.begin(),
        frameRegion->spans.end(),
        [&label](const MemoryMapSpan& span) { return span.label == label; });
  };
  const auto consumed
      = frameSpanWithLabel("frame cursor-consumed backing extent");
  ASSERT_NE(consumed, frameRegion->spans.end());
  EXPECT_EQ(consumed->storageState, MemoryStorageState::Allocated);
  EXPECT_EQ(consumed->dataCategory, MemoryDataCategory::Scratch);
  const auto available
      = frameSpanWithLabel("frame reserved available backing extent");
  ASSERT_NE(available, frameRegion->spans.end());
  EXPECT_EQ(available->storageState, MemoryStorageState::Reserved);
  EXPECT_EQ(available->dataCategory, MemoryDataCategory::None);
  for (const auto& region : snapshot.allocatorMemoryMap) {
    EXPECT_GT(region.sizeBytes, 0u);
    EXPECT_EQ(region.quality, MetricQuality::Measured);
    EXPECT_NE(region.limitation.find("classic DART 6"), std::string::npos);
    EXPECT_EQ(region.id.find("0x"), std::string::npos);
    EXPECT_EQ(region.label.find("0x"), std::string::npos);
    EXPECT_EQ(region.source.find("0x"), std::string::npos);
    EXPECT_TRUE(region.observations.empty());
    for (const auto& span : region.spans) {
      EXPECT_LE(span.offsetBytes, region.sizeBytes);
      EXPECT_LE(span.sizeBytes, region.sizeBytes - span.offsetBytes);
      EXPECT_EQ(span.quality, MetricQuality::Measured);
      EXPECT_EQ(span.extentKind, MemoryExtentKind::ExactByteRange);
      if (span.storageState == MemoryStorageState::Free
          || span.storageState == MemoryStorageState::Padding) {
        EXPECT_EQ(span.dataCategory, MemoryDataCategory::None);
      }
    }
  }

  ASSERT_FALSE(snapshot.objectAddressAtlas.empty());
  const auto atlasHasCategory = [&snapshot](MemoryDataCategory category) {
    for (const auto& region : snapshot.objectAddressAtlas) {
      for (const auto& span : region.observations) {
        if (span.dataCategory == category) {
          return true;
        }
      }
    }
    return false;
  };
  EXPECT_TRUE(atlasHasCategory(MemoryDataCategory::Model));
  EXPECT_TRUE(atlasHasCategory(MemoryDataCategory::State));
  EXPECT_TRUE(atlasHasCategory(MemoryDataCategory::Geometry));
  EXPECT_TRUE(atlasHasCategory(MemoryDataCategory::ConstraintSolver));
  const auto findAtlasSpan = [&snapshot](const std::string& labelFragment) {
    for (const auto& region : snapshot.objectAddressAtlas) {
      for (const auto& span : region.observations) {
        if (span.label.find(labelFragment) != std::string::npos) {
          return &span;
        }
      }
    }
    return static_cast<const MemoryMapSpan*>(nullptr);
  };
  EXPECT_NE(findAtlasSpan("Skeleton"), nullptr);
  EXPECT_NE(findAtlasSpan("BodyNode"), nullptr);
  EXPECT_NE(findAtlasSpan("DegreeOfFreedom"), nullptr);
  EXPECT_NE(findAtlasSpan("ShapeNode"), nullptr);
  EXPECT_NE(findAtlasSpan("PointMass"), nullptr);
  const MemoryMapSpan* jointPoint = findAtlasSpan("Joint address point");
  const MemoryMapSpan* shapePoint = findAtlasSpan("Shape address point");
  const MemoryMapSpan* constraintPoint
      = findAtlasSpan("ConstraintBase address point");
  ASSERT_NE(jointPoint, nullptr);
  ASSERT_NE(shapePoint, nullptr);
  ASSERT_NE(constraintPoint, nullptr);
  EXPECT_EQ(jointPoint->extentKind, MemoryExtentKind::AddressPoint);
  EXPECT_EQ(shapePoint->extentKind, MemoryExtentKind::AddressPoint);
  EXPECT_EQ(constraintPoint->extentKind, MemoryExtentKind::AddressPoint);
  EXPECT_EQ(jointPoint->quality, MetricQuality::Proxy);
  for (const auto& region : snapshot.objectAddressAtlas) {
    ASSERT_TRUE(region.pageSizeBytes);
    EXPECT_GT(*region.pageSizeBytes, 0u);
    EXPECT_EQ(region.sizeBytes % *region.pageSizeBytes, 0u);
    EXPECT_FALSE(region.pageSizeEvidence.empty());
    EXPECT_NE(region.quality, MetricQuality::Measured);
    EXPECT_NE(region.limitation.find("lower bounds"), std::string::npos);
    EXPECT_EQ(region.id.find("0x"), std::string::npos);
    EXPECT_EQ(region.label.find("0x"), std::string::npos);
    EXPECT_EQ(region.source.find("0x"), std::string::npos);
    std::size_t partitionCursor = 0;
    for (const auto& span : region.spans) {
      EXPECT_EQ(span.offsetBytes, partitionCursor);
      ASSERT_LE(span.sizeBytes, region.sizeBytes - partitionCursor);
      partitionCursor += span.sizeBytes;
      EXPECT_NE(span.storageState, MemoryStorageState::Observed);
    }
    EXPECT_EQ(partitionCursor, region.sizeBytes);
    ASSERT_FALSE(region.observations.empty());
    for (const auto& observation : region.observations) {
      ASSERT_LE(observation.offsetBytes, region.sizeBytes);
      EXPECT_LE(
          observation.sizeBytes, region.sizeBytes - observation.offsetBytes);
      EXPECT_EQ(observation.storageState, MemoryStorageState::Observed);
      EXPECT_NE(observation.dataCategory, MemoryDataCategory::None);
    }
  }

  world.reset();
  EXPECT_DOUBLE_EQ(requireValue("world.body_node_count"), 3.0);
}

//==============================================================================
TEST(MemoryDiagnostics, MissingWorldKeepsUnavailableValuesDistinctFromZero)
{
  const simulation::WorldPtr world;
  const DiagnosticSnapshot snapshot
      = collectMemoryDiagnostics(world, 4u, makeSyntheticProcessReading());

  const DiagnosticMetric* worldCount = findMetric(snapshot, "world.count");
  ASSERT_NE(worldCount, nullptr);
  ASSERT_TRUE(worldCount->value);
  EXPECT_DOUBLE_EQ(*worldCount->value, 0.0);

  const DiagnosticMetric* activeBytes
      = findMetric(snapshot, "allocation.active_bytes");
  ASSERT_NE(activeBytes, nullptr);
  EXPECT_FALSE(activeBytes->value);
  EXPECT_FALSE(activeBytes->limitation.empty());
  EXPECT_TRUE(snapshot.allocatorMemoryMap.empty());
  EXPECT_TRUE(snapshot.objectAddressAtlas.empty());
  EXPECT_FALSE(snapshot.guidance.empty());
}

//==============================================================================
TEST(MemoryDiagnostics, DenseMapStaysBelowTheOpenGL2DrawIndexLimit)
{
  constexpr std::size_t kRegionBytes = 4096u;
  constexpr std::size_t kObservationCount = 512u;
  MemoryMapRegion region;
  region.id = "dense";
  region.label = "Dense synthetic region";
  region.sizeBytes = kRegionBytes;
  region.quality = MetricQuality::Proxy;
  region.scope = "synthetic renderer stress";
  region.source = "unit test";
  region.limitation = "synthetic renderer stress";
  region.spans.push_back(MemoryMapSpan{
      0u,
      kRegionBytes,
      MemoryStorageState::Unobserved,
      MemoryDataCategory::None,
      MemoryExtentKind::UnobservedRange,
      "background",
      MetricQuality::Proxy,
      "synthetic background"});
  region.observations.assign(
      kObservationCount,
      MemoryMapSpan{
          0u,
          kRegionBytes,
          MemoryStorageState::Observed,
          MemoryDataCategory::Model,
          MemoryExtentKind::ShallowLowerBound,
          "overlapping observation",
          MetricQuality::Estimate,
          "synthetic overlap"});

  DiagnosticSnapshot snapshot;
  snapshot.generation = 1u;
  snapshot.allocatorMemoryMap.push_back(std::move(region));
  DiagnosticSession session([snapshot] { return snapshot; }, 1u);
  session.setEnabled(true);
  ASSERT_TRUE(session.captureNow(0.0));

  ImGuiContext* previousContext = ImGui::GetCurrentContext();
  ImGuiContext* testContext = ImGui::CreateContext();
  ImGui::SetCurrentContext(testContext);
  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize = ImVec2(1600.0f, 1400.0f);
  io.DeltaTime = 1.0f / 60.0f;
  io.IniFilename = nullptr;
  unsigned char* pixels = nullptr;
  int textureWidth = 0;
  int textureHeight = 0;
  io.Fonts->GetTexDataAsRGBA32(&pixels, &textureWidth, &textureHeight);

  struct FrameDrawStats
  {
    bool windowVisible{false};
    int drawListCount{0};
    int maximumVertices{0};
    int lastDrawListVertices{0};
  };
  const auto renderFrame = [&](const ImVec2& mousePosition) {
    io.MousePos = mousePosition;
    ImGui::NewFrame();
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
    ImGui::SetNextWindowSize(io.DisplaySize);
    const bool windowVisible = ImGui::Begin(
        "Memory diagnostics draw-list bound",
        nullptr,
        ImGuiWindowFlags_NoSavedSettings);
    if (windowVisible) {
      renderMemoryDiagnostics(session, 0.0, 1.0);
    }
    ImGui::End();
    ImGui::Render();

    FrameDrawStats stats;
    stats.windowVisible = windowVisible;
    const ImDrawData* drawData = ImGui::GetDrawData();
    if (drawData != nullptr) {
      stats.drawListCount = drawData->CmdListsCount;
      for (int index = 0; index < drawData->CmdListsCount; ++index) {
        stats.maximumVertices = std::max(
            stats.maximumVertices, drawData->CmdLists[index]->VtxBuffer.Size);
      }
      if (drawData->CmdListsCount > 0) {
        stats.lastDrawListVertices
            = drawData->CmdLists[drawData->CmdListsCount - 1]->VtxBuffer.Size;
      }
    }
    return stats;
  };

  const FrameDrawStats baseline = renderFrame(ImVec2(-1000.0f, -1000.0f));
  int maximumDrawListVertices = baseline.maximumVertices;
  bool tooltipExercised = false;
  for (float mouseY = 180.0f; mouseY <= 800.0f; mouseY += 8.0f) {
    const FrameDrawStats hovered = renderFrame(ImVec2(300.0f, mouseY));
    maximumDrawListVertices
        = std::max(maximumDrawListVertices, hovered.maximumVertices);
    // A map-cell tooltip contains the bounded evidence list and is therefore
    // materially larger than the short checkbox/metric tooltips encountered
    // while scanning down the window.
    if (hovered.drawListCount > baseline.drawListCount
        && hovered.lastDrawListVertices > 2048) {
      tooltipExercised = true;
      break;
    }
  }

  EXPECT_TRUE(baseline.windowVisible);
  EXPECT_GT(baseline.drawListCount, 0);
  EXPECT_GT(maximumDrawListVertices, 32 * 1024)
      << "the dense synthetic map did not exercise the renderer cap";
  EXPECT_TRUE(tooltipExercised)
      << "the dense synthetic cell tooltip was not exercised";
  EXPECT_LT(maximumDrawListVertices, 60 * 1024);
  EXPECT_LT(
      maximumDrawListVertices,
      static_cast<std::uintmax_t>(std::numeric_limits<ImDrawIdx>::max()));

  ImGui::DestroyContext(testContext);
  ImGui::SetCurrentContext(previousContext);
}

} // namespace
} // namespace dart::examples::demos
