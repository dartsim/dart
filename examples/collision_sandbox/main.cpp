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

#include "pair_registry.hpp"

#include <dart/gui/application.hpp>
#include <dart/gui/debug.hpp>
#include <dart/gui/gizmo.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/collision/native/broad_phase/broad_phase.hpp>
#include <dart/collision/native/collision_world.hpp>
#include <dart/collision/native/narrow_phase/narrow_phase.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cmath>

namespace collision = dart::collision::native;
namespace dynamics = dart::dynamics;
namespace gui = dart::gui;
namespace sandbox = dart::examples::collision_sandbox;
namespace simulation = dart::simulation;

namespace {

using ShapeParameters = sandbox::ShapeParameters;
using ShapeType = collision::ShapeType;

struct BroadPhaseOption
{
  collision::BroadPhaseType type;
  std::string_view id;
  std::string_view label;
};

constexpr std::array<BroadPhaseOption, 4> kBroadPhaseOptions{{
    {collision::BroadPhaseType::AabbTree, "aabb_tree", "AABB Tree"},
    {collision::BroadPhaseType::SpatialHash, "spatial_hash", "Spatial Hash"},
    {collision::BroadPhaseType::SweepAndPrune, "sweep_and_prune", "Sweep"},
    {collision::BroadPhaseType::BruteForce, "brute_force", "Brute Force"},
}};

constexpr std::array<std::string_view, 3> kPairCoverageColumns{{
    "Pair",
    "Support",
    "Mode",
}};

constexpr double kObjectGizmoSize = 0.65;
constexpr double kContactObjectAlpha = 0.45;
constexpr double kContactMarkerRadius = 0.018;
constexpr double kDistanceMarkerRadius = 0.04;

Eigen::Vector4d rgba(double r, double g, double b, double a = 1.0)
{
  return {r, g, b, a};
}

struct ContactRow
{
  std::size_t manifoldIndex = 0;
  std::size_t contactIndex = 0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double depth = 0.0;
};

struct QuerySummary
{
  const sandbox::PairCase* pair = nullptr;
  bool hit = false;
  bool distanceValid = false;
  double distance = 0.0;
  std::size_t numManifolds = 0;
  std::size_t numContacts = 0;
  std::size_t broadPhaseObjects = 0;
  std::size_t broadPhaseNodes = 0;
  std::size_t broadPhaseLeaves = 0;
  std::size_t broadPhaseInternals = 0;
  std::size_t broadPhaseCandidatePairs = 0;
  std::size_t broadPhaseCells = 0;
  std::size_t broadPhaseEndpoints = 0;
  bool broadPhaseHasTreeTopology = false;
  bool broadPhaseHasSpatialHashCells = false;
  bool broadPhaseHasSweepEndpoints = false;
  bool pairFiltered = false;
  std::vector<ContactRow> contacts;
};

struct SandboxState
{
  simulation::WorldPtr world = simulation::World::create("collision_sandbox");
  dynamics::SimpleFramePtr objectAFrame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), "object_a");
  dynamics::SimpleFramePtr objectBFrame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), "object_b");
  std::vector<dynamics::SimpleFramePtr> debugFrames;
  std::size_t pairCaseIndex = 0;
  std::size_t broadPhaseIndex = 0;
  ShapeParameters objectAParams
      = sandbox::defaultShapeParameters(ShapeType::Sphere);
  ShapeParameters objectBParams
      = sandbox::defaultShapeParameters(ShapeType::Sphere);
  double maxContacts = 16.0;
  bool showAabbs = true;
  bool showBroadPhase = true;
  bool showBvhNodes = true;
  bool showBvhEdges = true;
  bool showCandidatePairs = true;
  bool showSpatialHashCells = true;
  bool showSweepEndpoints = true;
  bool showDebugLabels = true;
  bool filterPair = false;
  bool dirty = true;
  QuerySummary summary;
  std::vector<gui::DebugLabelDescriptor> debugLabels;
};

void addDebugLabel(
    SandboxState& state,
    std::string text,
    const Eigen::Vector3d& position,
    const Eigen::Vector4d& color)
{
  if (!position.allFinite() || text.empty()) {
    return;
  }

  gui::DebugLabelDescriptor label;
  label.text = std::move(text);
  label.position = position;
  label.rgba = color;
  state.debugLabels.push_back(std::move(label));
}

const sandbox::PairCase& selectedPair(const SandboxState& state)
{
  const auto pairs = sandbox::pairCases();
  return pairs[std::min(state.pairCaseIndex, pairs.size() - 1)];
}

const BroadPhaseOption& selectedBroadPhase(const SandboxState& state)
{
  return kBroadPhaseOptions[std::min(
      state.broadPhaseIndex, kBroadPhaseOptions.size() - 1)];
}

std::string_view pairCoverageMode(const sandbox::PairCase& pair)
{
  switch (pair.status) {
    case sandbox::PairStatus::Contact:
      return "dedicated native contact path";
    case sandbox::PairStatus::AdaptedFallback:
      return "native adapter contact path";
    case sandbox::PairStatus::DistanceOnly:
      return "native distance query only";
    case sandbox::PairStatus::Unsupported:
      return "unsupported placeholder";
  }
  return "unknown";
}

const BroadPhaseOption* findBroadPhaseOption(std::string_view id)
{
  const auto it = std::find_if(
      kBroadPhaseOptions.begin(),
      kBroadPhaseOptions.end(),
      [id](const BroadPhaseOption& option) { return option.id == id; });
  if (it == kBroadPhaseOptions.end()) {
    return nullptr;
  }
  return &*it;
}

void attachObjectFrames(SandboxState& state)
{
  if (state.world->getSimpleFrame(state.objectAFrame->getName()) == nullptr) {
    state.world->addSimpleFrame(state.objectAFrame);
  }
  if (state.world->getSimpleFrame(state.objectBFrame->getName()) == nullptr) {
    state.world->addSimpleFrame(state.objectBFrame);
  }
}

void resetPairControls(SandboxState& state)
{
  const sandbox::PairPose pose = sandbox::defaultPairPose(selectedPair(state));
  state.objectAFrame->setTransform(pose.transformA);
  state.objectBFrame->setTransform(pose.transformB);
  state.objectAParams
      = sandbox::defaultShapeParameters(selectedPair(state).shapeA);
  state.objectBParams
      = sandbox::defaultShapeParameters(selectedPair(state).shapeB);
  state.maxContacts = 16.0;
  state.dirty = true;
}

std::string formatVec3(const Eigen::Vector3d& value)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << value.x() << " " << value.y()
      << " " << value.z();
  return out.str();
}

std::string formatFramePosition(const dynamics::SimpleFramePtr& frame)
{
  if (frame == nullptr) {
    return "unavailable";
  }
  return formatVec3(frame->getWorldTransform().translation());
}

Eigen::Vector4d objectColor(
    const sandbox::PairCase& pair,
    bool objectA,
    bool hit,
    bool filtered,
    bool showContactOverlay)
{
  const double alpha = showContactOverlay ? kContactObjectAlpha : 1.0;
  if (filtered) {
    return objectA ? rgba(0.62, 0.62, 0.68) : rgba(0.48, 0.5, 0.56);
  }
  if (pair.status == sandbox::PairStatus::Unsupported) {
    return rgba(0.42, 0.44, 0.46);
  }
  if (pair.status == sandbox::PairStatus::DistanceOnly) {
    return objectA ? rgba(0.1, 0.75, 0.9) : rgba(0.95, 0.85, 0.25);
  }
  if (hit) {
    return objectA ? rgba(0.95, 0.18, 0.18, alpha)
                   : rgba(0.95, 0.18, 0.78, alpha);
  }
  if (pair.status == sandbox::PairStatus::AdaptedFallback) {
    return objectA ? rgba(0.95, 0.52, 0.16) : rgba(0.18, 0.75, 0.9);
  }
  return objectA ? rgba(0.18, 0.38, 0.95) : rgba(0.18, 0.75, 0.34);
}

std::shared_ptr<dynamics::Shape> makeVisualShape(
    ShapeType type, const ShapeParameters& params)
{
  switch (type) {
    case ShapeType::Sphere:
      return std::make_shared<dynamics::SphereShape>(params.radius);
    case ShapeType::Box:
      return std::make_shared<dynamics::BoxShape>(params.halfExtents * 2.0);
    case ShapeType::Capsule:
      return std::make_shared<dynamics::CapsuleShape>(
          params.radius, params.height);
    case ShapeType::Cylinder:
      return std::make_shared<dynamics::CylinderShape>(
          params.radius, params.height);
    case ShapeType::Plane:
      return std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(
          params.halfExtents.x() * 2.0,
          params.halfExtents.y() * 2.0,
          params.halfExtents.z() * 2.0));
    case ShapeType::Convex:
      return std::make_shared<dynamics::SphereShape>(params.radius * 0.87);
    case ShapeType::Mesh:
      return std::make_shared<dynamics::BoxShape>(params.halfExtents * 2.0);
    case ShapeType::Sdf:
      return std::make_shared<dynamics::SphereShape>(params.radius);
    case ShapeType::Compound:
      return std::make_shared<dynamics::BoxShape>(
          Eigen::Vector3d(0.7, 0.38, 0.28) * (params.radius / 0.45));
  }
  return std::make_shared<dynamics::SphereShape>(params.radius);
}

void addLine(
    SandboxState& state,
    std::string name,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const Eigen::Vector4d& color,
    float thickness = 2.0f)
{
  if (!from.allFinite() || !to.allFinite()) {
    return;
  }
  if ((to - from).squaredNorm() < 1e-12) {
    return;
  }

  const Eigen::Vector3d segment = to - from;
  const double length = segment.norm();
  if (!std::isfinite(length) || length < 1e-9) {
    return;
  }

  const Eigen::Vector3d direction = segment / length;
  const Eigen::Quaterniond rotation
      = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), direction);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = rotation.toRotationMatrix();
  tf.translation() = (from + to) * 0.5;

  const double radius = std::clamp<double>(thickness * 0.003, 0.003, 0.02);
  auto frame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), std::move(name), tf);
  frame->setShape(std::make_shared<dynamics::CylinderShape>(radius, length));
  gui::applyDebugVisualStyle(*frame, color);
  state.world->addSimpleFrame(frame);
  state.debugFrames.push_back(std::move(frame));
}

void addPointMarker(
    SandboxState& state,
    std::string name,
    const Eigen::Vector3d& point,
    const Eigen::Vector4d& color,
    double radius)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = point;
  auto frame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), std::move(name), tf);
  frame->setShape(std::make_shared<dynamics::SphereShape>(radius));
  gui::applyDebugVisualStyle(*frame, color);
  state.world->addSimpleFrame(frame);
  state.debugFrames.push_back(std::move(frame));
}

void addAabbLines(
    SandboxState& state,
    const std::string& prefix,
    const collision::Aabb& aabb,
    const Eigen::Vector4d& color)
{
  if (!aabb.min.allFinite() || !aabb.max.allFinite()) {
    return;
  }

  const Eigen::Vector3d min = aabb.min.cwiseMin(aabb.max);
  const Eigen::Vector3d max = aabb.min.cwiseMax(aabb.max);
  const std::array<Eigen::Vector3d, 8> corners{
      Eigen::Vector3d(min.x(), min.y(), min.z()),
      Eigen::Vector3d(max.x(), min.y(), min.z()),
      Eigen::Vector3d(max.x(), max.y(), min.z()),
      Eigen::Vector3d(min.x(), max.y(), min.z()),
      Eigen::Vector3d(min.x(), min.y(), max.z()),
      Eigen::Vector3d(max.x(), min.y(), max.z()),
      Eigen::Vector3d(max.x(), max.y(), max.z()),
      Eigen::Vector3d(min.x(), max.y(), max.z())};
  const std::array<std::pair<std::size_t, std::size_t>, 12> edges{
      std::pair<std::size_t, std::size_t>{0, 1},
      {1, 2},
      {2, 3},
      {3, 0},
      {4, 5},
      {5, 6},
      {6, 7},
      {7, 4},
      {0, 4},
      {1, 5},
      {2, 6},
      {3, 7}};

  for (std::size_t i = 0; i < edges.size(); ++i) {
    addLine(
        state,
        prefix + ".edge." + std::to_string(i),
        corners[edges[i].first],
        corners[edges[i].second],
        color,
        1.0f);
  }
}

Eigen::Vector4d sweepAxisColor(int axis, bool isMin)
{
  const double intensity = isMin ? 0.95 : 0.55;
  switch (axis) {
    case 0:
      return rgba(intensity, 0.16, 0.16);
    case 1:
      return rgba(0.16, intensity, 0.16);
    case 2:
      return rgba(0.16, 0.35, intensity);
  }
  return rgba(0.85, 0.85, 0.85);
}

void addSweepEndpointOverlay(
    SandboxState& state, const collision::BroadPhaseDebugSnapshot& snapshot)
{
  if (!snapshot.hasSweepEndpoints || snapshot.endpoints.empty()) {
    return;
  }

  for (int axis = 0; axis < 3; ++axis) {
    std::vector<const collision::BroadPhaseDebugEndpoint*> endpoints;
    for (const auto& endpoint : snapshot.endpoints) {
      if (endpoint.axis == axis && std::isfinite(endpoint.value)) {
        endpoints.push_back(&endpoint);
      }
    }

    if (endpoints.empty()) {
      continue;
    }

    double minValue = endpoints.front()->value;
    double maxValue = endpoints.front()->value;
    for (const auto* endpoint : endpoints) {
      minValue = std::min(minValue, endpoint->value);
      maxValue = std::max(maxValue, endpoint->value);
    }

    const double span = std::max(maxValue - minValue, 1.0);
    const double railY = -1.15 - static_cast<double>(axis) * 0.16;
    constexpr double railZ = -0.55;
    constexpr double railMinX = -1.25;
    constexpr double railMaxX = 1.25;
    const Eigen::Vector4d railColor = sweepAxisColor(axis, true);
    addLine(
        state,
        "sweep.axis." + std::to_string(axis),
        Eigen::Vector3d(railMinX, railY, railZ),
        Eigen::Vector3d(railMaxX, railY, railZ),
        railColor,
        1.0f);
    addDebugLabel(
        state,
        std::string("Sweep ") + static_cast<char>('X' + axis),
        Eigen::Vector3d(railMinX, railY, railZ),
        railColor);

    for (const auto* endpoint : endpoints) {
      const double t = (endpoint->value - minValue) / span;
      const double x = railMinX + t * (railMaxX - railMinX);
      addLine(
          state,
          "sweep.endpoint." + std::to_string(axis) + "."
              + std::to_string(endpoint->order),
          Eigen::Vector3d(x, railY, railZ - 0.065),
          Eigen::Vector3d(x, railY, railZ + 0.065),
          sweepAxisColor(axis, endpoint->isMin),
          endpoint->isMin ? 2.5f : 1.5f);
    }
  }
}

void addBroadPhaseOverlay(
    SandboxState& state, const collision::BroadPhaseDebugSnapshot& snapshot)
{
  std::unordered_map<std::size_t, Eigen::Vector3d> nodeCenters;
  std::unordered_map<std::size_t, Eigen::Vector3d> leafCentersByObjectId;
  nodeCenters.reserve(snapshot.nodes.size());
  leafCentersByObjectId.reserve(snapshot.nodes.size());

  for (const auto& node : snapshot.nodes) {
    nodeCenters.emplace(node.nodeId, node.aabb.center());
    if (node.isLeaf()) {
      leafCentersByObjectId.emplace(node.objectId, node.tightAabb.center());
      if (state.showAabbs) {
        const Eigen::Vector4d aabbColor = rgba(0.95, 0.62, 0.18);
        addAabbLines(
            state,
            "broadphase.leaf." + std::to_string(node.objectId),
            node.tightAabb,
            aabbColor);
        addDebugLabel(
            state,
            "AABB " + std::to_string(node.objectId),
            node.tightAabb.center(),
            aabbColor);
      }
    } else if (state.showBvhNodes) {
      const Eigen::Vector4d nodeColor = rgba(0.16, 0.36, 0.95);
      addAabbLines(
          state,
          "broadphase.internal." + std::to_string(node.nodeId),
          node.aabb,
          nodeColor);
      addDebugLabel(
          state,
          "BVH " + std::to_string(node.nodeId),
          node.aabb.center(),
          nodeColor);
    }
  }

  if (state.showBvhEdges && snapshot.hasTreeTopology) {
    for (const auto& node : snapshot.nodes) {
      if (node.isLeaf()) {
        continue;
      }
      const auto parentCenter = nodeCenters.find(node.nodeId);
      if (parentCenter == nodeCenters.end()) {
        continue;
      }
      for (const std::size_t child : {node.left, node.right}) {
        const auto childCenter = nodeCenters.find(child);
        if (childCenter != nodeCenters.end()) {
          addLine(
              state,
              "broadphase.edge." + std::to_string(node.nodeId) + "."
                  + std::to_string(child),
              parentCenter->second,
              childCenter->second,
              rgba(0.95, 0.95, 0.95),
              1.0f);
        }
      }
    }
  }

  if (state.showCandidatePairs) {
    for (std::size_t i = 0; i < snapshot.candidatePairs.size(); ++i) {
      const auto& pair = snapshot.candidatePairs[i];
      const auto first = leafCentersByObjectId.find(pair.first);
      const auto second = leafCentersByObjectId.find(pair.second);
      if (first != leafCentersByObjectId.end()
          && second != leafCentersByObjectId.end()) {
        const Eigen::Vector4d pairColor = rgba(0.95, 0.1, 0.85);
        addLine(
            state,
            "broadphase.candidate." + std::to_string(i),
            first->second,
            second->second,
            pairColor,
            2.5f);
        addDebugLabel(
            state,
            "Pair " + std::to_string(i),
            (first->second + second->second) * 0.5,
            pairColor);
      }
    }
  }

  if (state.showSpatialHashCells && snapshot.hasSpatialHashCells) {
    for (std::size_t i = 0; i < snapshot.cells.size(); ++i) {
      const Eigen::Vector4d cellColor = rgba(0.1, 0.85, 0.95);
      addAabbLines(
          state,
          "spatial_hash.cell." + std::to_string(i),
          snapshot.cells[i].aabb,
          cellColor);
      addDebugLabel(
          state,
          "Cell " + std::to_string(i),
          snapshot.cells[i].aabb.center(),
          cellColor);
    }
  }

  if (state.showSweepEndpoints) {
    addSweepEndpointOverlay(state, snapshot);
  }
}

void clearDebugFrames(SandboxState& state)
{
  for (const auto& frame : state.debugFrames) {
    if (frame != nullptr) {
      state.world->removeSimpleFrame(frame);
    }
  }
  state.debugFrames.clear();
}

void updateObjectVisuals(SandboxState& state)
{
  const sandbox::PairCase& pair = selectedPair(state);
  const bool showContactOverlay = state.summary.numContacts > 0;
  state.objectAFrame->setShape(
      makeVisualShape(pair.shapeA, state.objectAParams));
  state.objectBFrame->setShape(
      makeVisualShape(pair.shapeB, state.objectBParams));
  state.objectAFrame->getVisualAspect(true)->setRGBA(objectColor(
      pair,
      true,
      state.summary.hit,
      state.summary.pairFiltered,
      showContactOverlay));
  state.objectBFrame->getVisualAspect(true)->setRGBA(objectColor(
      pair,
      false,
      state.summary.hit,
      state.summary.pairFiltered,
      showContactOverlay));
}

void summarizeContacts(
    QuerySummary& summary, const collision::CollisionResult& result)
{
  summary.numManifolds = result.numManifolds();
  summary.numContacts = result.numContacts();
  summary.contacts.clear();
  for (std::size_t m = 0; m < result.numManifolds(); ++m) {
    const auto& manifold = result.getManifold(m);
    for (std::size_t c = 0; c < manifold.numContacts(); ++c) {
      const auto& contact = manifold.getContact(c);
      summary.contacts.push_back(
          ContactRow{m, c, contact.position, contact.normal, contact.depth});
    }
  }
}

void summarizeBroadPhase(
    QuerySummary& summary, const collision::BroadPhaseDebugSnapshot& snapshot)
{
  summary.broadPhaseObjects = snapshot.numObjects;
  summary.broadPhaseNodes = snapshot.nodes.size();
  summary.broadPhaseCells = snapshot.cells.size();
  summary.broadPhaseEndpoints = snapshot.endpoints.size();
  summary.broadPhaseCandidatePairs = snapshot.candidatePairs.size();
  summary.broadPhaseHasTreeTopology = snapshot.hasTreeTopology;
  summary.broadPhaseHasSpatialHashCells = snapshot.hasSpatialHashCells;
  summary.broadPhaseHasSweepEndpoints = snapshot.hasSweepEndpoints;
  summary.broadPhaseLeaves = 0;
  summary.broadPhaseInternals = 0;
  for (const auto& node : snapshot.nodes) {
    if (node.isLeaf()) {
      ++summary.broadPhaseLeaves;
    } else {
      ++summary.broadPhaseInternals;
    }
  }
}

void rebuildScene(SandboxState& state)
{
  attachObjectFrames(state);
  clearDebugFrames(state);
  state.debugLabels.clear();

  const sandbox::PairCase& pair = selectedPair(state);
  state.summary = QuerySummary{};
  state.summary.pair = &pair;

  const Eigen::Isometry3d tfA = state.objectAFrame->getWorldTransform();
  const Eigen::Isometry3d tfB = state.objectBFrame->getWorldTransform();

  collision::CollisionWorld nativeWorld(selectedBroadPhase(state).type);
  auto objA = nativeWorld.createObject(
      sandbox::makeShape(pair.shapeA, state.objectAParams), tfA);
  auto objB = nativeWorld.createObject(
      sandbox::makeShape(pair.shapeB, state.objectBParams), tfB);
  if (state.filterPair) {
    objA.setCollisionFilter(
        collision::FilterGroup::Static, collision::FilterGroup::Static);
    objB.setCollisionFilter(
        collision::FilterGroup::Dynamic, collision::FilterGroup::Dynamic);
  } else {
    objA.setCollisionFilterData(collision::CollisionFilterData::all());
    objB.setCollisionFilterData(collision::CollisionFilterData::all());
  }
  state.summary.pairFiltered = !collision::shouldCollide(
      objA.getCollisionFilterData(), objB.getCollisionFilterData());

  collision::CollisionResult collisionResult;
  collision::DistanceResult distanceResult;
  if (pair.supportsContact()) {
    collision::CollisionOption option;
    option.maxNumContacts
        = static_cast<std::size_t>(std::clamp(state.maxContacts, 1.0, 128.0));
    state.summary.hit
        = nativeWorld.collide(objA, objB, option, collisionResult);
    summarizeContacts(state.summary, collisionResult);
  } else if (pair.supportsDistance()) {
    collision::DistanceOption option;
    collision::NarrowPhase::distance(objA, objB, option, distanceResult);
    state.summary.distanceValid = distanceResult.isValid();
    state.summary.distance = distanceResult.distance;
  }

  updateObjectVisuals(state);

  if (pair.supportsContact()) {
    for (const ContactRow& contact : state.summary.contacts) {
      const std::string prefix = "contact.m"
                                 + std::to_string(contact.manifoldIndex) + ".c"
                                 + std::to_string(contact.contactIndex);
      addPointMarker(
          state,
          prefix + ".point",
          contact.point,
          rgba(0.95, 0.12, 0.12),
          kContactMarkerRadius);
      addDebugLabel(
          state,
          "Contact " + std::to_string(contact.manifoldIndex) + ":"
              + std::to_string(contact.contactIndex),
          contact.point,
          rgba(0.95, 0.12, 0.12));
      if (contact.normal.squaredNorm() > 1e-12) {
        addLine(
            state,
            prefix + ".normal",
            contact.point,
            contact.point + contact.normal.normalized() * 0.32,
            rgba(0.95, 0.9, 0.12),
            2.5f);
        addLine(
            state,
            prefix + ".depth",
            contact.point,
            contact.point
                - contact.normal.normalized() * std::max(contact.depth, 0.035),
            rgba(0.95, 0.12, 0.85),
            2.0f);
      }
    }
  } else if (distanceResult.isValid()) {
    addLine(
        state,
        "distance.segment",
        distanceResult.pointOnObject1,
        distanceResult.pointOnObject2,
        rgba(0.1, 0.82, 0.92),
        2.5f);
    addDebugLabel(
        state,
        "Distance",
        (distanceResult.pointOnObject1 + distanceResult.pointOnObject2) * 0.5,
        rgba(0.1, 0.82, 0.92));
    addPointMarker(
        state,
        "distance.point_a",
        distanceResult.pointOnObject1,
        rgba(0.95, 0.82, 0.12),
        kDistanceMarkerRadius);
    addPointMarker(
        state,
        "distance.point_b",
        distanceResult.pointOnObject2,
        rgba(0.95, 0.82, 0.12),
        kDistanceMarkerRadius);
  }

  const collision::BroadPhaseDebugSnapshot broadPhase
      = nativeWorld.buildBroadPhaseDebugSnapshot();
  summarizeBroadPhase(state.summary, broadPhase);
  if (state.showBroadPhase) {
    addBroadPhaseOverlay(state, broadPhase);
  }

  state.dirty = false;
}

bool parseExampleArgs(int argc, char* argv[], SandboxState& state)
{
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--pair" && i + 1 < argc) {
      const std::string_view id(argv[++i]);
      const sandbox::PairCase* pair = sandbox::findPairCase(id);
      if (!pair) {
        std::cerr << "Unknown pair id: " << id << "\n";
        return false;
      }
      state.pairCaseIndex = pair->index;
      resetPairControls(state);
    } else if (arg == "--broad-phase" && i + 1 < argc) {
      const std::string_view id(argv[++i]);
      const BroadPhaseOption* option = findBroadPhaseOption(id);
      if (!option) {
        std::cerr << "Unknown broad-phase id: " << id << "\n";
        return false;
      }
      state.broadPhaseIndex
          = static_cast<std::size_t>(option - kBroadPhaseOptions.data());
      state.dirty = true;
    } else if (arg == "--filter-pair") {
      state.filterPair = true;
      state.dirty = true;
    }
  }
  return true;
}

bool slider3(
    gui::PanelBuilder& panel,
    const char* prefix,
    Eigen::Vector3d& values,
    double min,
    double max)
{
  bool changed = false;
  changed |= panel.slider(std::string(prefix) + " X", values.x(), min, max);
  changed |= panel.slider(std::string(prefix) + " Y", values.y(), min, max);
  changed |= panel.slider(std::string(prefix) + " Z", values.z(), min, max);
  return changed;
}

bool shapeParameterControls(
    gui::PanelBuilder& panel,
    const char* prefix,
    ShapeType type,
    ShapeParameters& params)
{
  bool changed = false;
  switch (type) {
    case ShapeType::Sphere:
    case ShapeType::Convex:
    case ShapeType::Sdf:
    case ShapeType::Compound:
      changed |= panel.slider(
          std::string(prefix) + " Radius", params.radius, 0.05, 2.0);
      break;
    case ShapeType::Capsule:
    case ShapeType::Cylinder:
      changed |= panel.slider(
          std::string(prefix) + " Radius", params.radius, 0.05, 2.0);
      changed |= panel.slider(
          std::string(prefix) + " Height", params.height, 0.05, 4.0);
      break;
    case ShapeType::Box:
    case ShapeType::Mesh: {
      const std::string label = std::string(prefix) + " Half Extent";
      changed |= slider3(panel, label.c_str(), params.halfExtents, 0.025, 2.0);
      break;
    }
    case ShapeType::Plane: {
      const std::string label = std::string(prefix) + " Visual Half Extent";
      changed |= slider3(panel, label.c_str(), params.halfExtents, 0.01, 5.0);
      break;
    }
  }
  return changed;
}

std::vector<gui::Gizmo> createObjectGizmos(
    const std::shared_ptr<SandboxState>& state)
{
  const auto makeGizmo = [state](
                             std::string label,
                             const dynamics::SimpleFramePtr& target,
                             const gui::GizmoAxisColors& colors) {
    gui::Gizmo gizmo;
    gizmo.label = std::move(label);
    gizmo.target = target;
    gizmo.flags = gui::GizmoFlags::All;
    gizmo.size = kObjectGizmoSize;
    gizmo.colors = colors;
    gizmo.onChanged = [state](const Eigen::Isometry3d&) {
      state->dirty = true;
    };
    return gizmo;
  };

  gui::GizmoAxisColors colorsA;
  colorsA.highlight = rgba(1.0, 0.82, 0.18);
  gui::GizmoAxisColors colorsB;
  colorsB.x = rgba(0.95, 0.22, 0.55);
  colorsB.y = rgba(0.18, 0.75, 0.45);
  colorsB.z = rgba(0.35, 0.55, 0.98);
  colorsB.highlight = rgba(1.0, 0.82, 0.18);

  return {
      makeGizmo("Object A", state->objectAFrame, colorsA),
      makeGizmo("Object B", state->objectBFrame, colorsB)};
}

gui::Panel createControlsPanel(const std::shared_ptr<SandboxState>& state)
{
  gui::Panel panel;
  panel.title = "Native Collision";
  panel.initialPosition = std::array<double, 2>{820.0, 16.0};
  panel.initialSize = std::array<double, 2>{420.0, 620.0};
  panel.build = [state](gui::PanelBuilder& panelBuilder) {
    const auto pairs = sandbox::pairCases();
    const sandbox::PairCase& pair = selectedPair(*state);

    panelBuilder.text(
        pair.label + " [" + std::string(sandbox::pairStatusLabel(pair.status))
        + "]");
    panelBuilder.text(pair.note);
    panelBuilder.text(
        "Support: " + std::string(sandbox::pairStatusDescription(pair.status)));
    panelBuilder.text("Path: " + std::string(pairCoverageMode(pair)));
    panelBuilder.text("Left-drag object gizmo arrows, planes, and rings.");
    if (panelBuilder.button("Previous")) {
      state->pairCaseIndex
          = (state->pairCaseIndex + pairs.size() - 1) % pairs.size();
      resetPairControls(*state);
    }
    panelBuilder.sameLine();
    if (panelBuilder.button("Next")) {
      state->pairCaseIndex = (state->pairCaseIndex + 1) % pairs.size();
      resetPairControls(*state);
    }
    if (panelBuilder.beginMenu("Pair Selector")) {
      for (const sandbox::PairCase& candidate : pairs) {
        if (panelBuilder.menuItem(
                candidate.label + " ["
                + std::string(sandbox::pairStatusLabel(candidate.status))
                + "]")) {
          state->pairCaseIndex = candidate.index;
          resetPairControls(*state);
        }
      }
      panelBuilder.endMenu();
    }
    if (panelBuilder.collapsingHeader("Pair Coverage", false)) {
      const bool useTable
          = panelBuilder.beginTable("pair_coverage", kPairCoverageColumns);
      if (!useTable) {
        panelBuilder.text("Pair | Support | Mode");
      }
      for (const sandbox::PairCase& candidate : pairs) {
        std::string pairLabel = candidate.label;
        if (candidate.index == state->pairCaseIndex) {
          pairLabel += " (current)";
        }
        const std::string statusLabel
            = std::string(sandbox::pairStatusLabel(candidate.status));
        const std::string modeLabel = std::string(pairCoverageMode(candidate));
        if (useTable) {
          panelBuilder.tableNextRow();
          panelBuilder.tableNextColumn();
          panelBuilder.text(pairLabel);
          panelBuilder.tableNextColumn();
          panelBuilder.text(statusLabel);
          panelBuilder.tableNextColumn();
          panelBuilder.text(modeLabel);
        } else {
          panelBuilder.text(
              pairLabel + " | " + statusLabel + " | " + modeLabel);
        }
      }
      if (useTable) {
        panelBuilder.endTable();
      }
    }

    panelBuilder.separator();
    panelBuilder.text(
        "Broad-phase: " + std::string(selectedBroadPhase(*state).label));
    if (panelBuilder.beginMenu("Broad-phase Selector")) {
      for (std::size_t i = 0; i < kBroadPhaseOptions.size(); ++i) {
        const BroadPhaseOption& option = kBroadPhaseOptions[i];
        if (panelBuilder.menuItem(std::string(option.label))) {
          state->broadPhaseIndex = i;
          state->dirty = true;
        }
      }
      panelBuilder.endMenu();
    }

    panelBuilder.separator();
    panelBuilder.text("Object A");
    panelBuilder.text("Position: " + formatFramePosition(state->objectAFrame));
    state->dirty |= shapeParameterControls(
        panelBuilder, "A", pair.shapeA, state->objectAParams);

    panelBuilder.separator();
    panelBuilder.text("Object B");
    panelBuilder.text("Position: " + formatFramePosition(state->objectBFrame));
    state->dirty |= shapeParameterControls(
        panelBuilder, "B", pair.shapeB, state->objectBParams);
    state->dirty
        |= panelBuilder.slider("Max Contacts", state->maxContacts, 1.0, 64.0);
    state->dirty |= panelBuilder.checkbox("Filter Pair", state->filterPair);
    if (panelBuilder.button("Reset Pair")) {
      resetPairControls(*state);
    }

    panelBuilder.separator();
    state->dirty |= panelBuilder.checkbox("AABBs", state->showAabbs);
    state->dirty
        |= panelBuilder.checkbox("Broad-phase Overlay", state->showBroadPhase);
    state->dirty |= panelBuilder.checkbox("BVH Nodes", state->showBvhNodes);
    state->dirty |= panelBuilder.checkbox("BVH Edges", state->showBvhEdges);
    state->dirty
        |= panelBuilder.checkbox("Candidate Pairs", state->showCandidatePairs);
    state->dirty |= panelBuilder.checkbox(
        "Spatial Hash Cells", state->showSpatialHashCells);
    state->dirty
        |= panelBuilder.checkbox("Sweep Endpoints", state->showSweepEndpoints);
    panelBuilder.checkbox("Name Tags", state->showDebugLabels);

    panelBuilder.separator();
    const QuerySummary& summary = state->summary;
    panelBuilder.text(
        std::string("Collision: ") + (summary.hit ? "yes" : "no"));
    panelBuilder.text(
        std::string("Filtered: ") + (summary.pairFiltered ? "yes" : "no"));
    panelBuilder.text(
        "Manifolds: " + std::to_string(summary.numManifolds)
        + " Contacts: " + std::to_string(summary.numContacts));
    if (summary.distanceValid) {
      panelBuilder.text("Distance: " + std::to_string(summary.distance));
    }
    if (panelBuilder.collapsingHeader("Contacts", true)) {
      for (const ContactRow& contact : summary.contacts) {
        panelBuilder.text(
            "M" + std::to_string(contact.manifoldIndex) + " C"
            + std::to_string(contact.contactIndex) + " p "
            + formatVec3(contact.point));
        panelBuilder.text(
            "    n " + formatVec3(contact.normal) + " d "
            + std::to_string(contact.depth));
      }
    }
    if (panelBuilder.collapsingHeader("Broad-phase", true)) {
      panelBuilder.text(
          "Objects: " + std::to_string(summary.broadPhaseObjects));
      panelBuilder.text(
          "Candidates: " + std::to_string(summary.broadPhaseCandidatePairs));
      panelBuilder.text(
          "Nodes: " + std::to_string(summary.broadPhaseNodes) + " ("
          + std::to_string(summary.broadPhaseLeaves) + " leaf, "
          + std::to_string(summary.broadPhaseInternals) + " internal)");
      panelBuilder.text("Cells: " + std::to_string(summary.broadPhaseCells));
      panelBuilder.text(
          "Endpoints: " + std::to_string(summary.broadPhaseEndpoints));
      panelBuilder.text(
          std::string("Tree topology: ")
          + (summary.broadPhaseHasTreeTopology ? "yes" : "no"));
      panelBuilder.text(
          std::string("Spatial cells: ")
          + (summary.broadPhaseHasSpatialHashCells ? "yes" : "no"));
      panelBuilder.text(
          std::string("Sweep order: ")
          + (summary.broadPhaseHasSweepEndpoints ? "yes" : "no"));
    }
  };
  return panel;
}

gui::RunOptions makeRunDefaults()
{
  gui::RunOptions options;
  options.windowTitle = "DART Native Collision Sandbox";
  options.width = 1280;
  options.height = 720;
  return options;
}

gui::OrbitCamera makeCamera()
{
  gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.4, 0.0, 0.25);
  camera.yaw = -0.72;
  camera.pitch = 0.38;
  camera.distance = 5.3;
  return camera;
}

} // namespace

int main(int argc, char* argv[])
{
  auto state = std::make_shared<SandboxState>();
  resetPairControls(*state);
  if (!parseExampleArgs(argc, argv, *state)) {
    return 2;
  }
  attachObjectFrames(*state);
  rebuildScene(*state);

  gui::ApplicationOptions options;
  options.world = state->world;
  options.runDefaults = makeRunDefaults();
  options.camera = makeCamera();
  options.simulateWorld = false;
  options.gizmos = createObjectGizmos(state);
  options.debugLabels = [state] {
    return state->showDebugLabels ? state->debugLabels
                                  : std::vector<gui::DebugLabelDescriptor>{};
  };
  options.panels.push_back(createControlsPanel(state));
  options.preRender = [state] {
    if (state->dirty) {
      rebuildScene(*state);
    }
  };

  return gui::runApplication(argc, argv, options);
}
