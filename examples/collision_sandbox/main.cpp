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

#include <dart/math/constants.hpp>

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
  bool broadPhaseHasTreeTopology = false;
  std::vector<ContactRow> contacts;
};

struct SandboxState
{
  simulation::WorldPtr world = simulation::World::create("collision_sandbox");
  std::size_t pairCaseIndex = 0;
  std::array<double, 3> objectATranslation = {0.0, 0.0, 0.0};
  std::array<double, 3> objectBTranslation = {0.55, 0.0, 0.0};
  std::array<double, 3> objectARotation = {0.0, 0.0, 0.0};
  std::array<double, 3> objectBRotation = {0.0, 0.0, 0.0};
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
  bool dirty = true;
  QuerySummary summary;
};

const sandbox::PairCase& selectedPair(const SandboxState& state)
{
  const auto pairs = sandbox::pairCases();
  return pairs[std::min(state.pairCaseIndex, pairs.size() - 1)];
}

Eigen::Isometry3d makeTransform(
    const std::array<double, 3>& translation,
    const std::array<double, 3>& rotationDegrees)
{
  const double roll = rotationDegrees[0] * dart::math::pi / 180.0;
  const double pitch = rotationDegrees[1] * dart::math::pi / 180.0;
  const double yaw = rotationDegrees[2] * dart::math::pi / 180.0;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                 * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
  tf.translation()
      = Eigen::Vector3d(translation[0], translation[1], translation[2]);
  return tf;
}

void copyTranslation(std::array<double, 3>& out, const Eigen::Vector3d& in)
{
  out = {in.x(), in.y(), in.z()};
}

void resetPairControls(SandboxState& state)
{
  const sandbox::PairPose pose = sandbox::defaultPairPose(selectedPair(state));
  copyTranslation(state.objectATranslation, pose.transformA.translation());
  copyTranslation(state.objectBTranslation, pose.transformB.translation());
  state.objectARotation = {0.0, 0.0, 0.0};
  state.objectBRotation = {0.0, 0.0, 0.0};
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

Eigen::Vector4d objectColor(
    const sandbox::PairCase& pair, bool objectA, bool hit)
{
  if (pair.status == sandbox::PairStatus::Unsupported) {
    return rgba(0.42, 0.44, 0.46);
  }
  if (pair.status == sandbox::PairStatus::DistanceOnly) {
    return objectA ? rgba(0.1, 0.75, 0.9) : rgba(0.95, 0.85, 0.25);
  }
  if (hit) {
    return objectA ? rgba(0.95, 0.18, 0.18) : rgba(0.95, 0.18, 0.78);
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

void addFrame(
    const simulation::WorldPtr& world,
    std::string name,
    const std::shared_ptr<dynamics::Shape>& shape,
    const Eigen::Isometry3d& transform,
    const Eigen::Vector4d& color)
{
  auto frame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), std::move(name), transform);
  frame->setShape(shape);
  frame->getVisualAspect(true)->setRGBA(color);
  world->addSimpleFrame(frame);
}

void addLine(
    const simulation::WorldPtr& world,
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
  frame->getVisualAspect(true)->setRGBA(color);
  world->addSimpleFrame(frame);
}

void addPointMarker(
    const simulation::WorldPtr& world,
    std::string name,
    const Eigen::Vector3d& point,
    const Eigen::Vector4d& color,
    double radius)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = point;
  addFrame(
      world,
      std::move(name),
      std::make_shared<dynamics::SphereShape>(radius),
      tf,
      color);
}

void addAabbLines(
    const simulation::WorldPtr& world,
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
        world,
        prefix + ".edge." + std::to_string(i),
        corners[edges[i].first],
        corners[edges[i].second],
        color,
        1.0f);
  }
}

void addBroadPhaseOverlay(
    const simulation::WorldPtr& world,
    const SandboxState& state,
    const collision::BroadPhaseDebugSnapshot& snapshot)
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
        addAabbLines(
            world,
            "broadphase.leaf." + std::to_string(node.objectId),
            node.tightAabb,
            rgba(0.95, 0.62, 0.18));
      }
    } else if (state.showBvhNodes) {
      addAabbLines(
          world,
          "broadphase.internal." + std::to_string(node.nodeId),
          node.aabb,
          rgba(0.16, 0.36, 0.95));
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
              world,
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
        addLine(
            world,
            "broadphase.candidate." + std::to_string(i),
            first->second,
            second->second,
            rgba(0.95, 0.1, 0.85),
            2.5f);
      }
    }
  }
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
  summary.broadPhaseCandidatePairs = snapshot.candidatePairs.size();
  summary.broadPhaseHasTreeTopology = snapshot.hasTreeTopology;
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
  state.world->removeAllSimpleFrames();

  const sandbox::PairCase& pair = selectedPair(state);
  state.summary = QuerySummary{};
  state.summary.pair = &pair;

  const Eigen::Isometry3d tfA
      = makeTransform(state.objectATranslation, state.objectARotation);
  const Eigen::Isometry3d tfB
      = makeTransform(state.objectBTranslation, state.objectBRotation);

  collision::CollisionWorld nativeWorld;
  auto objA = nativeWorld.createObject(
      sandbox::makeShape(pair.shapeA, state.objectAParams), tfA);
  auto objB = nativeWorld.createObject(
      sandbox::makeShape(pair.shapeB, state.objectBParams), tfB);

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

  addFrame(
      state.world,
      "object_a",
      makeVisualShape(pair.shapeA, state.objectAParams),
      tfA,
      objectColor(pair, true, state.summary.hit));
  addFrame(
      state.world,
      "object_b",
      makeVisualShape(pair.shapeB, state.objectBParams),
      tfB,
      objectColor(pair, false, state.summary.hit));

  if (pair.supportsContact()) {
    for (const ContactRow& contact : state.summary.contacts) {
      const std::string prefix = "contact.m"
                                 + std::to_string(contact.manifoldIndex) + ".c"
                                 + std::to_string(contact.contactIndex);
      addPointMarker(
          state.world,
          prefix + ".point",
          contact.point,
          rgba(0.95, 0.12, 0.12),
          0.045);
      if (contact.normal.squaredNorm() > 1e-12) {
        addLine(
            state.world,
            prefix + ".normal",
            contact.point,
            contact.point + contact.normal.normalized() * 0.32,
            rgba(0.95, 0.9, 0.12),
            2.5f);
        addLine(
            state.world,
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
        state.world,
        "distance.segment",
        distanceResult.pointOnObject1,
        distanceResult.pointOnObject2,
        rgba(0.1, 0.82, 0.92),
        2.5f);
    addPointMarker(
        state.world,
        "distance.point_a",
        distanceResult.pointOnObject1,
        rgba(0.95, 0.82, 0.12),
        0.04);
    addPointMarker(
        state.world,
        "distance.point_b",
        distanceResult.pointOnObject2,
        rgba(0.95, 0.82, 0.12),
        0.04);
  }

  const collision::BroadPhaseDebugSnapshot broadPhase
      = nativeWorld.buildBroadPhaseDebugSnapshot();
  summarizeBroadPhase(state.summary, broadPhase);
  if (state.showBroadPhase) {
    addBroadPhaseOverlay(state.world, state, broadPhase);
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
    }
  }
  return true;
}

bool slider3(
    gui::PanelBuilder& panel,
    const char* prefix,
    std::array<double, 3>& values,
    double min,
    double max)
{
  bool changed = false;
  changed |= panel.slider(std::string(prefix) + " X", values[0], min, max);
  changed |= panel.slider(std::string(prefix) + " Y", values[1], min, max);
  changed |= panel.slider(std::string(prefix) + " Z", values[2], min, max);
  return changed;
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

    panelBuilder.separator();
    panelBuilder.text("Object A");
    state->dirty |= slider3(
        panelBuilder, "A Position", state->objectATranslation, -2.0, 2.0);
    state->dirty |= slider3(
        panelBuilder, "A Rotation", state->objectARotation, -180.0, 180.0);
    state->dirty |= shapeParameterControls(
        panelBuilder, "A", pair.shapeA, state->objectAParams);

    panelBuilder.separator();
    panelBuilder.text("Object B");
    state->dirty |= slider3(
        panelBuilder, "B Position", state->objectBTranslation, -2.0, 2.0);
    state->dirty |= slider3(
        panelBuilder, "B Rotation", state->objectBRotation, -180.0, 180.0);
    state->dirty |= shapeParameterControls(
        panelBuilder, "B", pair.shapeB, state->objectBParams);
    state->dirty
        |= panelBuilder.slider("Max Contacts", state->maxContacts, 1.0, 64.0);
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

    panelBuilder.separator();
    const QuerySummary& summary = state->summary;
    panelBuilder.text(
        std::string("Collision: ") + (summary.hit ? "yes" : "no"));
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
      panelBuilder.text(
          std::string("Tree topology: ")
          + (summary.broadPhaseHasTreeTopology ? "yes" : "no"));
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
  if (!parseExampleArgs(argc, argv, *state)) {
    return 2;
  }
  rebuildScene(*state);

  gui::ApplicationOptions options;
  options.world = state->world;
  options.runDefaults = makeRunDefaults();
  options.camera = makeCamera();
  options.simulateWorld = false;
  options.panels.push_back(createControlsPanel(state));
  options.preRender = [state] {
    if (state->dirty) {
      rebuildScene(*state);
    }
  };

  return gui::runApplication(argc, argv, options);
}
