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

#include "Inspector.hpp"

#include <osg/Group>
#include <osg/NodeVisitor>
#include <osg/PolygonMode>

#include <algorithm>
#include <utility>

#include <cmath>

namespace dart_demos {

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::DegreeOfFreedom;
using dart::dynamics::Joint;
using dart::dynamics::ShapeNode;

//==============================================================================
/// Finds every OSG node named exactly `name` under `root`. Used to locate a
/// selected body's rendered nodes for the wireframe toggle: gui-osg's
/// ShapeFrameNode constructor names each node "<ShapeFrame name> [frame]"
/// (dart/gui/osg/ShapeFrameNode.cpp), which is the only host-facing way to
/// reach a specific body's OSG subtree without a dart/ change (WorldNode
/// keeps its Frame->ShapeFrameNode map private). Relies on ShapeNode names
/// being unique within the world; two different skeletons that happen to
/// reuse the same body/shape-node name would both match (a documented, rare
/// edge case -- most scenes use distinct body names).
class NamedNodeFinder : public ::osg::NodeVisitor
{
public:
  explicit NamedNodeFinder(std::string name)
    : ::osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), mName(std::move(name))
  {
  }

  void apply(::osg::Node& node) override
  {
    if (node.getName() == mName)
      mFound.push_back(&node);
    traverse(node);
  }

  std::string mName;
  std::vector<::osg::Node*> mFound;
};

//==============================================================================
/// Returns a finite, correctly-ordered [lo, hi] position range for `dof`'s
/// slider. Infinite limits (continuous revolute, FreeJoint dofs) become a
/// window around the current position so the slider stays usable without
/// teleporting the dof to a fixed bound; the ordering guarantee (lo <= hi) is
/// required because std::clamp(v, lo, hi) with lo > hi is undefined behavior
/// (aborts under _GLIBCXX_ASSERTIONS), which the "never crash" contract
/// forbids.
std::pair<double, double> positionRange(
    const dart::dynamics::DegreeOfFreedom* dof)
{
  const double pos = dof->getPosition();
  double lo = dof->getPositionLowerLimit();
  double hi = dof->getPositionUpperLimit();
  if (!std::isfinite(lo))
    lo = std::min(pos - 10.0, std::isfinite(hi) ? hi : pos);
  if (!std::isfinite(hi))
    hi = std::max(pos + 10.0, lo);
  if (lo > hi)
    std::swap(lo, hi);
  return {lo, hi};
}

} // namespace

//==============================================================================
void Inspector::reset()
{
  setSelection(nullptr);
  mBodyVisualState.clear();
  mPendingEdits.clear();
}

//==============================================================================
void Inspector::setSelection(BodyNode* body)
{
  if (body == mSelected)
    return;

  restoreHighlight();
  removeAllSubjects();

  mSelected = body;
  if (mSelected) {
    addSubject(mSelected);
    snapshotHighlightColors();
  }
}

//==============================================================================
void Inspector::handleDestructionNotification(
    const dart::common::Subject* subject)
{
  if (subject != mSelected)
    return;

  // The body is already being destroyed -- do not touch its (now-invalid)
  // ShapeNodes. Just drop every reference to it.
  mSelected = nullptr;
  mHighlightSnapshot.clear();
  mPendingEdits.clear();
}

//==============================================================================
void Inspector::renderTree(const dart::simulation::WorldPtr& world)
{
  if (!world || world->getNumSkeletons() == 0) {
    ImGui::TextDisabled("(no skeletons)");
    return;
  }

  for (std::size_t s = 0; s < world->getNumSkeletons(); ++s) {
    const auto& skel = world->getSkeleton(s);
    if (!skel)
      continue;

    const std::string header = skel->getName() + "##skel_" + std::to_string(s);
    if (ImGui::TreeNodeEx(header.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
      for (std::size_t t = 0; t < skel->getNumTrees(); ++t) {
        if (auto* root = skel->getRootBodyNode(t))
          renderBodyRow(root);
      }
      ImGui::TreePop();
    }
  }
}

//==============================================================================
void Inspector::renderBodyRow(BodyNode* body)
{
  ImGuiTreeNodeFlags flags
      = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_DefaultOpen;
  if (body == mSelected)
    flags |= ImGuiTreeNodeFlags_Selected;

  auto* joint = body->getParentJoint();
  const std::size_t numDofs = joint ? joint->getNumDofs() : 0;
  if (body->getNumChildBodyNodes() == 0 && numDofs == 0)
    flags |= ImGuiTreeNodeFlags_Leaf;

  const bool open = ImGui::TreeNodeEx(
      static_cast<void*>(body), flags, "%s", body->getName().c_str());
  if (ImGui::IsItemClicked())
    setSelection(body);

  if (open) {
    if (joint)
      renderJointRow(joint);
    for (std::size_t c = 0; c < body->getNumChildBodyNodes(); ++c)
      renderBodyRow(body->getChildBodyNode(c));
    ImGui::TreePop();
  }
}

//==============================================================================
void Inspector::renderJointRow(Joint* joint)
{
  if (joint->getNumDofs() == 0)
    return;

  const std::string label
      = joint->getName() + " (" + joint->getType() + ")##joint";
  if (ImGui::TreeNodeEx(
          label.c_str(),
          ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_Bullet)) {
    for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
      auto* dof = joint->getDof(i);
      ImGui::BulletText(
          "%s = %.3f", dof->getName().c_str(), dof->getPosition());
    }
    ImGui::TreePop();
  }
}

//==============================================================================
void Inspector::renderDetail(bool paused, ::osg::Group* worldNode)
{
  if (!mSelected) {
    ImGui::TextWrapped("No body selected. Click a body in the tree above.");
    return;
  }

  BodyNode* body = mSelected;
  ImGui::TextWrapped("Body: %s", body->getName().c_str());

  const Eigen::Isometry3d worldTf = body->getWorldTransform();
  const Eigen::Isometry3d relTf = body->getRelativeTransform();
  const Eigen::Vector3d worldPos = worldTf.translation();
  const Eigen::Vector3d worldRpy = worldTf.linear().eulerAngles(0, 1, 2) * 180.0
                                   / dart::math::constantsd::pi();
  const Eigen::Vector3d relPos = relTf.translation();

  ImGui::Text(
      "World pos   (%.3f, %.3f, %.3f) m",
      worldPos.x(),
      worldPos.y(),
      worldPos.z());
  ImGui::Text(
      "World rpy   (%.1f, %.1f, %.1f) deg",
      worldRpy.x(),
      worldRpy.y(),
      worldRpy.z());
  ImGui::Text(
      "Rel. pos    (%.3f, %.3f, %.3f) m", relPos.x(), relPos.y(), relPos.z());

  const Eigen::Vector3d linVel = body->getLinearVelocity();
  const Eigen::Vector3d angVel = body->getAngularVelocity();
  ImGui::Text(
      "Lin. vel    (%.3f, %.3f, %.3f) m/s", linVel.x(), linVel.y(), linVel.z());
  ImGui::Text(
      "Ang. vel    (%.3f, %.3f, %.3f) rad/s",
      angVel.x(),
      angVel.y(),
      angVel.z());
  ImGui::Text("Mass        %.3f kg", body->getMass());

  ImGui::Separator();
  if (auto* joint = body->getParentJoint()) {
    ImGui::TextWrapped(
        "Joint: %s (%s)", joint->getName().c_str(), joint->getType().c_str());
    for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
      auto* dof = joint->getDof(i);
      const auto [lower, upper] = positionRange(dof);

      const auto pending = mPendingEdits.find(dof);
      float position = static_cast<float>(
          pending != mPendingEdits.end() ? pending->second
                                         : dof->getPosition());

      ImGui::PushID(dof);
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
      ImGui::BeginDisabled(!paused);
      if (ImGui::SliderFloat(
              dof->getName().c_str(),
              &position,
              static_cast<float>(lower),
              static_cast<float>(upper),
              "%.3f",
              ImGuiSliderFlags_AlwaysClamp)
          && std::isfinite(position)) {
        position = std::clamp(
            position, static_cast<float>(lower), static_cast<float>(upper));
        mPendingEdits[dof] = static_cast<double>(position);
      }
      ImGui::EndDisabled();
      ImGui::SameLine();
      ImGui::Text("vel %.3f", dof->getVelocity());
      ImGui::PopID();
    }
    if (!paused)
      ImGui::TextDisabled("Pause the simulation to edit joint positions.");
  } else {
    ImGui::TextDisabled("No parent joint (root of a free-floating tree).");
  }

  ImGui::Separator();
  BodyVisualState& visualState = mBodyVisualState[body];
  bool wireframe = visualState.wireframe;
  if (ImGui::Checkbox("Wireframe", &wireframe))
    setWireframe(body, wireframe, worldNode);
  ImGui::SameLine();
  bool hidden = visualState.hidden;
  if (ImGui::Checkbox("Hidden", &hidden))
    setHidden(body, hidden);
}

//==============================================================================
void Inspector::applyQueuedEdits()
{
  if (mPendingEdits.empty())
    return;

  for (const auto& [dof, value] : mPendingEdits) {
    if (!dof || !std::isfinite(value))
      continue;
    const auto [lower, upper] = positionRange(dof);
    dof->setPosition(std::clamp(value, lower, upper));
  }
  mPendingEdits.clear();
}

//==============================================================================
void Inspector::setWireframe(
    BodyNode* body, bool enabled, ::osg::Group* worldNode)
{
  mBodyVisualState[body].wireframe = enabled;
  if (!worldNode)
    return;

  for (std::size_t i = 0; i < body->getNumShapeNodes(); ++i) {
    auto* sn = body->getShapeNode(i);
    if (!sn)
      continue;

    NamedNodeFinder finder(sn->getName() + " [frame]");
    worldNode->accept(finder);
    for (auto* node : finder.mFound) {
      auto* stateSet = node->getOrCreateStateSet();
      if (enabled) {
        stateSet->setAttributeAndModes(
            new ::osg::PolygonMode(
                ::osg::PolygonMode::FRONT_AND_BACK, ::osg::PolygonMode::LINE),
            ::osg::StateAttribute::ON | ::osg::StateAttribute::OVERRIDE);
      } else {
        stateSet->removeAttribute(::osg::StateAttribute::POLYGONMODE);
      }
    }
  }
}

//==============================================================================
void Inspector::setHidden(BodyNode* body, bool hidden)
{
  mBodyVisualState[body].hidden = hidden;
  for (std::size_t i = 0; i < body->getNumShapeNodes(); ++i) {
    auto* sn = body->getShapeNode(i);
    if (sn) {
      if (auto* visual = sn->getVisualAspect(true))
        visual->setHidden(hidden);
    }
  }
}

//==============================================================================
void Inspector::snapshotHighlightColors()
{
  mHighlightSnapshot.clear();
  if (!mSelected)
    return;

  for (std::size_t i = 0; i < mSelected->getNumShapeNodes(); ++i) {
    auto* sn = mSelected->getShapeNode(i);
    if (!sn)
      continue;
    if (auto* visual = sn->getVisualAspect())
      mHighlightSnapshot.emplace_back(sn, visual->getRGBA());
  }
}

//==============================================================================
void Inspector::restoreHighlight()
{
  for (const auto& [shapeNode, color] : mHighlightSnapshot) {
    if (auto* visual = shapeNode->getVisualAspect())
      visual->setRGBA(color);
  }
  mHighlightSnapshot.clear();
}

//==============================================================================
void Inspector::updateHighlight(double wallTimeSeconds)
{
  if (!mSelected || mHighlightSnapshot.empty())
    return;

  // A gentle ~0.6 Hz pulse blended toward a bright cyan-white tint, so the
  // selected body stays identifiable without permanently changing its color
  // (the base color is restored on deselect/switch).
  const double pulse = 0.5 + 0.5 * std::sin(wallTimeSeconds * 4.0);
  const Eigen::Vector3d tint(0.35, 0.92, 1.0);
  const double blend = 0.35 * pulse;

  for (const auto& [shapeNode, baseColor] : mHighlightSnapshot) {
    auto* visual = shapeNode->getVisualAspect();
    if (!visual)
      continue;
    Eigen::Vector4d color = baseColor;
    color.head<3>() = (1.0 - blend) * baseColor.head<3>() + blend * tint;
    visual->setRGBA(color);
  }
}

} // namespace dart_demos
