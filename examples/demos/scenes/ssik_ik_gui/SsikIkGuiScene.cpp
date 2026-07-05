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

// Ported from examples/ssik_ik_gui: analytical IK against 19 prebuilt robot
// arms (from the optional ssik Python package) driven by a draggable
// InteractiveFrame gizmo target, with a full ImGui control panel (arm
// selector, target pose fields, solver options, solution browser).
//
// Deviations from the original: see SsikBridge.hpp's file comment for the
// Python-interpreter lifetime and threading deviations (lazy one-time
// Py_Initialize, never finalized; this host is single-threaded end to end
// already, satisfying the original's SingleThreaded requirement). The
// original's own headless mode (--headless/--shot), --arm CLI flag, ImGui
// menu bar (Exit/About), and --gui-scale flag are dropped in favor of the
// host's generic --cycle-scenes/--headless/--gui-scale equivalents and
// Rebuild button; the Arm combo box already covers picking an initial arm
// interactively. The original's --self-test is also dropped, and unlike the
// others it has no equivalent here: --cycle-scenes visits this scene once
// with the default arm and cannot fail the process, so the original's CI
// gate (every one of the 19 arms loads and yields >0 total solutions,
// enforced via a nonzero exit code) is lost with no replacement -- arm-load
// and solve failures only surface as on-panel status text. World gravity is
// zero (never stepped, purely
// kinematic; see the file comment on this pattern in
// ContactInverseDynamicsScene.cpp) and physics stepping is disabled for this
// scene the same way as WamIkFastScene.cpp. The gizmo-drag detection and
// solver-options panel are ported field-for-field from the original's
// SsikIkWidget; only the ImGui window chrome (auto-resize, menu bar) is
// simplified to fit inside the host's fixed scene panel.

// SsikBridge.hpp includes Python.h, which must be the first thing this
// translation unit sees (see SsikBridge.hpp's file comment) -- ahead of
// Scenes.hpp/dart.hpp and the standard headers they pull in transitively.
// clang-format off
#include "SsikBridge.hpp"
// clang-format on
#include "../Scenes.hpp"
#include "ArmSkeleton.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <algorithm>
#include <array>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <cmath>

namespace dart_demos {

namespace {

using namespace ssik_ik_gui; // NOLINT: scene-local convenience.

constexpr double kPi = 3.14159265358979323846;

//==============================================================================
Eigen::Isometry3d makeTransform(
    const std::array<float, 3>& xyz, const std::array<float, 3>& rpyDegrees)
{
  const double roll = rpyDegrees[0] * kPi / 180.0;
  const double pitch = rpyDegrees[1] * kPi / 180.0;
  const double yaw = rpyDegrees[2] * kPi / 180.0;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  tf.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                 * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
  return tf;
}

//==============================================================================
void setTargetControlsFromTransform(
    const Eigen::Isometry3d& tf,
    std::array<float, 3>& xyz,
    std::array<float, 3>& rpyDegrees)
{
  xyz[0] = static_cast<float>(tf.translation().x());
  xyz[1] = static_cast<float>(tf.translation().y());
  xyz[2] = static_cast<float>(tf.translation().z());

  const Eigen::Vector3d zyx = tf.linear().eulerAngles(2, 1, 0);
  rpyDegrees[0] = static_cast<float>(zyx[2] * 180.0 / kPi);
  rpyDegrees[1] = static_cast<float>(zyx[1] * 180.0 / kPi);
  rpyDegrees[2] = static_cast<float>(zyx[0] * 180.0 / kPi);
}

//==============================================================================
std::string formatVector(const Eigen::VectorXd& vector)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4);
  for (int i = 0; i < vector.size(); ++i) {
    if (i > 0)
      stream << ", ";
    stream << vector[i];
  }
  return stream.str();
}

//==============================================================================
Eigen::Isometry3d offsetPose(
    const Eigen::Isometry3d& pose, const Eigen::Vector3d& offset)
{
  Eigen::Isometry3d out = pose;
  out.translation() += offset;
  return out;
}

//==============================================================================
/// Per-instance state, ported from the original's SsikIkWidget (minus the
/// app chrome dropped per the file comment).
struct SsikState
{
  dart::gui::osg::InteractiveFramePtr target;
  dart::dynamics::SimpleFramePtr solvedFrame;
  ArmSkeleton arm;

  // Constructed lazily in onActivate: SsikPythonBridge's constructor touches
  // the interpreter, which this scene only needs once actually activated.
  std::unique_ptr<SsikPythonBridge> bridge;

  std::vector<ArmSpec> arms = getArmSpecs();
  int armIndex = 0;
  ArmInfo info;
  bool loaded = false;

  bool autoSolve = true;
  bool respectLimits = true;
  bool allowRefinement = false;
  bool useSeed = false;
  int maxSolutions = 0;
  int seedMetricIndex = 0;
  int selectedSolution = 0;

  Eigen::Isometry3d appliedTargetTf = Eigen::Isometry3d::Identity();
  std::array<float, 3> targetXyz{{0.0f, 0.0f, 0.0f}};
  std::array<float, 3> targetRpyDegrees{{0.0f, 0.0f, 0.0f}};
  Eigen::VectorXd seed;
  std::vector<SsikSolution> solutions;
  std::string status;
};

//==============================================================================
void syncTargetFrame(SsikState& state)
{
  state.target->setTransform(
      makeTransform(state.targetXyz, state.targetRpyDegrees));
  // Record the panel-driven pose so the drag-detection in renderPanel only
  // reacts to direct gizmo dragging, not to our own control edits.
  state.appliedTargetTf = state.target->getTransform();
}

//==============================================================================
void updateSolvedFrame(SsikState& state)
{
  state.arm.showSolutions(state.solutions, state.selectedSolution);

  if (state.selectedSolution < 0
      || state.selectedSolution >= static_cast<int>(state.solutions.size()))
    return;

  const SsikSolution& solution = state.solutions[state.selectedSolution];
  if (solution.hasFkPose)
    state.solvedFrame->setTransform(solution.fkPose);
}

//==============================================================================
void solveCurrent(SsikState& state)
{
  if (!state.loaded || !state.bridge)
    return;

  if (state.seed.size() != state.info.dof)
    state.seed = Eigen::VectorXd::Zero(state.info.dof);

  static const char* const metrics[] = {"wrap_linf", "wrap_l2"};
  std::string error;
  if (!state.bridge->solve(
          state.target->getTransform(),
          state.maxSolutions,
          state.useSeed,
          state.seed,
          state.respectLimits,
          state.allowRefinement,
          metrics[state.seedMetricIndex],
          state.solutions,
          error)) {
    state.solutions.clear();
    state.status = "Solve failed: " + error;
    state.arm.showSolutions(state.solutions, -1);
    return;
  }

  std::ostringstream stream;
  stream << "Solutions: " << state.solutions.size();
  if (state.maxSolutions == 0)
    stream << " (all branches)";
  state.status = stream.str();

  if (!state.solutions.empty())
    state.selectedSolution = std::min<int>(
        state.selectedSolution, static_cast<int>(state.solutions.size()) - 1);
  updateSolvedFrame(state);
}

//==============================================================================
void loadSelectedArm(SsikState& state)
{
  state.loaded = false;
  state.solutions.clear();
  state.selectedSolution = 0;
  state.status = "Loading ssik.prebuilt." + state.arms[state.armIndex].module;

  std::string error;
  if (!state.bridge
      || !state.bridge->loadArm(
          state.arms[state.armIndex].module, state.info, error)) {
    state.status
        = "Install ssik in this Python environment to use this scene: " + error;
    state.info = ArmInfo();
    state.arm.build({});
    setTargetControlsFromTransform(
        state.info.home, state.targetXyz, state.targetRpyDegrees);
    syncTargetFrame(state);
    state.solvedFrame->setTransform(
        offsetPose(state.info.home, Eigen::Vector3d(0.0, 0.25, 0.0)));
    return;
  }

  state.loaded = true;
  state.seed = Eigen::VectorXd::Zero(state.info.dof);
  state.arm.build(state.info.chain);
  setTargetControlsFromTransform(
      state.info.home, state.targetXyz, state.targetRpyDegrees);
  syncTargetFrame(state);
  state.solvedFrame->setTransform(state.info.home);
  solveCurrent(state);
}

//==============================================================================
void itemTooltip(const char* text)
{
  ImGui::SetItemTooltip("%s", text);
}

//==============================================================================
void renderArmSelector(SsikState& state)
{
  if (ImGui::BeginCombo("Arm", state.arms[state.armIndex].label.c_str())) {
    for (std::size_t i = 0; i < state.arms.size(); ++i) {
      const bool selected = static_cast<int>(i) == state.armIndex;
      if (ImGui::Selectable(state.arms[i].label.c_str(), selected)) {
        state.armIndex = static_cast<int>(i);
        loadSelectedArm(state);
      }
      if (selected)
        ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }
  itemTooltip(
      "Robot arm to control. Each entry is an ssik prebuilt analytical IK "
      "module; switching rebuilds the arm and re-solves immediately.");

  if (state.loaded) {
    ImGui::Text(
        "Module: ssik.prebuilt.%s", state.arms[state.armIndex].module.c_str());
    itemTooltip(
        "The Python module providing this arm's closed-form IK and "
        "kinematics.");
    ImGui::Text(
        "Chain: %s -> %s",
        state.info.baseLink.c_str(),
        state.info.eeLink.c_str());
    itemTooltip(
        "Kinematic chain solved: from the base link to the end-effector "
        "(tip) link.");
    ImGui::Text("DOF: %d", state.info.dof);
    itemTooltip(
        "Degrees of freedom: the number of independently actuated joints in "
        "the arm.");
  } else {
    ImGui::TextColored(
        ImVec4(1.0f, 0.78f, 0.20f, 1.0f), "ssik is not available");
    ImGui::TextWrapped("%s", state.status.c_str());
    ImGui::TextWrapped(
        "Install into this Pixi environment with: "
        "pixi run python -m pip install ssik");
  }
}

//==============================================================================
void renderTargetControls(SsikState& state)
{
  bool changed = false;

  // Commit each drag only when every component is finite: a ctrl+click-typed
  // 'nan'/'inf' would otherwise flow through makeTransform into the gizmo's
  // OSG transform (corrupting its bounds) and make appliedTargetTf NaN, so
  // the drag-detection check never settles and Auto solve fails every frame.
  std::array<float, 3> xyz = state.targetXyz;
  if (ImGui::DragFloat3("Position", xyz.data(), 0.005f) && std::isfinite(xyz[0])
      && std::isfinite(xyz[1]) && std::isfinite(xyz[2])) {
    state.targetXyz = xyz;
    changed = true;
  }
  itemTooltip(
      "Desired end-effector position (x, y, z) in meters, in the arm's base "
      "frame. Drag a value to edit, or drag the gizmo in the 3D view.");
  std::array<float, 3> rpy = state.targetRpyDegrees;
  if (ImGui::DragFloat3("RPY deg", rpy.data(), 0.5f) && std::isfinite(rpy[0])
      && std::isfinite(rpy[1]) && std::isfinite(rpy[2])) {
    state.targetRpyDegrees = rpy;
    changed = true;
  }
  itemTooltip(
      "Desired end-effector orientation as roll, pitch, yaw (degrees) about "
      "the X, Y, Z axes.");

  if (changed) {
    syncTargetFrame(state);
    if (state.autoSolve)
      solveCurrent(state);
  }

  if (ImGui::Button("Home target")) {
    setTargetControlsFromTransform(
        state.info.home, state.targetXyz, state.targetRpyDegrees);
    syncTargetFrame(state);
    if (state.autoSolve)
      solveCurrent(state);
  }
  itemTooltip("Reset the target to the arm's home (rest) pose.");
  ImGui::SameLine();
  if (ImGui::Button("Read dragged")) {
    setTargetControlsFromTransform(
        state.target->getTransform(), state.targetXyz, state.targetRpyDegrees);
    if (state.autoSolve)
      solveCurrent(state);
  }
  itemTooltip(
      "Copy the pose of the draggable gizmo in the 3D view back into these "
      "fields.");
}

//==============================================================================
void renderSolverControls(SsikState& state)
{
  bool changed = false;
  changed |= ImGui::Checkbox("Auto solve", &state.autoSolve);
  itemTooltip(
      "Re-run the IK automatically whenever the target or an option changes "
      "(including while dragging the gizmo).");
  changed |= ImGui::Checkbox("Respect limits", &state.respectLimits);
  itemTooltip(
      "Discard solutions that violate the arm's joint limits. Turn off to "
      "also see out-of-limit branches.");
  changed |= ImGui::Checkbox("Allow refinement", &state.allowRefinement);
  itemTooltip(
      "Let ssik numerically refine near-singular solutions for extra "
      "accuracy, at some extra cost.");
  changed |= ImGui::Checkbox("Use q seed", &state.useSeed);
  itemTooltip(
      "Provide a seed (reference) joint configuration so solutions are "
      "ranked by closeness to it, keeping motion continuous.");
  changed |= ImGui::SliderInt(
      "Max solutions",
      &state.maxSolutions,
      0,
      256,
      "%d",
      ImGuiSliderFlags_AlwaysClamp);
  itemTooltip(
      "Maximum number of IK branches to return (0 = all). Analytical IK "
      "yields several distinct elbow/wrist configurations for one target.");

  static const char* const metrics[] = {"wrap_linf", "wrap_l2"};
  changed |= ImGui::Combo("Seed metric", &state.seedMetricIndex, metrics, 2);
  itemTooltip(
      "How distance to the seed is measured when ranking solutions: "
      "wrap_linf = largest single-joint angular distance; wrap_l2 = "
      "Euclidean distance across joints.");

  if (ImGui::Button("Solve"))
    solveCurrent(state);
  itemTooltip("Run the analytical IK once for the current target.");

  ImGui::SameLine();
  if (ImGui::Button("Seed from selection")) {
    if (state.selectedSolution >= 0
        && state.selectedSolution < static_cast<int>(state.solutions.size())) {
      state.seed = state.solutions[state.selectedSolution].q;
      state.useSeed = true;
      if (state.autoSolve)
        solveCurrent(state);
    }
  }
  itemTooltip(
      "Use the currently selected solution below as the seed configuration "
      "and enable 'Use q seed'.");

  if (changed && state.autoSolve)
    solveCurrent(state);
}

//==============================================================================
void renderSolutions(SsikState& state)
{
  ImGui::TextWrapped("%s", state.status.c_str());
  if (state.solutions.empty())
    return;

  const int maxIndex = static_cast<int>(state.solutions.size()) - 1;

  if (state.solutions.size() > 1) {
    if (ImGui::SliderInt(
            "Solution",
            &state.selectedSolution,
            0,
            maxIndex,
            "%d",
            ImGuiSliderFlags_AlwaysClamp))
      updateSolvedFrame(state);
    itemTooltip(
        "Browse the IK solutions: distinct joint configurations that all "
        "reach the same target. The selected one is opaque; the others are "
        "shown translucent.");
  }

  // Clamp before the same-frame index below: AlwaysClamp still lets a
  // ctrl+click-typed out-of-range value land for one frame (and a typed
  // negative would persist), which would be an out-of-bounds solutions read.
  state.selectedSolution = std::clamp(state.selectedSolution, 0, maxIndex);

  const SsikSolution& solution = state.solutions[state.selectedSolution];
  ImGui::Text("fk residual: %.3e", solution.fkResidual);
  itemTooltip(
      "Forward-kinematics residual: how far this solution's actual "
      "end-effector pose is from the target (smaller is better).");
  ImGui::Text("refinement: %s", solution.refinementUsed.c_str());
  itemTooltip("Whether ssik numerically refined this solution.");
  ImGui::TextWrapped("q: %s", formatVector(solution.q).c_str());
  itemTooltip("Joint angles of this solution in radians, one per DOF.");

  if (solution.hasFkPose) {
    const Eigen::Vector3d& p = solution.fkPose.translation();
    ImGui::Text("FK xyz: %.3f, %.3f, %.3f", p.x(), p.y(), p.z());
    itemTooltip(
        "End-effector position computed by forward kinematics from q; should "
        "match the target position.");
  }
}

} // namespace

//==============================================================================
DemoScene makeSsikIkGuiScene()
{
  DemoScene scene;
  scene.id = "ssik_ik_gui";
  scene.title = "ssik Analytical IK";
  scene.category = "Control & IK";
  scene.summary
      = "Analytical IK against 19 prebuilt robot arms (optional "
        "ssik package).";

  scene.factory = [] {
    auto world = dart::simulation::World::create();
    world->setGravity(Eigen::Vector3d::Zero());

    Eigen::Isometry3d defaultTarget = Eigen::Isometry3d::Identity();
    defaultTarget.translation() = Eigen::Vector3d(0.45, 0.0, 0.35);

    auto state = std::make_shared<SsikState>();
    state->target = std::make_shared<dart::gui::osg::InteractiveFrame>(
        dart::dynamics::Frame::World(), "ssik_target", defaultTarget, 0.25);
    world->addSimpleFrame(state->target);

    state->solvedFrame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(),
        "ssik_fk_solution",
        offsetPose(defaultTarget, Eigen::Vector3d(0.0, 0.25, 0.0)));
    state->solvedFrame->setShape(
        std::make_shared<dart::dynamics::SphereShape>(0.05));
    state->solvedFrame->createVisualAspect()->setColor(
        Eigen::Vector4d(0.15, 0.85, 1.0, 0.85));
    world->addSimpleFrame(state->solvedFrame);

    state->arm.setWorld(world);

    ::osg::ref_ptr<dart::gui::osg::GridVisual> grid
        = new dart::gui::osg::GridVisual();
    grid->setPlaneType(dart::gui::osg::GridVisual::PlaneType::XY);

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(2.6, -3.0, 1.6),
        ::osg::Vec3d(0.1, 0.0, 0.55),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.onActivate = [state, grid](DemoHostContext& ctx) {
      auto* viewer = ctx.viewer();

      // Purely kinematic: keep the host's global Play button from stepping
      // physics under this scene (restored on teardown).
      viewer->allowSimulation(false);
      ctx.addTeardown([viewer] { viewer->allowSimulation(true); });

      ctx.addAttachment(grid.get());

      if (auto* dnd = viewer->enableDragAndDrop(state->target.get()))
        ctx.addTeardown([viewer, dnd] { viewer->disableDragAndDrop(dnd); });

      // The Python bridge is constructed on first activation only (numpy
      // must be importable; ssik itself is optional and checked per-arm).
      try {
        state->bridge = std::make_unique<SsikPythonBridge>();
        loadSelectedArm(*state);
      } catch (const std::exception& e) {
        state->status
            = std::string("Failed to start the Python bridge: ") + e.what();
      }
    };

    setup.renderPanel = [state] {
      // If the target gizmo was dragged in the 3D view, its frame transform
      // no longer matches the panel controls. Pick that up, reflect it in
      // the controls, and re-solve so the arm follows the dragged target --
      // ported from the original's render(), which ran this check at the top
      // of the ImGui widget every frame; here it runs at the top of
      // renderPanel for the same per-frame cadence (see
      // ContactInverseDynamicsScene.cpp's file comment).
      const Eigen::Isometry3d targetTf = state->target->getTransform();
      if (!targetTf.isApprox(state->appliedTargetTf, 1e-9)) {
        setTargetControlsFromTransform(
            targetTf, state->targetXyz, state->targetRpyDegrees);
        state->appliedTargetTf = targetTf;
        if (state->autoSolve)
          solveCurrent(*state);
      }

      renderArmSelector(*state);
      ImGui::Separator();
      renderTargetControls(*state);
      ImGui::Separator();
      renderSolverControls(*state);
      ImGui::Separator();
      renderSolutions(*state);
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
