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

#include <dart/gui/grid_visual.hpp>
#include <dart/gui/im_gui_handler.hpp>
#include <dart/gui/im_gui_viewer.hpp>
#include <dart/gui/im_gui_widget.hpp>
#include <dart/gui/real_time_world_node.hpp>

#include <dart/io/read.hpp>

#if DART_HAVE_BULLET
  #include <dart/collision/bullet/bullet_collision_detector.hpp>
#endif
#if DART_HAVE_ODE
  #include <dart/collision/ode/ode_collision_detector.hpp>
#endif
#include <dart/gui/include_im_gui.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>
#include <dart/constraint/mimic_motor_constraint.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/math/helpers.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>

#include <dart/common/uri.hpp>

#include <CLI/CLI.hpp>
#include <Eigen/Core>
#include <osg/GraphicsContext>
#include <osg/Vec3>

#include <algorithm>
#include <iostream>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

using dart::common::Uri;
using dart::dynamics::BodyNode;
using dart::dynamics::Joint;
using dart::dynamics::MimicConstraintType;
using dart::dynamics::SkeletonPtr;
using dart::gui::ImGuiViewer;
using dart::gui::RealTimeWorldNode;
using dart::simulation::WorldPtr;
namespace {

struct MimicPairView
{
  std::string label;
  const Joint* follower{};
  const Joint* reference{};
  std::size_t followerDof = 0;
  std::size_t referenceDof = 0;
  const BodyNode* base{};
  Eigen::Vector3d baseStart = Eigen::Vector3d::Zero();
  Eigen::Vector3d baseNow = Eigen::Vector3d::Zero();
};

struct PaletteEntry
{
  std::string model;
  Eigen::Vector3d color;
  std::string label;
};

const std::vector<PaletteEntry>& getPalette()
{
  static const std::vector<PaletteEntry> palette = {
      {"pendulum_with_base", Eigen::Vector3d(0.7, 0.7, 0.7), "uncoupled"},
      {"pendulum_with_base_mimic_slow_follows_fast",
       Eigen::Vector3d(0.9, 0.35, 0.35),
       "slow follows fast"},
      {"pendulum_with_base_mimic_fast_follows_slow",
       Eigen::Vector3d(0.35, 0.5, 0.95),
       "fast follows slow"},
  };
  return palette;
}

Eigen::Vector3d translationOf(const BodyNode* bn)
{
  if (bn == nullptr) {
    return Eigen::Vector3d::Zero();
  }
  return bn->getWorldTransform().translation();
}

struct SolverConfig
{
  bool useOdeCollision = true;
  bool usePgsSolver = false;
};

void applyCollisionDetector(
    const SolverConfig& cfg, const dart::simulation::WorldPtr& world)
{
#if DART_HAVE_ODE
  if (cfg.useOdeCollision) {
    world->getConstraintSolver()->setCollisionDetector(
        dart::collision::OdeCollisionDetector::create());
    return;
  }
#endif

#if DART_HAVE_BULLET
  world->getConstraintSolver()->setCollisionDetector(
      dart::collision::BulletCollisionDetector::create());
#else
  (void)cfg;
  (void)world;
#endif
}

void applyLcpSolver(
    const SolverConfig& cfg, const dart::simulation::WorldPtr& world)
{
  auto* boxedSolver = dynamic_cast<dart::constraint::ConstraintSolver*>(
      world->getConstraintSolver());
  if (!boxedSolver) {
    return;
  }

  if (cfg.usePgsSolver) {
    boxedSolver->setLcpSolver(std::make_shared<dart::math::PgsSolver>());
    boxedSolver->setSecondaryLcpSolver(nullptr);
  } else {
    boxedSolver->setLcpSolver(std::make_shared<dart::math::DantzigSolver>());
    boxedSolver->setSecondaryLcpSolver(
        std::make_shared<dart::math::PgsSolver>());
  }
}

void retargetMimicsToBaseline(
    const WorldPtr& world, const std::string& baselineName)
{
  const auto baseline = world->getSkeleton(baselineName);
  if (!baseline) {
    std::cerr << "Baseline skeleton [" << baselineName << "] not found; "
              << "leaving parsed mimic joints untouched.\n";
    return;
  }

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (!skeleton) {
      continue;
    }

    for (std::size_t j = 0; j < skeleton->getNumJoints(); ++j) {
      auto* joint = skeleton->getJoint(j);
      if (!joint) {
        continue;
      }

      const auto props = joint->getMimicDofProperties();
      if (props.empty()) {
        continue;
      }

      if (skeleton == baseline) {
        // Leave the baseline uncoupled so it serves as the reference.
        std::vector<dart::dynamics::MimicDofProperties> clearedProps(
            joint->getNumDofs());
        joint->setMimicJointDofs(
            std::span<const dart::dynamics::MimicDofProperties>(clearedProps));
        joint->setActuatorType(dart::dynamics::Joint::FORCE);
        joint->setUseCouplerConstraint(false);
        continue;
      }

      bool updated = false;
      for (std::size_t dofIndex = 0; dofIndex < props.size(); ++dofIndex) {
        auto prop = props[dofIndex];
        if (prop.mReferenceJoint == nullptr) {
          continue;
        }

        auto* ref = baseline->getJoint(prop.mReferenceJoint->getName());
        if (!ref) {
          continue;
        }

        prop.mReferenceJoint = ref;
        joint->setMimicJointDof(dofIndex, prop);
        updated = true;
      }

      if (updated) {
        joint->setActuatorType(dart::dynamics::Joint::MIMIC);
        joint->setUseCouplerConstraint(false);
      }
    }
  }
}

std::vector<MimicPairView> collectMimicPairs(const WorldPtr& world)
{
  std::vector<MimicPairView> pairs;
  if (!world) {
    return pairs;
  }

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (!skeleton) {
      continue;
    }

    const auto* base = skeleton->getBodyNode("base");
    for (std::size_t j = 0; j < skeleton->getNumJoints(); ++j) {
      const auto* follower = skeleton->getJoint(j);
      if (!follower) {
        continue;
      }

      const auto props = follower->getMimicDofProperties();
      for (std::size_t k = 0; k < props.size(); ++k) {
        const auto& prop = props[k];
        if (prop.mReferenceJoint == nullptr) {
          continue;
        }

        MimicPairView view;
        view.label = skeleton->getName() + ": " + follower->getName() + "["
                     + std::to_string(k) + "] -> "
                     + prop.mReferenceJoint->getName() + "["
                     + std::to_string(prop.mReferenceDofIndex) + "]";
        view.follower = follower;
        view.reference = prop.mReferenceJoint;
        view.followerDof = k;
        view.referenceDof = prop.mReferenceDofIndex;
        view.base = base;
        view.baseStart = translationOf(base);
        pairs.push_back(view);
      }
    }
  }

  return pairs;
}

void tintBases(const WorldPtr& world)
{
  for (const auto& entry : getPalette()) {
    const auto skeleton = world->getSkeleton(entry.model);
    if (!skeleton) {
      continue;
    }

    auto* base = skeleton->getBodyNode("base");
    if (!base) {
      continue;
    }

    base->setColor(entry.color);
  }
}

class MimicOverlay : public dart::gui::ImGuiWidget
{
public:
  MimicOverlay(
      ImGuiViewer* viewer,
      WorldPtr world,
      std::vector<MimicPairView> pairs,
      std::string worldPath,
      SolverConfig cfg)
    : mViewer(viewer),
      mWorld(std::move(world)),
      mPairs(std::move(pairs)),
      mWorldPath(std::move(worldPath)),
      mConfig(std::move(cfg))
  {
    applyCollisionDetector(mConfig, mWorld);
    applyLcpSolver(mConfig, mWorld);
  }

  void render() override
  {
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_FirstUseEver);
    const auto windowFlags = ImGuiWindowFlags_NoCollapse;
    if (!ImGui::Begin("Mimic constraint debugger", nullptr, windowFlags)) {
      ImGui::End();
      return;
    }

    renderSimControls();
    ImGui::Text(
        "World time %.3f s | dt %.4f",
        mWorld->getTime(),
        mWorld->getTimeStep());
    ImGui::TextWrapped("SDF: %s", mWorldPath.c_str());

    renderSolverControls();
    renderLegend();
    ImGui::Separator();

    if (mPairs.empty()) {
      ImGui::TextWrapped(
          "No mimic joints were parsed from %s", mWorldPath.c_str());
    }

    renderMimicTable();

    ImGui::End();
  }

private:
  void resetAnchors()
  {
    for (auto& pair : mPairs) {
      pair.baseStart = translationOf(pair.base);
    }
  }

  void renderSimControls()
  {
    bool simulating = mViewer->isSimulating();
    if (ImGui::Checkbox("Run simulation", &simulating)) {
      mViewer->simulate(simulating);
    }
    ImGui::SameLine();
    if (ImGui::Button("Step 1")) {
      mViewer->simulate(false);
      mWorld->step();
      updateBasePositions();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset world")) {
      mWorld->reset();
      resetAnchors();
    }
  }

  void renderSolverControls()
  {
    ImGui::Separator();
    ImGui::Text("Collision / solver");

    bool odeSelected = mConfig.useOdeCollision;
#if DART_HAVE_ODE
    if (ImGui::Checkbox(
            "Use ODE collision (closer to Gazebo repro)", &odeSelected)) {
      mConfig.useOdeCollision = odeSelected;
      applyCollisionDetector(mConfig, mWorld);
    }
#else
    (void)odeSelected;
    ImGui::TextDisabled("ODE collision detector not built");
#endif
    bool pgs = mConfig.usePgsSolver;
    ImGui::SameLine();
    if (ImGui::Checkbox("Force PGS solver", &pgs)) {
      mConfig.usePgsSolver = pgs;
      applyLcpSolver(mConfig, mWorld);
    }

    const auto contacts = mWorld->getLastCollisionResult().getNumContacts();
    ImGui::Text("Contacts last step: %zu", contacts);

#if DART_HAVE_ODE
    if (mConfig.useOdeCollision) {
      ImGui::TextColored(
          ImVec4(0.9f, 0.7f, 0.2f, 1.0f),
          "ODE collision detector requested; requires "
          "DART_BUILD_COLLISION_ODE.");
    }
#endif
  }

  void renderMimicTable()
  {
    if (mPairs.empty()) {
      return;
    }

    ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                            | ImGuiTableFlags_SizingStretchProp
                            | ImGuiTableFlags_Resizable;
    if (ImGui::BeginTable("mimic_table", 6, flags)) {
      ImGui::TableSetupColumn("Pair");
      ImGui::TableSetupColumn("Reference (rad)");
      ImGui::TableSetupColumn("Follower (rad)");
      ImGui::TableSetupColumn("Error (rad)");
      ImGui::TableSetupColumn("Velocity error (rad/s)");
      ImGui::TableSetupColumn("Base drift (m)");
      ImGui::TableHeadersRow();

      for (auto& pair : mPairs) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(pair.label.c_str());

        const double middleRefPos
            = pair.reference->getPosition(pair.referenceDof);
        const double middleRefVel
            = pair.reference->getVelocity(pair.referenceDof);
        const double follower = pair.follower->getPosition(pair.followerDof);
        const double followerVel = pair.follower->getVelocity(pair.followerDof);
        const double error = follower - middleRefPos;
        const double velError = followerVel - middleRefVel;

        ImGui::TableSetColumnIndex(1);
        ImGui::Text(
            "%.3f (%.1f deg)",
            middleRefPos,
            dart::math::toDegree(middleRefPos));
        ImGui::TableSetColumnIndex(2);
        ImGui::Text(
            "%.3f (%.1f deg)", follower, dart::math::toDegree(follower));
        ImGui::TableSetColumnIndex(3);
        ImGui::TextColored(
            std::abs(error) > 0.2 ? ImVec4(1, 0.4f, 0.4f, 1)
                                  : ImVec4(0.7f, 0.9f, 0.7f, 1),
            "%.3f",
            error);
        ImGui::TableSetColumnIndex(4);
        ImGui::TextColored(
            std::abs(velError) > 0.2 ? ImVec4(1, 0.4f, 0.4f, 1)
                                     : ImVec4(0.7f, 0.9f, 0.7f, 1),
            "%.3f",
            velError);

        pair.baseNow = translationOf(pair.base);
        const double drift = (pair.baseNow - pair.baseStart).norm();
        ImGui::TableSetColumnIndex(5);
        ImGui::TextColored(
            drift > 0.25 ? ImVec4(1, 0.4f, 0.4f, 1)
                         : ImVec4(0.7f, 0.9f, 0.7f, 1),
            "%.3f (%.2f, %.2f, %.2f)",
            drift,
            pair.baseNow.x(),
            pair.baseNow.y(),
            pair.baseNow.z());
      }

      ImGui::EndTable();
    }
  }

  void renderLegend()
  {
    ImGui::Separator();
    ImGui::Text("Rigs:");
    for (const auto& entry : getPalette()) {
      ImGui::Bullet();
      ImGui::SameLine();
      ImGui::ColorButton(
          ("##color_" + entry.model).c_str(),
          ImVec4(entry.color.x(), entry.color.y(), entry.color.z(), 1.0f),
          ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoInputs,
          ImVec2(18, 18));
      ImGui::SameLine();
      ImGui::Text("%s (%s)", entry.model.c_str(), entry.label.c_str());
    }
  }

  void updateBasePositions()
  {
    for (auto& pair : mPairs) {
      pair.baseNow = translationOf(pair.base);
    }
  }

  ImGuiViewer* mViewer;
  WorldPtr mWorld;
  std::vector<MimicPairView> mPairs;
  std::string mWorldPath;
  SolverConfig mConfig;
};

} // namespace

//==============================================================================
int main(int argc, char* argv[])
{
  CLI::App app("Mimic pendulums example");
  double guiScale = 1.0;
  app.add_option("--gui-scale", guiScale, "Scale factor for ImGui widgets")
      ->check(CLI::PositiveNumber);
  CLI11_PARSE(app, argc, argv);

  const std::string worldUri
      = "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf";

  const auto world = dart::io::readWorld(Uri(worldUri));
  if (!world) {
    std::cerr << "Failed to load world from " << worldUri << "\n";
    return EXIT_FAILURE;
  }

  retargetMimicsToBaseline(world, "pendulum_with_base");
  tintBases(world);

  auto mimicPairs = collectMimicPairs(world);
  if (mimicPairs.empty()) {
    std::cerr << "No mimic joints found in " << worldUri << "\n";
  }

  auto* wsi = osg::GraphicsContext::getWindowingSystemInterface();
  if (wsi == nullptr) {
    std::cerr << "No OSG windowing system detected. "
              << "A valid display server is required to run this example.\n";
    return EXIT_FAILURE;
  }

  osg::ref_ptr<RealTimeWorldNode> worldNode = new RealTimeWorldNode(world);
  osg::ref_ptr<ImGuiViewer> viewer = new ImGuiViewer();
  viewer->setImGuiScale(static_cast<float>(guiScale));
  viewer->getImGuiHandler()->setFontScale(static_cast<float>(guiScale));
  viewer->addWorldNode(worldNode);
  viewer->addInstructionText("space: toggle simulation\n");

  auto grid
      = ::osg::ref_ptr<dart::gui::GridVisual>(new dart::gui::GridVisual());
  grid->setPlaneType(dart::gui::GridVisual::PlaneType::XY);
  grid->setNumCells(20);
  viewer->addAttachment(grid);

  viewer->setUpViewInWindow(0, 0, 1280, 720);
  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(8.0f, -7.0f, 4.0f),
      ::osg::Vec3(0.5f, 0.0f, 1.5f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  viewer->simulate(true);
  viewer->getImGuiHandler()->addWidget(
      std::make_shared<MimicOverlay>(
          viewer.get(),
          world,
          std::move(mimicPairs),
          worldUri,
          SolverConfig{}));

  if (!viewer->isRealized()) {
    viewer->realize();
  }

  osg::ref_ptr<osg::GraphicsContext> gc
      = viewer->getCamera() ? viewer->getCamera()->getGraphicsContext()
                            : nullptr;
  if (!viewer->isRealized() || !gc || !gc->valid()) {
    std::cerr << "Failed to create an OSG window. Ensure DISPLAY is set or "
              << "use a virtual framebuffer.\n";
    return EXIT_FAILURE;
  }

  const int runResult = viewer->run();
  if (runResult != 0) {
    std::cerr << "Viewer exited early (status " << runResult << ")\n";
  }

  return runResult;
}
