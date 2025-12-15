/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/gui/GridVisual.hpp>
#include <dart/gui/ImGuiHandler.hpp>
#include <dart/gui/ImGuiViewer.hpp>
#include <dart/gui/ImGuiWidget.hpp>
#include <dart/gui/IncludeImGui.hpp>
#include <dart/gui/WorldNode.hpp>

#include <dart/All.hpp>

#include <CLI/CLI.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

namespace {

struct CaseMetrics
{
  double translationalBlockError{0.0};
  double worldJacobianMaxError{0.0};
  double worldJacobianClassicDerivMaxError{0.0};
  double relativeJacobianDotMaxError{0.0};
};

struct FreeJointCase
{
  std::string name;
  dart::dynamics::SkeletonPtr skeleton;
  dart::dynamics::FreeJoint* joint{nullptr};
  dart::dynamics::BodyNode* body{nullptr};

  dart::dynamics::SkeletonPtr referenceSkeleton;
  dart::dynamics::FreeJoint* referenceJoint{nullptr};
  dart::dynamics::ShapeNode* referenceShapeNode{nullptr};

  Eigen::Vector6d initialPositions{Eigen::Vector6d::Zero()};
  Eigen::Vector6d initialVelocities{Eigen::Vector6d::Zero()};
  double initialEnergy{0.0};

  Eigen::Vector3d jacobianOffset{Eigen::Vector3d(0.1, 0.0, 0.0)};
  CaseMetrics metrics;
};

std::string formatDouble(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

FreeJointCase makeCase(
    const std::string& name,
    const Eigen::Isometry3d& pose,
    const Eigen::Vector6d& velocities,
    const Eigen::Vector4d& color)
{
  using namespace dart::dynamics;

  FreeJointCase data;
  data.name = name;

  const Eigen::Vector3d boxSize = Eigen::Vector3d(0.3, 0.2, 0.15);
  data.skeleton = Skeleton::create(name);
  auto pair = data.skeleton->createJointAndBodyNodePair<FreeJoint>();
  data.joint = pair.first;
  data.body = pair.second;

  data.body->setMass(1.0);
  data.body->setMomentOfInertia(0.02, 0.04, 0.06);
  data.body->setCollidable(false);

  auto shapeNode = data.body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(std::make_shared<BoxShape>(boxSize));
  shapeNode->getVisualAspect()->setColor(color);

  data.joint->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
  data.joint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
  data.initialPositions = FreeJoint::convertToPositions(pose);
  data.joint->setPositions(data.initialPositions);

  data.initialVelocities = velocities;
  data.joint->setVelocities(data.initialVelocities);

  data.initialEnergy = data.skeleton->computeKineticEnergy();

  data.referenceSkeleton = Skeleton::create(name + " (ground truth)");
  data.referenceSkeleton->setMobile(false);
  auto referencePair
      = data.referenceSkeleton->createJointAndBodyNodePair<FreeJoint>();
  data.referenceJoint = referencePair.first;

  auto* referenceBody = referencePair.second;
  referenceBody->setCollidable(false);

  Eigen::Vector4d referenceColor = color;
  referenceColor.head<3>()
      = 0.25 * Eigen::Vector3d::Ones() + 0.75 * referenceColor.head<3>();
  referenceColor[3] = std::min(referenceColor[3], 0.15);

  data.referenceShapeNode = referenceBody->createShapeNodeWith<VisualAspect>(
      std::make_shared<BoxShape>(boxSize * 1.05));
  data.referenceShapeNode->getVisualAspect()->setColor(referenceColor);
  data.referenceShapeNode->getVisualAspect()->setShadowed(false);

  data.referenceJoint->setTransformFromParentBodyNode(
      Eigen::Isometry3d::Identity());
  data.referenceJoint->setTransformFromChildBodyNode(
      Eigen::Isometry3d::Identity());
  data.referenceJoint->setPositions(data.initialPositions);
  data.referenceJoint->setVelocities(Eigen::Vector6d::Zero());

  return data;
}

Eigen::Isometry3d computeExpectedTransform(const FreeJointCase& data, double t)
{
  using dart::dynamics::FreeJoint;

  const Eigen::Isometry3d initial
      = FreeJoint::convertToTransform(data.initialPositions);
  Eigen::Isometry3d expected = initial;
  expected.linear()
      = dart::math::expMapRot(data.initialVelocities.head<3>() * t)
        * initial.linear();
  expected.translation()
      = initial.translation() + data.initialVelocities.tail<3>() * t;
  return expected;
}

CaseMetrics computeMetrics(const FreeJointCase& data, double dt)
{
  using dart::dynamics::FreeJoint;

  CaseMetrics metrics;

  const dart::math::Jacobian worldJacobian = data.body->getWorldJacobian();
  const Eigen::Matrix3d translationBlock
      = worldJacobian.bottomRows<3>().rightCols<3>();
  metrics.translationalBlockError
      = (translationBlock - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();

  auto clone = data.skeleton->cloneSkeleton(data.name + "_clone");
  clone->setPositions(data.skeleton->getPositions());
  clone->setVelocities(data.skeleton->getVelocities());

  auto* cloneBody = clone->getBodyNode(0);
  auto* cloneJoint = dynamic_cast<FreeJoint*>(clone->getJoint(0));

  const Eigen::Vector6d savedPositions = cloneJoint->getPositions();
  const Eigen::Vector6d savedVelocities = cloneJoint->getVelocities();

  const Eigen::Vector3d offset = data.jacobianOffset;
  const dart::math::Jacobian J = cloneBody->getWorldJacobian(offset);

  double maxJacError = 0.0;
  for (int i = 0; i < 6; ++i) {
    Eigen::Vector6d velocities = Eigen::Vector6d::Zero();
    velocities[i] = 1.0;
    cloneJoint->setVelocities(velocities);

    const Eigen::Isometry3d T0 = cloneBody->getWorldTransform();
    const Eigen::Matrix3d R0 = T0.linear();
    const Eigen::Vector3d p0 = T0 * offset;

    clone->integratePositions(dt);

    const Eigen::Isometry3d T1 = cloneBody->getWorldTransform();
    const Eigen::Matrix3d R1 = T1.linear();
    const Eigen::Vector3d p1 = T1 * offset;

    Eigen::Vector6d numeric;
    numeric.head<3>() = dart::math::logMap(R1 * R0.transpose()) / dt;
    numeric.tail<3>() = (p1 - p0) / dt;

    maxJacError
        = std::max(maxJacError, (numeric - J.col(i)).cwiseAbs().maxCoeff());

    cloneJoint->setPositions(savedPositions);
  }
  metrics.worldJacobianMaxError = maxJacError;

  cloneJoint->setPositions(savedPositions);
  cloneJoint->setVelocities(savedVelocities);

  {
    const Eigen::Vector6d q0 = cloneJoint->getPositions();
    const Eigen::Vector6d qdot = cloneJoint->getVelocities();
    const dart::math::Jacobian dJ = cloneBody->getJacobianClassicDeriv(offset);

    clone->integratePositions(dt);
    const Eigen::Vector6d VPlus = cloneBody->getSpatialVelocity(
        offset, dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
    cloneJoint->setPositions(q0);

    clone->integratePositions(-dt);
    const Eigen::Vector6d VMinus = cloneBody->getSpatialVelocity(
        offset, dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
    cloneJoint->setPositions(q0);

    const Eigen::Vector6d numeric = (VPlus - VMinus) / (2.0 * dt);
    const Eigen::Vector6d predicted = dJ * qdot;
    metrics.worldJacobianClassicDerivMaxError
        = (numeric - predicted).cwiseAbs().maxCoeff();
  }

  const Eigen::Matrix6d relJacobian0 = cloneJoint->getRelativeJacobian();
  const Eigen::Matrix6d relJacobianDot
      = cloneJoint->getRelativeJacobianTimeDeriv();
  clone->integratePositions(dt);
  const Eigen::Matrix6d relJacobian1 = cloneJoint->getRelativeJacobian();
  const Eigen::Matrix6d relJacobianFd = (relJacobian1 - relJacobian0) / dt;

  metrics.relativeJacobianDotMaxError
      = (relJacobianDot - relJacobianFd).cwiseAbs().maxCoeff();

  return metrics;
}

} // namespace

class FreeJointCasesWidget : public dart::gui::ImGuiWidget
{
public:
  FreeJointCasesWidget(
      osg::ref_ptr<dart::gui::ImGuiViewer> viewer,
      osg::ref_ptr<dart::gui::GridVisual> grid,
      dart::simulation::WorldPtr world,
      std::vector<FreeJointCase> cases,
      double numericDt,
      double guiScale)
    : mViewer(std::move(viewer)),
      mGrid(std::move(grid)),
      mWorld(std::move(world)),
      mCases(std::move(cases)),
      mNumericDt(numericDt),
      mGuiScale(guiScale)
  {
    const int scaleKey = static_cast<int>(std::lround(guiScale * 100.0));
    mWindowLabel
        = "FreeJoint cases##free_joint_cases_" + std::to_string(scaleKey);

    applyGroundTruthVisibility();
    recomputeMetrics();
  }

  void render() override
  {
    updateGroundTruth();

    const float windowScale = static_cast<float>(mGuiScale);
    const ImGuiIO& io = ImGui::GetIO();
    const ImVec2 displaySize = io.DisplaySize;

    const ImVec2 desiredPos(10.0f * windowScale, 20.0f * windowScale);
    const ImVec2 desiredSize(520.0f * windowScale, 540.0f * windowScale);
    const ImVec2 maxSize(
        std::max(1.0f, displaySize.x - desiredPos.x),
        std::max(1.0f, displaySize.y - desiredPos.y));
    const ImVec2 clampedSize(
        std::min(desiredSize.x, maxSize.x), std::min(desiredSize.y, maxSize.y));

    ImGui::SetNextWindowPos(desiredPos, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(clampedSize, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.85f);

    if (!ImGui::Begin(
            mWindowLabel.c_str(),
            nullptr,
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_MenuBar)) {
      ImGui::End();
      return;
    }

    if (ImGui::BeginMenuBar()) {
      if (ImGui::BeginMenu("Menu")) {
        if (ImGui::MenuItem("Exit"))
          mViewer->setDone(true);
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("About DART"))
          mViewer->showAbout();
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }

    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
      int state = mViewer->isSimulating() ? 0 : 1;
      if (mViewer->isAllowingSimulation()) {
        if (ImGui::RadioButton("Play", &state, 0) && !mViewer->isSimulating())
          mViewer->simulate(true);
        ImGui::SameLine();
        if (ImGui::RadioButton("Pause", &state, 1) && mViewer->isSimulating())
          mViewer->simulate(false);
      }

      ImGui::Text("Time: %s", formatDouble(mWorld->getTime(), 3).c_str());

      if (ImGui::Button("Reset cases")) {
        resetCases();
      }
      ImGui::SameLine();
      if (ImGui::Button("Numeric checks")) {
        recomputeMetrics();
      }
    }

    if (ImGui::CollapsingHeader(
            "Ground truth", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Checkbox("Show reference bodies", &mShowGroundTruth)) {
        applyGroundTruthVisibility();
        updateGroundTruth();
      }

      ImGui::TextWrapped(
          "Reference bodies are kinematic visuals whose pose is computed from "
          "the initial world-frame velocities (ω, v).");
    }

    if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
      bool showGrid = mGrid && mGrid->isDisplayed();
      if (ImGui::Checkbox("Show grid", &showGrid) && mGrid)
        mGrid->display(showGrid);
    }

    if (ImGui::CollapsingHeader("Cases", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::BeginTable(
              "freejoint_cases_table",
              7,
              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                  | ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("Case");
        ImGui::TableSetupColumn("KE");
        ImGui::TableSetupColumn("dKE/KE0");
        ImGui::TableSetupColumn("|Jt-I|inf");
        ImGui::TableSetupColumn("|Jnum-J|inf");
        ImGui::TableSetupColumn("|dVdot-FD|inf");
        ImGui::TableSetupColumn("|relJdot-FD|inf");
        ImGui::TableHeadersRow();

        for (const auto& c : mCases) {
          const double energy = c.skeleton->computeKineticEnergy();
          const double denom = std::max(1.0, c.initialEnergy);
          const double relEnergyError = (energy - c.initialEnergy) / denom;

          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(c.name.c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(formatDouble(energy, 4).c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(formatDouble(relEnergyError, 4).c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(
              formatDouble(c.metrics.translationalBlockError, 3).c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(
              formatDouble(c.metrics.worldJacobianMaxError, 3).c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(
              formatDouble(c.metrics.worldJacobianClassicDerivMaxError, 3)
                  .c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(
              formatDouble(c.metrics.relativeJacobianDotMaxError, 3).c_str());
        }

        ImGui::EndTable();
      }
    }

    ImGui::End();
  }

private:
  void resetCases()
  {
    mWorld->reset();
    for (auto& c : mCases) {
      c.joint->setPositions(c.initialPositions);
      c.joint->setVelocities(c.initialVelocities);
      c.joint->setAccelerations(Eigen::Vector6d::Zero());
      c.initialEnergy = c.skeleton->computeKineticEnergy();

      if (c.referenceJoint) {
        c.referenceJoint->setPositions(c.initialPositions);
        c.referenceJoint->setVelocities(Eigen::Vector6d::Zero());
      }
    }

    updateGroundTruth();
  }

  void applyGroundTruthVisibility()
  {
    for (auto& c : mCases) {
      if (!c.referenceShapeNode)
        continue;
      c.referenceShapeNode->getVisualAspect()->setHidden(!mShowGroundTruth);
    }
  }

  void updateGroundTruth()
  {
    if (!mShowGroundTruth)
      return;

    const double t = mWorld->getTime();
    for (auto& c : mCases) {
      if (!c.referenceJoint)
        continue;

      const Eigen::Isometry3d expected = computeExpectedTransform(c, t);
      c.referenceJoint->setPositions(
          dart::dynamics::FreeJoint::convertToPositions(expected));
    }
  }

  void recomputeMetrics()
  {
    for (auto& c : mCases)
      c.metrics = computeMetrics(c, mNumericDt);
  }

  osg::ref_ptr<dart::gui::ImGuiViewer> mViewer;
  osg::ref_ptr<dart::gui::GridVisual> mGrid;
  dart::simulation::WorldPtr mWorld;
  std::vector<FreeJointCase> mCases;
  double mNumericDt;
  double mGuiScale;
  std::string mWindowLabel;
  bool mShowGroundTruth{true};
};

int main(int argc, char* argv[])
{
  using namespace dart;

  CLI::App app("FreeJoint cases viewer");
  double guiScale = 1.0;
  double numericDt = 1e-6;
  double simulationDt = 1e-3;
  app.add_option("--gui-scale", guiScale, "Scale factor for ImGui widgets")
      ->check(CLI::PositiveNumber);
  app.add_option("--numeric-dt", numericDt, "Finite difference dt for checks")
      ->check(CLI::PositiveNumber);
  app.add_option("--dt", simulationDt, "Simulation timestep")
      ->check(CLI::PositiveNumber);
  CLI11_PARSE(app, argc, argv);

  simulation::WorldPtr world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(simulationDt);

  std::vector<FreeJointCase> cases;

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector6d vel = Eigen::Vector6d::Zero();
    vel.tail<3>() = Eigen::Vector3d(0.7, 0.0, 0.0);
    cases.push_back(makeCase("Linear only", pose, vel, dart::Color::Blue(0.3)));
  }

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
    Eigen::Vector6d vel = Eigen::Vector6d::Zero();
    vel.head<3>() = Eigen::Vector3d(0.0, 0.9, 0.0);
    cases.push_back(makeCase("Angular only", pose, vel, dart::Color::Red(0.3)));
  }

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(4.0, 0.0, 0.0);
    pose.linear() = math::expMapRot(Eigen::Vector3d(0.6, -0.3, 0.2));
    Eigen::Vector6d vel = Eigen::Vector6d::Zero();
    vel.head<3>() = Eigen::Vector3d(0.6, -0.3, 0.4);
    vel.tail<3>() = Eigen::Vector3d(0.5, 0.2, -0.1);
    cases.push_back(
        makeCase("Linear + angular", pose, vel, dart::Color::Green(0.3)));
  }

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(6.0, 0.0, 0.0);
    pose.linear() = math::expMapRot(Eigen::Vector3d(math::pi - 1e-6, 0.0, 0.0));
    Eigen::Vector6d vel = Eigen::Vector6d::Zero();
    vel.head<3>() = Eigen::Vector3d(0.2, 0.8, -0.4);
    vel.tail<3>() = Eigen::Vector3d(0.3, -0.1, 0.2);
    cases.push_back(
        makeCase("Near π rotation", pose, vel, dart::Color::Orange(0.3)));
  }

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(8.0, 0.0, 0.0);
    pose.linear() = math::expMapRot(Eigen::Vector3d(0.3, 1.2, -0.4));
    Eigen::Vector6d vel = Eigen::Vector6d::Zero();
    vel.head<3>() = Eigen::Vector3d(1.2, 0.9, -1.0);
    vel.tail<3>() = Eigen::Vector3d(0.2, 0.3, 0.0);
    cases.push_back(
        makeCase("High ω multi-axis", pose, vel, dart::Color::Fuchsia(0.3)));
  }

  for (const auto& c : cases) {
    world->addSkeleton(c.skeleton);
    world->addSkeleton(c.referenceSkeleton);
  }

  osg::ref_ptr<gui::WorldNode> node = new gui::WorldNode(world);

  osg::ref_ptr<gui::ImGuiViewer> viewer = new gui::ImGuiViewer();
  viewer->setImGuiScale(static_cast<float>(guiScale));
  viewer->addWorldNode(node);

  auto grid = ::osg::ref_ptr<gui::GridVisual>(new gui::GridVisual());
  grid->setPlaneType(gui::GridVisual::PlaneType::XY);
  grid->setNumCells(20);
  viewer->addAttachment(grid);

  viewer->getImGuiHandler()->addWidget(std::make_shared<FreeJointCasesWidget>(
      viewer, grid, world, cases, numericDt, guiScale));

  viewer->setUpViewInWindow(0, 0, 1280, 720);
  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(4.5f, -10.0f, 4.0f),
      ::osg::Vec3(4.5f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  viewer->simulate(true);
  viewer->run();
}
