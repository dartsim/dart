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

#include <dart/gui/grid_visual.hpp>
#include <dart/gui/im_gui_handler.hpp>
#include <dart/gui/im_gui_viewer.hpp>
#include <dart/gui/im_gui_widget.hpp>
#include <dart/gui/include_im_gui.hpp>
#include <dart/gui/world_node.hpp>

#include <dart/all.hpp>

#include <CLI/CLI.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>

namespace {

struct CaseMetrics
{
  double translationalBlockError{0.0};
  double worldJacobianMaxError{0.0};
  double worldJacobianClassicDerivMaxError{0.0};
  double relativeJacobianDotMaxError{0.0};
};

enum class GroundTruthModel
{
  ConstantWorldTwist,
  TorqueFreeRigidBody,
};

enum class IntegrationMode
{
  DefaultStep,
  Substeps,
  UnconstrainedRungeKutta4,
};

struct TorqueFreeState
{
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d omegaBody{Eigen::Vector3d::Zero()};
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

  Eigen::Matrix3d anisotropicInertia{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d sphericalInertia{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d inertia{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d inertiaInv{Eigen::Matrix3d::Identity()};
  TorqueFreeState torqueFreeState;

  Eigen::Vector6d initialPositions{Eigen::Vector6d::Zero()};
  Eigen::Vector6d initialVelocities{Eigen::Vector6d::Zero()};
  double initialEnergy{0.0};

  Eigen::Vector3d jacobianOffset{Eigen::Vector3d(0.1, 0.0, 0.0)};
  CaseMetrics metrics;
};

struct LiveCaseMetrics
{
  double energy{0.0};
  double relativeEnergyDrift{0.0};
  double referenceTranslationError{0.0};
  double referenceRotationError{0.0};
};

std::string formatDouble(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string formatScientific(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::scientific << std::setprecision(precision) << value;
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
  data.anisotropicInertia = Eigen::Vector3d(0.02, 0.04, 0.06).asDiagonal();
  data.sphericalInertia
      = data.anisotropicInertia.diagonal().mean() * Eigen::Matrix3d::Identity();
  data.inertia = data.anisotropicInertia;
  data.inertiaInv = data.inertia.inverse();

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

Eigen::Isometry3d computeConstantTwistTransform(
    const FreeJointCase& data, double t)
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

struct TorqueFreeDeriv
{
  Eigen::Vector4d orientationDot{Eigen::Vector4d::Zero()};
  Eigen::Vector3d omegaBodyDot{Eigen::Vector3d::Zero()};
};

Eigen::Vector3d computeTorqueFreeOmegaBodyDot(
    const Eigen::Matrix3d& inertia,
    const Eigen::Matrix3d& inertiaInv,
    const Eigen::Vector3d& omegaBody)
{
  return -inertiaInv * (omegaBody.cross(inertia * omegaBody));
}

Eigen::Vector4d computeQuaternionCoeffDot(
    const Eigen::Quaterniond& orientation, const Eigen::Vector3d& omegaBody)
{
  const Eigen::Quaterniond omegaQuat(
      0.0, omegaBody.x(), omegaBody.y(), omegaBody.z());
  Eigen::Quaterniond qdot = orientation * omegaQuat;
  qdot.coeffs() *= 0.5;
  return qdot.coeffs();
}

TorqueFreeDeriv evaluateTorqueFreeDeriv(
    const TorqueFreeState& state,
    const Eigen::Matrix3d& inertia,
    const Eigen::Matrix3d& inertiaInv)
{
  TorqueFreeDeriv deriv;
  deriv.orientationDot
      = computeQuaternionCoeffDot(state.orientation, state.omegaBody);
  deriv.omegaBodyDot
      = computeTorqueFreeOmegaBodyDot(inertia, inertiaInv, state.omegaBody);
  return deriv;
}

void integrateTorqueFreeStep(
    TorqueFreeState& state,
    const Eigen::Matrix3d& inertia,
    const Eigen::Matrix3d& inertiaInv,
    double dt)
{
  if (!(dt > 0.0)) {
    return;
  }

  const TorqueFreeDeriv k1
      = evaluateTorqueFreeDeriv(state, inertia, inertiaInv);

  TorqueFreeState s2 = state;
  s2.orientation.coeffs() += 0.5 * dt * k1.orientationDot;
  s2.omegaBody += 0.5 * dt * k1.omegaBodyDot;
  s2.orientation.normalize();
  const TorqueFreeDeriv k2 = evaluateTorqueFreeDeriv(s2, inertia, inertiaInv);

  TorqueFreeState s3 = state;
  s3.orientation.coeffs() += 0.5 * dt * k2.orientationDot;
  s3.omegaBody += 0.5 * dt * k2.omegaBodyDot;
  s3.orientation.normalize();
  const TorqueFreeDeriv k3 = evaluateTorqueFreeDeriv(s3, inertia, inertiaInv);

  TorqueFreeState s4 = state;
  s4.orientation.coeffs() += dt * k3.orientationDot;
  s4.omegaBody += dt * k3.omegaBodyDot;
  s4.orientation.normalize();
  const TorqueFreeDeriv k4 = evaluateTorqueFreeDeriv(s4, inertia, inertiaInv);

  state.orientation.coeffs()
      += (dt / 6.0)
         * (k1.orientationDot + 2.0 * k2.orientationDot
            + 2.0 * k3.orientationDot + k4.orientationDot);
  state.omegaBody += (dt / 6.0)
                     * (k1.omegaBodyDot + 2.0 * k2.omegaBodyDot
                        + 2.0 * k3.omegaBodyDot + k4.omegaBodyDot);
  state.orientation.normalize();
}

void resetTorqueFreeState(FreeJointCase& data)
{
  using dart::dynamics::FreeJoint;

  const Eigen::Isometry3d initial
      = FreeJoint::convertToTransform(data.initialPositions);

  data.torqueFreeState.orientation = Eigen::Quaterniond(initial.linear());
  data.torqueFreeState.orientation.normalize();

  const Eigen::Vector3d omegaWorld = data.initialVelocities.head<3>();
  data.torqueFreeState.omegaBody = initial.linear().transpose() * omegaWorld;
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

class SubsteppingWorldNode : public dart::gui::WorldNode
{
public:
  explicit SubsteppingWorldNode(dart::simulation::WorldPtr world)
    : dart::gui::WorldNode(std::move(world))
  {
  }

  void setSubsteps(std::size_t substeps)
  {
    mSubsteps = std::max<std::size_t>(1u, substeps);
  }

  void setIntegrationMode(IntegrationMode mode)
  {
    mIntegrationMode = mode;
  }

  IntegrationMode getIntegrationMode() const
  {
    return mIntegrationMode;
  }

  std::size_t getSubsteps() const
  {
    return mSubsteps;
  }

  void refresh() override
  {
    customPreRefresh();

    clearChildUtilizationFlags();

    if (mSimulating) {
      for (std::size_t i = 0; i < mNumStepsPerCycle; ++i) {
        customPreStep();
        stepWorld();
        customPostStep();
      }
    }

    refreshSkeletons();
    refreshSimpleFrames();

    clearUnusedNodes();

    customPostRefresh();
  }

private:
  void stepWorld()
  {
    switch (mIntegrationMode) {
      case IntegrationMode::DefaultStep:
        mWorld->step();
        break;
      case IntegrationMode::Substeps:
        mWorld->stepSubsteps(mSubsteps);
        break;
      case IntegrationMode::UnconstrainedRungeKutta4:
        mWorld->stepUnconstrainedRungeKutta4();
        break;
    }
  }

  IntegrationMode mIntegrationMode{IntegrationMode::Substeps};
  std::size_t mSubsteps{1u};
};

class FreeJointCasesWidget : public dart::gui::ImGuiWidget
{
public:
  FreeJointCasesWidget(
      osg::ref_ptr<dart::gui::ImGuiViewer> viewer,
      osg::ref_ptr<dart::gui::GridVisual> grid,
      osg::ref_ptr<SubsteppingWorldNode> node,
      dart::simulation::WorldPtr world,
      std::vector<FreeJointCase> cases,
      double numericDt,
      double simulationDt,
      GroundTruthModel groundTruthModel,
      bool useSphericalInertia,
      int torqueFreeSubsteps,
      IntegrationMode integrationMode,
      int simulationSubsteps,
      double guiScale)
    : mViewer(std::move(viewer)),
      mGrid(std::move(grid)),
      mNode(std::move(node)),
      mWorld(std::move(world)),
      mCases(std::move(cases)),
      mNumericDt(numericDt),
      mSimulationDt(simulationDt),
      mGroundTruthModel(groundTruthModel),
      mUseSphericalInertia(useSphericalInertia),
      mTorqueFreeSubsteps(std::max(1, torqueFreeSubsteps)),
      mIntegrationMode(integrationMode),
      mSimulationSubsteps(std::max(1, simulationSubsteps)),
      mGuiScale(guiScale)
  {
    const int scaleKey = static_cast<int>(std::lround(guiScale * 100.0));
    mWindowLabel
        = "FreeJoint cases##free_joint_cases_" + std::to_string(scaleKey);

    applyInertiaMode();
    applyGroundTruthVisibility();
    mNode->setIntegrationMode(mIntegrationMode);
    mNode->setSubsteps(static_cast<std::size_t>(mSimulationSubsteps));
    resetCases();
    recomputeMetrics();
  }

  void render() override
  {
    advanceTorqueFreeGroundTruth();
    updateGroundTruth();
    updatePerformanceAndHistory();

    const float windowScale = static_cast<float>(mGuiScale);
    const ImGuiIO& io = ImGui::GetIO();
    const ImVec2 displaySize = io.DisplaySize;

    const ImVec2 desiredPos(10.0f * windowScale, 20.0f * windowScale);
    const ImVec2 desiredSize(620.0f * windowScale, 680.0f * windowScale);
    constexpr float kMaxDisplayWidthFraction = 0.50f;
    const ImVec2 maxSize(
        std::max(
            1.0f, (displaySize.x - desiredPos.x) * kMaxDisplayWidthFraction),
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
        if (ImGui::MenuItem("Exit")) {
          mViewer->setDone(true);
        }
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("About DART")) {
          mViewer->showAbout();
        }
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }

    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
      int state = mViewer->isSimulating() ? 0 : 1;
      if (mViewer->isAllowingSimulation()) {
        if (ImGui::RadioButton("Play", &state, 0) && !mViewer->isSimulating()) {
          mViewer->simulate(true);
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Pause", &state, 1) && mViewer->isSimulating()) {
          mViewer->simulate(false);
        }
      }

      ImGui::Text("Time: %s", formatDouble(mWorld->getTime(), 3).c_str());

      int integrationModeIndex = 0;
      switch (mIntegrationMode) {
        case IntegrationMode::DefaultStep:
          integrationModeIndex = 0;
          break;
        case IntegrationMode::Substeps:
          integrationModeIndex = 1;
          break;
        case IntegrationMode::UnconstrainedRungeKutta4:
          integrationModeIndex = 2;
          break;
      }

      constexpr const char* integrationModeItems
          = "Default step\0Substeps\0Unconstrained RK4\0";
      if (ImGui::Combo(
              "Integration mode",
              &integrationModeIndex,
              integrationModeItems)) {
        if (integrationModeIndex == 0) {
          mIntegrationMode = IntegrationMode::DefaultStep;
        } else if (integrationModeIndex == 1) {
          mIntegrationMode = IntegrationMode::Substeps;
        } else {
          mIntegrationMode = IntegrationMode::UnconstrainedRungeKutta4;
        }
        mNode->setIntegrationMode(mIntegrationMode);
      }

      if (ImGui::SliderInt(
              "Simulation substeps", &mSimulationSubsteps, 1, 50)) {
        mSimulationSubsteps = std::max(1, mSimulationSubsteps);
        mNode->setSubsteps(static_cast<std::size_t>(mSimulationSubsteps));
      }

      if (ImGui::Button("Reset cases")) {
        resetCases();
      }
      ImGui::SameLine();
      if (ImGui::Button("Numeric checks")) {
        recomputeMetrics();
      }

      if (ImGui::Checkbox("Use spherical inertia", &mUseSphericalInertia)) {
        applyInertiaMode();
        resetCases();
      }
    }

    if (ImGui::CollapsingHeader(
            "Energy and reference", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Text(
          "Max |dKE/KE0|: %s",
          formatScientific(mMaxAbsRelativeEnergyDrift).c_str());
      ImGui::Text(
          "Max reference translation: %s m",
          formatScientific(mMaxReferenceTranslationError).c_str());
      ImGui::Text(
          "Max reference rotation: %s rad",
          formatScientific(mMaxReferenceRotationError).c_str());

      const float energyScale
          = static_cast<float>(std::max(1e-14, 1.2 * mEnergyPlotMax));
      ImGui::PlotLines(
          "Energy drift",
          mEnergyDriftHistory.data(),
          static_cast<int>(mHistoryCount),
          0,
          nullptr,
          0.0f,
          energyScale,
          ImVec2(-1.0f, 58.0f * windowScale));

      const float referenceScale
          = static_cast<float>(std::max(1e-14, 1.2 * mReferencePlotMax));
      ImGui::PlotLines(
          "Reference error",
          mReferenceErrorHistory.data(),
          static_cast<int>(mHistoryCount),
          0,
          nullptr,
          0.0f,
          referenceScale,
          ImVec2(-1.0f, 58.0f * windowScale));
    }

    if (ImGui::CollapsingHeader(
            "Performance", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Text("Render FPS: %s", formatDouble(mRenderFps, 1).c_str());
      ImGui::Text(
          "Simulation steps/s: %s",
          formatDouble(mSimulationStepsPerSecond, 1).c_str());
      ImGui::Text(
          "Real-time factor: %s", formatDouble(mRealtimeFactor, 2).c_str());

      const float stepRateScale
          = static_cast<float>(std::max(1.0, 1.2 * mStepRatePlotMax));
      ImGui::PlotLines(
          "Simulation steps/s",
          mStepRateHistory.data(),
          static_cast<int>(mHistoryCount),
          0,
          nullptr,
          0.0f,
          stepRateScale,
          ImVec2(-1.0f, 58.0f * windowScale));
    }

    if (ImGui::CollapsingHeader(
            "Ground truth", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Checkbox("Show reference bodies", &mShowGroundTruth)) {
        applyGroundTruthVisibility();
        updateGroundTruth();
      }

      int modelIndex
          = (mGroundTruthModel == GroundTruthModel::TorqueFreeRigidBody) ? 0
                                                                         : 1;
      constexpr const char* modelItems
          = "Torque-free rigid body (RK4)\0Constant world twist\0";
      if (ImGui::Combo("Model", &modelIndex, modelItems)) {
        mGroundTruthModel = (modelIndex == 0)
                                ? GroundTruthModel::TorqueFreeRigidBody
                                : GroundTruthModel::ConstantWorldTwist;
        updateGroundTruth();
      }

      if (mGroundTruthModel == GroundTruthModel::TorqueFreeRigidBody) {
        if (ImGui::SliderInt("Substeps", &mTorqueFreeSubsteps, 1, 200)) {
          mTorqueFreeSubsteps = std::max(1, mTorqueFreeSubsteps);
        }

        ImGui::TextWrapped(
            "Integrates Euler's torque-free rigid-body equations (RK4) with "
            "the current inertia. Translation uses p(t)=p0+v0 t.");
      } else {
        ImGui::TextWrapped(
            "Uses R(t)=Exp(omega t) R0 and p(t)=p0+v t, assuming the "
            "world-frame twist (omega, v) stays constant. This holds for "
            "spherical inertia "
            "(Ixx=Iyy=Izz) or special initial conditions.");
      }
    }

    if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
      bool showGrid = mGrid && mGrid->isDisplayed();
      if (ImGui::Checkbox("Show grid", &showGrid) && mGrid) {
        mGrid->display(showGrid);
      }
    }

    if (ImGui::CollapsingHeader("Cases", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::BeginTable(
              "freejoint_live_cases_table",
              4,
              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                  | ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("Case");
        ImGui::TableSetupColumn("dKE/KE0");
        ImGui::TableSetupColumn("|dx-ref|");
        ImGui::TableSetupColumn("|dR-ref|");
        ImGui::TableHeadersRow();

        for (const auto& c : mCases) {
          const LiveCaseMetrics live = computeLiveMetrics(c);

          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(c.name.c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(
              formatScientific(live.relativeEnergyDrift).c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(
              formatScientific(live.referenceTranslationError).c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(
              formatScientific(live.referenceRotationError).c_str());
        }

        ImGui::EndTable();
      }
    }

    if (ImGui::CollapsingHeader("Numeric checks")) {
      if (ImGui::BeginTable(
              "freejoint_numeric_checks_table",
              6,
              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                  | ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("Case");
        ImGui::TableSetupColumn("|Jt-I|inf");
        ImGui::TableSetupColumn("|Jnum-J|inf");
        ImGui::TableSetupColumn("|dVdot-FD|inf");
        ImGui::TableSetupColumn("|relJdot-FD|inf");
        ImGui::TableHeadersRow();

        for (const auto& c : mCases) {
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(c.name.c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(
              formatScientific(c.metrics.translationalBlockError).c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(
              formatScientific(c.metrics.worldJacobianMaxError).c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(
              formatScientific(c.metrics.worldJacobianClassicDerivMaxError)
                  .c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(
              formatScientific(c.metrics.relativeJacobianDotMaxError).c_str());
        }

        ImGui::EndTable();
      }
    }

    ImGui::End();
  }

private:
  void applyInertiaMode()
  {
    for (auto& c : mCases) {
      const Eigen::Matrix3d& inertia
          = mUseSphericalInertia ? c.sphericalInertia : c.anisotropicInertia;

      c.body->setMomentOfInertia(
          inertia(0, 0),
          inertia(1, 1),
          inertia(2, 2),
          inertia(0, 1),
          inertia(0, 2),
          inertia(1, 2));
      c.inertia = inertia;
      c.inertiaInv = inertia.inverse();
    }
  }

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

    resetTorqueFreeGroundTruth();
    updateGroundTruth();
    resetHistory();
  }

  void applyGroundTruthVisibility()
  {
    for (auto& c : mCases) {
      if (!c.referenceShapeNode) {
        continue;
      }
      c.referenceShapeNode->getVisualAspect()->setHidden(!mShowGroundTruth);
    }
  }

  void resetTorqueFreeGroundTruth()
  {
    mTorqueFreeTime = 0.0;
    for (auto& c : mCases) {
      resetTorqueFreeState(c);
    }
  }

  void advanceTorqueFreeGroundTruth()
  {
    const double targetTime = mWorld->getTime();
    if (targetTime < mTorqueFreeTime) {
      resetTorqueFreeGroundTruth();
      return;
    }

    if (!(targetTime > mTorqueFreeTime)) {
      return;
    }

    const double baseStep = (mSimulationDt > 0.0)
                                ? (mSimulationDt / mTorqueFreeSubsteps)
                                : (targetTime - mTorqueFreeTime);
    if (!(baseStep > 0.0)) {
      return;
    }

    double time = mTorqueFreeTime;
    while (time + 1e-12 < targetTime) {
      const double dt = std::min(baseStep, targetTime - time);
      for (auto& c : mCases) {
        integrateTorqueFreeStep(c.torqueFreeState, c.inertia, c.inertiaInv, dt);
      }
      time += dt;
    }

    mTorqueFreeTime = targetTime;
  }

  void updateGroundTruth()
  {
    const double t = mWorld->getTime();

    if (!mShowGroundTruth) {
      return;
    }

    for (auto& c : mCases) {
      if (!c.referenceJoint) {
        continue;
      }

      Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
      if (mGroundTruthModel == GroundTruthModel::TorqueFreeRigidBody) {
        expected.linear() = c.torqueFreeState.orientation.toRotationMatrix();
        expected.translation()
            = c.initialPositions.tail<3>() + c.initialVelocities.tail<3>() * t;
      } else {
        expected = computeConstantTwistTransform(c, t);
      }
      c.referenceJoint->setPositions(
          dart::dynamics::FreeJoint::convertToPositions(expected));
    }
  }

  void recomputeMetrics()
  {
    for (auto& c : mCases) {
      c.metrics = computeMetrics(c, mNumericDt);
    }
  }

  Eigen::Isometry3d computeGroundTruthTransform(const FreeJointCase& c) const
  {
    const double t = mWorld->getTime();

    if (mGroundTruthModel == GroundTruthModel::TorqueFreeRigidBody) {
      Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
      expected.linear() = c.torqueFreeState.orientation.toRotationMatrix();
      expected.translation()
          = c.initialPositions.tail<3>() + c.initialVelocities.tail<3>() * t;
      return expected;
    }

    return computeConstantTwistTransform(c, t);
  }

  LiveCaseMetrics computeLiveMetrics(const FreeJointCase& c) const
  {
    LiveCaseMetrics live;

    live.energy = c.skeleton->computeKineticEnergy();
    const double energyScale = std::max(1.0, c.initialEnergy);
    live.relativeEnergyDrift = (live.energy - c.initialEnergy) / energyScale;

    const Eigen::Isometry3d actual = c.body->getWorldTransform();
    const Eigen::Isometry3d expected = computeGroundTruthTransform(c);
    live.referenceTranslationError
        = (actual.translation() - expected.translation()).norm();
    live.referenceRotationError
        = dart::math::logMap(actual.linear() * expected.linear().transpose())
              .norm();

    return live;
  }

  void updatePerformanceAndHistory()
  {
    const auto now = std::chrono::steady_clock::now();
    const double worldTime = mWorld->getTime();

    if (mHavePerformanceSample) {
      const double wallDt
          = std::chrono::duration<double>(now - mLastPerformanceSampleTime)
                .count();
      if (wallDt > 0.0) {
        const double simDt
            = std::max(0.0, worldTime - mLastPerformanceWorldTime);
        const double stepRate
            = (mSimulationDt > 0.0) ? (simDt / mSimulationDt) / wallDt : 0.0;

        updateSmoothedValue(mRenderFps, 1.0 / wallDt);
        updateSmoothedValue(mSimulationStepsPerSecond, stepRate);
        updateSmoothedValue(mRealtimeFactor, simDt / wallDt);
      }
    } else {
      mHavePerformanceSample = true;
    }

    mLastPerformanceSampleTime = now;
    mLastPerformanceWorldTime = worldTime;

    double maxAbsRelativeEnergyDrift = 0.0;
    double maxReferenceTranslationError = 0.0;
    double maxReferenceRotationError = 0.0;
    for (const auto& c : mCases) {
      const LiveCaseMetrics live = computeLiveMetrics(c);
      maxAbsRelativeEnergyDrift = std::max(
          maxAbsRelativeEnergyDrift, std::abs(live.relativeEnergyDrift));
      maxReferenceTranslationError = std::max(
          maxReferenceTranslationError, live.referenceTranslationError);
      maxReferenceRotationError
          = std::max(maxReferenceRotationError, live.referenceRotationError);
    }

    mMaxAbsRelativeEnergyDrift
        = std::max(mMaxAbsRelativeEnergyDrift, maxAbsRelativeEnergyDrift);
    mMaxReferenceTranslationError
        = std::max(mMaxReferenceTranslationError, maxReferenceTranslationError);
    mMaxReferenceRotationError
        = std::max(mMaxReferenceRotationError, maxReferenceRotationError);

    const double referenceError
        = std::max(maxReferenceTranslationError, maxReferenceRotationError);
    mEnergyPlotMax = std::max(mEnergyPlotMax, maxAbsRelativeEnergyDrift);
    mReferencePlotMax = std::max(mReferencePlotMax, referenceError);
    mStepRatePlotMax = std::max(mStepRatePlotMax, mSimulationStepsPerSecond);

    appendHistorySamples(
        maxAbsRelativeEnergyDrift, referenceError, mSimulationStepsPerSecond);
  }

  void updateSmoothedValue(double& value, double sample)
  {
    constexpr double alpha = 0.12;
    if (!(value > 0.0)) {
      value = sample;
      return;
    }
    value += alpha * (sample - value);
  }

  void appendHistorySamples(
      double energyDrift, double referenceError, double stepRate)
  {
    const float energySample = static_cast<float>(std::min(energyDrift, 1e12));
    const float referenceSample
        = static_cast<float>(std::min(referenceError, 1e12));
    const float stepRateSample = static_cast<float>(std::min(stepRate, 1e12));

    if (mHistoryCount < kHistoryLength) {
      mEnergyDriftHistory[mHistoryCount] = energySample;
      mReferenceErrorHistory[mHistoryCount] = referenceSample;
      mStepRateHistory[mHistoryCount] = stepRateSample;
      ++mHistoryCount;
      return;
    }

    std::move(
        mEnergyDriftHistory.begin() + 1,
        mEnergyDriftHistory.end(),
        mEnergyDriftHistory.begin());
    std::move(
        mReferenceErrorHistory.begin() + 1,
        mReferenceErrorHistory.end(),
        mReferenceErrorHistory.begin());
    std::move(
        mStepRateHistory.begin() + 1,
        mStepRateHistory.end(),
        mStepRateHistory.begin());
    mEnergyDriftHistory.back() = energySample;
    mReferenceErrorHistory.back() = referenceSample;
    mStepRateHistory.back() = stepRateSample;
  }

  void resetHistory()
  {
    mHistoryCount = 0;
    mEnergyDriftHistory.fill(0.0f);
    mReferenceErrorHistory.fill(0.0f);
    mStepRateHistory.fill(0.0f);
    mMaxAbsRelativeEnergyDrift = 0.0;
    mMaxReferenceTranslationError = 0.0;
    mMaxReferenceRotationError = 0.0;
    mEnergyPlotMax = 0.0;
    mReferencePlotMax = 0.0;
    mStepRatePlotMax = 0.0;
    mHavePerformanceSample = false;
    mLastPerformanceWorldTime = mWorld ? mWorld->getTime() : 0.0;
    mRenderFps = 0.0;
    mSimulationStepsPerSecond = 0.0;
    mRealtimeFactor = 0.0;
  }

  osg::ref_ptr<dart::gui::ImGuiViewer> mViewer;
  osg::ref_ptr<dart::gui::GridVisual> mGrid;
  osg::ref_ptr<SubsteppingWorldNode> mNode;
  dart::simulation::WorldPtr mWorld;
  std::vector<FreeJointCase> mCases;
  static constexpr std::size_t kHistoryLength = 240;
  double mNumericDt;
  double mSimulationDt;
  GroundTruthModel mGroundTruthModel;
  bool mUseSphericalInertia;
  int mTorqueFreeSubsteps;
  IntegrationMode mIntegrationMode;
  int mSimulationSubsteps;
  double mGuiScale;
  std::string mWindowLabel;
  bool mShowGroundTruth{true};
  double mTorqueFreeTime{0.0};
  bool mHavePerformanceSample{false};
  std::chrono::steady_clock::time_point mLastPerformanceSampleTime{
      std::chrono::steady_clock::now()};
  double mLastPerformanceWorldTime{0.0};
  double mRenderFps{0.0};
  double mSimulationStepsPerSecond{0.0};
  double mRealtimeFactor{0.0};
  double mMaxAbsRelativeEnergyDrift{0.0};
  double mMaxReferenceTranslationError{0.0};
  double mMaxReferenceRotationError{0.0};
  double mEnergyPlotMax{0.0};
  double mReferencePlotMax{0.0};
  double mStepRatePlotMax{0.0};
  std::size_t mHistoryCount{0};
  std::array<float, kHistoryLength> mEnergyDriftHistory{};
  std::array<float, kHistoryLength> mReferenceErrorHistory{};
  std::array<float, kHistoryLength> mStepRateHistory{};
};

int main(int argc, char* argv[])
{
  using namespace dart;

  CLI::App app("FreeJoint cases viewer");
  double guiScale = 1.0;
  double numericDt = 1e-6;
  double simulationDt = 1e-3;
  bool sphericalInertia = false;
  std::string groundTruthMode = "torque-free";
  std::string integrationModeName = "substeps";
  int torqueFreeSubsteps = 10;
  int simulationSubsteps = 1;
  app.add_option("--gui-scale", guiScale, "Scale factor for ImGui widgets")
      ->check(CLI::PositiveNumber);
  app.add_option("--numeric-dt", numericDt, "Finite difference dt for checks")
      ->check(CLI::PositiveNumber);
  app.add_option("--dt", simulationDt, "Simulation timestep")
      ->check(CLI::PositiveNumber);
  app.add_flag(
      "--spherical-inertia",
      sphericalInertia,
      "Use spherical inertia (Ixx=Iyy=Izz) so the world-frame twist stays "
      "constant");
  app.add_option(
         "--ground-truth",
         groundTruthMode,
         "Ground truth model: torque-free|constant")
      ->check(CLI::IsMember({"torque-free", "constant"}));
  app.add_option(
         "--ground-truth-substeps",
         torqueFreeSubsteps,
         "Substeps per simulation step for torque-free ground truth")
      ->check(CLI::Range(1, 1000));
  app.add_option(
         "--integration-mode",
         integrationModeName,
         "Simulation integration mode: step|substeps|rk4")
      ->check(CLI::IsMember({"step", "substeps", "rk4"}));
  app.add_option(
         "--simulation-substeps",
         simulationSubsteps,
         "Internal simulation substeps per displayed simulation step")
      ->check(CLI::Range(1, 1000));
  CLI11_PARSE(app, argc, argv);
  const GroundTruthModel groundTruthModel
      = (groundTruthMode == "constant") ? GroundTruthModel::ConstantWorldTwist
                                        : GroundTruthModel::TorqueFreeRigidBody;
  IntegrationMode integrationMode = IntegrationMode::Substeps;
  if (integrationModeName == "step") {
    integrationMode = IntegrationMode::DefaultStep;
  } else if (integrationModeName == "rk4") {
    integrationMode = IntegrationMode::UnconstrainedRungeKutta4;
  }

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
        makeCase("Near pi rotation", pose, vel, dart::Color::Orange(0.3)));
  }

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(8.0, 0.0, 0.0);
    pose.linear() = math::expMapRot(Eigen::Vector3d(0.3, 1.2, -0.4));
    Eigen::Vector6d vel = Eigen::Vector6d::Zero();
    vel.head<3>() = Eigen::Vector3d(1.2, 0.9, -1.0);
    vel.tail<3>() = Eigen::Vector3d(0.2, 0.3, 0.0);
    cases.push_back(makeCase(
        "High omega multi-axis", pose, vel, dart::Color::Fuchsia(0.3)));
  }

  for (const auto& c : cases) {
    world->addSkeleton(c.skeleton);
    world->addSkeleton(c.referenceSkeleton);
  }

  osg::ref_ptr<SubsteppingWorldNode> node = new SubsteppingWorldNode(world);
  node->setIntegrationMode(integrationMode);
  node->setSubsteps(static_cast<std::size_t>(simulationSubsteps));

  osg::ref_ptr<gui::ImGuiViewer> viewer = new gui::ImGuiViewer();
  viewer->setImGuiScale(static_cast<float>(guiScale));
  viewer->getImGuiHandler()->setFontScale(static_cast<float>(guiScale));
  viewer->addWorldNode(node);

  auto grid = ::osg::ref_ptr<gui::GridVisual>(new gui::GridVisual());
  grid->setPlaneType(gui::GridVisual::PlaneType::XY);
  grid->setNumCells(20);
  viewer->addAttachment(grid);

  viewer->getImGuiHandler()->addWidget(
      std::make_shared<FreeJointCasesWidget>(
          viewer,
          grid,
          node,
          world,
          cases,
          numericDt,
          simulationDt,
          groundTruthModel,
          sphericalInertia,
          torqueFreeSubsteps,
          integrationMode,
          simulationSubsteps,
          guiScale));

  viewer->setUpViewInWindow(0, 0, 1280, 720);
  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.5f, -10.0f, 4.0f),
      ::osg::Vec3(2.5f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  viewer->simulate(true);
  viewer->run();
}
