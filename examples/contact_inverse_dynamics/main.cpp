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

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <osgViewer/ViewerEventHandlers>

#include <array>
#include <atomic>
#include <iostream>
#include <string>
#include <vector>

#include <cmath>
#include <cstdlib>

using namespace dart;

namespace {

constexpr double kDefaultGuiScale = 1.0;
constexpr double kMinGuiScale = 0.75;
constexpr double kMaxGuiScale = 4.0;

//==============================================================================
double parseGuiScale(const std::string& value)
{
  try {
    const double scale = std::stod(value);
    if (scale >= kMinGuiScale && scale <= kMaxGuiScale) {
      return scale;
    }
    std::cerr << "--gui-scale must be in [" << kMinGuiScale << ", "
              << kMaxGuiScale << "]; got '" << value << "'. Falling back to "
              << kDefaultGuiScale << ".\n";
  } catch (const std::exception&) {
    std::cerr << "Invalid --gui-scale value '" << value << "'. Falling back "
              << "to " << kDefaultGuiScale << ".\n";
  }
  return kDefaultGuiScale;
}

//==============================================================================
int scaleWindowExtent(int extent, double scale)
{
  return std::max(1, static_cast<int>(extent * scale + 0.5));
}

} // namespace

//==============================================================================
class ContactInverseDynamicsWidget : public dart::gui::osg::ImGuiWidget
{
public:
  ContactInverseDynamicsWidget(
      dart::gui::osg::ImGuiViewer* viewer,
      dart::simulation::WorldPtr world,
      dart::dynamics::SkeletonPtr biped,
      double guiScale,
      std::function<void()> regenerateCallback)
    : mViewer(viewer),
      mWorld(std::move(world)),
      mBiped(std::move(biped)),
      mGuiScale(static_cast<float>(guiScale)),
      mRegenerateCallback(std::move(regenerateCallback)),
      mPlaying(true),
      mPlaybackSpeed(1.0f),
      mSquatDepth(0.15f),
      mFrictionCoeff(0.8f),
      mForceScale(0.002f),
      mSampleIndex(0)
  {
    mPlotSamples.resize(240, 0.0f);
    mTorquePlots.resize(3);
    for (auto& plot : mTorquePlots) {
      plot.resize(240, 0.0f);
    }
  }

  void render() override
  {
    // ImGuiHandler works in framebuffer pixels, so the default style and
    // font are tiny on HiDPI/scaled displays; scale the style once and the
    // font every frame (matching the dynamic_joint_constraints example).
    if (!mStyleScaled) {
      ImGui::GetStyle().ScaleAllSizes(mGuiScale);
      mStyleScaled = true;
    }
#if IMGUI_VERSION_NUM >= 19200
    ImGui::GetStyle().FontScaleMain = mGuiScale;
#else
    ImGui::GetIO().FontGlobalScale = mGuiScale;
#endif

    ImGui::SetNextWindowPos(ImVec2(10 * mGuiScale, 20 * mGuiScale));
    ImGui::SetNextWindowSize(ImVec2(420 * mGuiScale, 680 * mGuiScale));
    ImGui::SetNextWindowBgAlpha(0.5f);
    if (!ImGui::Begin(
            "Contact-Aware Inverse Dynamics",
            nullptr,
            ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_HorizontalScrollbar)) {
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

    if (ImGui::CollapsingHeader("Controls", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Checkbox("Play", &mPlaying);
      ImGui::SliderFloat("Playback Speed", &mPlaybackSpeed, 0.1f, 2.0f);

      ImGui::SliderFloat("Squat Depth (m)", &mSquatDepth, 0.0f, 0.25f);
      if (ImGui::IsItemDeactivatedAfterEdit()) {
        if (mRegenerateCallback) {
          mRegenerateCallback();
        }
      }
      if (ImGui::Button("Regenerate Keyframes")) {
        if (mRegenerateCallback) {
          mRegenerateCallback();
        }
      }

      ImGui::SliderFloat("Friction Coefficient", &mFrictionCoeff, 0.0f, 1.5f);
      ImGui::SliderFloat("Force Arrow Scale", &mForceScale, 0.0001f, 0.01f);

      Eigen::Vector3d gravity = mWorld->getGravity();
      ImGui::Text(
          "Gravity: (%.2f, %.2f, %.2f)", gravity[0], gravity[1], gravity[2]);
    }

    if (ImGui::CollapsingHeader("Status", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (mFeasible) {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "FEASIBLE");
      } else {
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "INFEASIBLE");
      }

      ImGui::Text("Residual Norm: %.4f", mResidualNorm);

      double bodyMass = mBiped->getMass();
      double bodyWeight = bodyMass * mWorld->getGravity().norm();
      ImGui::Text("Total Vertical Force: %.2f N", mTotalVerticalForce);
      ImGui::Text("Body Weight: %.2f N", bodyWeight);
      ImGui::Text(
          "Ratio: %.2f",
          bodyWeight > 1e-6 ? mTotalVerticalForce / bodyWeight : 0.0);

      ImGui::Separator();
      ImGui::Text("Contact Forces:");
      for (std::size_t i = 0; i < mContactForces.size(); ++i) {
        double mag = mContactForces[i].norm();
        ImGui::Text("  Contact %zu: %.2f N", i, mag);
      }
    }

    if (ImGui::CollapsingHeader("Plots", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (!mTorqueNames.empty()) {
        for (std::size_t i = 0; i < mTorqueNames.size(); ++i) {
          ImGui::PlotLines(
              mTorqueNames[i].c_str(),
              mTorquePlots[i].data(),
              static_cast<int>(mTorquePlots[i].size()),
              0,
              nullptr,
              -200.0f,
              200.0f,
              ImVec2(0, 80 * mGuiScale));
        }
      }

      ImGui::PlotLines(
          "Total Vertical GRF (N)",
          mPlotSamples.data(),
          static_cast<int>(mPlotSamples.size()),
          0,
          nullptr,
          0.0f,
          1000.0f,
          ImVec2(0, 80 * mGuiScale));
    }

    ImGui::End();
  }

  void setStatus(
      bool feasible,
      double residualNorm,
      const std::vector<Eigen::Vector3d>& contactForces,
      const Eigen::VectorXd& jointForces,
      const std::vector<std::string>& torqueNames,
      const std::vector<std::size_t>& torqueDofs)
  {
    mFeasible = feasible;
    mResidualNorm = residualNorm;
    mContactForces = contactForces;

    Eigen::Vector3d upDir = -mWorld->getGravity().normalized();
    mTotalVerticalForce = 0.0;
    for (const auto& f : contactForces) {
      mTotalVerticalForce += f.dot(upDir);
    }

    mPlotSamples[mSampleIndex] = static_cast<float>(mTotalVerticalForce);

    mTorqueNames = torqueNames;
    for (std::size_t i = 0; i < torqueDofs.size() && i < mTorquePlots.size();
         ++i) {
      if (torqueDofs[i] < static_cast<std::size_t>(jointForces.size())) {
        mTorquePlots[i][mSampleIndex]
            = static_cast<float>(jointForces[torqueDofs[i]]);
      }
    }

    mSampleIndex = (mSampleIndex + 1) % mPlotSamples.size();
  }

  bool isPlaying() const
  {
    return mPlaying;
  }
  float getPlaybackSpeed() const
  {
    return mPlaybackSpeed;
  }
  float getSquatDepth() const
  {
    return mSquatDepth;
  }
  float getFrictionCoeff() const
  {
    return mFrictionCoeff;
  }
  float getForceScale() const
  {
    return mForceScale;
  }

protected:
  osg::ref_ptr<dart::gui::osg::ImGuiViewer> mViewer;
  dart::simulation::WorldPtr mWorld;
  dart::dynamics::SkeletonPtr mBiped;
  float mGuiScale;
  bool mStyleScaled = false;
  std::function<void()> mRegenerateCallback;

  bool mPlaying;
  float mPlaybackSpeed;
  float mSquatDepth;
  float mFrictionCoeff;
  float mForceScale;

  bool mFeasible = true;
  double mResidualNorm = 0.0;
  std::vector<Eigen::Vector3d> mContactForces;
  double mTotalVerticalForce = 0.0;

  std::vector<float> mPlotSamples;
  std::vector<std::vector<float>> mTorquePlots;
  std::vector<std::string> mTorqueNames;
  std::size_t mSampleIndex;
};

//==============================================================================
class ContactInverseDynamicsWorldNode : public dart::gui::osg::WorldNode
{
public:
  ContactInverseDynamicsWorldNode(
      dart::simulation::WorldPtr world, dart::dynamics::SkeletonPtr biped)
    : dart::gui::osg::WorldNode(std::move(world)),
      mBiped(std::move(biped)),
      mSolver(mBiped),
      mTime(0.0)
  {
    // A stronger-than-default regularization keeps the (statically
    // indeterminate) force distribution even and steady across frames; the
    // residual tolerance is widened to match the equality bias it introduces.
    mSolver.setRegularization(1e-4);
    mSolver.setResidualTolerance(1e-4);

    mLeftHeel = mBiped->getBodyNode("h_heel_left");
    mLeftToe = mBiped->getBodyNode("h_toe_left");
    mRightHeel = mBiped->getBodyNode("h_heel_right");
    mRightToe = mBiped->getBodyNode("h_toe_right");

    if (!mLeftHeel || !mLeftToe || !mRightHeel || !mRightToe) {
      std::cerr << "Error: Could not find foot body nodes" << std::endl;
      mValid = false;
      return;
    }

    for (std::size_t i = 0; i < 4; ++i) {
      auto frame
          = std::make_shared<dynamics::SimpleFrame>(dynamics::Frame::World());
      mContactFrames.push_back(frame);
      mWorld->addSimpleFrame(frame);

      auto arrow = std::make_shared<dynamics::ArrowShape>(
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::UnitZ() * 0.1,
          dynamics::ArrowShape::Properties(0.01, 2.0, 0.15),
          Eigen::Vector4d(0.0, 1.0, 0.0, 1.0));
      mArrows.push_back(arrow);
      frame->setShape(arrow);
      frame->getVisualAspect(true);
    }

    findTorqueDofs();
    storeInitialPose();
    generateSquatKeyframes(0.15);
  }

  void findTorqueDofs()
  {
    mTorqueNames.clear();
    mTorqueDofs.clear();

    std::vector<std::string> candidates
        = {"j_thigh_left_z",
           "j_shin_left",
           "j_heel_left_1",
           "j_thigh_right_z",
           "j_shin_right",
           "j_heel_right_1"};

    for (const auto& name : candidates) {
      auto dof = mBiped->getDof(name);
      if (dof && mTorqueNames.size() < 3) {
        mTorqueNames.push_back(name);
        mTorqueDofs.push_back(dof->getIndexInSkeleton());
      }
    }
  }

  void storeInitialPose()
  {
    mInitialPositions = mBiped->getPositions();
  }

  void generateSquatKeyframes(double depth)
  {
    std::cout << "Generating squat keyframes with depth " << depth << " m..."
              << std::endl;

    mBiped->setPositions(mInitialPositions);

    const std::size_t numKeyframes = 60;
    mKeyframes.clear();
    mKeyframes.reserve(numKeyframes);

    auto leftHeelIK = mLeftHeel->getIK(true);
    auto rightHeelIK = mRightHeel->getIK(true);

    auto leftHeelTarget = std::make_shared<dynamics::SimpleFrame>(
        dynamics::Frame::World(),
        "leftHeelTarget",
        mLeftHeel->getWorldTransform());
    auto rightHeelTarget = std::make_shared<dynamics::SimpleFrame>(
        dynamics::Frame::World(),
        "rightHeelTarget",
        mRightHeel->getWorldTransform());

    leftHeelIK->setTarget(leftHeelTarget);
    leftHeelIK->setHierarchyLevel(0);
    rightHeelIK->setTarget(rightHeelTarget);
    rightHeelIK->setHierarchyLevel(0);

    auto pelvis = mBiped->getBodyNode(0);
    auto pelvisIK = pelvis->getIK(true);
    auto pelvisTarget = std::make_shared<dynamics::SimpleFrame>(
        dynamics::Frame::World(), "pelvisTarget", pelvis->getWorldTransform());
    pelvisIK->setTarget(pelvisTarget);
    pelvisIK->setHierarchyLevel(1);

    Eigen::Isometry3d pelvisInitial = pelvis->getWorldTransform();

    auto wbIK = mBiped->getIK(true);
    if (auto solver = wbIK->getSolver()) {
      if (auto gd = std::dynamic_pointer_cast<optimizer::GradientDescentSolver>(
              solver)) {
        gd->setNumMaxIterations(200);
      }
    }

    for (std::size_t i = 0; i < numKeyframes; ++i) {
      double phase = static_cast<double>(i) / static_cast<double>(numKeyframes);
      double offset = depth * 0.5
                      * (1.0 - std::cos(2.0 * math::constantsd::pi() * phase));

      Eigen::Isometry3d pelvisTf = pelvisInitial;
      pelvisTf.translation()[1] -= offset;
      pelvisTarget->setTransform(pelvisTf);

      wbIK->solveAndApply(true);
      mKeyframes.push_back(mBiped->getPositions());

      if (i == 0) {
        auto solver = wbIK->getSolver();
        if (solver) {
          auto gd = std::dynamic_pointer_cast<optimizer::GradientDescentSolver>(
              solver);
          if (gd) {
            gd->setNumMaxIterations(50);
          }
        }
      }
    }

    mBiped->setPositions(mInitialPositions);
    std::cout << "Generated " << mKeyframes.size() << " keyframes" << std::endl;
  }

  Eigen::VectorXd catmullRomInterpolate(double t) const
  {
    if (mKeyframes.empty()) {
      return mBiped->getPositions();
    }

    std::size_t n = mKeyframes.size();
    double scaledT = t * static_cast<double>(n);
    std::size_t i1 = static_cast<std::size_t>(scaledT) % n;
    double u = scaledT - std::floor(scaledT);

    std::size_t i0 = (i1 + n - 1) % n;
    std::size_t i2 = (i1 + 1) % n;
    std::size_t i3 = (i1 + 2) % n;

    const auto& p0 = mKeyframes[i0];
    const auto& p1 = mKeyframes[i1];
    const auto& p2 = mKeyframes[i2];
    const auto& p3 = mKeyframes[i3];

    double u2 = u * u;
    double u3 = u2 * u;

    Eigen::VectorXd result = 0.5
                             * ((-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u3
                                + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u2
                                + (-p0 + p2) * u + 2.0 * p1);

    return result;
  }

  Eigen::VectorXd catmullRomVelocity(double t) const
  {
    if (mKeyframes.empty()) {
      return Eigen::VectorXd::Zero(mBiped->getNumDofs());
    }

    std::size_t n = mKeyframes.size();
    double scaledT = t * static_cast<double>(n);
    std::size_t i1 = static_cast<std::size_t>(scaledT) % n;
    double u = scaledT - std::floor(scaledT);

    std::size_t i0 = (i1 + n - 1) % n;
    std::size_t i2 = (i1 + 1) % n;
    std::size_t i3 = (i1 + 2) % n;

    const auto& p0 = mKeyframes[i0];
    const auto& p1 = mKeyframes[i1];
    const auto& p2 = mKeyframes[i2];
    const auto& p3 = mKeyframes[i3];

    double u2 = u * u;

    Eigen::VectorXd result
        = 0.5 * static_cast<double>(n)
          * ((3.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3)) * u2
             + (2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3)) * u + (-p0 + p2));

    return result;
  }

  Eigen::VectorXd catmullRomAcceleration(double t) const
  {
    if (mKeyframes.empty()) {
      return Eigen::VectorXd::Zero(mBiped->getNumDofs());
    }

    std::size_t n = mKeyframes.size();
    double scaledT = t * static_cast<double>(n);
    std::size_t i1 = static_cast<std::size_t>(scaledT) % n;
    double u = scaledT - std::floor(scaledT);

    std::size_t i0 = (i1 + n - 1) % n;
    std::size_t i2 = (i1 + 1) % n;
    std::size_t i3 = (i1 + 2) % n;

    const auto& p0 = mKeyframes[i0];
    const auto& p1 = mKeyframes[i1];
    const auto& p2 = mKeyframes[i2];
    const auto& p3 = mKeyframes[i3];

    double nSq = static_cast<double>(n * n);

    Eigen::VectorXd result = 0.5 * nSq
                             * ((6.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3)) * u
                                + 2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3));

    return result;
  }

  bool isValid() const
  {
    return mValid;
  }

  void customPreRefresh() override
  {
    if (!mValid) {
      return;
    }

    if (mRegenerateRequested.exchange(false) && mWidget) {
      generateSquatKeyframes(mWidget->getSquatDepth());
    }

    const double cycleDuration = 3.0;
    const bool playing = mWidget && mWidget->isPlaying();
    const double speed
        = mWidget ? static_cast<double>(mWidget->getPlaybackSpeed()) : 1.0;
    if (playing) {
      mTime += 1.0 / 60.0 * speed;
    }

    const double phase = std::fmod(mTime, cycleDuration) / cycleDuration;

    // The spline derivatives are taken with respect to the normalized phase;
    // convert them to time derivatives of the displayed motion. A paused
    // playback yields zero velocity/acceleration, i.e. gravity compensation.
    const double phaseRate = playing ? speed / cycleDuration : 0.0;

    Eigen::VectorXd q = catmullRomInterpolate(phase);
    Eigen::VectorXd dq = catmullRomVelocity(phase) * phaseRate;
    Eigen::VectorXd ddq = catmullRomAcceleration(phase) * phaseRate * phaseRate;

    mBiped->setPositions(q);
    mBiped->setVelocities(dq);
    mBiped->setAccelerations(ddq);

    Eigen::Vector3d upDir = -mWorld->getGravity().normalized();

    std::vector<dynamics::ContactInverseDynamics::Contact> contacts;
    std::array<dynamics::BodyNode*, 4> contactBodies
        = {mLeftHeel, mLeftToe, mRightHeel, mRightToe};

    for (auto* body : contactBodies) {
      dynamics::ContactInverseDynamics::Contact contact;
      contact.bodyNode = body;
      contact.localOffset = Eigen::Vector3d::Zero();
      contact.normal = upDir;
      contact.frictionCoeff = mWidget ? mWidget->getFrictionCoeff() : 0.8;
      contact.numBasis = 8;
      contacts.push_back(contact);
    }

    mSolver.setContacts(contacts);
    auto result = mSolver.compute();

    if (mWidget) {
      double residualNorm = result.unactuatedResidual.lpNorm<Eigen::Infinity>();
      mWidget->setStatus(
          result.feasible,
          residualNorm,
          result.contactForces,
          result.jointForces,
          mTorqueNames,
          mTorqueDofs);

      float forceScale = mWidget->getForceScale();
      for (std::size_t i = 0;
           i < result.contactForces.size() && i < mContactFrames.size();
           ++i) {
        auto* visual = mContactFrames[i]->getVisualAspect(true);
        const Eigen::Vector3d force = result.contactForces[i];
        if (force.norm() < 1e-8) {
          visual->setHidden(true);
          continue;
        }
        visual->setHidden(false);

        Eigen::Vector3d worldPos
            = contactBodies[i]->getWorldTransform() * contacts[i].localOffset;
        Eigen::Vector3d headPos = worldPos + forceScale * force;

        mArrows[i]->setPositions(worldPos, headPos);

        Eigen::Vector4d color = result.feasible
                                    ? Eigen::Vector4d(0.0, 1.0, 0.0, 1.0)
                                    : Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
        mArrows[i]->notifyColorUpdated(color);
      }
    }
  }

  void setWidget(std::shared_ptr<ContactInverseDynamicsWidget> widget)
  {
    mWidget = std::move(widget);
  }

  /// Called from the ImGui widget (draw callback). The actual regeneration
  /// runs at the start of the next update traversal to avoid touching the
  /// skeleton and keyframes from the draw thread.
  void requestKeyframeRegeneration()
  {
    mRegenerateRequested = true;
  }

protected:
  dart::dynamics::SkeletonPtr mBiped;
  dynamics::ContactInverseDynamics mSolver;

  dynamics::BodyNode* mLeftHeel;
  dynamics::BodyNode* mLeftToe;
  dynamics::BodyNode* mRightHeel;
  dynamics::BodyNode* mRightToe;

  std::vector<dynamics::SimpleFramePtr> mContactFrames;
  std::vector<std::shared_ptr<dynamics::ArrowShape>> mArrows;

  std::vector<Eigen::VectorXd> mKeyframes;
  Eigen::VectorXd mInitialPositions;

  std::vector<std::string> mTorqueNames;
  std::vector<std::size_t> mTorqueDofs;

  double mTime;
  std::shared_ptr<ContactInverseDynamicsWidget> mWidget;
  std::atomic<bool> mRegenerateRequested{false};
  bool mValid{true};
};

//==============================================================================
int main(int argc, char* argv[])
{
  int maxFrames = -1;
  std::string screenshotPath;
  double guiScale = kDefaultGuiScale;
  if (const char* envScale = std::getenv("DART_GUI_SCALE")) {
    guiScale = parseGuiScale(envScale);
  }

  const std::string guiScalePrefix = "--gui-scale=";
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--frames" && i + 1 < argc) {
      maxFrames = std::atoi(argv[++i]);
    } else if (arg == "--screenshot" && i + 1 < argc) {
      screenshotPath = argv[++i];
    } else if (arg == "--gui-scale" && i + 1 < argc) {
      guiScale = parseGuiScale(argv[++i]);
    } else if (arg.rfind(guiScalePrefix, 0) == 0) {
      guiScale = parseGuiScale(arg.substr(guiScalePrefix.size()));
    }
  }

  const int windowWidth = scaleWindowExtent(1280, guiScale);
  const int windowHeight = scaleWindowExtent(720, guiScale);

  auto world
      = dart::utils::SkelParser::readWorld("dart://sample/skel/fullbody1.skel");
  if (!world) {
    std::cerr << "Error: Could not load dart://sample/skel/fullbody1.skel"
              << std::endl;
    return 1;
  }
  world->setGravity(Eigen::Vector3d(0, -9.81, 0));

  auto biped = world->getSkeleton("fullbody1");
  if (!biped) {
    std::cerr << "Error: Could not find the fullbody1 skeleton" << std::endl;
    return 1;
  }

  const auto setDofPosition = [&](const std::string& name, double position) {
    if (auto* dof = biped->getDof(name)) {
      dof->setPosition(position);
    } else {
      std::cerr << "Warning: missing DOF " << name << std::endl;
    }
  };
  setDofPosition("j_pelvis_rot_y", -0.20);
  setDofPosition("j_thigh_left_z", 0.15);
  setDofPosition("j_shin_left", -0.40);
  setDofPosition("j_heel_left_1", 0.25);
  setDofPosition("j_thigh_right_z", 0.15);
  setDofPosition("j_shin_right", -0.40);
  setDofPosition("j_heel_right_1", 0.25);
  setDofPosition("j_abdomen_2", 0.00);

  ::osg::ref_ptr<ContactInverseDynamicsWorldNode> node
      = new ContactInverseDynamicsWorldNode(world, biped);
  if (!node->isValid()) {
    return 1;
  }

  if (maxFrames > 0) {
    osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
        = new dart::gui::osg::ImGuiViewer();
    viewer->addWorldNode(node);

    auto widget = std::make_shared<ContactInverseDynamicsWidget>(
        viewer, world, biped, guiScale, [nodePtr = node.get()]() {
          nodePtr->requestKeyframeRegeneration();
        });
    viewer->getImGuiHandler()->addWidget(widget);
    node->setWidget(widget);

    viewer->setUpViewInWindow(0, 0, windowWidth, windowHeight);
    viewer->getCameraManipulator()->setHomePosition(
        ::osg::Vec3(3.0f, 1.5f, 3.0f),
        ::osg::Vec3(0.0f, 0.0f, 0.0f),
        ::osg::Vec3(0.0f, 1.0f, 0.0f));
    viewer->setCameraManipulator(viewer->getCameraManipulator());

    ::osg::ref_ptr<osgViewer::ScreenCaptureHandler> captureHandler;
    if (!screenshotPath.empty()) {
      // WriteToFile appends "_<context>_<frame>.<ext>" to the base name.
      std::string basePath = screenshotPath;
      const std::string extension = ".png";
      if (basePath.size() > extension.size()
          && basePath.compare(
                 basePath.size() - extension.size(),
                 extension.size(),
                 extension)
                 == 0) {
        basePath = basePath.substr(0, basePath.size() - extension.size());
      }
      captureHandler = new osgViewer::ScreenCaptureHandler(
          new osgViewer::ScreenCaptureHandler::WriteToFile(basePath, "png"));
      viewer->addEventHandler(captureHandler);
    }

    for (int i = 0; i < maxFrames; ++i) {
      if (captureHandler && i == maxFrames - 1) {
        captureHandler->captureNextFrame(*viewer);
      }
      viewer->frame();
    }

    if (captureHandler) {
      std::cout << "Screenshot saved with base path: " << screenshotPath
                << std::endl;
    }

    return 0;
  }

  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
      = new dart::gui::osg::ImGuiViewer();
  auto shadow = gui::osg::WorldNode::createDefaultShadowTechnique(viewer);
  node->setShadowTechnique(shadow);
  viewer->addWorldNode(node);

  auto widget = std::make_shared<ContactInverseDynamicsWidget>(
      viewer, world, biped, guiScale, [nodePtr = node.get()]() {
        nodePtr->requestKeyframeRegeneration();
      });
  viewer->getImGuiHandler()->addWidget(widget);
  node->setWidget(widget);

  viewer->setUpViewInWindow(0, 0, windowWidth, windowHeight);
  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(3.0f, 1.5f, 3.0f),
      ::osg::Vec3(0.0f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 1.0f, 0.0f));
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  std::cout << "Contact-Aware Inverse Dynamics Demo\n";
  std::cout << "Use the ImGui widget to control playback and parameters\n";
  std::cout << "Panel too small on a HiDPI display? Re-run with "
            << "--gui-scale 2 (or set DART_GUI_SCALE).\n";

  viewer->run();

  return 0;
}
