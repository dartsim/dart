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

#include <dart/gui/all.hpp>
#include <dart/gui/include_im_gui.hpp>

#include <dart/all.hpp>

#include <CLI/CLI.hpp>
#include <osg/Image>
#include <osgDB/WriteFile>

#include <algorithm>
#include <chrono>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <cmath>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::constraint;

namespace {

constexpr int kMaxHistorySize = 120;
constexpr float kDefaultFontSize = 13.0f;

enum class Scenario
{
  MassRatio,
  BoxStack,
  BallDrop,
  Dominos,
  InclinedPlane
};

enum class SolverType
{
  Dantzig,
  Pgs
};

struct ScenarioInfo
{
  Scenario type;
  std::string name;
  std::string description;
  std::string explanation;
};

struct SolverInfo
{
  SolverType type;
  std::string name;
  std::string description;
};

std::vector<ScenarioInfo> GetScenarios()
{
  return {
      {Scenario::MassRatio,
       "Mass Ratio (1000:1)",
       "Heavy box on light box",
       "Tests solver stability with extreme mass ratios.\n\n"
       "A 1000kg box sits on a 1kg box. Poor solvers exhibit:\n"
       "- Jittering or vibration\n"
       "- Light box sinking into ground\n"
       "- Numerical instability\n\n"
       "Reference: SimBenchmark, Silcowitz et al. 2009"},
      {Scenario::BoxStack,
       "Box Pyramid",
       "Stacked box pyramid",
       "Tests shock propagation through contact graph.\n\n"
       "A pyramid of boxes must maintain stable stacking.\n"
       "Challenges:\n"
       "- Multiple simultaneous contacts\n"
       "- Load distribution through stack\n"
       "- Sensitivity to solver order\n\n"
       "Reference: Guendelman et al. 2003"},
      {Scenario::BallDrop,
       "Ball Drop (75 balls)",
       "Many balls dropping",
       "Tests many-contact performance and stability.\n\n"
       "75 balls drop and settle. Challenges:\n"
       "- O(n²) potential contacts\n"
       "- Solver iteration scaling\n"
       "- Contact graph complexity\n\n"
       "Reference: SimBenchmark '666 balls'"},
      {Scenario::Dominos,
       "Domino Chain",
       "Sequential domino toppling",
       "Tests impulse propagation accuracy.\n\n"
       "First domino is tilted to start chain reaction.\n"
       "Challenges:\n"
       "- Sequential impulse transfer\n"
       "- Timing accuracy\n"
       "- Energy conservation\n\n"
       "Reference: Classic benchmark"},
      {Scenario::InclinedPlane,
       "Inclined Plane",
       "Block sliding on ramp",
       "Tests friction model accuracy.\n\n"
       "A block slides down a 23° ramp (angle > arctan(μ)).\n"
       "Verifies:\n"
       "- Coulomb friction cone\n"
       "- Sliding vs sticking transition\n"
       "- Friction coefficient accuracy\n\n"
       "Reference: Stewart & Trinkle 1996"}};
}

std::vector<SolverInfo> GetSolvers()
{
  return {
      {SolverType::Dantzig,
       "Dantzig",
       "Pivoting method, exact for well-posed problems"},
      {SolverType::Pgs,
       "PGS (Projected Gauss-Seidel)",
       "Iterative method, fast but approximate"}};
}

SkeletonPtr createGround()
{
  SkeletonPtr ground = Skeleton::create("ground");

  BodyNodePtr body = ground->createJointAndBodyNodePair<WeldJoint>().second;

  double thickness = 0.1;
  auto shape
      = std::make_shared<BoxShape>(Eigen::Vector3d(20.0, thickness, 20.0));
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.8, 0.8));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation().y() = -thickness / 2.0;
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  auto* dynamics = shapeNode->getDynamicsAspect();
  dynamics->setRestitutionCoeff(0.3);
  dynamics->setFrictionCoeff(0.8);
  dynamics->setPrimarySlipCompliance(0.0);
  dynamics->setSecondarySlipCompliance(0.0);

  return ground;
}

SkeletonPtr createBox(
    const std::string& name,
    const Eigen::Vector3d& size,
    double mass,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color = Eigen::Vector3d(0.6, 0.6, 0.8))
{
  SkeletonPtr box = Skeleton::create(name);

  BodyNodePtr body = box->createJointAndBodyNodePair<FreeJoint>().second;

  auto shape = std::make_shared<BoxShape>(size);
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(color);

  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  auto* dynamics = shapeNode->getDynamicsAspect();
  dynamics->setRestitutionCoeff(0.3);
  dynamics->setFrictionCoeff(0.8);
  dynamics->setPrimarySlipCompliance(0.0);
  dynamics->setSecondarySlipCompliance(0.0);

  return box;
}

SkeletonPtr createSphere(
    const std::string& name,
    double radius,
    double mass,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color = Eigen::Vector3d(0.8, 0.4, 0.4))
{
  SkeletonPtr sphere = Skeleton::create(name);

  BodyNodePtr body = sphere->createJointAndBodyNodePair<FreeJoint>().second;

  auto shape = std::make_shared<SphereShape>(radius);
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(color);

  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  auto* dynamics = shapeNode->getDynamicsAspect();
  dynamics->setRestitutionCoeff(0.5);
  dynamics->setFrictionCoeff(0.6);
  dynamics->setPrimarySlipCompliance(0.0);
  dynamics->setSecondarySlipCompliance(0.0);

  return sphere;
}

WorldPtr createMassRatioScenario()
{
  auto world = World::create("mass_ratio");
  world->setGravity(Eigen::Vector3d(0, -9.81, 0));

  world->addSkeleton(createGround());

  double boxSize = 0.5;
  world->addSkeleton(createBox(
      "light_box",
      Eigen::Vector3d(boxSize, boxSize, boxSize),
      1.0,
      Eigen::Vector3d(0, boxSize / 2.0, 0),
      Eigen::Vector3d(0.4, 0.8, 0.4)));

  world->addSkeleton(createBox(
      "heavy_box",
      Eigen::Vector3d(boxSize, boxSize, boxSize),
      1000.0,
      Eigen::Vector3d(0, boxSize * 1.6, 0),
      Eigen::Vector3d(0.8, 0.2, 0.2)));

  return world;
}

WorldPtr createBoxStackScenario()
{
  auto world = World::create("box_stack");
  world->setGravity(Eigen::Vector3d(0, -9.81, 0));

  world->addSkeleton(createGround());

  double boxSize = 0.3;
  int layers = 5;

  int boxIndex = 0;
  for (int layer = 0; layer < layers; ++layer) {
    int boxesInLayer = layers - layer;
    double y = boxSize / 2.0 + layer * boxSize * 1.05;
    double startX = -(boxesInLayer - 1) * boxSize * 0.55;

    for (int i = 0; i < boxesInLayer; ++i) {
      double x = startX + i * boxSize * 1.1;
      double hue = static_cast<double>(boxIndex) / (layers * (layers + 1) / 2);
      Eigen::Vector3d color(0.3 + 0.5 * hue, 0.5, 0.8 - 0.5 * hue);

      world->addSkeleton(createBox(
          "box_" + std::to_string(boxIndex),
          Eigen::Vector3d(boxSize, boxSize, boxSize),
          1.0,
          Eigen::Vector3d(x, y, 0),
          color));
      ++boxIndex;
    }
  }

  return world;
}

WorldPtr createBallDropScenario()
{
  auto world = World::create("ball_drop");
  world->setGravity(Eigen::Vector3d(0, -9.81, 0));

  world->addSkeleton(createGround());

  double radius = 0.1;
  int gridSize = 5;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> jitter(-0.02, 0.02);

  int ballIndex = 0;
  for (int x = 0; x < gridSize; ++x) {
    for (int z = 0; z < gridSize; ++z) {
      for (int y = 0; y < 3; ++y) {
        double px = (x - gridSize / 2.0) * radius * 2.5 + jitter(rng);
        double py = radius + y * radius * 2.2 + 0.5;
        double pz = (z - gridSize / 2.0) * radius * 2.5 + jitter(rng);

        double hue = static_cast<double>(ballIndex) / (gridSize * gridSize * 3);
        Eigen::Vector3d color(0.8 - 0.4 * hue, 0.4 + 0.4 * hue, 0.4);

        world->addSkeleton(createSphere(
            "ball_" + std::to_string(ballIndex),
            radius,
            0.5,
            Eigen::Vector3d(px, py, pz),
            color));
        ++ballIndex;
      }
    }
  }

  return world;
}

WorldPtr createDominosScenario()
{
  auto world = World::create("dominos");
  world->setGravity(Eigen::Vector3d(0, -9.81, 0));

  world->addSkeleton(createGround());

  double width = 0.05;
  double height = 0.3;
  double depth = 0.15;
  double spacing = 0.12;
  int count = 20;

  for (int i = 0; i < count; ++i) {
    double x = (i - count / 2.0) * spacing;
    double hue = static_cast<double>(i) / count;
    Eigen::Vector3d color(0.2 + 0.6 * hue, 0.3, 0.8 - 0.5 * hue);

    auto domino = createBox(
        "domino_" + std::to_string(i),
        Eigen::Vector3d(width, height, depth),
        0.5,
        Eigen::Vector3d(x, height / 2.0, 0),
        color);

    if (i == 0) {
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation() = Eigen::Vector3d(x, height / 2.0, 0);
      Eigen::AngleAxisd rotation(0.3, Eigen::Vector3d::UnitZ());
      tf.linear() = rotation.toRotationMatrix();
      domino->getBodyNode(0)->getParentJoint()->setTransformFromParentBodyNode(
          tf);
    }

    world->addSkeleton(domino);
  }

  return world;
}

WorldPtr createInclinedPlaneScenario()
{
  auto world = World::create("inclined_plane");
  world->setGravity(Eigen::Vector3d(0, -9.81, 0));

  SkeletonPtr ramp = Skeleton::create("ramp");
  BodyNodePtr rampBody = ramp->createJointAndBodyNodePair<WeldJoint>().second;

  auto rampShape = std::make_shared<BoxShape>(Eigen::Vector3d(3.0, 0.1, 1.0));
  auto rampNode = rampBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(rampShape);
  rampNode->getVisualAspect()->setColor(Eigen::Vector3d(0.5, 0.5, 0.5));

  double angle = 0.4;
  Eigen::Isometry3d rampTf = Eigen::Isometry3d::Identity();
  rampTf.translation() = Eigen::Vector3d(0, 0.5, 0);
  Eigen::AngleAxisd rampRotation(angle, Eigen::Vector3d::UnitZ());
  rampTf.linear() = rampRotation.toRotationMatrix();
  rampBody->getParentJoint()->setTransformFromParentBodyNode(rampTf);

  auto* rampDynamics = rampNode->getDynamicsAspect();
  rampDynamics->setFrictionCoeff(0.5);
  rampDynamics->setPrimarySlipCompliance(0.0);
  rampDynamics->setSecondarySlipCompliance(0.0);
  world->addSkeleton(ramp);

  double boxSize = 0.2;
  auto box = createBox(
      "sliding_box",
      Eigen::Vector3d(boxSize, boxSize, boxSize),
      1.0,
      Eigen::Vector3d(
          -1.0 * std::cos(angle) + 0.5 * std::sin(angle),
          1.0 * std::sin(angle) + 0.5 * std::cos(angle) + boxSize / 2.0,
          0),
      Eigen::Vector3d(0.8, 0.6, 0.2));
  box->getBodyNode(0)->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(
      0.3);
  world->addSkeleton(box);

  world->addSkeleton(createGround());

  return world;
}

WorldPtr createScenario(Scenario scenario)
{
  switch (scenario) {
    case Scenario::MassRatio:
      return createMassRatioScenario();
    case Scenario::BoxStack:
      return createBoxStackScenario();
    case Scenario::BallDrop:
      return createBallDropScenario();
    case Scenario::Dominos:
      return createDominosScenario();
    case Scenario::InclinedPlane:
      return createInclinedPlaneScenario();
  }
  return createMassRatioScenario();
}

void applySolver(WorldPtr world, SolverType solverType)
{
  auto* solver = world->getConstraintSolver();
  if (!solver) {
    return;
  }

  switch (solverType) {
    case SolverType::Dantzig:
      solver->setLcpSolver(std::make_shared<dart::math::DantzigSolver>());
      break;
    case SolverType::Pgs:
      solver->setLcpSolver(std::make_shared<dart::math::PgsSolver>());
      break;
  }
}

class LcpPhysicsWorldNode : public RealTimeWorldNode
{
public:
  LcpPhysicsWorldNode(
      const WorldPtr& world,
      const osg::ref_ptr<osgShadow::ShadowTechnique>& shadower)
    : RealTimeWorldNode(world, shadower),
      mStepTime(0.0),
      mContactCount(0),
      mStepCount(0)
  {
  }

  void customPreStep() override
  {
    mStepStart = std::chrono::high_resolution_clock::now();
  }

  void customPostStep() override
  {
    auto end = std::chrono::high_resolution_clock::now();
    mStepTime
        = std::chrono::duration<double, std::milli>(end - mStepStart).count();

    auto* solver = mWorld->getConstraintSolver();
    if (solver) {
      mContactCount = solver->getLastCollisionResult().getNumContacts();
    }
    ++mStepCount;
  }

  double getStepTimeMs() const
  {
    return mStepTime;
  }
  std::size_t getContactCount() const
  {
    return mContactCount;
  }
  std::size_t getStepCount() const
  {
    return mStepCount;
  }
  void resetStepCount()
  {
    mStepCount = 0;
  }

private:
  std::chrono::high_resolution_clock::time_point mStepStart;
  double mStepTime;
  std::size_t mContactCount;
  std::size_t mStepCount;
};

class LcpPhysicsWidget : public ImGuiWidget
{
public:
  LcpPhysicsWidget(
      ImGuiViewer* viewer,
      osg::ref_ptr<LcpPhysicsWorldNode> worldNode,
      int initialScenario,
      int initialSolver)
    : mViewer(viewer),
      mWorldNode(worldNode),
      mWorld(worldNode->getWorld()),
      mScenarios(GetScenarios()),
      mSolvers(GetSolvers()),
      mCurrentScenario(initialScenario),
      mCurrentSolver(initialSolver),
      mTimeStep(1000),
      mFrameCount(0),
      mLastFrameTime(std::chrono::high_resolution_clock::now())
  {
    mTimeStep = static_cast<int>(1.0 / mWorld->getTimeStep());
  }

  void render() override
  {
    updateFps();
    mUiScale = getUiScale();

    ImGui::SetNextWindowPos(
        ImVec2(10.0f * mUiScale, 10.0f * mUiScale), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(
        ImVec2(340.0f * mUiScale, 600.0f * mUiScale), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.85f);

    if (!ImGui::Begin("LCP Physics Control")) {
      ImGui::End();
      return;
    }

    renderSimulationControl();
    renderScenarioSection();
    renderSolverSection();
    renderParameterSection();
    renderPerformanceSection();
    renderDebugSection();
    renderExplanationSection();

    ImGui::End();
  }

private:
  void updateFps()
  {
    ++mFrameCount;
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed
        = std::chrono::duration<double>(now - mLastFrameTime).count();
    if (elapsed >= 0.5) {
      mFps = mFrameCount / elapsed;
      mFrameCount = 0;
      mLastFrameTime = now;
    }

    double stepTime = mWorldNode->getStepTimeMs();
    mStepTimeHistory.push_back(static_cast<float>(stepTime));
    if (mStepTimeHistory.size() > kMaxHistorySize) {
      mStepTimeHistory.pop_front();
    }
  }

  void renderSimulationControl()
  {
    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
      bool simulating = mViewer->isSimulating();

      if (ImGui::Button(
              simulating ? "Pause" : "Play", ImVec2(70.0f * mUiScale, 0.0f))) {
        mViewer->simulate(!simulating);
      }
      ImGui::SameLine();
      if (ImGui::Button("Step", ImVec2(70.0f * mUiScale, 0.0f))) {
        if (simulating) {
          mViewer->simulate(false);
        }
        mWorld->step();
      }
      ImGui::SameLine();
      if (ImGui::Button("Reset", ImVec2(70.0f * mUiScale, 0.0f))) {
        resetScenario();
      }

      ImGui::Text("Time: %.3f s", mWorld->getTime());
      ImGui::Text(
          "Steps: %zu (%.1f steps/frame)",
          mWorldNode->getStepCount(),
          mWorldNode->getStepCount() / std::max(1.0, mWorld->getTime() * 60.0));
    }
  }

  void renderScenarioSection()
  {
    if (ImGui::CollapsingHeader("Scenario", ImGuiTreeNodeFlags_DefaultOpen)) {
      int prevScenario = mCurrentScenario;

      for (int i = 0; i < static_cast<int>(mScenarios.size()); ++i) {
        if (ImGui::RadioButton(
                mScenarios[i].name.c_str(), &mCurrentScenario, i)) {
          if (prevScenario != mCurrentScenario) {
            switchScenario(static_cast<Scenario>(mCurrentScenario));
          }
        }
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip("%s", mScenarios[i].description.c_str());
        }
      }
    }
  }

  void renderSolverSection()
  {
    if (ImGui::CollapsingHeader("LCP Solver", ImGuiTreeNodeFlags_DefaultOpen)) {
      int prevSolver = mCurrentSolver;

      for (int i = 0; i < static_cast<int>(mSolvers.size()); ++i) {
        if (ImGui::RadioButton(mSolvers[i].name.c_str(), &mCurrentSolver, i)) {
          if (prevSolver != mCurrentSolver) {
            applySolver(mWorld, mSolvers[mCurrentSolver].type);
          }
        }
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip("%s", mSolvers[i].description.c_str());
        }
      }
    }
  }

  void renderParameterSection()
  {
    if (ImGui::CollapsingHeader("Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::SliderInt("Timestep (Hz)", &mTimeStep, 100, 2000)) {
        mWorld->setTimeStep(1.0 / mTimeStep);
      }

      Eigen::Vector3d g = mWorld->getGravity();
      float gravity = static_cast<float>(-g.y());
      if (ImGui::SliderFloat("Gravity (m/s²)", &gravity, 0.0f, 20.0f)) {
        mWorld->setGravity(Eigen::Vector3d(0, -gravity, 0));
      }
    }
  }

  void renderPerformanceSection()
  {
    if (ImGui::CollapsingHeader(
            "Performance", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Text("Render FPS: %.1f", mFps);
      ImGui::Text("Contacts: %zu", mWorldNode->getContactCount());
      ImGui::Text("Bodies: %zu", mWorld->getNumSkeletons());

      if (!mStepTimeHistory.empty()) {
        std::vector<float> history(
            mStepTimeHistory.begin(), mStepTimeHistory.end());
        float maxTime
            = *std::max_element(history.begin(), history.end()) * 1.2f;
        maxTime = std::max(maxTime, 1.0f);

        ImGui::Text("Step time (ms):");
        ImGui::PlotLines(
            "##steptime",
            history.data(),
            static_cast<int>(history.size()),
            0,
            nullptr,
            0.0f,
            maxTime,
            ImVec2(300.0f * mUiScale, 60.0f * mUiScale));

        float avgTime = 0.0f;
        for (float t : history) {
          avgTime += t;
        }
        avgTime /= static_cast<float>(history.size());
        ImGui::Text("Avg: %.2f ms, Max: %.2f ms", avgTime, maxTime / 1.2f);
      }
    }
  }

  void renderExplanationSection()
  {
    if (ImGui::CollapsingHeader("About This Scenario")) {
      ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 310.0f * mUiScale);
      ImGui::TextUnformatted(mScenarios[mCurrentScenario].explanation.c_str());
      ImGui::PopTextWrapPos();
    }
  }

  void renderDebugSection()
  {
    if (!ImGui::CollapsingHeader("ImGui Debug")) {
      return;
    }

    const ImGuiIO& io = ImGui::GetIO();
    ImGui::Text("DisplaySize: %.0f x %.0f", io.DisplaySize.x, io.DisplaySize.y);
    ImGui::Text(
        "FramebufferScale: %.2f x %.2f",
        io.DisplayFramebufferScale.x,
        io.DisplayFramebufferScale.y);
    ImGui::Text("FontSize: %.1f", ImGui::GetFontSize());
    ImGui::Text("FontGlobalScale: %.3f", io.FontGlobalScale);
    ImGui::Text("UiScale: %.2f", mUiScale);
    if (io.Fonts && io.Fonts->TexData) {
      ImGui::Text(
          "FontTex: %d x %d",
          io.Fonts->TexData->Width,
          io.Fonts->TexData->Height);
    }
  }

  float getUiScale() const
  {
    const float fontSize = ImGui::GetFontSize();
    if (!std::isfinite(fontSize) || fontSize <= 0.0f) {
      return 1.0f;
    }
    return fontSize / kDefaultFontSize;
  }

  void resetScenario()
  {
    switchScenario(static_cast<Scenario>(mCurrentScenario));
  }

  void switchScenario(Scenario scenario)
  {
    bool wasSimulating = mViewer->isSimulating();
    mViewer->simulate(false);

    auto newWorld = createScenario(scenario);
    newWorld->setTimeStep(1.0 / mTimeStep);
    applySolver(newWorld, mSolvers[mCurrentSolver].type);

    mWorld = newWorld;
    mWorldNode->setWorld(newWorld);
    mWorldNode->resetStepCount();
    mStepTimeHistory.clear();

    if (wasSimulating) {
      mViewer->simulate(true);
    }
  }

  ImGuiViewer* mViewer;
  osg::ref_ptr<LcpPhysicsWorldNode> mWorldNode;
  WorldPtr mWorld;

  std::vector<ScenarioInfo> mScenarios;
  std::vector<SolverInfo> mSolvers;
  int mCurrentScenario;
  int mCurrentSolver;
  int mTimeStep;
  float mUiScale{1.0f};

  double mFps{0.0};
  int mFrameCount;
  std::chrono::high_resolution_clock::time_point mLastFrameTime;
  std::deque<float> mStepTimeHistory;
};

void listScenarios()
{
  std::cout << "Available scenarios:\n";
  for (const auto& s : GetScenarios()) {
    std::cout << "  " << s.name << ": " << s.description << "\n";
  }
}

void listSolvers()
{
  std::cout << "\nAvailable solvers:\n";
  for (const auto& s : GetSolvers()) {
    std::cout << "  " << s.name << ": " << s.description << "\n";
  }
}

int parseScenario(const std::string& name)
{
  auto scenarios = GetScenarios();
  for (int i = 0; i < static_cast<int>(scenarios.size()); ++i) {
    std::string lowerName = scenarios[i].name;
    std::transform(
        lowerName.begin(), lowerName.end(), lowerName.begin(), ::tolower);
    std::string lowerInput = name;
    std::transform(
        lowerInput.begin(), lowerInput.end(), lowerInput.begin(), ::tolower);
    if (lowerName.find(lowerInput) != std::string::npos) {
      return i;
    }
  }
  return 0;
}

int parseSolver(const std::string& name)
{
  auto solvers = GetSolvers();
  for (int i = 0; i < static_cast<int>(solvers.size()); ++i) {
    std::string lowerName = solvers[i].name;
    std::transform(
        lowerName.begin(), lowerName.end(), lowerName.begin(), ::tolower);
    std::string lowerInput = name;
    std::transform(
        lowerInput.begin(), lowerInput.end(), lowerInput.begin(), ::tolower);
    if (lowerName.find(lowerInput) != std::string::npos) {
      return i;
    }
  }
  return 0;
}

bool writeHeadlessFrame(
    const std::string& outDir,
    std::size_t frameIndex,
    const std::vector<uint8_t>& pixels,
    int width,
    int height)
{
  if (outDir.empty() || pixels.empty() || width <= 0 || height <= 0) {
    return false;
  }

  std::ostringstream path;
  path << outDir << "/frame_" << std::setfill('0') << std::setw(6) << frameIndex
       << std::setw(0) << ".png";

  ::osg::ref_ptr<::osg::Image> image = new ::osg::Image;
  image->setImage(
      width,
      height,
      1,
      GL_RGBA,
      GL_RGBA,
      GL_UNSIGNED_BYTE,
      const_cast<unsigned char*>(pixels.data()),
      ::osg::Image::NO_DELETE);

  return ::osgDB::writeImageFile(*image, path.str());
}

int runHeadless(
    int scenarioIdx,
    int solverIdx,
    int frames,
    const std::string& outDir,
    int width,
    int height,
    double guiScale)
{
  namespace fs = std::filesystem;

  if (!outDir.empty()) {
    std::error_code ec;
    fs::create_directories(outDir, ec);
    if (ec) {
      std::cerr << "Failed to create output directory: " << ec.message()
                << "\n";
      return EXIT_FAILURE;
    }
  }

  auto scenarios = GetScenarios();
  auto solvers = GetSolvers();

  auto world = createScenario(scenarios[scenarioIdx].type);
  world->setTimeStep(1.0 / 1000.0);
  applySolver(world, solvers[solverIdx].type);

  ImGuiViewer viewer(ViewerConfig::headless(width, height));
  viewer.setThreadingModel(::osgViewer::Viewer::SingleThreaded);
  viewer.setImGuiScale(static_cast<float>(guiScale));
  viewer.getImGuiHandler()->setFontScale(static_cast<float>(guiScale));
  auto shadow = WorldNode::createDefaultShadowTechnique(&viewer);
  osg::ref_ptr<LcpPhysicsWorldNode> worldNode
      = new LcpPhysicsWorldNode(world, shadow);
  viewer.addWorldNode(worldNode);

  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(3.0f, 2.0f, 3.0f),
      ::osg::Vec3(0.0f, 0.3f, 0.0f),
      ::osg::Vec3(0.0f, 1.0f, 0.0f));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  std::cout << "Scenario: " << scenarios[scenarioIdx].name << "\n";
  std::cout << "Solver: " << solvers[solverIdx].name << "\n";
  std::cout << "Frames: " << frames << ", Size: " << width << "x" << height
            << "\n";
  if (!outDir.empty()) {
    std::cout << "Output: " << outDir << "\n";
  }

  viewer.simulate(true);
  viewer.getImGuiHandler()->addWidget(
      std::make_shared<LcpPhysicsWidget>(
          &viewer, worldNode, scenarioIdx, solverIdx));

  double totalStepTime = 0.0;
  for (int i = 0; i < frames; ++i) {
    viewer.frame();
    totalStepTime += worldNode->getStepTimeMs();
    if (!outDir.empty()) {
      int captureWidth = 0;
      int captureHeight = 0;
      const auto pixels = viewer.captureBuffer(&captureWidth, &captureHeight);
      if (!writeHeadlessFrame(
              outDir,
              static_cast<std::size_t>(i),
              pixels,
              captureWidth,
              captureHeight)) {
        std::cerr << "Failed to capture frame " << i << "\n";
      }
    }
  }

  std::cout << "Completed " << frames << " frames\n";
  std::cout << "Sim time: " << std::fixed << std::setprecision(2)
            << world->getTime() << "s\n";
  std::cout << "Avg step: " << std::setprecision(3) << totalStepTime / frames
            << " ms\n";
  std::cout << "Contacts (final): " << worldNode->getContactCount() << "\n";

  return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char* argv[])
{
  CLI::App app(
      "LCP Physics - Challenging simulation scenarios with solver comparison");
  bool headless = false;
  bool listMode = false;
  std::string scenarioName = "mass";
  std::string solverName = "dantzig";
  int frames = 300;
  std::string outDir;
  int width = 1280;
  int height = 720;
  double guiScale = 1.0;

  app.add_flag("--headless", headless, "Run without display window");
  app.add_flag("--list", listMode, "List available scenarios and solvers");
  app.add_option(
      "--scenario", scenarioName, "Scenario (mass/box/ball/domino/incline)");
  app.add_option("--solver", solverName, "Solver (dantzig/pgs)");
  app.add_option("--frames", frames, "Number of frames")
      ->check(CLI::PositiveNumber);
  app.add_option("--out", outDir, "Output directory for frame capture");
  app.add_option("--width", width, "Viewport width")
      ->check(CLI::PositiveNumber);
  app.add_option("--height", height, "Viewport height")
      ->check(CLI::PositiveNumber);
  app.add_option("--gui-scale", guiScale, "GUI scale factor")
      ->check(CLI::PositiveNumber);
  CLI11_PARSE(app, argc, argv);

  if (listMode) {
    listScenarios();
    listSolvers();
    return EXIT_SUCCESS;
  }

  int scenarioIdx = parseScenario(scenarioName);
  int solverIdx = parseSolver(solverName);

  if (headless) {
    return runHeadless(
        scenarioIdx, solverIdx, frames, outDir, width, height, guiScale);
  }

  auto scenarios = GetScenarios();
  auto solvers = GetSolvers();

  auto world = createScenario(scenarios[scenarioIdx].type);
  world->setTimeStep(1.0 / 1000.0);
  applySolver(world, solvers[solverIdx].type);

  ImGuiViewer viewer;
  viewer.setImGuiScale(static_cast<float>(guiScale));
  viewer.getImGuiHandler()->setFontScale(static_cast<float>(guiScale));
  auto shadow = WorldNode::createDefaultShadowTechnique(&viewer);
  osg::ref_ptr<LcpPhysicsWorldNode> worldNode
      = new LcpPhysicsWorldNode(world, shadow);
  viewer.addWorldNode(worldNode);

  viewer.getImGuiHandler()->addWidget(
      std::make_shared<LcpPhysicsWidget>(
          &viewer, worldNode, scenarioIdx, solverIdx));

  viewer.setUpViewInWindow(100, 100, 1280, 720);

  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(3.0f, 2.0f, 3.0f),
      ::osg::Vec3(0.0f, 0.3f, 0.0f),
      ::osg::Vec3(0.0f, 1.0f, 0.0f));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  std::cout << "LCP Physics Example\n";
  std::cout << "Scenario: " << scenarios[scenarioIdx].name << "\n";
  std::cout << "Solver: " << solvers[solverIdx].name << "\n";
  std::cout << "Press SPACE to start/stop simulation\n";

  return viewer.run();
}
