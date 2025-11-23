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

#include <dart/config.hpp>

#include <dart/gui/osg/GridVisual.hpp>
#include <dart/gui/osg/ImGuiViewer.hpp>
#include <dart/gui/osg/ImGuiWidget.hpp>
#include <dart/gui/osg/RealTimeWorldNode.hpp>

#include <dart/utils/DartResourceRetriever.hpp>
#include <dart/utils/sdf/SdfParser.hpp>
#include <dart/utils/sdf/detail/SdfHelpers.hpp>

#if HAVE_BULLET
  #include <dart/collision/bullet/BulletCollisionDetector.hpp>
#endif
#if HAVE_ODE
  #include <dart/collision/ode/OdeCollisionDetector.hpp>
#endif
#include <dart/simulation/World.hpp>

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/DantzigBoxedLcpSolver.hpp>
#include <dart/constraint/MimicMotorConstraint.hpp>
#include <dart/constraint/PgsBoxedLcpSolver.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/math/Helpers.hpp>

#include <dart/common/Uri.hpp>

#include <Eigen/Core>
#include <imgui.h>
#include <osg/GraphicsContext>
#include <osg/Vec3>
#include <sdf/Root.hh>
#include <sdf/sdf.hh>

#include <algorithm>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using dart::common::Uri;
using dart::dynamics::BodyNode;
using dart::dynamics::Joint;
using dart::dynamics::MimicConstraintType;
using dart::dynamics::SkeletonPtr;
using dart::gui::osg::ImGuiViewer;
using dart::gui::osg::RealTimeWorldNode;
using dart::simulation::WorldPtr;
using dart::utils::SdfParser::detail::ElementEnumerator;
using dart::utils::SdfParser::detail::getAttributeString;
using dart::utils::SdfParser::detail::getElement;
using dart::utils::SdfParser::detail::getValueDouble;
using dart::utils::SdfParser::detail::hasAttribute;
using dart::utils::SdfParser::detail::hasElement;

namespace {

struct MimicSpec
{
  std::string model;
  std::string followerJoint;
  std::string referenceJoint;
  std::size_t referenceDof = 0;
  double multiplier = 1.0;
  double offset = 0.0;
};

struct MimicPairView
{
  std::string label;
  Joint* follower{};
  Joint* reference{};
  BodyNode* base{};
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
  if (bn == nullptr)
    return Eigen::Vector3d::Zero();
  return bn->getWorldTransform().translation();
}

std::string formatErrors(const sdf::Errors& errors)
{
  std::stringstream ss;
  for (const auto& err : errors)
    ss << err.Message() << "\n";
  return ss.str();
}

std::string readSdfText(
    const dart::common::Uri& uri,
    const std::shared_ptr<dart::utils::DartResourceRetriever>& retriever)
{
  auto resource = retriever->retrieve(uri);
  if (!resource) {
    std::cerr << "Failed to retrieve SDF: " << uri.toString() << "\n";
    return {};
  }

  std::string text(resource->getSize(), '\0');
  const auto read = resource->read(text.data(), 1, text.size());
  if (read != resource->getSize()) {
    std::cerr << "Failed to read SDF bytes for " << uri.toString() << "\n";
    return {};
  }

  return text;
}

struct SolverConfig
{
  bool useOdeCollision = true;
  bool usePgsSolver = false;
};

void applyCollisionDetector(
    const SolverConfig& cfg, const dart::simulation::WorldPtr& world)
{
#if HAVE_ODE
  if (cfg.useOdeCollision) {
    world->getConstraintSolver()->setCollisionDetector(
        dart::collision::OdeCollisionDetector::create());
    return;
  }
#endif

#if HAVE_BULLET
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
  auto* boxedSolver = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(
      world->getConstraintSolver());
  if (!boxedSolver)
    return;

  if (cfg.usePgsSolver) {
    boxedSolver->setBoxedLcpSolver(
        std::make_shared<dart::constraint::PgsBoxedLcpSolver>());
    boxedSolver->setSecondaryBoxedLcpSolver(nullptr);
  } else {
    boxedSolver->setBoxedLcpSolver(
        std::make_shared<dart::constraint::DantzigBoxedLcpSolver>());
    boxedSolver->setSecondaryBoxedLcpSolver(
        std::make_shared<dart::constraint::PgsBoxedLcpSolver>());
  }
}

std::vector<MimicSpec> parseMimicSpecs(const std::string& sdfText)
{
  sdf::Root root;
  const auto errors = root.LoadSdfString(sdfText);
  if (!errors.empty()) {
    std::cerr << "Failed to parse provided SDF text:\n" << formatErrors(errors);
    return {};
  }

  const auto rootElement = root.Element();
  if (!rootElement)
    return {};

  const auto worldElement = getElement(rootElement, "world");
  if (!worldElement)
    return {};

  std::vector<MimicSpec> specs;
  ElementEnumerator modelEnum(worldElement, "model");
  while (modelEnum.next()) {
    const auto modelElement = modelEnum.get();
    if (!modelElement)
      continue;

    const auto modelName = getAttributeString(modelElement, "name");
    ElementEnumerator jointEnum(modelElement, "joint");
    while (jointEnum.next()) {
      const auto jointElement = jointEnum.get();
      if (!jointElement || !hasElement(jointElement, "axis"))
        continue;

      const auto axisElement = getElement(jointElement, "axis");
      if (!hasElement(axisElement, "mimic"))
        continue;

      const auto mimicElement = getElement(axisElement, "mimic");
      if (!mimicElement)
        continue;

      MimicSpec spec;
      spec.model = modelName;
      spec.followerJoint = getAttributeString(jointElement, "name");
      spec.referenceJoint = getAttributeString(mimicElement, "joint");
      const auto axisAttribute = hasAttribute(mimicElement, "axis")
                                     ? getAttributeString(mimicElement, "axis")
                                     : std::string();
      spec.referenceDof = axisAttribute == "axis2" ? 1u : 0u;
      spec.multiplier = hasElement(mimicElement, "multiplier")
                            ? getValueDouble(mimicElement, "multiplier")
                            : 1.0;
      spec.offset = hasElement(mimicElement, "offset")
                        ? getValueDouble(mimicElement, "offset")
                        : 0.0;

      specs.push_back(spec);
    }
  }

  return specs;
}

void configureMimicMotors(
    const std::vector<MimicSpec>& specs, const WorldPtr& world)
{
  // Get the middle pendulum (uncoupled) which contains the true reference
  // joints
  const auto middlePendulum = world->getSkeleton("pendulum_with_base");

  for (const auto& spec : specs) {
    const auto skeleton = world->getSkeleton(spec.model);
    if (!skeleton) {
      std::cerr << "Skipping missing model [" << spec.model << "]\n";
      continue;
    }

    auto* follower = skeleton->getJoint(spec.followerJoint);
    if (!follower) {
      std::cerr << "Missing follower joint [" << spec.followerJoint
                << "] in model [" << spec.model << "]\n";
      continue;
    }

    // Get reference from middle pendulum (not from the same skeleton)
    Joint* reference = nullptr;
    if (middlePendulum) {
      reference = middlePendulum->getJoint(spec.referenceJoint);
    }

    if (!reference) {
      std::cerr << "Missing reference joint [" << spec.referenceJoint
                << "] in middle pendulum\n";
      continue;
    }

    if (follower->getNumDofs() == 0 || reference->getNumDofs() == 0) {
      std::cerr << "Ignoring mimic joint with no DoFs\n";
      continue;
    }

    std::vector<dart::dynamics::MimicDofProperties> mimicProps
        = follower->getMimicDofProperties();
    mimicProps.resize(follower->getNumDofs());

    const std::size_t followerIndex
        = std::min(spec.referenceDof, follower->getNumDofs() - 1);
    const std::size_t referenceIndex
        = std::min(spec.referenceDof, reference->getNumDofs() - 1);

    auto& prop = mimicProps[followerIndex];
    prop.mReferenceJoint = reference;
    prop.mReferenceDofIndex = referenceIndex;
    prop.mMultiplier = spec.multiplier;
    prop.mOffset = spec.offset;
    prop.mConstraintType = MimicConstraintType::Motor;

    follower->setMimicJointDofs(mimicProps);
    follower->setActuatorType(Joint::MIMIC);
    follower->setUseCouplerConstraint(false);
  }
}

std::vector<MimicPairView> collectMimicPairs(
    const WorldPtr& world, const std::vector<MimicSpec>& specs)
{
  std::vector<MimicPairView> pairs;

  // Get the middle pendulum (uncoupled) which contains the true reference
  // joints
  const auto middlePendulum = world->getSkeleton("pendulum_with_base");

  for (const auto& spec : specs) {
    const auto skeleton = world->getSkeleton(spec.model);
    if (!skeleton)
      continue;

    auto* reference = middlePendulum
                          ? middlePendulum->getJoint(spec.referenceJoint)
                          : nullptr;
    auto* follower = skeleton->getJoint(spec.followerJoint);
    auto* base = skeleton->getBodyNode("base");
    if (!follower || !reference || !base)
      continue;

    MimicPairView view;
    view.label = spec.model + ": " + spec.followerJoint + " -> middle "
                 + spec.referenceJoint;
    view.follower = follower;
    view.reference = reference;
    view.base = base;
    view.baseStart = translationOf(base);

    pairs.push_back(view);
  }

  return pairs;
}

void tintBases(const WorldPtr& world)
{
  for (const auto& entry : getPalette()) {
    const auto skeleton = world->getSkeleton(entry.model);
    if (!skeleton)
      continue;

    auto* base = skeleton->getBodyNode("base");
    if (!base)
      continue;

    base->setColor(entry.color);
  }
}

class MimicOverlay : public dart::gui::osg::ImGuiWidget
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
    for (auto& pair : mPairs)
      pair.baseStart = translationOf(pair.base);
  }

  void renderSimControls()
  {
    bool simulating = mViewer->isSimulating();
    if (ImGui::Checkbox("Run simulation", &simulating))
      mViewer->simulate(simulating);
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
#if HAVE_ODE
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

#if HAVE_ODE
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
    if (mPairs.empty())
      return;

    ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                            | ImGuiTableFlags_SizingStretchProp
                            | ImGuiTableFlags_Resizable;
    if (ImGui::BeginTable("mimic_table", 6, flags)) {
      ImGui::TableSetupColumn("Pair");
      ImGui::TableSetupColumn("Middle Ref (rad)");
      ImGui::TableSetupColumn("Follower (rad)");
      ImGui::TableSetupColumn("Error (rad)");
      ImGui::TableSetupColumn("Velocity error (rad/s)");
      ImGui::TableSetupColumn("Base drift (m)");
      ImGui::TableHeadersRow();

      for (auto& pair : mPairs) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(pair.label.c_str());

        const double middleRefPos = pair.reference->getPosition(0);
        const double middleRefVel = pair.reference->getVelocity(0);
        const double follower = pair.follower->getPosition(0);
        const double followerVel = pair.follower->getVelocity(0);
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
    for (auto& pair : mPairs)
      pair.baseNow = translationOf(pair.base);
  }

  ImGuiViewer* mViewer;
  WorldPtr mWorld;
  std::vector<MimicPairView> mPairs;
  std::string mWorldPath;
  SolverConfig mConfig;
};

} // namespace

//==============================================================================
int main(int /*argc*/, char*[] /*argv*/)
{
  const std::string worldUri
      = "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf";

  auto retriever = std::make_shared<dart::utils::DartResourceRetriever>();
  dart::utils::SdfParser::Options options(retriever);

  const auto world = dart::utils::SdfParser::readWorld(Uri(worldUri), options);
  if (!world) {
    std::cerr << "Failed to load world from " << worldUri << "\n";
    return EXIT_FAILURE;
  }

  const std::string sdfText = readSdfText(Uri(worldUri), retriever);
  if (sdfText.empty())
    return EXIT_FAILURE;

  const auto mimicSpecs = parseMimicSpecs(sdfText);
  if (mimicSpecs.empty())
    std::cerr << "No mimic joints found in " << worldUri << "\n";

  configureMimicMotors(mimicSpecs, world);
  tintBases(world);

  auto mimicPairs = collectMimicPairs(world, mimicSpecs);

  auto* wsi = osg::GraphicsContext::getWindowingSystemInterface();
  if (wsi == nullptr) {
    std::cerr << "No OSG windowing system detected. "
              << "A valid display server is required to run this example.\n";
    return EXIT_FAILURE;
  }

  osg::ref_ptr<RealTimeWorldNode> worldNode = new RealTimeWorldNode(world);
  osg::ref_ptr<ImGuiViewer> viewer = new ImGuiViewer();
  viewer->addWorldNode(worldNode);
  viewer->addInstructionText("space: toggle simulation\n");

  auto grid = ::osg::ref_ptr<dart::gui::osg::GridVisual>(
      new dart::gui::osg::GridVisual());
  grid->setPlaneType(dart::gui::osg::GridVisual::PlaneType::XY);
  grid->setNumCells(20);
  viewer->addAttachment(grid);

  viewer->setUpViewInWindow(0, 0, 1280, 720);
  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(8.0f, -7.0f, 4.0f),
      ::osg::Vec3(0.5f, 0.0f, 1.5f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  viewer->simulate(true);
  viewer->getImGuiHandler()->addWidget(std::make_shared<MimicOverlay>(
      viewer.get(), world, std::move(mimicPairs), worldUri, SolverConfig{}));

  if (!viewer->isRealized())
    viewer->realize();

  osg::ref_ptr<osg::GraphicsContext> gc
      = viewer->getCamera() ? viewer->getCamera()->getGraphicsContext()
                            : nullptr;
  if (!viewer->isRealized() || !gc || !gc->valid()) {
    std::cerr << "Failed to create an OSG window. Ensure DISPLAY is set or "
              << "use a virtual framebuffer.\n";
    return EXIT_FAILURE;
  }

  const int runResult = viewer->run();
  if (runResult != 0)
    std::cerr << "Viewer exited early (status " << runResult << ")\n";

  return runResult;
}
