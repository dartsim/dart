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

#include <dart/dart.hpp>

#include <osg/GraphicsContext>
#include <osg/Viewport>
#include <osgViewer/Viewer>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::BoxShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::FreeJoint;
using dart::dynamics::Joint;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::SoftBodyNode;
using dart::dynamics::SoftBodyNodeHelper;
using dart::dynamics::SoftMeshShape;
using dart::dynamics::VisualAspect;
using dart::dynamics::WeldJoint;
using dart::simulation::World;
using dart::simulation::WorldPtr;

constexpr std::size_t kNumLinks = 5;
constexpr std::size_t kCheckpointInterval = 250;
constexpr double kTimeStep = 0.001;
constexpr double kLinkLength = 0.52;
constexpr double kLinkWidth = 0.34;
constexpr double kLinkHeight = 0.28;
constexpr double kRigidCoreMass = 0.16;
constexpr double kFleshMass = 0.54;
constexpr double kVertexStiffness = 850.0;
constexpr double kEdgeStiffness = 120.0;
constexpr double kFleshDamping = 8.0;
constexpr double kGaitAmplitude = 0.56;
constexpr double kGaitFrequencyHz = 1.35;
constexpr double kGaitPhaseOffset = 1.05;
constexpr double kTrackingGain = 7.0;
constexpr double kMaxJointSpeed = 4.0;
constexpr double kGroundFriction = 1.25;

//==============================================================================
struct Options
{
  double scale = 1.0;
  bool showHelp = false;
  bool valid = true;
  bool headless = false;
  std::string shotPath;
  std::size_t steps = 3000;
  int width = 800;
  int height = 500;
  bool showWidget = true;
};

//==============================================================================
struct WormState
{
  bool gaitEnabled = true;
  double initialRootX = 0.0;
  double lastStepMs = 0.0;
  double smoothedStepMs = 0.0;
  std::size_t measuredSteps = 0;
};

//==============================================================================
void printUsage(const char* executable)
{
  dart::gui::osg::printGuiScaleUsage(std::cout, executable);
  std::cout
      << "\nAdditional options:\n"
      << "  --headless       Step without opening a window and exit.\n"
      << "  --steps N        Simulation steps (default 3000).\n"
      << "  --shot PATH      Capture the final state off-screen to PATH; "
         "implies "
         "--headless.\n"
      << "  --width W        Capture width before --gui-scale (default 800).\n"
      << "  --height H       Capture height before --gui-scale (default 500).\n"
      << "  --hide-widget    Hide the ImGui gait and statistics panel.\n"
      << "\nGUI controls:\n"
      << "  Space            Run/pause simulation.\n"
      << "  g                Toggle the traveling-wave gait.\n";
}

//==============================================================================
bool parseNonnegativeSize(const char* text, std::size_t& value)
{
  if (!text || !text[0])
    return false;

  // strtoull silently wraps whitespace-prefixed negatives, so require the
  // first non-space character to be a digit or an explicit plus sign.
  const char* scan = text;
  while (*scan == ' ' || *scan == '\t')
    ++scan;
  if (*scan != '+' && !std::isdigit(static_cast<unsigned char>(*scan)))
    return false;

  errno = 0;
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(text, &end, 10);
  if (!end || *end != '\0' || errno == ERANGE
      || parsed > std::numeric_limits<std::size_t>::max()) {
    return false;
  }
  value = static_cast<std::size_t>(parsed);
  return true;
}

//==============================================================================
bool parsePositiveInt(const char* text, int& value)
{
  char* end = nullptr;
  const long parsed = std::strtol(text, &end, 10);
  if (!text[0] || !end || *end != '\0' || parsed <= 0
      || parsed > std::numeric_limits<int>::max()) {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

//==============================================================================
Options parseOptions(int argc, char* argv[])
{
  Options options;
  std::vector<char*> forwarded{argv[0]};

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--headless") == 0) {
      options.headless = true;
    } else if (std::strcmp(argv[i], "--shot") == 0) {
      if (i + 1 >= argc) {
        std::cerr << "--shot requires a path\n";
        options.valid = false;
        break;
      }
      options.shotPath = argv[++i];
      options.headless = true;
    } else if (std::strcmp(argv[i], "--steps") == 0) {
      if (i + 1 >= argc || !parseNonnegativeSize(argv[++i], options.steps)) {
        std::cerr << "--steps requires a nonnegative integer\n";
        options.valid = false;
        break;
      }
    } else if (std::strcmp(argv[i], "--width") == 0) {
      if (i + 1 >= argc || !parsePositiveInt(argv[++i], options.width)) {
        std::cerr << "--width requires a positive integer\n";
        options.valid = false;
        break;
      }
    } else if (std::strcmp(argv[i], "--height") == 0) {
      if (i + 1 >= argc || !parsePositiveInt(argv[++i], options.height)) {
        std::cerr << "--height requires a positive integer\n";
        options.valid = false;
        break;
      }
    } else if (std::strcmp(argv[i], "--hide-widget") == 0) {
      options.showWidget = false;
    } else {
      forwarded.push_back(argv[i]);
    }
  }

  const auto gui = dart::gui::osg::parseGuiScaleOptions(
      static_cast<int>(forwarded.size()), forwarded.data(), &std::cerr);
  options.scale = gui.scale;
  options.showHelp = gui.showHelp;
  return options;
}

//==============================================================================
void setBoxInertia(BodyNode* body, const Eigen::Vector3d& size, double mass)
{
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  const BoxShape shape{size};
  inertia.setMoment(shape.computeInertia(mass));
  body->setInertia(inertia);
}

//==============================================================================
SkeletonPtr createGround()
{
  auto ground = Skeleton::create("ground");
  auto* body = ground->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(14.0, 5.0, 0.08));
  auto* shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.33, 0.36, 0.40));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kGroundFriction);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.0);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = -0.04;
  body->getParentJoint()->setTransformFromParentBodyNode(transform);
  ground->setMobile(false);
  return ground;
}

//==============================================================================
SoftBodyNode::Properties makeSoftLinkProperties(std::size_t index)
{
  const std::string name = "soft_link_" + std::to_string(index);
  BodyNode::Properties bodyProperties{BodyNode::AspectProperties(name)};
  bodyProperties.mInertia.setMass(kRigidCoreMass);
  bodyProperties.mInertia.setMoment(
      BoxShape(Eigen::Vector3d(kLinkLength, kLinkWidth, kLinkHeight))
          .computeInertia(kRigidCoreMass));

  auto fleshProperties = SoftBodyNodeHelper::makeBoxProperties(
      Eigen::Vector3d(kLinkLength, kLinkWidth, kLinkHeight),
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3i(3, 3, 3),
      kFleshMass,
      kVertexStiffness,
      kEdgeStiffness,
      kFleshDamping);
  return SoftBodyNode::Properties(bodyProperties, fleshProperties);
}

//==============================================================================
void styleSoftLink(SoftBodyNode* body, std::size_t index)
{
  const std::array<Eigen::Vector4d, kNumLinks> colors = {
      Eigen::Vector4d(0.18, 0.66, 0.83, 0.88),
      Eigen::Vector4d(0.24, 0.77, 0.63, 0.88),
      Eigen::Vector4d(0.78, 0.82, 0.31, 0.88),
      Eigen::Vector4d(0.95, 0.62, 0.25, 0.88),
      Eigen::Vector4d(0.88, 0.35, 0.42, 0.88),
  };

  for (std::size_t i = 0; i < body->getNumShapeNodes(); ++i) {
    auto* shapeNode = body->getShapeNode(i);
    if (auto* dynamics = shapeNode->getDynamicsAspect(false)) {
      dynamics->setFrictionCoeff(kGroundFriction);
      dynamics->setRestitutionCoeff(0.0);
    }
    auto* visual = shapeNode->getVisualAspect(false);
    if (!visual)
      continue;
    const auto shape = shapeNode->getShape();
    if (shape && shape->getType() == SoftMeshShape::getStaticType())
      visual->setRGBA(colors[index % colors.size()]);
  }
}

//==============================================================================
SkeletonPtr createWorm()
{
  auto worm = Skeleton::create("soft_worm");

  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties rootJoint;
  rootJoint.mName = "root_free_joint";
  auto rootPair = worm->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
      nullptr, rootJoint, makeSoftLinkProperties(0));
  rootPair.first->setActuatorType(Joint::PASSIVE);
  setBoxInertia(
      rootPair.second,
      Eigen::Vector3d(kLinkLength, kLinkWidth, kLinkHeight),
      kRigidCoreMass);
  styleSoftLink(rootPair.second, 0);

  Eigen::Isometry3d rootTransform = Eigen::Isometry3d::Identity();
  rootTransform.translation() = Eigen::Vector3d(0.0, 0.0, 0.19);
  rootPair.first->setPositions(FreeJoint::convertToPositions(rootTransform));

  BodyNode* parent = rootPair.second;
  for (std::size_t i = 1; i < kNumLinks; ++i) {
    RevoluteJoint::Properties joint;
    joint.mName = "spine_joint_" + std::to_string(i);
    joint.mAxis = Eigen::Vector3d::UnitY();
    joint.mT_ParentBodyToJoint.translation().x() = 0.5 * kLinkLength;
    joint.mT_ChildBodyToJoint.translation().x() = -0.5 * kLinkLength;
    joint.mPositionLowerLimits[0] = -0.92;
    joint.mPositionUpperLimits[0] = 0.92;

    auto pair = worm->createJointAndBodyNodePair<RevoluteJoint, SoftBodyNode>(
        parent, joint, makeSoftLinkProperties(i));
    pair.first->setActuatorType(Joint::SERVO);
    pair.first->setLimitEnforcement(true);
    pair.first->setVelocityLowerLimit(0, -kMaxJointSpeed);
    pair.first->setVelocityUpperLimit(0, kMaxJointSpeed);
    setBoxInertia(
        pair.second,
        Eigen::Vector3d(kLinkLength, kLinkWidth, kLinkHeight),
        kRigidCoreMass);
    styleSoftLink(pair.second, i);
    parent = pair.second;
  }

  return worm;
}

//==============================================================================
WorldPtr createWorld(SkeletonPtr& worm)
{
  auto world = World::create("soft_worm_world");
  world->setTimeStep(kTimeStep);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(createGround());
  worm = createWorm();
  world->addSkeleton(worm);
  return world;
}

//==============================================================================
void applyGait(const SkeletonPtr& worm, double time, bool enabled)
{
  const double omega = 2.0 * dart::math::constantsd::pi() * kGaitFrequencyHz;
  for (std::size_t i = 1; i < kNumLinks; ++i) {
    auto* joint = worm->getJoint("spine_joint_" + std::to_string(i));
    if (!joint)
      continue;

    double command = 0.0;
    if (enabled) {
      const double phase = omega * time - kGaitPhaseOffset * (i - 1);
      const double target = kGaitAmplitude * std::sin(phase);
      const double targetVelocity = kGaitAmplitude * omega * std::cos(phase);
      command
          = targetVelocity + kTrackingGain * (target - joint->getPosition(0));
      command = std::clamp(command, -kMaxJointSpeed, kMaxJointSpeed);
    }
    joint->setCommand(0, command);
  }
}

//==============================================================================
double getRootX(const SkeletonPtr& worm)
{
  return worm->getRootBodyNode()->getTransform().translation().x();
}

//==============================================================================
double positionChecksum(const SkeletonPtr& worm)
{
  long double checksum = 0.0;
  std::size_t component = 1;
  const auto positions = worm->getPositions();
  for (Eigen::Index i = 0; i < positions.size(); ++i)
    checksum += static_cast<long double>(component++) * positions[i];

  for (std::size_t i = 0; i < worm->getNumSoftBodyNodes(); ++i) {
    const auto* softBody = worm->getSoftBodyNode(i);
    for (std::size_t j = 0; j < softBody->getNumPointMasses(); ++j) {
      const Eigen::Vector3d& position
          = softBody->getPointMass(j)->getWorldPosition();
      for (int axis = 0; axis < 3; ++axis)
        checksum += static_cast<long double>(component++) * position[axis];
    }
  }
  return static_cast<double>(checksum);
}

//==============================================================================
bool isFinite(const SkeletonPtr& worm)
{
  if (!worm->getPositions().allFinite() || !worm->getVelocities().allFinite())
    return false;

  for (std::size_t i = 0; i < worm->getNumSoftBodyNodes(); ++i) {
    const auto* softBody = worm->getSoftBodyNode(i);
    if (!softBody->getTransform().matrix().allFinite())
      return false;
    for (std::size_t j = 0; j < softBody->getNumPointMasses(); ++j) {
      const auto* point = softBody->getPointMass(j);
      if (!point->getPositions().allFinite()
          || !point->getVelocities().allFinite()
          || !point->getWorldPosition().allFinite()) {
        return false;
      }
    }
  }
  return true;
}

//==============================================================================
void printCheckpoint(
    std::size_t step, const SkeletonPtr& worm, const WormState& state)
{
  const double rootX = getRootX(worm);
  std::cout << std::fixed << std::setprecision(12) << "soft_worm step=" << step
            << " root_x=" << rootX
            << " displacement=" << rootX - state.initialRootX
            << " position_checksum=" << positionChecksum(worm)
            << " finite=" << (isFinite(worm) ? "true" : "false") << '\n';
}

//==============================================================================
class WormWorldNode final : public dart::gui::osg::RealTimeWorldNode
{
public:
  WormWorldNode(
      const WorldPtr& world, SkeletonPtr worm, std::shared_ptr<WormState> state)
    : dart::gui::osg::RealTimeWorldNode(world),
      mWorm(std::move(worm)),
      mState(std::move(state))
  {
  }

  void customPreStep() override
  {
    applyGait(mWorm, mWorld->getTime(), mState->gaitEnabled);
    mStepStart = Clock::now();
  }

  void customPostStep() override
  {
    const double stepMs
        = std::chrono::duration<double, std::milli>(Clock::now() - mStepStart)
              .count();
    mState->lastStepMs = stepMs;
    if (mState->measuredSteps == 0)
      mState->smoothedStepMs = stepMs;
    else
      mState->smoothedStepMs = 0.92 * mState->smoothedStepMs + 0.08 * stepMs;
    ++mState->measuredSteps;
  }

private:
  using Clock = std::chrono::steady_clock;
  SkeletonPtr mWorm;
  std::shared_ptr<WormState> mState;
  Clock::time_point mStepStart;
};

//==============================================================================
class WormEventHandler final : public osgGA::GUIEventHandler
{
public:
  explicit WormEventHandler(std::shared_ptr<WormState> state)
    : mState(std::move(state))
  {
  }

  bool handle(
      const osgGA::GUIEventAdapter& event, osgGA::GUIActionAdapter&) override
  {
    if (event.getEventType() == osgGA::GUIEventAdapter::KEYDOWN
        && event.getKey() == 'g') {
      mState->gaitEnabled = !mState->gaitEnabled;
      return true;
    }
    return false;
  }

private:
  std::shared_ptr<WormState> mState;
};

//==============================================================================
class WormWidget final : public dart::gui::osg::ImGuiWidget
{
public:
  WormWidget(
      dart::gui::osg::ImGuiViewer* viewer,
      WorldPtr world,
      SkeletonPtr worm,
      std::shared_ptr<WormState> state)
    : mViewer(viewer),
      mWorld(std::move(world)),
      mWorm(std::move(worm)),
      mState(std::move(state))
  {
  }

  void render() override
  {
    const float scale
        = static_cast<float>(mViewer->getImGuiHandler()->getGuiScale());
    ImGui::SetNextWindowPos(
        ImVec2(12.0f * scale, 12.0f * scale), ImGuiCond_Once);
    ImGui::SetNextWindowSize(
        ImVec2(350.0f * scale, 235.0f * scale), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.92f);
    if (!ImGui::Begin(
            "Soft Worm",
            nullptr,
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings)) {
      ImGui::End();
      return;
    }

    ImGui::Checkbox("Traveling-wave gait", &mState->gaitEnabled);
    ImGui::Text("Links %zu   driven joints %zu", kNumLinks, kNumLinks - 1);
    ImGui::Text(
        "Root x %.3f m   displacement %.3f m",
        getRootX(mWorm),
        getRootX(mWorm) - mState->initialRootX);
    ImGui::Text(
        "Step %.3f ms avg   %.3f ms last",
        mState->smoothedStepMs,
        mState->lastStepMs);
    ImGui::Text(
        "Frame %d   contacts %zu",
        mWorld->getSimFrames(),
        mWorld->getLastCollisionResult().getNumContacts());
    ImGui::Text(
        "Gait %.2f Hz   amplitude %.2f rad", kGaitFrequencyHz, kGaitAmplitude);
    ImGui::Text("Position checksum %.6e", positionChecksum(mWorm));
    ImGui::TextWrapped(
        "Each articulated link carries 3x3x3 box-shaped soft flesh. "
        "Press g to pause/resume the phase-offset sinusoidal gait.");
    ImGui::End();
  }

private:
  dart::gui::osg::ImGuiViewer* mViewer;
  WorldPtr mWorld;
  SkeletonPtr mWorm;
  std::shared_ptr<WormState> mState;
};

//==============================================================================
void applyCamera(dart::gui::osg::Viewer& viewer)
{
  const ::osg::Vec3 eye(3.6f, -4.2f, 2.4f);
  const ::osg::Vec3 center(0.8f, 0.0f, 0.25f);
  const ::osg::Vec3 up(0.0f, 0.0f, 1.0f);
  viewer.setUpwardsDirection(up);
  viewer.getCameraManipulator()->setHomePosition(eye, center, up);
  viewer.setCameraManipulator(viewer.getCameraManipulator());
  viewer.getCamera()->setViewMatrixAsLookAt(eye, center, up);
}

//==============================================================================
int runHeadless(
    const WorldPtr& world,
    const SkeletonPtr& worm,
    WormState& state,
    std::size_t steps)
{
  printCheckpoint(0, worm, state);
  for (std::size_t step = 1; step <= steps; ++step) {
    applyGait(worm, world->getTime(), state.gaitEnabled);
    world->step();
    if (!isFinite(worm)) {
      printCheckpoint(step, worm, state);
      std::cerr << "soft_worm became non-finite at step " << step << '\n';
      return 2;
    }
    if (step % kCheckpointInterval == 0 || step == steps)
      printCheckpoint(step, worm, state);
  }
  return 0;
}

//==============================================================================
int captureFinalState(
    dart::gui::osg::ImGuiViewer& viewer, const Options& options)
{
  const std::filesystem::path shotPath(options.shotPath);
  std::error_code error;
  if (std::filesystem::exists(shotPath, error)) {
    if (!std::filesystem::is_regular_file(shotPath, error)) {
      std::cerr << "Capture target is not a regular file: " << options.shotPath
                << '\n';
      return 1;
    }
    std::filesystem::remove(shotPath, error);
  }
  if (error) {
    std::cerr << "Failed to prepare capture target: " << options.shotPath
              << '\n';
    return 1;
  }

  const int width
      = dart::gui::osg::scaleWindowExtent(options.width, options.scale);
  const int height
      = dart::gui::osg::scaleWindowExtent(options.height, options.scale);
  ::osg::ref_ptr<::osg::GraphicsContext::Traits> traits
      = new ::osg::GraphicsContext::Traits;
  traits->readDISPLAY();
  traits->setUndefinedScreenDetailsToDefaultScreen();
  traits->width = width;
  traits->height = height;
  traits->red = traits->green = traits->blue = 8;
  traits->alpha = 8;
  traits->depth = 24;
  traits->windowDecoration = false;
  traits->pbuffer = true;
  traits->doubleBuffer = true;

  auto context = ::osg::GraphicsContext::createGraphicsContext(traits.get());
  if (!context) {
    std::cerr << "Failed to create an off-screen GL context (no usable "
                 "DISPLAY?).\n";
    return 1;
  }

  auto* camera = viewer.getCamera();
  camera->setGraphicsContext(context);
  camera->setViewport(new ::osg::Viewport(0, 0, width, height));
  camera->setProjectionMatrixAsPerspective(
      30.0, static_cast<double>(width) / height, 0.1, 1000.0);
  const GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
  camera->setDrawBuffer(buffer);
  camera->setReadBuffer(buffer);

  viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
  viewer.simulate(false);
  applyCamera(viewer);
  viewer.realize();
  if (!viewer.isRealized()) {
    std::cerr << "Viewer failed to realize off-screen.\n";
    return 1;
  }

  viewer.frame();
  viewer.frame();
  viewer.captureScreen(options.shotPath);
  viewer.frame();

  error.clear();
  const bool wroteFile = std::filesystem::is_regular_file(shotPath, error)
                         && std::filesystem::file_size(shotPath, error) > 0;
  if (error || !wroteFile) {
    std::cerr << "Failed to write capture: " << options.shotPath << '\n';
    return 1;
  }

  std::cout << "soft_worm shot=" << options.shotPath << " width=" << width
            << " height=" << height << '\n';
  return 0;
}

} // namespace

//==============================================================================
int main(int argc, char* argv[])
{
  const Options options = parseOptions(argc, argv);
  if (!options.valid)
    return 1;
  if (options.showHelp) {
    printUsage(argv[0]);
    return 0;
  }

  SkeletonPtr worm;
  WorldPtr world = createWorld(worm);
  auto state = std::make_shared<WormState>();
  state->initialRootX = getRootX(worm);

  if (options.headless) {
    const int result = runHeadless(world, worm, *state, options.steps);
    if (result != 0 || options.shotPath.empty())
      return result;
  }

  osg::ref_ptr<WormWorldNode> node = new WormWorldNode(world, worm, state);
  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
      = new dart::gui::osg::ImGuiViewer();
  viewer->addWorldNode(node);
  viewer->addEventHandler(new WormEventHandler(state));
  viewer->getImGuiHandler()->setGuiScale(options.scale);
  if (options.showWidget) {
    viewer->getImGuiHandler()->addWidget(
        std::make_shared<WormWidget>(viewer.get(), world, worm, state));
  }
  applyCamera(*viewer);

  if (!options.shotPath.empty())
    return captureFinalState(*viewer, options);

  viewer->setUpViewInWindow(
      0,
      0,
      dart::gui::osg::scaleWindowExtent(options.width, options.scale),
      dart::gui::osg::scaleWindowExtent(options.height, options.scale));
  viewer->simulate(true);
  std::cout << viewer->getInstructions() << '\n'
            << "Soft worm: press g to toggle the traveling-wave gait.\n";
  viewer->run();
  return 0;
}
