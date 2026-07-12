/*
 * Copyright (c) 2011-2026, The DART development contributors
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

#include <dart/gui/osg/OffscreenViewer.hpp>
#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::BoxShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::Frame;
using dart::dynamics::FreeJoint;
using dart::dynamics::SimpleFrame;
using dart::dynamics::SimpleFramePtr;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::SoftBodyNode;
using dart::dynamics::SoftBodyNodeHelper;
using dart::dynamics::SphereShape;
using dart::dynamics::VisualAspect;
using dart::dynamics::WeldJoint;
using dart::simulation::World;
using dart::simulation::WorldPtr;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTimeStep = 0.001;
constexpr double kSoftBodyRadius = 0.36;
constexpr double kCompareTolerance = 0.25;

struct Options
{
  double guiScale = 1.0;
  bool showHelp = false;
  bool valid = true;
  bool headless = false;
  bool adaptive = true;
  bool compare = false;
  bool showWidget = true;
  std::size_t steps = 2000;
  std::size_t checkpoint = 250;
  std::size_t ringCount = 1;
  std::size_t lingerSteps = 12;
  int width = 960;
  int height = 720;
  std::string shotPath;
  double compareTolerance = kCompareTolerance;
};

struct Scene
{
  WorldPtr world;
  SkeletonPtr softSkeleton;
  SkeletonPtr pusherSkeleton;
  SoftBodyNode* softBody = nullptr;
  FreeJoint* pusherJoint = nullptr;
  std::vector<SimpleFramePtr> pointMarkers;
};

struct Checksum
{
  double skeletonPositionL1 = 0.0;
  double skeletonPositionSquared = 0.0;
  double pointWorldPositionL1 = 0.0;
  double pointWorldPositionSquared = 0.0;
  std::size_t active = 0;
  std::size_t total = 0;
  std::size_t contacts = 0;
  bool finite = true;
};

bool parseSize(const char* text, std::size_t& value)
{
  if (!text || text[0] == '\0')
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
  const auto parsed = std::strtoull(text, &end, 10);
  if (end == text || *end != '\0' || errno == ERANGE
      || parsed > std::numeric_limits<std::size_t>::max()) {
    return false;
  }

  value = static_cast<std::size_t>(parsed);
  return true;
}

bool parsePositiveInt(const char* text, int& value)
{
  if (!text || text[0] == '\0')
    return false;

  errno = 0;
  char* end = nullptr;
  const long parsed = std::strtol(text, &end, 10);
  if (end == text || *end != '\0' || errno == ERANGE || parsed <= 0
      || parsed > std::numeric_limits<int>::max()) {
    return false;
  }

  value = static_cast<int>(parsed);
  return true;
}

bool parseNonnegativeDouble(const char* text, double& value)
{
  if (!text || text[0] == '\0')
    return false;

  errno = 0;
  char* end = nullptr;
  const double parsed = std::strtod(text, &end);
  if (end == text || *end != '\0' || errno == ERANGE || !std::isfinite(parsed)
      || parsed < 0.0) {
    return false;
  }

  value = parsed;
  return true;
}

void printUsage(const char* executable)
{
  dart::gui::osg::printGuiScaleUsage(std::cout, executable);
  std::cout
      << "\nAdaptive soft-contact options:\n"
      << "  --adaptive       Enable adaptive contact activation (default).\n"
      << "  --all-active     Disable activation and simulate every point "
         "mass.\n"
      << "  --compare        Run adaptive and all-active worlds together.\n"
      << "  --headless       Run without constructing a viewer or requiring "
         "DISPLAY.\n"
      << "  --steps N        Number of simulation steps (default 2000).\n"
      << "  --checkpoint N   Deterministic report interval (default 250).\n"
      << "  --shot PATH      Capture the final state off-screen (requires "
         "DISPLAY).\n"
      << "  --width W        Capture/window width before GUI scaling.\n"
      << "  --height H       Capture/window height before GUI scaling.\n"
      << "  --ring-count N   Contact-neighborhood rings (default 1).\n"
      << "  --linger N       Out-of-contact linger steps (default 12).\n"
      << "  --compare-tolerance X  Maximum surface-pose delta (default "
         "0.25).\n"
      << "  --hide-widget    Hide the on-screen controls and statistics.\n\n"
      << "GUI control: A toggles adaptive activation.\n";
}

Options parseOptions(int argc, char* argv[])
{
  Options options;
  std::vector<char*> forwarded{argv[0]};

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--adaptive") == 0) {
      options.adaptive = true;
    } else if (std::strcmp(argv[i], "--all-active") == 0) {
      options.adaptive = false;
    } else if (std::strcmp(argv[i], "--compare") == 0) {
      options.compare = true;
      options.headless = true;
    } else if (std::strcmp(argv[i], "--headless") == 0) {
      options.headless = true;
    } else if (std::strcmp(argv[i], "--steps") == 0) {
      if (i + 1 >= argc || !parseSize(argv[++i], options.steps)) {
        std::cerr << "--steps requires a nonnegative integer\n";
        options.valid = false;
        break;
      }
    } else if (std::strcmp(argv[i], "--checkpoint") == 0) {
      if (i + 1 >= argc || !parseSize(argv[++i], options.checkpoint)) {
        std::cerr << "--checkpoint requires a nonnegative integer\n";
        options.valid = false;
        break;
      }
    } else if (std::strcmp(argv[i], "--shot") == 0) {
      if (i + 1 >= argc) {
        std::cerr << "--shot requires a path\n";
        options.valid = false;
        break;
      }
      options.shotPath = argv[++i];
      options.headless = true;
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
    } else if (std::strcmp(argv[i], "--ring-count") == 0) {
      if (i + 1 >= argc || !parseSize(argv[++i], options.ringCount)) {
        std::cerr << "--ring-count requires a nonnegative integer\n";
        options.valid = false;
        break;
      }
    } else if (std::strcmp(argv[i], "--linger") == 0) {
      if (i + 1 >= argc || !parseSize(argv[++i], options.lingerSteps)) {
        std::cerr << "--linger requires a nonnegative integer\n";
        options.valid = false;
        break;
      }
    } else if (std::strcmp(argv[i], "--compare-tolerance") == 0) {
      if (i + 1 >= argc
          || !parseNonnegativeDouble(argv[++i], options.compareTolerance)) {
        std::cerr
            << "--compare-tolerance requires a nonnegative finite number\n";
        options.valid = false;
        break;
      }
    } else if (std::strcmp(argv[i], "--hide-widget") == 0) {
      options.showWidget = false;
    } else {
      forwarded.push_back(argv[i]);
    }
  }

  const auto guiOptions = dart::gui::osg::parseGuiScaleOptions(
      static_cast<int>(forwarded.size()), forwarded.data(), &std::cerr);
  options.guiScale = guiOptions.scale;
  options.showHelp = guiOptions.showHelp;
  return options;
}

SkeletonPtr createGround()
{
  auto ground = Skeleton::create("ground");
  auto* body = ground->createJointAndBodyNodePair<WeldJoint>().second;
  auto* shape = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(6.0, 4.0, 0.1)));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = -0.05;
  shape->setRelativeTransform(transform);
  shape->getVisualAspect()->setColor(Eigen::Vector3d(0.62, 0.65, 0.69));
  shape->getDynamicsAspect()->setFrictionCoeff(1.0);
  return ground;
}

SkeletonPtr createSoftEllipsoid(const Options& options, SoftBodyNode*& softBody)
{
  auto skeleton = Skeleton::create("adaptive_ellipsoid");
  FreeJoint::Properties jointProperties;
  jointProperties.mName = "adaptive_ellipsoid_joint";

  BodyNode::Properties bodyProperties(
      BodyNode::AspectProperties("adaptive_ellipsoid_body"));
  const auto ellipsoidShape = std::make_shared<dart::dynamics::EllipsoidShape>(
      Eigen::Vector3d(0.72, 0.54, 0.60));
  bodyProperties.mInertia = dart::dynamics::Inertia(
      0.8, Eigen::Vector3d::Zero(), ellipsoidShape->computeInertia(0.8));

  const auto uniqueProperties = SoftBodyNodeHelper::makeEllipsoidProperties(
      Eigen::Vector3d(0.72, 0.54, 0.60), 12, 8, 1.6, 4000.0, 400.0, 500.0);
  const SoftBodyNode::Properties properties(bodyProperties, uniqueProperties);

  const auto pair
      = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
          nullptr, jointProperties, properties);
  softBody = pair.second;
  softBody->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(1.1);
  softBody->setAdaptiveContactActivationRingCount(options.ringCount);
  softBody->setAdaptiveContactActivationLingerSteps(options.lingerSteps);
  softBody->setAdaptiveContactActivationVelocityTolerance(0.035);
  softBody->setAdaptiveContactActivationPositionTolerance(0.018);
  softBody->setAdaptiveContactActivationEnabled(options.adaptive);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, 1.10);
  pair.first->setPositions(FreeJoint::convertToPositions(transform));

  auto* visual = softBody->getShapeNode(0)->getVisualAspect();
  visual->setColor(Eigen::Vector4d(0.20, 0.48, 0.95, 0.58));
  return skeleton;
}

SkeletonPtr createPusher(FreeJoint*& pusherJoint)
{
  auto skeleton = Skeleton::create("periodic_pusher");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pusherJoint = pair.first;
  auto* body = pair.second;
  const auto shape
      = std::make_shared<BoxShape>(Eigen::Vector3d(0.20, 0.70, 0.34));
  auto* shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  body->setInertia(dart::dynamics::Inertia(
      20.0, Eigen::Vector3d::Zero(), shape->computeInertia(20.0)));
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.96, 0.48, 0.16));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.8);
  skeleton->setMobile(false);
  return skeleton;
}

void setPusherPose(Scene& scene, double time)
{
  // A smooth 1.4-second in/out sweep repeatedly makes and breaks side contact.
  const double phase = 2.0 * kPi * time / 1.4;
  const double x = -1.10 + 0.325 * (1.0 - std::cos(phase));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(x, 0.0, 0.37);
  scene.pusherJoint->setPositions(FreeJoint::convertToPositions(transform));
}

void createPointMarkers(Scene& scene)
{
  const std::size_t count = scene.softBody->getNumPointMasses();
  scene.pointMarkers.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    auto marker = std::make_shared<SimpleFrame>(
        Frame::World(), "activation_marker_" + std::to_string(i));
    marker->setShape(std::make_shared<SphereShape>(0.018));
    marker->createVisualAspect();
    scene.world->addSimpleFrame(marker);
    scene.pointMarkers.push_back(marker);
  }
}

std::shared_ptr<Scene> createScene(const Options& options, bool withMarkers)
{
  auto scene = std::make_shared<Scene>();
  scene->world = World::create("adaptive_soft_contact");
  scene->world->setTimeStep(kTimeStep);
  scene->world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  scene->softSkeleton = createSoftEllipsoid(options, scene->softBody);
  scene->pusherSkeleton = createPusher(scene->pusherJoint);
  scene->world->addSkeleton(createGround());
  scene->world->addSkeleton(scene->softSkeleton);
  scene->world->addSkeleton(scene->pusherSkeleton);
  setPusherPose(*scene, 0.0);
  if (withMarkers)
    createPointMarkers(*scene);
  return scene;
}

Checksum computeChecksum(const Scene& scene)
{
  Checksum result;
  const Eigen::VectorXd positions = scene.softSkeleton->getPositions();
  const Eigen::VectorXd velocities = scene.softSkeleton->getVelocities();
  result.skeletonPositionL1 = positions.cwiseAbs().sum();
  result.skeletonPositionSquared = positions.squaredNorm();
  result.finite = positions.allFinite() && velocities.allFinite();
  result.active = scene.softBody->getNumActivePointMasses();
  result.total = scene.softBody->getNumPointMasses();
  result.contacts = scene.world->getLastCollisionResult().getNumContacts();

  for (std::size_t i = 0; i < result.total; ++i) {
    const auto* point = scene.softBody->getPointMass(i);
    const Eigen::Vector3d worldPosition = point->getWorldPosition();
    result.pointWorldPositionL1 += worldPosition.cwiseAbs().sum();
    result.pointWorldPositionSquared += worldPosition.squaredNorm();
    result.finite = result.finite && worldPosition.allFinite()
                    && point->getPositions().allFinite()
                    && point->getVelocities().allFinite();
  }
  return result;
}

void printCheckpoint(std::size_t step, const Scene& scene)
{
  const Checksum checksum = computeChecksum(scene);
  std::printf(
      "step %6zu  mode %10s  active %4zu  inactive %4zu  contacts %3zu  "
      "skelPosL1 %.17g  skelPosSq %.17g  pointWorldPosL1 %.17g  "
      "pointWorldPosSq %.17g  finite %s\n",
      step,
      scene.softBody->isAdaptiveContactActivationEnabled() ? "adaptive"
                                                           : "all-active",
      checksum.active,
      checksum.total - checksum.active,
      checksum.contacts,
      checksum.skeletonPositionL1,
      checksum.skeletonPositionSquared,
      checksum.pointWorldPositionL1,
      checksum.pointWorldPositionSquared,
      checksum.finite ? "yes" : "no");
}

void updatePointMarkers(Scene& scene)
{
  if (scene.pointMarkers.empty())
    return;

  std::vector<Eigen::Vector3d> contactPoints;
  for (const auto& contact :
       scene.world->getLastCollisionResult().getContacts()) {
    if (contact.point.allFinite())
      contactPoints.push_back(contact.point);
  }

  std::vector<std::pair<double, std::size_t>> ranked;
  ranked.reserve(scene.pointMarkers.size());
  for (std::size_t i = 0; i < scene.pointMarkers.size(); ++i) {
    const Eigen::Vector3d position
        = scene.softBody->getPointMass(i)->getWorldPosition();
    scene.pointMarkers[i]->setRelativeTranslation(position);
    double distance = contactPoints.empty()
                          ? position.z()
                          : std::numeric_limits<double>::infinity();
    for (const auto& contactPoint : contactPoints)
      distance = std::min(distance, (position - contactPoint).squaredNorm());
    ranked.emplace_back(distance, i);
  }
  std::stable_sort(ranked.begin(), ranked.end());

  std::vector<bool> highlighted(scene.pointMarkers.size(), false);
  const std::size_t active = std::min(
      scene.softBody->getNumActivePointMasses(), scene.pointMarkers.size());
  for (std::size_t i = 0; i < active; ++i)
    highlighted[ranked[i].second] = true;

  for (std::size_t i = 0; i < scene.pointMarkers.size(); ++i) {
    const Eigen::Vector4d color = highlighted[i]
                                      ? Eigen::Vector4d(1.0, 0.78, 0.08, 1.0)
                                      : Eigen::Vector4d(0.08, 0.18, 0.48, 0.45);
    scene.pointMarkers[i]->getVisualAspect(true)->setColor(color);
  }
}

bool stepScene(Scene& scene)
{
  setPusherPose(scene, scene.world->getTime());
  scene.world->step();
  return computeChecksum(scene).finite;
}

class AdaptiveWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  explicit AdaptiveWorldNode(std::shared_ptr<Scene> scene)
    : dart::gui::osg::RealTimeWorldNode(scene->world), mScene(std::move(scene))
  {
    updatePointMarkers(*mScene);
  }

  double getLastStepMilliseconds() const
  {
    return mLastStepMilliseconds;
  }

protected:
  void customPreStep() override
  {
    setPusherPose(*mScene, mWorld->getTime());
    mStepStart = std::chrono::steady_clock::now();
  }

  void customPostStep() override
  {
    mLastStepMilliseconds = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - mStepStart)
                                .count();
    updatePointMarkers(*mScene);
  }

private:
  std::shared_ptr<Scene> mScene;
  std::chrono::steady_clock::time_point mStepStart;
  double mLastStepMilliseconds = 0.0;
};

void setAdaptiveEnabled(Scene& scene, bool enabled)
{
  scene.softBody->setAdaptiveContactActivationEnabled(enabled);
  updatePointMarkers(scene);
  std::cout << "Adaptive contact activation: "
            << (enabled ? "ON" : "OFF (all active)") << "\n";
}

class AdaptiveEventHandler : public osgGA::GUIEventHandler
{
public:
  explicit AdaptiveEventHandler(std::shared_ptr<Scene> scene)
    : mScene(std::move(scene))
  {
  }

  bool handle(
      const osgGA::GUIEventAdapter& event, osgGA::GUIActionAdapter&) override
  {
    if (event.getEventType() != osgGA::GUIEventAdapter::KEYDOWN
        || (event.getKey() != 'a' && event.getKey() != 'A')) {
      return false;
    }

    setAdaptiveEnabled(
        *mScene, !mScene->softBody->isAdaptiveContactActivationEnabled());
    return true;
  }

private:
  std::shared_ptr<Scene> mScene;
};

class AdaptiveStatsWidget : public dart::gui::osg::ImGuiWidget
{
public:
  AdaptiveStatsWidget(
      dart::gui::osg::ImGuiViewer* viewer,
      AdaptiveWorldNode* node,
      std::shared_ptr<Scene> scene)
    : mViewer(viewer), mNode(node), mScene(std::move(scene))
  {
  }

  void render() override
  {
    const float scale
        = static_cast<float>(mViewer->getImGuiHandler()->getGuiScale());
    ImGui::SetNextWindowPos(
        ImVec2(12.0f * scale, 12.0f * scale), ImGuiCond_Once);
    ImGui::SetNextWindowSize(
        ImVec2(360.0f * scale, 230.0f * scale), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.92f);
    if (!ImGui::Begin(
            "Adaptive Soft Contact",
            nullptr,
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings)) {
      ImGui::End();
      return;
    }

    bool simulate = mViewer->isSimulating();
    if (ImGui::Checkbox("Run simulation", &simulate))
      mViewer->simulate(simulate);

    bool adaptive = mScene->softBody->isAdaptiveContactActivationEnabled();
    if (ImGui::Checkbox("Adaptive activation (A)", &adaptive))
      setAdaptiveEnabled(*mScene, adaptive);

    const std::size_t active = mScene->softBody->getNumActivePointMasses();
    const std::size_t total = mScene->softBody->getNumPointMasses();
    ImGui::Separator();
    ImGui::Text("Active point masses:   %zu", active);
    ImGui::Text("Inactive point masses: %zu", total - active);
    ImGui::Text(
        "Contacts:              %zu",
        mScene->world->getLastCollisionResult().getNumContacts());
    ImGui::Text(
        "Last step:             %.3f ms", mNode->getLastStepMilliseconds());
    ImGui::Text(
        "Simulation step:       %zu",
        static_cast<std::size_t>(mScene->world->getSimFrames()));
    ImGui::Separator();
    ImGui::TextWrapped(
        "Gold markers are a contact-nearest region estimate sized to the exact "
        "public active count; blue markers are the remainder. The moving "
        "orange pusher makes and breaks side contact.");
    ImGui::End();
  }

private:
  dart::gui::osg::ImGuiViewer* mViewer;
  AdaptiveWorldNode* mNode;
  std::shared_ptr<Scene> mScene;
};

void applyCamera(dart::gui::osg::Viewer& viewer)
{
  const ::osg::Vec3 eye(3.1f, 2.4f, 2.0f);
  const ::osg::Vec3 center(0.0f, 0.0f, 0.42f);
  const ::osg::Vec3 up(0.0f, 0.0f, 1.0f);
  viewer.setUpwardsDirection(up);
  viewer.getCameraManipulator()->setHomePosition(eye, center, up);
  viewer.setCameraManipulator(viewer.getCameraManipulator());
  viewer.getCamera()->setViewMatrixAsLookAt(eye, center, up);
}

bool captureScene(
    const Options& options,
    const std::shared_ptr<Scene>& scene,
    osg::ref_ptr<AdaptiveWorldNode> node)
{
  std::cout << "Activation markers are a contact-nearest estimate sized to the "
               "exact public active count.\n";
  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
      = new dart::gui::osg::ImGuiViewer();
  viewer->addWorldNode(node);
  viewer->simulate(false);
  viewer->getImGuiHandler()->setGuiScale(options.guiScale);
  if (options.showWidget) {
    viewer->getImGuiHandler()->addWidget(
        std::make_shared<AdaptiveStatsWidget>(viewer.get(), node.get(), scene));
  }

  dart::gui::osg::OffscreenSetup setup;
  setup.width = dart::gui::osg::scaleWindowExtent(
      std::max(1, options.width), options.guiScale);
  setup.height = dart::gui::osg::scaleWindowExtent(
      std::max(1, options.height), options.guiScale);
  const bool captured = dart::gui::osg::captureOffscreen(
      *viewer,
      options.shotPath,
      ::osg::Vec3(3.1f, 2.4f, 2.0f),
      ::osg::Vec3(0.0f, 0.0f, 0.42f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f),
      setup,
      4);
  if (captured)
    std::cout << "[headless] wrote " << options.shotPath << "\n";
  return captured;
}

int runHeadless(const Options& options)
{
  auto scene = createScene(options, !options.shotPath.empty());
  const auto start = std::chrono::steady_clock::now();
  for (std::size_t step = 1; step <= options.steps; ++step) {
    if (!stepScene(*scene)) {
      std::cerr << "Non-finite state at step " << step << "\n";
      return 2;
    }
    if ((options.checkpoint != 0 && step % options.checkpoint == 0)
        || step == options.steps) {
      printCheckpoint(step, *scene);
    }
  }
  const double elapsedMilliseconds
      = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start)
            .count();
  std::printf(
      "timing elapsed_ms %.3f  steps_per_s %.1f  nondeterministic yes\n",
      elapsedMilliseconds,
      elapsedMilliseconds > 0.0
          ? 1000.0 * static_cast<double>(options.steps) / elapsedMilliseconds
          : 0.0);

  if (!options.shotPath.empty()) {
    updatePointMarkers(*scene);
    osg::ref_ptr<AdaptiveWorldNode> node = new AdaptiveWorldNode(scene);
    if (!captureScene(options, scene, node))
      return 3;
  }
  return 0;
}

int runComparison(const Options& options)
{
  Options adaptiveOptions = options;
  adaptiveOptions.adaptive = true;
  Options allActiveOptions = options;
  allActiveOptions.adaptive = false;
  auto adaptiveScene = createScene(adaptiveOptions, false);
  auto allActiveScene = createScene(allActiveOptions, false);

  double maximumCoordinateDelta = 0.0;
  double maximumTranslationDelta = 0.0;
  double maximumRotationDelta = 0.0;
  double maximumSurfacePoseDelta = 0.0;
  for (std::size_t step = 1; step <= options.steps; ++step) {
    if (!stepScene(*adaptiveScene) || !stepScene(*allActiveScene)) {
      std::cerr << "Non-finite comparison state at step " << step << "\n";
      return 2;
    }

    const Eigen::VectorXd delta
        = adaptiveScene->softSkeleton->getPositions()
          - allActiveScene->softSkeleton->getPositions();
    maximumCoordinateDelta
        = std::max(maximumCoordinateDelta, delta.cwiseAbs().maxCoeff());
    const double translationDelta
        = (adaptiveScene->softBody->getWorldTransform().translation()
           - allActiveScene->softBody->getWorldTransform().translation())
              .norm();
    maximumTranslationDelta
        = std::max(maximumTranslationDelta, translationDelta);
    const Eigen::Matrix3d relativeRotation
        = adaptiveScene->softBody->getWorldTransform().linear().transpose()
          * allActiveScene->softBody->getWorldTransform().linear();
    const double rotationDelta = Eigen::AngleAxisd(relativeRotation).angle();
    maximumRotationDelta = std::max(maximumRotationDelta, rotationDelta);
    const double surfacePoseDelta
        = translationDelta + kSoftBodyRadius * rotationDelta;
    maximumSurfacePoseDelta
        = std::max(maximumSurfacePoseDelta, surfacePoseDelta);
    if ((options.checkpoint != 0 && step % options.checkpoint == 0)
        || step == options.steps) {
      const Checksum adaptiveChecksum = computeChecksum(*adaptiveScene);
      const Checksum allActiveChecksum = computeChecksum(*allActiveScene);
      std::printf(
          "compare step %6zu  adaptive_active %4zu  adaptive_skelPosL1 %.17g  "
          "adaptive_contacts %3zu  all_active %4zu  all_skelPosL1 %.17g  "
          "all_contacts %3zu  max_translation_delta %.17g  "
          "max_rotation_delta_rad %.17g  max_surface_pose_delta %.17g  "
          "max_coordinate_delta %.17g  within_tolerance %s\n",
          step,
          adaptiveChecksum.active,
          adaptiveChecksum.skeletonPositionL1,
          adaptiveChecksum.contacts,
          allActiveChecksum.active,
          allActiveChecksum.skeletonPositionL1,
          allActiveChecksum.contacts,
          maximumTranslationDelta,
          maximumRotationDelta,
          maximumSurfacePoseDelta,
          maximumCoordinateDelta,
          maximumSurfacePoseDelta <= options.compareTolerance ? "yes" : "no");
    }
  }

  std::printf(
      "compare_result max_translation_delta %.17g max_rotation_delta_rad %.17g "
      "max_surface_pose_delta %.17g max_coordinate_delta %.17g tolerance %.17g "
      "pass %s\n",
      maximumTranslationDelta,
      maximumRotationDelta,
      maximumSurfacePoseDelta,
      maximumCoordinateDelta,
      options.compareTolerance,
      maximumSurfacePoseDelta <= options.compareTolerance ? "yes" : "no");
  return maximumSurfacePoseDelta <= options.compareTolerance ? 0 : 4;
}

int runGui(const Options& options)
{
  auto scene = createScene(options, true);
  osg::ref_ptr<AdaptiveWorldNode> node = new AdaptiveWorldNode(scene);
  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
      = new dart::gui::osg::ImGuiViewer();
  viewer->addWorldNode(node);
  viewer->addEventHandler(new AdaptiveEventHandler(scene));
  viewer->getImGuiHandler()->setGuiScale(options.guiScale);
  if (options.showWidget) {
    viewer->getImGuiHandler()->addWidget(
        std::make_shared<AdaptiveStatsWidget>(viewer.get(), node.get(), scene));
  }
  viewer->addInstructionText(
      "A: toggle adaptive/all-active soft contact\n"
      "Space: pause/resume\n");
  viewer->simulate(true);
  viewer->setUpViewInWindow(
      0,
      0,
      dart::gui::osg::scaleWindowExtent(options.width, options.guiScale),
      dart::gui::osg::scaleWindowExtent(options.height, options.guiScale));
  applyCamera(*viewer);
  std::cout << viewer->getInstructions() << "\n";
  viewer->run();
  return 0;
}

} // namespace

int main(int argc, char* argv[])
{
  const Options options = parseOptions(argc, argv);
  if (!options.valid)
    return 1;
  if (options.showHelp) {
    printUsage(argv[0]);
    return 0;
  }
  if (options.compare)
    return runComparison(options);
  if (options.headless)
    return runHeadless(options);
  return runGui(options);
}
