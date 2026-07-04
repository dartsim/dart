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

// Ported from examples/sleeping: a visual demo of automatic body deactivation
// ("sleeping") and the solver-island partition. Four separated box stacks
// each form an independent solver island; keys/panel controls let you shoot a
// sphere or drop a box onto the currently-aimed stack, toggle automatic
// deactivation, and toggle per-island coloring.
//
// examples/sleeping/main.cpp is a full standalone app (its own ImGui panel,
// CLI options, and headless capture mode) that predates and duplicates much
// of what dart-demos' host now provides generically (headless --shot
// capture, a Simulation toolbar, viewer theming). Per
// docs/dev_tasks/dart6_consolidated_demos/BRIEF-phase2.md, only the scene
// itself and its bespoke panel controls (stacks, shoot sphere, drop box,
// deactivation toggle, island coloring) are ported here; the original example
// is left in place (its removal is a later cleanup phase). Also dropped, all
// because they are host-owned chrome shared across every scene rather than
// this scene's own state: the custom clear color/headlights/key+fill
// lights/SoftShadowMap, the global configureImGuiStyle() override (would
// restyle every other scene's panel too), the SingleThreaded threading-model
// call and viewer->simulate(true) autostart, and the "Run simulation"
// checkbox (the host's Simulation toolbar already owns Play/Pause), and the
// 's' key that printed awake/asleep stats to stdout (the scene panel shows the
// same awake/asleep counts continuously, and there is no GUI console). Also
// dropped: customPreRefresh's island-recolor/aim-line-update ran every render
// frame regardless of pause state; DemoSceneSetup only exposes
// preStep/postStep (once per simulation step, only while running), so here
// they run from postStep instead -- like RigidCubesScene's documented
// omission of its customPreRefresh-based playback mode, the visuals now hold
// their last state while paused rather than continuing to refresh every
// frame.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <cmath>

namespace dart_demos {

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::BoxShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::FreeJoint;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::VisualAspect;
using dart::dynamics::WeldJoint;
using dart::simulation::WorldPtr;

constexpr double kBox = 0.3;
constexpr double kDensity = 1000.0; // kg/m^3, water-like
constexpr int kProjectilePoolSize = 1;
constexpr int kAimPoints = 32;
const double kStackX[] = {-2.0, -0.7, 0.7, 2.0};

//==============================================================================
void setShapeInertia(BodyNode* body, const dart::dynamics::ShapePtr& shape)
{
  const double mass = kDensity * shape->getVolume();
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);
}

//==============================================================================
SkeletonPtr createFloor()
{
  auto floor = Skeleton::create("floor");
  auto body = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(20.0, 20.0, 0.1));
  auto* sn = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  sn->getVisualAspect()->setColor(Eigen::Vector3d(0.40, 0.43, 0.47));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0, -0.05);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);
  floor->setMobile(false);
  return floor;
}

//==============================================================================
SkeletonPtr createBox(const std::string& name, const Eigen::Vector3d& pos)
{
  auto skel = Skeleton::create(name);
  auto* body = skel->createJointAndBodyNodePair<FreeJoint>(nullptr).second;
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(kBox));
  body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      shape);
  setShapeInertia(body, shape);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = pos;
  skel->getJoint(0)->setPositions(FreeJoint::convertToPositions(tf));
  return skel;
}

//==============================================================================
bool isShotProjectile(const SkeletonPtr& skel)
{
  return skel->getName().rfind("proj_shot", 0) == 0;
}

//==============================================================================
bool isDroppedProjectile(const SkeletonPtr& skel)
{
  return skel->getName().rfind("proj_drop", 0) == 0;
}

//==============================================================================
bool isProjectile(const SkeletonPtr& skel)
{
  return isShotProjectile(skel) || isDroppedProjectile(skel);
}

//==============================================================================
Eigen::Vector4d shotColor()
{
  return Eigen::Vector4d(1.0, 0.84, 0.08, 1.0);
}

//==============================================================================
Eigen::Vector4d droppedBoxColor()
{
  return Eigen::Vector4d(0.0, 0.96, 1.0, 1.0);
}

//==============================================================================
void setSkeletonColor(const SkeletonPtr& skel, const Eigen::Vector4d& color)
{
  for (std::size_t b = 0; b < skel->getNumBodyNodes(); ++b) {
    auto* body = skel->getBodyNode(b);
    for (std::size_t s = 0; s < body->getNumShapeNodes(); ++s) {
      auto* sn = body->getShapeNode(s);
      if (auto* visual = sn->getVisualAspect())
        visual->setRGBA(color);
    }
  }
}

//==============================================================================
Eigen::Vector4d projectileColor(const SkeletonPtr& skel, bool active)
{
  if (!active)
    return Eigen::Vector4d(0.05, 0.07, 0.10, 1.0);
  return isShotProjectile(skel) ? shotColor() : droppedBoxColor();
}

//==============================================================================
SkeletonPtr createProjectile(
    const std::string& name,
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& linearVelocity,
    bool sphere)
{
  auto skel = Skeleton::create(name);
  auto* body = skel->createJointAndBodyNodePair<FreeJoint>(nullptr).second;
  dart::dynamics::ShapePtr shape;
  if (sphere)
    shape = std::make_shared<dart::dynamics::EllipsoidShape>(
        Eigen::Vector3d::Constant(1.10));
  else
    shape = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.62));
  auto* sn = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  const Eigen::Vector4d base = sphere ? shotColor() : droppedBoxColor();
  auto* visual = sn->getVisualAspect();
  visual->setRGBA(base);
  visual->setShadowed(false);
  setShapeInertia(body, shape);

  auto* joint = static_cast<FreeJoint*>(skel->getJoint(0));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = pos;
  joint->setPositions(FreeJoint::convertToPositions(tf));

  Eigen::Vector6d v = Eigen::Vector6d::Zero();
  v.tail<3>() = linearVelocity;
  joint->setVelocities(v);
  return skel;
}

//==============================================================================
bool isProjectileInactive(const SkeletonPtr& skel)
{
  return skel->getNumBodyNodes() == 0 || !skel->getBodyNode(0)->isCollidable();
}

//==============================================================================
void setProjectileState(
    const SkeletonPtr& skel,
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& linearVelocity,
    bool active)
{
  auto* joint = static_cast<FreeJoint*>(skel->getJoint(0));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = pos;
  joint->setPositions(FreeJoint::convertToPositions(tf));

  Eigen::Vector6d v = Eigen::Vector6d::Zero();
  v.tail<3>() = linearVelocity;
  joint->setVelocities(v);

  auto* body = skel->getBodyNode(0);
  body->setCollidable(active);

  skel->setResting(!active);
  setSkeletonColor(skel, projectileColor(skel, active));
}

//==============================================================================
void resetProjectile(
    const SkeletonPtr& skel,
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& linearVelocity)
{
  setProjectileState(skel, pos, linearVelocity, /*active=*/true);
}

//==============================================================================
Eigen::Vector3d hsvToRgb(double hue, double saturation, double value)
{
  hue = std::fmod(hue, 1.0);
  if (hue < 0.0)
    hue += 1.0;

  const double h = hue * 6.0;
  const int sector = static_cast<int>(std::floor(h));
  const double f = h - sector;
  const double p = value * (1.0 - saturation);
  const double q = value * (1.0 - saturation * f);
  const double t = value * (1.0 - saturation * (1.0 - f));

  switch (sector % 6) {
    case 0:
      return Eigen::Vector3d(value, t, p);
    case 1:
      return Eigen::Vector3d(q, value, p);
    case 2:
      return Eigen::Vector3d(p, value, t);
    case 3:
      return Eigen::Vector3d(p, q, value);
    case 4:
      return Eigen::Vector3d(t, p, value);
    default:
      return Eigen::Vector3d(value, p, q);
  }
}

//==============================================================================
double clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}

//==============================================================================
Eigen::Vector4d rgba(const Eigen::Vector3d& color)
{
  return Eigen::Vector4d(
      clamp01(color[0]), clamp01(color[1]), clamp01(color[2]), 1.0);
}

//==============================================================================
Eigen::Vector4d islandColor(int idx)
{
  static const Eigen::Vector3d palette[]
      = {{0.20, 0.62, 1.00},
         {1.00, 0.48, 0.18},
         {0.16, 0.78, 0.50},
         {0.95, 0.30, 0.58},
         {1.00, 0.78, 0.18},
         {0.66, 0.42, 1.00},
         {0.00, 0.84, 0.86}};
  constexpr int n = static_cast<int>(sizeof(palette) / sizeof(palette[0]));
  if (idx < 0)
    return Eigen::Vector4d(0.74, 0.76, 0.78, 1.0);
  const Eigen::Vector3d& c = palette[idx % n];
  return rgba(c);
}

//==============================================================================
Eigen::Vector4d objectColor(const SkeletonPtr& skel, std::size_t worldIndex)
{
  if (isShotProjectile(skel))
    return shotColor();
  if (isDroppedProjectile(skel))
    return droppedBoxColor();

  const Eigen::Vector3d c
      = hsvToRgb(0.58 + 0.173 * static_cast<double>(worldIndex), 0.54, 0.92);
  return Eigen::Vector4d(c[0], c[1], c[2], 1.0);
}

//==============================================================================
Eigen::Vector4d awakeTint(const Eigen::Vector4d& base)
{
  const Eigen::Vector3d light(0.98, 0.99, 1.0);
  return rgba(0.84 * base.head<3>() + 0.18 * light);
}

//==============================================================================
Eigen::Vector4d asleepTint(const Eigen::Vector4d& base)
{
  const Eigen::Vector3d shadow(0.015, 0.020, 0.030);
  return rgba(0.72 * base.head<3>() + shadow);
}

//==============================================================================
int initialStackIndex(const SkeletonPtr& skel)
{
  const std::string name = skel->getName();
  if (name.rfind("box", 0) != 0 || name.size() <= 3)
    return -1;

  int id = 0;
  for (std::size_t i = 3; i < name.size(); ++i) {
    if (name[i] < '0' || name[i] > '9')
      return -1;
    id = 10 * id + (name[i] - '0');
  }
  return id / 4;
}

//==============================================================================
struct Launch
{
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
};

//==============================================================================
Launch sphereLaunch(int targetIdx)
{
  const double x = kStackX[((targetIdx % 4) + 4) % 4];
  return {Eigen::Vector3d(x, 3.0, 2.0), Eigen::Vector3d(0.0, -4.2, 1.7)};
}

//==============================================================================
/// Per-instance state captured by this scene's preStep/postStep/key-action
/// lambdas (one instance per factory() call, i.e. per Rebuild/Reset too).
struct SleepingState
{
  int targetIdx = 0;
  bool colorByIsland = false;
  int shotSlot = 0;
  int dropSlot = 0;
  std::unordered_map<std::string, int> lastDisplayIsland;

  dart::dynamics::SimpleFramePtr aimFrame;
  dart::dynamics::LineSegmentShapePtr aimLine;
};

//==============================================================================
void parkProjectilePool(const WorldPtr& world, const std::string& prefix)
{
  const Eigen::Vector3d parkedBase(60.0, 60.0, 10.0);
  for (int i = 0; i < kProjectilePoolSize; ++i) {
    const std::string name = prefix + std::to_string(i);
    if (const auto& skel = world->getSkeleton(name)) {
      const double yOffset = prefix == "proj_drop" ? 1.0 : 0.0;
      const Eigen::Vector3d parked
          = parkedBase + Eigen::Vector3d(i, yOffset, 0.0);
      setProjectileState(
          skel, parked, Eigen::Vector3d::Zero(), /*active=*/false);
    }
  }
}

//==============================================================================
void spawnSphere(const WorldPtr& world, SleepingState& state)
{
  parkProjectilePool(world, "proj_shot");
  parkProjectilePool(world, "proj_drop");

  const Launch L = sphereLaunch(state.targetIdx);
  const std::string name = "proj_shot" + std::to_string(state.shotSlot);
  state.shotSlot = (state.shotSlot + 1) % kProjectilePoolSize;
  if (const auto& skel = world->getSkeleton(name))
    resetProjectile(skel, L.pos, L.vel);
  ++state.targetIdx;
}

//==============================================================================
void spawnDropBox(const WorldPtr& world, SleepingState& state)
{
  parkProjectilePool(world, "proj_shot");
  parkProjectilePool(world, "proj_drop");

  const double targetX = kStackX[((state.targetIdx % 4) + 4) % 4];
  const double spawnX = std::min(targetX - 0.45, -3.6);
  const double targetTime = 0.65;
  const Eigen::Vector3d pos(spawnX, 1.2, 2.7);
  const Eigen::Vector3d vel(
      (targetX - spawnX) / targetTime, -1.2 / targetTime, 0.0);
  const std::string name = "proj_drop" + std::to_string(state.dropSlot);
  state.dropSlot = (state.dropSlot + 1) % kProjectilePoolSize;
  if (const auto& skel = world->getSkeleton(name))
    resetProjectile(skel, pos, vel);
  ++state.targetIdx;
}

//==============================================================================
void recolorIslands(const WorldPtr& world, SleepingState& state)
{
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto& skel = world->getSkeleton(i);
    if (!skel->isMobile())
      continue; // skip the static floor

    Eigen::Vector4d base = objectColor(skel, i);
    if (state.colorByIsland && !isProjectile(skel)) {
      int island = -1;
      const int solverIsland = skel->getIslandIndex();
      if (!skel->isResting() && solverIsland >= 0) {
        island = solverIsland;
        state.lastDisplayIsland[skel->getName()] = island;
      } else {
        island = initialStackIndex(skel);
        if (island < 0) {
          const auto it = state.lastDisplayIsland.find(skel->getName());
          if (it != state.lastDisplayIsland.end())
            island = it->second;
        }
      }
      if (island >= 0)
        base = islandColor(island);
    }

    const bool keepProjectileBright
        = isProjectile(skel) && !isProjectileInactive(skel);
    const Eigen::Vector4d color = (!keepProjectileBright && skel->isResting())
                                      ? asleepTint(base)
                                      : awakeTint(base);
    setSkeletonColor(skel, color);
  }
}

//==============================================================================
void createAimLine(const WorldPtr& world, SleepingState& state)
{
  auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(), "aim");
  auto line = std::make_shared<dart::dynamics::LineSegmentShape>(3.0f);
  for (int i = 0; i < kAimPoints; ++i)
    line->addVertex(Eigen::Vector3d::Zero());
  for (int i = 0; i + 1 < kAimPoints; ++i)
    line->addConnection(i, i + 1);
  line->addDataVariance(dart::dynamics::Shape::DYNAMIC_VERTICES);
  frame->setShape(line);
  frame->createVisualAspect();
  frame->getVisualAspect()->setColor(Eigen::Vector3d(1.0, 0.92, 0.20));
  world->addSimpleFrame(frame);

  state.aimFrame = frame;
  state.aimLine = line;
}

//==============================================================================
void updateAimLine(const WorldPtr& world, SleepingState& state)
{
  const Launch L = sphereLaunch(state.targetIdx);
  const Eigen::Vector3d g = world->getGravity();

  double tEnd = 1.0;
  const double a = 0.5 * g.z();
  const double b = L.vel.z();
  const double c = L.pos.z();
  if (a < 0.0) {
    const double disc = b * b - 4.0 * a * c;
    if (disc >= 0.0)
      tEnd = (-b - std::sqrt(disc)) / (2.0 * a);
  }
  if (!(tEnd > 0.0))
    tEnd = 1.0;

  for (int i = 0; i < kAimPoints; ++i) {
    const double t = tEnd * static_cast<double>(i) / (kAimPoints - 1);
    Eigen::Vector3d p = L.pos + L.vel * t + 0.5 * g * (t * t);
    if (p.z() < 0.0)
      p.z() = 0.0;
    state.aimLine->setVertex(static_cast<std::size_t>(i), p);
  }
}

//==============================================================================
void countStacks(const WorldPtr& world, int& awake, int& asleep)
{
  awake = 0;
  asleep = 0;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto& skel = world->getSkeleton(i);
    if (!skel->isMobile() || isProjectile(skel))
      continue;
    (skel->isResting() ? asleep : awake)++;
  }
}

//==============================================================================
void countProjectiles(const WorldPtr& world, int& shots, int& drops)
{
  shots = 0;
  drops = 0;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto& skel = world->getSkeleton(i);
    if (isProjectile(skel) && isProjectileInactive(skel))
      continue;
    if (isShotProjectile(skel))
      ++shots;
    else if (isDroppedProjectile(skel))
      ++drops;
  }
}

//==============================================================================
void addProjectilePool(const WorldPtr& world)
{
  const Eigen::Vector3d parkedBase(60.0, 60.0, 10.0);
  for (int i = 0; i < kProjectilePoolSize; ++i) {
    const Eigen::Vector3d parked = parkedBase + Eigen::Vector3d(i, 0.0, 0.0);

    auto shot = createProjectile(
        "proj_shot" + std::to_string(i),
        parked,
        Eigen::Vector3d::Zero(),
        /*sphere=*/true);
    setProjectileState(shot, parked, Eigen::Vector3d::Zero(), /*active=*/false);
    world->addSkeleton(shot);

    auto drop = createProjectile(
        "proj_drop" + std::to_string(i),
        parked + Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d::Zero(),
        /*sphere=*/false);
    setProjectileState(
        drop,
        parked + Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d::Zero(),
        /*active=*/false);
    world->addSkeleton(drop);
  }
}

} // namespace

//==============================================================================
DemoScene makeSleepingScene()
{
  DemoScene scene;
  scene.id = "sleeping";
  scene.title = "Sleeping";
  scene.category = "Rigid Body";
  scene.summary
      = "Automatic body deactivation across four independent box-stack "
        "solver islands.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();
    world->setTimeStep(1.0 / 60.0);
    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world->addSkeleton(createFloor());

    auto opts = world->getDeactivationOptions();
    opts.mTimeUntilSleep = 0.3;
    world->setDeactivationOptions(opts);

    const double s = kBox + 1e-3;
    const Eigen::Vector3d centers[] = {
        {-2.0, 0.0, 0.0}, {-0.7, 0.0, 0.0}, {0.7, 0.0, 0.0}, {2.0, 0.0, 0.0}};
    int id = 0;
    for (const auto& c : centers) {
      for (int k = 0; k < 4; ++k) {
        world->addSkeleton(createBox(
            "box" + std::to_string(id++),
            c + Eigen::Vector3d(0, 0, kBox / 2 + 0.01 + k * s)));
      }
    }
    addProjectilePool(world);

    auto state = std::make_shared<SleepingState>();
    createAimLine(world, *state);
    // Prime the island coloring and the aim-line geometry once at build time.
    // These normally refresh from postStep, which does not run until the sim is
    // stepped, so without this a freshly built (and still paused) scene would
    // show default colors and a degenerate zero-length aim line.
    recolorIslands(world, *state);
    updateAimLine(world, *state);

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(6.0, 8.0, 4.0),
        ::osg::Vec3d(0.0, 0.0, 1.0),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.postStep = [world, state] {
      recolorIslands(world, *state);
      updateAimLine(world, *state);
    };

    setup.keyActions.push_back(KeyAction{'f', "Shoot sphere", [world, state] {
                                           spawnSphere(world, *state);
                                         }});
    setup.keyActions.push_back(KeyAction{'d', "Drop box", [world, state] {
                                           spawnDropBox(world, *state);
                                         }});
    setup.keyActions.push_back(
        KeyAction{'t', "Toggle sleeping", [world] {
                    auto o = world->getDeactivationOptions();
                    o.mEnabled = !o.mEnabled;
                    world->setDeactivationOptions(o);
                  }});
    setup.keyActions.push_back(KeyAction{'c', "Toggle island colors", [state] {
                                           state->colorByIsland
                                               = !state->colorByIsland;
                                         }});

    setup.renderPanel = [world, state] {
      int awake = 0, asleep = 0;
      countStacks(world, awake, asleep);
      int shots = 0, drops = 0;
      countProjectiles(world, shots, drops);

      ImGui::Text("Stacks  %d awake / %d asleep", awake, asleep);
      ImGui::Text("Active  %d sphere(s) / %d box(es)", shots, drops);

      // Size the legend swatches off the current font size so they track the
      // active GUI scale (GetFontSize() is already scaled by the ImGui
      // handler's gui scale); a hard-coded 16 px would stay tiny on a
      // high-DPI/scaled UI.
      const float swatch = ImGui::GetFontSize();
      ImGui::ColorButton(
          "awake_color",
          ImVec4(0.48f, 0.76f, 1.0f, 1.0f),
          ImGuiColorEditFlags_NoTooltip,
          ImVec2(swatch, swatch));
      ImGui::SameLine();
      ImGui::TextUnformatted("awake");
      ImGui::SameLine();
      ImGui::ColorButton(
          "asleep_color",
          ImVec4(0.16f, 0.22f, 0.30f, 1.0f),
          ImGuiColorEditFlags_NoTooltip,
          ImVec2(swatch, swatch));
      ImGui::SameLine();
      ImGui::TextUnformatted("asleep");

      auto opts = world->getDeactivationOptions();
      bool enabled = opts.mEnabled;
      if (ImGui::Checkbox("Automatic deactivation (sleeping)", &enabled)) {
        opts.mEnabled = enabled;
        world->setDeactivationOptions(opts);
      }

      bool colorByIsland = state->colorByIsland;
      if (ImGui::Checkbox("Color by solver island", &colorByIsland))
        state->colorByIsland = colorByIsland;

      auto dwell = static_cast<float>(opts.mTimeUntilSleep);
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
      if (ImGui::SliderFloat(
              "Dwell time",
              &dwell,
              0.05f,
              2.0f,
              "%.3f s",
              ImGuiSliderFlags_AlwaysClamp)
          && std::isfinite(dwell)) {
        opts.mTimeUntilSleep = std::clamp(dwell, 0.05f, 2.0f);
        world->setDeactivationOptions(opts);
      }

      ImGui::TextWrapped(
          "The yellow arc previews the next sphere shot. Awake objects are "
          "bright; sleeping objects use a cooler muted tint.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
