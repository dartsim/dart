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

// Visual demo of automatic body deactivation ("sleeping") and the solver-island
// partition. Several separated box stacks each form an independent solver
// island. Each island is drawn in a distinct color; when an island goes to
// sleep (deactivates) it is tinted to a dim, cool shade. Press keys to drop a
// box (which wakes the island it lands on, and merges islands if it bridges
// two), toggle the feature, or print stats.

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <osg/Camera>
#include <osg/GraphicsContext>
#include <osg/Light>
#include <osg/Viewport>
#include <osgShadow/SoftShadowMap>

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>

#include <cmath>
#include <cstdlib>
#include <cstring>

using namespace dart;
using dart::dynamics::BodyNode;
using dart::dynamics::BoxShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::FreeJoint;
using dart::dynamics::ShapeNode;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::VisualAspect;
using dart::dynamics::WeldJoint;
using dart::simulation::World;
using dart::simulation::WorldPtr;

namespace {

constexpr double kBox = 0.3;
constexpr double kDensity = 1000.0; // kg/m^3, water-like

//==============================================================================
// Sets a body's mass and moment of inertia from its shape's volume, so small
// boxes/spheres get physically-sized inertia instead of the default (1,1,1)
// moment (which is far too large and makes them tumble unrealistically).
void setShapeInertia(dynamics::BodyNode* body, const dynamics::ShapePtr& shape)
{
  const double mass = kDensity * shape->getVolume();
  dynamics::Inertia inertia;
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
// A projectile sphere launched with an initial world-frame linear velocity.
// Projectiles keep a fixed bright color (they are skipped by the island
// recolor pass) so the user can see the object they fired.
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
    shape = std::make_shared<dart::dynamics::SphereShape>(0.25);
  else
    shape = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.35));
  auto* sn = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  sn->getVisualAspect()->setColor(Eigen::Vector3d(0.97, 0.55, 0.10));
  setShapeInertia(body, shape);

  auto* joint = static_cast<FreeJoint*>(skel->getJoint(0));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = pos;
  joint->setPositions(FreeJoint::convertToPositions(tf));

  // FreeJoint generalized velocity is [angular(3), linear(3)]; with the sphere
  // spawned axis-aligned the linear part is the world-frame velocity.
  Eigen::Vector6d v = Eigen::Vector6d::Zero();
  v.tail<3>() = linearVelocity;
  joint->setVelocities(v);
  return skel;
}

//==============================================================================
// True if this skeleton is a user-fired projectile (kept at a fixed color).
bool isProjectile(const SkeletonPtr& skel)
{
  return skel->getName().rfind("proj", 0) == 0;
}

//==============================================================================
// Distinct base color per island index; a neutral gray for "not in any island".
Eigen::Vector4d islandColor(int idx)
{
  static const Eigen::Vector3d palette[]
      = {{0.25, 0.55, 0.95},
         {0.95, 0.48, 0.22},
         {0.18, 0.72, 0.58},
         {0.86, 0.36, 0.62},
         {0.92, 0.72, 0.22},
         {0.58, 0.42, 0.86},
         {0.35, 0.70, 0.86}};
  constexpr int n = static_cast<int>(sizeof(palette) / sizeof(palette[0]));
  if (idx < 0)
    return Eigen::Vector4d(0.74, 0.76, 0.78, 1.0);
  const Eigen::Vector3d& c = palette[idx % n];
  return Eigen::Vector4d(c[0], c[1], c[2], 1.0);
}

// Dim, cool tint applied to an island that is asleep (deactivated).
Eigen::Vector4d asleepTint(const Eigen::Vector4d& base)
{
  const Eigen::Vector3d cool(0.48, 0.56, 0.66);
  const Eigen::Vector3d c = 0.60 * base.head<3>() + 0.40 * cool;
  return Eigen::Vector4d(c[0], c[1], c[2], 1.0);
}

// X centers of the four stacks; shots cycle through them.
const double kStackX[] = {-2.0, -0.7, 0.7, 2.0};

struct Launch
{
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
};

// Sphere launch parameters aimed at the stack selected by targetIdx. Tuned so
// the parabola arcs into the stack body rather than the floor in front of it.
Launch sphereLaunch(int targetIdx)
{
  const double x = kStackX[((targetIdx % 4) + 4) % 4];
  return {Eigen::Vector3d(x, 5.0, 1.6), Eigen::Vector3d(0.0, -12.0, 0.0)};
}

// Shared aim selection: the key handler advances targetIdx on each shot and the
// world node draws the predicted trajectory for the current selection.
struct AimState
{
  int targetIdx = 0;
};

//==============================================================================
class DeactivationWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  static constexpr int kAimPoints = 32;

  DeactivationWorldNode(const WorldPtr& world, std::shared_ptr<AimState> aim)
    : dart::gui::osg::RealTimeWorldNode(world),
      mWorld(world),
      mAim(std::move(aim))
  {
    createAimLine();
  }

  void customPreRefresh() override
  {
    recolorIslands();
    updateAimLine();
  }

  // Fire a sphere along the current aim arc; advances aim to the next stack.
  void fireSphere()
  {
    const Launch L = sphereLaunch(mAim->targetIdx);
    mWorld->addSkeleton(createProjectile(
        "proj_shot" + std::to_string(++mFired), L.pos, L.vel, /*sphere=*/true));
    ++mAim->targetIdx;
  }

  // Drop a box onto the currently-aimed stack; advances aim.
  void dropBox()
  {
    const double targetX = kStackX[((mAim->targetIdx % 4) + 4) % 4];
    mWorld->addSkeleton(createProjectile(
        "proj_drop" + std::to_string(++mFired),
        Eigen::Vector3d(targetX, 0.0, 4.0),
        Eigen::Vector3d::Zero(),
        /*sphere=*/false));
    ++mAim->targetIdx;
  }

  // Counts awake vs. asleep stack skeletons (mobile, non-projectile).
  void countStacks(int& awake, int& asleep) const
  {
    awake = 0;
    asleep = 0;
    for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i) {
      const auto& skel = mWorld->getSkeleton(i);
      if (!skel->isMobile() || isProjectile(skel))
        continue;
      (skel->isResting() ? asleep : awake)++;
    }
  }

private:
  // Recolor every box each frame from its current island index and sleep state.
  void recolorIslands()
  {
    for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i) {
      const auto& skel = mWorld->getSkeleton(i);
      if (!skel->isMobile())
        continue; // skip the static floor
      if (isProjectile(skel))
        continue; // keep the fired projectile its own bright color

      Eigen::Vector4d color = islandColor(skel->getIslandIndex());
      if (skel->isResting())
        color = asleepTint(color);

      for (std::size_t b = 0; b < skel->getNumBodyNodes(); ++b) {
        auto* body = skel->getBodyNode(b);
        for (std::size_t s = 0; s < body->getNumShapeNodes(); ++s) {
          auto* sn = body->getShapeNode(s);
          if (auto* visual = sn->getVisualAspect())
            visual->setRGBA(color);
        }
      }
    }
  }

  // A polyline that predicts where the next fired sphere will go (a parabola
  // under gravity). It is only an estimate - the real impact differs once the
  // sphere collides with a stack.
  void createAimLine()
  {
    auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "aim");
    mAimLine = std::make_shared<dart::dynamics::LineSegmentShape>(3.0f);
    for (int i = 0; i < kAimPoints; ++i)
      mAimLine->addVertex(Eigen::Vector3d::Zero());
    for (int i = 0; i + 1 < kAimPoints; ++i)
      mAimLine->addConnection(i, i + 1);
    mAimLine->addDataVariance(dart::dynamics::Shape::DYNAMIC_VERTICES);
    frame->setShape(mAimLine);
    frame->createVisualAspect();
    frame->getVisualAspect()->setColor(Eigen::Vector3d(1.0, 0.92, 0.20));
    mWorld->addSimpleFrame(frame);
  }

  void updateAimLine()
  {
    const Launch L = sphereLaunch(mAim->targetIdx);
    const Eigen::Vector3d g = mWorld->getGravity();

    // Time at which the parabola reaches the floor (z = 0), so the drawn arc
    // ends at the predicted ground impact.
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
      mAimLine->setVertex(static_cast<std::size_t>(i), p);
    }
  }

  WorldPtr mWorld;
  std::shared_ptr<AimState> mAim;
  dart::dynamics::LineSegmentShapePtr mAimLine;
  int mFired{0};
};

//==============================================================================
class KeyHandler : public osgGA::GUIEventHandler
{
public:
  KeyHandler(DeactivationWorldNode* node, WorldPtr world)
    : mNode(node), mWorld(std::move(world))
  {
  }

  bool handle(
      const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() != osgGA::GUIEventAdapter::KEYDOWN)
      return false;

    switch (ea.getKey()) {
      case 'f':
        mNode->fireSphere();
        return true;
      case 'd':
        mNode->dropBox();
        return true;
      case 't': {
        auto opts = mWorld->getDeactivationOptions();
        opts.mEnabled = !opts.mEnabled;
        mWorld->setDeactivationOptions(opts);
        return true;
      }
      case 's': {
        int awake = 0, asleep = 0;
        mNode->countStacks(awake, asleep);
        std::cout << "[stats] awake=" << awake << " asleep=" << asleep << "\n";
        return true;
      }
      default:
        return false;
    }
  }

private:
  DeactivationWorldNode* mNode;
  WorldPtr mWorld;
};

//==============================================================================
// On-screen panel: instructions, live status, and option controls. Mirrors the
// keyboard shortcuts with buttons so the demo is usable without the keyboard.
class IslandWidget : public dart::gui::osg::ImGuiWidget
{
public:
  IslandWidget(
      dart::gui::osg::ImGuiViewer* viewer,
      DeactivationWorldNode* node,
      WorldPtr world,
      float scale = 1.0f)
    : mViewer(viewer), mNode(node), mWorld(std::move(world)), mScale(scale)
  {
  }

  void render() override
  {
    const ImGuiIO& io = ImGui::GetIO();
    const float margin = 12.0f * mScale;
    const float maxWidth = std::max(1.0f, io.DisplaySize.x - 2.0f * margin);
    const float maxHeight = std::max(1.0f, io.DisplaySize.y - 2.0f * margin);
    const float width = std::min(320.0f * mScale, maxWidth);
    const float height = std::min(400.0f * mScale, maxHeight);

    ImGui::SetNextWindowPos(ImVec2(margin, margin), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.96f);
    if (!ImGui::Begin(
            "Sleeping Demo",
            nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse
                | ImGuiWindowFlags_NoSavedSettings)) {
      ImGui::End();
      return;
    }

    if (ImGui::CollapsingHeader("Status", ImGuiTreeNodeFlags_DefaultOpen)) {
      int awake = 0, asleep = 0;
      mNode->countStacks(awake, asleep);
      ImGui::Text("Time  %.2f s", mWorld->getTime());
      ImGui::SameLine();
      ImGui::Text("FPS  %.0f", ImGui::GetIO().Framerate);
      ImGui::Text("Stacks  %d awake / %d asleep", awake, asleep);

      ImGui::ColorButton(
          "awake_color",
          ImVec4(0.25f, 0.55f, 0.95f, 1.0f),
          ImGuiColorEditFlags_NoTooltip,
          ImVec2(16.0f * mScale, 16.0f * mScale));
      ImGui::SameLine();
      ImGui::TextUnformatted("awake");
      ImGui::SameLine();
      ImGui::ColorButton(
          "asleep_color",
          ImVec4(0.34f, 0.55f, 0.83f, 1.0f),
          ImGuiColorEditFlags_NoTooltip,
          ImVec2(16.0f * mScale, 16.0f * mScale));
      ImGui::SameLine();
      ImGui::TextUnformatted("asleep");
    }

    if (ImGui::CollapsingHeader("Options", ImGuiTreeNodeFlags_DefaultOpen)) {
      bool simulating = mViewer->isSimulating();
      if (ImGui::Checkbox("Run simulation", &simulating))
        mViewer->simulate(simulating);

      auto opts = mWorld->getDeactivationOptions();
      bool enabled = opts.mEnabled;
      if (ImGui::Checkbox("Automatic deactivation (sleeping)", &enabled)) {
        opts.mEnabled = enabled;
        mWorld->setDeactivationOptions(opts);
      }

      float dwell = static_cast<float>(opts.mTimeUntilSleep);
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
      if (ImGui::SliderFloat("Dwell time (s)", &dwell, 0.05f, 2.0f)) {
        opts.mTimeUntilSleep = dwell;
        mWorld->setDeactivationOptions(opts);
      }
    }

    if (ImGui::CollapsingHeader("Actions", ImGuiTreeNodeFlags_DefaultOpen)) {
      const float spacing = ImGui::GetStyle().ItemSpacing.x;
      const float buttonWidth
          = (ImGui::GetContentRegionAvail().x - spacing) * 0.5f;
      if (ImGui::Button("Shoot sphere", ImVec2(buttonWidth, 0.0f)))
        mNode->fireSphere();
      ImGui::SameLine();
      if (ImGui::Button("Drop box", ImVec2(buttonWidth, 0.0f)))
        mNode->dropBox();
      ImGui::TextWrapped("The yellow arc previews the next sphere shot.");
    }

    if (ImGui::CollapsingHeader("Keys")) {
      ImGui::BulletText("f  Shoot sphere");
      ImGui::BulletText("d  Drop box");
      ImGui::BulletText("t  Toggle sleeping");
      ImGui::BulletText("s  Print stats");
    }

    ImGui::End();
  }

private:
  dart::gui::osg::ImGuiViewer* mViewer;
  DeactivationWorldNode* mNode;
  WorldPtr mWorld;
  float mScale;
};

//==============================================================================
struct Options
{
  float guiScale = 1.0f;
  bool headless = false;
  std::string shotPath = "sleeping.png";
  int settleSteps = 1500;
  int width = 1280;
  int height = 800;
};

//==============================================================================
void printUsage(const char* prog)
{
  std::cout
      << "Usage: " << prog << " [options]\n"
      << "  --gui-scale <f>  Scale the on-screen ImGui panel (default 1.0;\n"
      << "                   try 2.0 on a HiDPI/4K display).\n"
      << "  --headless       Render one frame off-screen to a PNG and exit\n"
      << "                   (no window); useful for smoke tests / CI.\n"
      << "  --shot <path>    Output PNG path for --headless (default "
         "sleeping.png).\n"
      << "  --steps <n>      Sim steps to settle before the headless shot "
         "(default 1500).\n"
      << "  --width <w> --height <h>  Render size (default 1280x800).\n"
      << "  -h, --help       Show this help.\n";
}

//==============================================================================
// Parse command-line options. Returns false if the program should exit
// immediately (e.g. after --help or on a parse error).
bool parseArgs(int argc, char** argv, Options& opt)
{
  auto needsValue = [&](int i) {
    if (i + 1 >= argc) {
      std::cerr << "Missing value for " << argv[i] << "\n";
      return false;
    }
    return true;
  };
  for (int i = 1; i < argc; ++i) {
    const char* a = argv[i];
    if (std::strcmp(a, "--gui-scale") == 0) {
      if (!needsValue(i))
        return false;
      opt.guiScale = std::stof(argv[++i]);
    } else if (std::strcmp(a, "--headless") == 0) {
      opt.headless = true;
    } else if (std::strcmp(a, "--shot") == 0) {
      if (!needsValue(i))
        return false;
      opt.shotPath = argv[++i];
    } else if (std::strcmp(a, "--steps") == 0) {
      if (!needsValue(i))
        return false;
      opt.settleSteps = std::stoi(argv[++i]);
    } else if (std::strcmp(a, "--width") == 0) {
      if (!needsValue(i))
        return false;
      opt.width = std::stoi(argv[++i]);
    } else if (std::strcmp(a, "--height") == 0) {
      if (!needsValue(i))
        return false;
      opt.height = std::stoi(argv[++i]);
    } else if (std::strcmp(a, "-h") == 0 || std::strcmp(a, "--help") == 0) {
      printUsage(argv[0]);
      return false;
    } else {
      std::cerr << "Unknown option: " << a << "\n";
      printUsage(argv[0]);
      return false;
    }
  }
  return true;
}

//==============================================================================
float sanitizeGuiScale(float scale)
{
  if (!std::isfinite(scale) || scale <= 0.0f)
    return 1.0f;

  return std::clamp(scale, 0.75f, 2.5f);
}

//==============================================================================
void configureImGuiStyle(float scale)
{
  ImGuiIO& io = ImGui::GetIO();
  io.FontGlobalScale = scale;

  ImGui::StyleColorsDark();
  ImGuiStyle& style = ImGui::GetStyle();
  style.WindowRounding = 4.0f;
  style.ChildRounding = 3.0f;
  style.FrameRounding = 3.0f;
  style.GrabRounding = 3.0f;
  style.ScrollbarRounding = 3.0f;
  style.WindowBorderSize = 1.0f;
  style.FrameBorderSize = 0.0f;
  style.WindowPadding = ImVec2(12.0f, 10.0f);
  style.FramePadding = ImVec2(8.0f, 5.0f);
  style.ItemSpacing = ImVec2(8.0f, 7.0f);
  style.ScrollbarSize = 14.0f;
  style.ScaleAllSizes(scale);

  auto& colors = style.Colors;
  colors[ImGuiCol_Text] = ImVec4(0.93f, 0.95f, 0.97f, 1.0f);
  colors[ImGuiCol_WindowBg] = ImVec4(0.075f, 0.085f, 0.105f, 1.0f);
  colors[ImGuiCol_Border] = ImVec4(0.35f, 0.42f, 0.52f, 0.80f);
  colors[ImGuiCol_FrameBg] = ImVec4(0.14f, 0.18f, 0.23f, 1.0f);
  colors[ImGuiCol_FrameBgHovered] = ImVec4(0.22f, 0.30f, 0.39f, 1.0f);
  colors[ImGuiCol_FrameBgActive] = ImVec4(0.26f, 0.38f, 0.50f, 1.0f);
  colors[ImGuiCol_TitleBg] = ImVec4(0.11f, 0.14f, 0.18f, 1.0f);
  colors[ImGuiCol_TitleBgActive] = ImVec4(0.14f, 0.19f, 0.25f, 1.0f);
  colors[ImGuiCol_Header] = ImVec4(0.18f, 0.26f, 0.34f, 1.0f);
  colors[ImGuiCol_HeaderHovered] = ImVec4(0.25f, 0.36f, 0.47f, 1.0f);
  colors[ImGuiCol_HeaderActive] = ImVec4(0.30f, 0.43f, 0.55f, 1.0f);
  colors[ImGuiCol_Button] = ImVec4(0.20f, 0.32f, 0.43f, 1.0f);
  colors[ImGuiCol_ButtonHovered] = ImVec4(0.28f, 0.45f, 0.60f, 1.0f);
  colors[ImGuiCol_ButtonActive] = ImVec4(0.32f, 0.52f, 0.68f, 1.0f);
  colors[ImGuiCol_CheckMark] = ImVec4(0.35f, 0.66f, 0.95f, 1.0f);
  colors[ImGuiCol_SliderGrab] = ImVec4(0.35f, 0.66f, 0.95f, 1.0f);
  colors[ImGuiCol_SliderGrabActive] = ImVec4(0.55f, 0.78f, 1.0f, 1.0f);
}

//==============================================================================
void configureViewerAppearance(dart::gui::osg::ImGuiViewer* viewer)
{
  viewer->getCamera()->setClearColor(::osg::Vec4(0.76f, 0.80f, 0.85f, 1.0f));
  viewer->switchHeadlights(false);

  auto key = viewer->getLightSource(0)->getLight();
  if (key) {
    key->setPosition(::osg::Vec4(-3.5f, -4.5f, 7.0f, 0.0f));
    key->setAmbient(::osg::Vec4(0.18f, 0.18f, 0.20f, 1.0f));
    key->setDiffuse(::osg::Vec4(0.82f, 0.80f, 0.74f, 1.0f));
    key->setSpecular(::osg::Vec4(0.35f, 0.35f, 0.35f, 1.0f));
  }

  auto fill = viewer->getLightSource(1)->getLight();
  if (fill) {
    fill->setPosition(::osg::Vec4(5.0f, 4.0f, 6.0f, 0.0f));
    fill->setAmbient(::osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    fill->setDiffuse(::osg::Vec4(0.24f, 0.28f, 0.34f, 1.0f));
    fill->setSpecular(::osg::Vec4(0.08f, 0.08f, 0.10f, 1.0f));
  }
}

//==============================================================================
::osg::ref_ptr<osgShadow::ShadowTechnique> createDemoShadowTechnique(
    const dart::gui::osg::Viewer* viewer)
{
  ::osg::ref_ptr<osgShadow::SoftShadowMap> sm = new osgShadow::SoftShadowMap;
  const auto mapResolution = static_cast<short>(std::pow(2, 13));
  sm->setTextureSize(::osg::Vec2s(mapResolution, mapResolution));
  sm->setLight(viewer->getLightSource(0));
  sm->setSoftnessWidth(0.004f);
  sm->setJitteringScale(16.0f);
  sm->setBias(0.001f);
  return sm;
}

//==============================================================================
// Render the scene once into an off-screen pbuffer and write a PNG, without
// opening a window. The world is stepped directly so the result is the same
// regardless of wall-clock speed. Returns a process exit code.
int runHeadless(
    dart::gui::osg::ImGuiViewer* viewer,
    const WorldPtr& world,
    const Options& opt,
    const ::osg::Vec3& eye,
    const ::osg::Vec3& center,
    const ::osg::Vec3& up)
{
  ::osg::ref_ptr<::osg::GraphicsContext::Traits> traits
      = new ::osg::GraphicsContext::Traits;
  traits->readDISPLAY();
  traits->setUndefinedScreenDetailsToDefaultScreen();
  traits->x = 0;
  traits->y = 0;
  traits->width = opt.width;
  traits->height = opt.height;
  traits->red = traits->green = traits->blue = 8;
  traits->alpha = 8;
  traits->depth = 24;
  traits->windowDecoration = false;
  traits->pbuffer = true;
  traits->doubleBuffer = true;

  ::osg::ref_ptr<::osg::GraphicsContext> gc
      = ::osg::GraphicsContext::createGraphicsContext(traits.get());
  if (!gc) {
    std::cerr << "[headless] Failed to create an off-screen GL context "
                 "(no usable DISPLAY?).\n";
    return 1;
  }

  auto* camera = viewer->getCamera();
  camera->setGraphicsContext(gc.get());
  camera->setViewport(new ::osg::Viewport(0, 0, opt.width, opt.height));
  camera->setProjectionMatrixAsPerspective(
      30.0, static_cast<double>(opt.width) / opt.height, 0.1, 1000.0);
  const GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
  camera->setDrawBuffer(buffer);
  camera->setReadBuffer(buffer);

  // Single-threaded so the screen-capture (a camera final-draw callback)
  // completes synchronously inside frame(); otherwise the draw thread may still
  // be writing the file as we tear down and exit, corrupting it.
  viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

  // Drive stepping ourselves and pin the view, so the screenshot is
  // deterministic and independent of the real-time clock or any manipulator.
  viewer->setCameraManipulator(nullptr);
  viewer->simulate(false);
  camera->setViewMatrixAsLookAt(eye, center, up);

  viewer->realize();
  if (!viewer->isRealized()) {
    std::cerr << "[headless] Viewer failed to realize off-screen.\n";
    return 1;
  }

  for (int i = 0; i < opt.settleSteps; ++i)
    world->step();

  // Re-pin the view (realize may have reset it) and draw, then capture.
  camera->setViewMatrixAsLookAt(eye, center, up);
  viewer->frame();
  viewer->frame();
  viewer->captureScreen(opt.shotPath);
  viewer->frame(); // SaveScreen writes the PNG during this frame.

  std::cout << "[headless] sim time " << world->getTime() << " s; wrote "
            << opt.shotPath << "\n";
  return 0;
}

} // namespace

//==============================================================================
int main(int argc, char** argv)
{
  Options opt;
  if (!parseArgs(argc, argv, opt))
    return 0;
  opt.guiScale = sanitizeGuiScale(opt.guiScale);

  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(createFloor());

  // Automatic deactivation is on by default. Use a short dwell here so islands
  // visibly go to sleep a fraction of a second after they settle.
  auto opts = world->getDeactivationOptions();
  opts.mEnabled = true;
  opts.mTimeUntilSleep = 0.3;
  world->setDeactivationOptions(opts);

  // Four separated stacks -> four independent solver islands, each colored
  // distinctly and tinted dim once it settles and sleeps.
  const double s = kBox + 1e-3;
  const Eigen::Vector3d centers[]
      = {{-2.0, 0.0, 0.0}, {-0.7, 0.0, 0.0}, {0.7, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  int id = 0;
  for (const auto& c : centers) {
    for (int k = 0; k < 4; ++k) {
      world->addSkeleton(createBox(
          "box" + std::to_string(id++),
          c + Eigen::Vector3d(0, 0, kBox / 2 + 0.01 + k * s)));
    }
  }

  std::cout
      << "\n=== Island deactivation (sleeping) demo ===\n"
      << "Each stack is an independent solver island, drawn in its own\n"
      << "color. When an island settles and goes to sleep it is tinted to\n"
      << "a dim, cool shade (its constraint solve, gravity, and\n"
      << "integration are skipped until it is disturbed).\n\n"
      << "The yellow arc shows the predicted path of the next sphere\n"
      << "(an estimate - gravity and the collision make the real hit\n"
      << "differ).\n\n"
      << "Keys:  f = shoot a sphere along the aim arc (wakes it on hit)\n"
      << "       d = drop a box onto the aimed stack\n"
      << "       t = toggle automatic deactivation on/off\n"
      << "       s = print awake/asleep stats\n\n";

  auto aim = std::make_shared<AimState>();
  osg::ref_ptr<DeactivationWorldNode> node
      = new DeactivationWorldNode(world, aim);

  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
      = new dart::gui::osg::ImGuiViewer();
  viewer->addWorldNode(node);
  configureViewerAppearance(viewer.get());

  // Scale the ImGui panel for HiDPI displays (the default 13px font is tiny on
  // a 4K screen). Scale both style metrics and font rendering from the same
  // value so controls and labels grow together.
  configureImGuiStyle(opt.guiScale);

  // Soft shadows for depth without the blocky, over-dark default demo look.
  node->setShadowTechnique(createDemoShadowTechnique(viewer.get()));

  // On-screen instructions / status / options panel.
  viewer->getImGuiHandler()->addWidget(
      std::make_shared<IslandWidget>(viewer, node.get(), world, opt.guiScale));

  viewer->addEventHandler(new KeyHandler(node.get(), world));

  const ::osg::Vec3 eye(6.0, 8.0, 4.0);
  const ::osg::Vec3 center(0.0, 0.0, 1.0);
  const ::osg::Vec3 up(0.0, 0.0, 1.0);

  if (opt.headless)
    return runHeadless(viewer.get(), world, opt, eye, center, up);

  viewer->setUpViewInWindow(0, 0, opt.width, opt.height);
  viewer->getCameraManipulator()->setHomePosition(eye, center, up);
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  // Start simulating immediately rather than opening paused.
  viewer->simulate(true);

  viewer->run();

  return 0;
}
