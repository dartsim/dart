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

#include <iostream>
#include <memory>

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
  sn->getVisualAspect()->setColor(Eigen::Vector3d(0.55, 0.55, 0.6));
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
      = {{0.86, 0.22, 0.22},
         {0.24, 0.70, 0.34},
         {0.24, 0.48, 0.92},
         {0.92, 0.72, 0.16},
         {0.72, 0.32, 0.82},
         {0.20, 0.76, 0.76},
         {0.93, 0.52, 0.20}};
  constexpr int n = static_cast<int>(sizeof(palette) / sizeof(palette[0]));
  if (idx < 0)
    return Eigen::Vector4d(0.82, 0.82, 0.82, 1.0);
  const Eigen::Vector3d& c = palette[idx % n];
  return Eigen::Vector4d(c[0], c[1], c[2], 1.0);
}

// Dim, cool tint applied to an island that is asleep (deactivated).
Eigen::Vector4d asleepTint(const Eigen::Vector4d& base)
{
  const Eigen::Vector3d cool(0.12, 0.15, 0.30);
  const Eigen::Vector3d c = 0.35 * base.head<3>() + 0.65 * cool;
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
      WorldPtr world)
    : mViewer(viewer), mNode(node), mWorld(std::move(world))
  {
  }

  void render() override
  {
    ImGui::SetNextWindowPos(ImVec2(10, 20));
    ImGui::SetNextWindowSize(ImVec2(310, 380));
    ImGui::SetNextWindowBgAlpha(0.6f);
    if (!ImGui::Begin(
            "Island deactivation", nullptr, ImGuiWindowFlags_NoResize)) {
      ImGui::End();
      return;
    }

    ImGui::TextWrapped(
        "Stacks settle into solver islands, each drawn in its own color and "
        "tinted dim when it goes to sleep (its dynamics are skipped).");
    ImGui::Spacing();
    ImGui::Separator();

    if (ImGui::CollapsingHeader("Status", ImGuiTreeNodeFlags_DefaultOpen)) {
      int awake = 0, asleep = 0;
      mNode->countStacks(awake, asleep);
      ImGui::Text("Sim time : %.2f s", mWorld->getTime());
      ImGui::Text("FPS      : %.1f", ImGui::GetIO().Framerate);
      ImGui::Text("Stacks   : %d awake, %d asleep", awake, asleep);
      ImGui::TextColored(
          asleep > 0 ? ImVec4(0.4f, 0.7f, 1.0f, 1.0f)
                     : ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
          "%s",
          asleep > 0 ? "(asleep islands are tinted blue/dim)"
                     : "(all islands active)");
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
      if (ImGui::SliderFloat("Time until sleep (s)", &dwell, 0.05f, 2.0f)) {
        opts.mTimeUntilSleep = dwell;
        mWorld->setDeactivationOptions(opts);
      }
    }

    if (ImGui::CollapsingHeader("Actions", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Shoot sphere (f)"))
        mNode->fireSphere();
      ImGui::SameLine();
      if (ImGui::Button("Drop box (d)"))
        mNode->dropBox();
      ImGui::TextWrapped(
          "A projectile wakes the island it hits; the yellow arc previews the "
          "next sphere's path.");
    }

    if (ImGui::CollapsingHeader("Keys")) {
      ImGui::Text("f - shoot a sphere along the aim arc");
      ImGui::Text("d - drop a box onto the aimed stack");
      ImGui::Text("t - toggle automatic deactivation");
      ImGui::Text("s - print awake/asleep stats");
    }

    ImGui::End();
  }

private:
  dart::gui::osg::ImGuiViewer* mViewer;
  DeactivationWorldNode* mNode;
  WorldPtr mWorld;
};

} // namespace

//==============================================================================
int main()
{
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

  // Soft shadows for depth.
  node->setShadowTechnique(
      dart::gui::osg::WorldNode::createDefaultShadowTechnique(viewer.get()));

  // On-screen instructions / status / options panel.
  viewer->getImGuiHandler()->addWidget(
      std::make_shared<IslandWidget>(viewer, node.get(), world));

  viewer->addEventHandler(new KeyHandler(node.get(), world));
  viewer->setUpViewInWindow(0, 0, 1280, 800);
  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(6.0, 8.0, 4.0),
      ::osg::Vec3(0.0, 0.0, 1.0),
      ::osg::Vec3(0.0, 0.0, 1.0));
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  // Start simulating immediately rather than opening paused.
  viewer->simulate(true);

  viewer->run();

  return 0;
}
