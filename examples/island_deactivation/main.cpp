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
  body->setMass(1.0);
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(kBox));
  body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      shape);
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
  body->setMass(3.0);
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
};

//==============================================================================
class KeyHandler : public osgGA::GUIEventHandler
{
public:
  KeyHandler(WorldPtr world, std::shared_ptr<AimState> aim)
    : mWorld(std::move(world)), mAim(std::move(aim))
  {
  }

  bool handle(
      const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() != osgGA::GUIEventAdapter::KEYDOWN)
      return false;

    switch (ea.getKey()) {
      case 'f': {
        // Shoot a sphere along the aim trajectory: it flies in, hits the
        // (sleeping) island, and wakes it on contact.
        const Launch L = sphereLaunch(mAim->targetIdx);
        mWorld->addSkeleton(createProjectile(
            "proj_shot" + std::to_string(++mFired),
            L.pos,
            L.vel,
            /*sphere=*/true));
        std::cout << "[fire] launched a sphere at the stack near x="
                  << L.pos.x() << "\n";
        ++mAim->targetIdx;
        return true;
      }
      case 'd': {
        // Drop a box from above onto the aimed stack: it wakes whatever island
        // it lands on (and merges islands if it bridges two).
        const double targetX = kStackX[((mAim->targetIdx % 4) + 4) % 4];
        mWorld->addSkeleton(createProjectile(
            "proj_drop" + std::to_string(++mFired),
            Eigen::Vector3d(targetX, 0.0, 4.0),
            Eigen::Vector3d::Zero(),
            /*sphere=*/false));
        std::cout << "[drop] released a box above the stack near x=" << targetX
                  << "\n";
        ++mAim->targetIdx;
        return true;
      }
      case 't': {
        auto opts = mWorld->getDeactivationOptions();
        opts.mEnabled = !opts.mEnabled;
        mWorld->setDeactivationOptions(opts);
        std::cout << "[toggle] automatic deactivation "
                  << (opts.mEnabled ? "ENABLED" : "DISABLED") << "\n";
        return true;
      }
      case 's': {
        std::size_t asleep = 0, awake = 0;
        for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i) {
          const auto& skel = mWorld->getSkeleton(i);
          if (!skel->isMobile())
            continue;
          (skel->isResting() ? asleep : awake)++;
        }
        std::cout << "[stats] awake skeletons=" << awake
                  << "  asleep=" << asleep << "\n";
        return true;
      }
      default:
        return false;
    }
  }

private:
  WorldPtr mWorld;
  std::shared_ptr<AimState> mAim;
  int mFired{0};
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

  dart::gui::osg::Viewer viewer;
  viewer.addWorldNode(node);
  viewer.addEventHandler(new KeyHandler(world, aim));
  viewer.setUpViewInWindow(0, 0, 1280, 800);
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(6.0, 8.0, 4.0),
      ::osg::Vec3(0.0, 0.0, 1.0),
      ::osg::Vec3(0.0, 0.0, 1.0));
  viewer.setCameraManipulator(viewer.getCameraManipulator());
  viewer.run();

  return 0;
}
