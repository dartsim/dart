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

// Friction-evaluation scenes and their references, shared by the friction_eval
// harness (built against DART 6.19.4 and the 6.20 line) and by the T0 test
// tests/integration/test_FrictionAnalytic.cpp, so only API common to both is
// used. Ids follow the evaluation design: A analytic, C coupled thresholds,
// R robustness, P performance. Angles in Params are degrees. Each scene
// reports measured metrics next to "pred_exact_*" (exact Coulomb) and
// "pred_box_*" (DART's box law on its fixed tangent basis) references. A
// metric that is undefined for a run (an onset that never happened, a ratio
// without samples) is omitted, so NaN only ever marks a failure.

#ifndef DART_TOOLS_FRICTION_EVAL_FRICTION_SCENES_HPP_
#define DART_TOOLS_FRICTION_EVAL_FRICTION_SCENES_HPP_

#include <dart/config.hpp>

#include <dart/simulation/World.hpp>

#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/constraint/ContactConstraint.hpp>
#include <dart/constraint/ContactSurface.hpp>

#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#if HAVE_BULLET
  #include <dart/collision/bullet/BulletCollisionDetector.hpp>
#endif
#if HAVE_ODE
  #include <dart/collision/ode/OdeCollisionDetector.hpp>
#endif

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/WeldJoint.hpp>

// The 6.20 line still reports version 6.19.4 until it is packaged, so detect
// it by a header it added.
#if __has_include(<dart/dynamics/ConvexMeshShape.hpp>)
  #include <dart/dynamics/ConvexMeshShape.hpp>
  #define FRICTION_EVAL_DART620 1
#else
  #define FRICTION_EVAL_DART620 0
#endif

#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <cmath>

namespace friction_eval {

constexpr double kGravity = 9.81;
constexpr double kPi = 3.14159265358979323846;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// Initial overlap so every detector reports resting contacts at the first step.
constexpr double kOverlap = 1e-5;
// Speed [m/s] above which a body or contact counts as sliding.
constexpr double kSlideSpeed = 1e-3;

using Params = std::map<std::string, double>;
using Metrics = std::map<std::string, double>;

struct Scene
{
  dart::simulation::WorldPtr world;
  int steps = 0;
  std::function<void(int)> preStep;  // Forces and drives before World::step().
  std::function<bool(int)> postStep; // Observation; false stops the run.
  std::function<void(Metrics&)> finish;
};

inline double param(const Params& p, const std::string& key, double fallback)
{
  const auto it = p.find(key);
  return it == p.end() ? fallback : it->second;
}

inline double rad(double degrees)
{
  return degrees * kPi / 180.0;
}

inline double deg(double radians)
{
  return radians * 180.0 / kPi;
}

inline int steps(double seconds, double dt)
{
  return std::max(1, static_cast<int>(std::lround(seconds / dt)));
}

inline double angleDeg(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  const double n = a.norm() * b.norm();
  return n > 0.0 ? deg(std::acos(std::clamp(a.dot(b) / n, -1.0, 1.0))) : 0.0;
}

inline Eigen::Vector3d planar(const Eigen::Vector3d& v)
{
  return {v.x(), v.y(), 0.0};
}

inline Eigen::Isometry3d pose(
    const Eigen::Vector3d& translation,
    const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity())
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = rotation;
  tf.translation() = translation;
  return tf;
}

inline Eigen::Matrix3d rotation(double angle, const Eigen::Vector3d& axis)
{
  return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

inline std::shared_ptr<dart::dynamics::BoxShape> boxShape(
    double x, double y, double z)
{
  return std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(x, y, z));
}

/// Static box capacity factor 1/max(|cos|, |sin|) along an angle to the
/// first friction axis (C §1, D App. B).
inline double boxFactor(double angle)
{
  return 1.0 / std::max(std::abs(std::cos(angle)), std::abs(std::sin(angle)));
}

/// Distance semi-implicit Euler covers in n steps from v0 at deceleration a
/// (D App. A). The block stops at step K = ceil(v0/(a h)) with v_K = 0 exactly,
/// because the last friction impulse is smaller; without a stop in the horizon
/// (including a = 0) it is h sum_{k=1..n} (v0 - k a h).
inline double discreteSlideDistance(double v0, double a, double h, int n)
{
  if (v0 <= 0.0)
    return 0.0;
  const double k = a > 0.0 ? std::ceil(v0 / (a * h)) : n + 1.0;
  if (k <= n)
    return h * ((k - 1.0) * v0 - a * h * (k - 1.0) * k / 2.0);
  return h * (n * v0 - a * h * n * (n + 1.0) / 2.0);
}

/// Detectors by harness name; FCL uses its analytic primitives on both lines
/// (6.19.4 defaults to meshes, 6.20 to primitives).
inline dart::collision::CollisionDetectorPtr makeDetector(std::string_view name)
{
  if (name == "dart")
    return dart::collision::DARTCollisionDetector::create();
  if (name == "fcl") {
    auto detector = dart::collision::FCLCollisionDetector::create();
    detector->setPrimitiveShapeType(
        dart::collision::FCLCollisionDetector::PRIMITIVE);
    return detector;
  }
#if HAVE_BULLET
  if (name == "bullet")
    return dart::collision::BulletCollisionDetector::create();
#endif
#if HAVE_ODE
  if (name == "ode")
    return dart::collision::OdeCollisionDetector::create();
#endif
  return nullptr;
}

/// Builds a contact constraint the way DefaultContactSurfaceHandler does
/// (slip compliance scaled by the contact count) from handler's parameters.
inline dart::constraint::ContactConstraintPtr makeContactConstraint(
    const dart::constraint::ContactSurfaceHandler& handler,
    dart::collision::Contact& contact,
    std::size_t numContacts,
    double timeStep)
{
  auto params = handler.createParams(contact, numContacts);
  params.mPrimarySlipCompliance *= static_cast<double>(numContacts);
  params.mSecondarySlipCompliance *= static_cast<double>(numContacts);
  return std::make_shared<dart::constraint::ContactConstraint>(
      contact, timeStep, params);
}

/// Conveyor (A10): sets the contact surface motion velocity, expressed as
/// (normal, first friction axis, second friction axis).
class SurfaceVelocityHandler : public dart::constraint::ContactSurfaceHandler
{
public:
  explicit SurfaceVelocityHandler(const Eigen::Vector3d& velocity)
    : mVelocity(velocity)
  {
  }

  dart::constraint::ContactSurfaceParams createParams(
      const dart::collision::Contact& contact,
      std::size_t numContacts) const override
  {
    auto params = ContactSurfaceHandler::createParams(contact, numContacts);
    params.mContactSurfaceMotionVelocity = mVelocity;
    return params;
  }

  dart::constraint::ContactConstraintPtr createConstraint(
      dart::collision::Contact& contact,
      std::size_t numContacts,
      double timeStep) const override
  {
    return makeContactConstraint(*this, contact, numContacts, timeStep);
  }

private:
  Eigen::Vector3d mVelocity;
};

inline dart::simulation::WorldPtr makeWorld(double dt)
{
  auto world = dart::simulation::World::create("friction_eval");
  world->setTimeStep(dt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  // Accuracy runs keep every island awake; the harness can turn this back on.
  auto deactivation = world->getDeactivationOptions();
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);
  return world;
}

inline dart::dynamics::ShapeNode* addShape(
    dart::dynamics::BodyNode* body,
    const dart::dynamics::ShapePtr& shape,
    double mu,
    double mu2)
{
  auto* node = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  node->getDynamicsAspect()->setPrimaryFrictionCoeff(mu);
  node->getDynamicsAspect()->setSecondaryFrictionCoeff(mu2);
  return node;
}

inline dart::dynamics::BodyNode* addStatic(
    dart::simulation::World& world,
    const std::string& name,
    const dart::dynamics::ShapePtr& shape,
    const Eigen::Isometry3d& tf,
    double mu,
    double mu2)
{
  auto skeleton = dart::dynamics::Skeleton::create(name);
  auto* body = skeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>()
                   .second;
  body->getParentJoint()->setTransformFromParentBodyNode(tf);
  addShape(body, shape, mu, mu2);
  skeleton->setMobile(false);
  world.addSkeleton(skeleton);
  return body;
}

/// Ground slab with its top face at z = 0.
inline dart::dynamics::BodyNode* addGround(
    dart::simulation::World& world, double mu, double mu2)
{
  return addStatic(
      world,
      "ground",
      boxShape(100.0, 100.0, 1.0),
      pose({0.0, 0.0, -0.5}),
      mu,
      mu2);
}

inline dart::dynamics::BodyNode* addFree(
    dart::simulation::World& world,
    const std::string& name,
    const dart::dynamics::ShapePtr& shape,
    double mass,
    const Eigen::Isometry3d& tf,
    double mu,
    double mu2)
{
  auto skeleton = dart::dynamics::Skeleton::create(name);
  auto* body = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
                   .second;
  addShape(body, shape, mu, mu2);
  body->setInertia(dart::dynamics::Inertia(
      mass, Eigen::Vector3d::Zero(), shape->computeInertia(mass)));
  dart::dynamics::FreeJoint::setTransformOf(body, tf);
  world.addSkeleton(skeleton);
  return body;
}

inline dart::dynamics::FreeJoint* freeJoint(dart::dynamics::BodyNode* body)
{
  return static_cast<dart::dynamics::FreeJoint*>(body->getParentJoint());
}

/// Net contact force [N] on body from the last step's contacts (world frame).
/// Contact::force acts on the first body and its negative on the second.
inline Eigen::Vector3d contactForce(
    const dart::simulation::World& world, const dart::dynamics::BodyNode* body)
{
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  const auto& result = world.getLastCollisionResult();
  for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
    const auto& contact = result.getContact(i);
    if (contact.getBodyNodePtr1().get() == body)
      force += contact.force;
    else if (contact.getBodyNodePtr2().get() == body)
      force -= contact.force;
  }
  return force;
}

inline Eigen::Vector3d position(const dart::dynamics::BodyNode* body)
{
  return body->getWorldTransform().translation();
}

//==============================================================================
// L2 analytic scenes
//==============================================================================

// A1/A2: 1 m cube released on a tan(theta) = 0.5 incline. The incline is
// gravity tilted over flat ground, so DART's default basis stays world X/Y and
// phi rotates the downslope direction against it (A2 without fdir1).
inline Scene incline(const Params& p, double dt)
{
  const double mu = param(p, "mu", 0.3);
  const double tanTheta = param(p, "tan", 0.5);
  const double phi = rad(param(p, "phi", 0.0));
  const double theta = std::atan(tanTheta);
  const Eigen::Vector3d down(std::cos(phi), std::sin(phi), 0.0);
  Scene s;
  s.world = makeWorld(dt);
  s.world->setGravity(
      kGravity
      * (std::sin(theta) * down - std::cos(theta) * Eigen::Vector3d::UnitZ()));
  addGround(*s.world, mu, mu);
  const Eigen::Vector3d start(0.0, 0.0, 0.5 - kOverlap);
  auto* box = addFree(
      *s.world, "box", boxShape(1.0, 1.0, 1.0), 1000.0, pose(start), mu, mu);
  s.steps = steps(param(p, "T", 2.0), dt);
  const double n = s.steps;
  const int halfway = s.steps / 2;
  auto half = std::make_shared<Eigen::Vector3d>(start);
  s.postStep = [=](int i) {
    if (i + 1 == halfway)
      *half = position(box);
    return true;
  };
  s.finish = [=](Metrics& m) {
    const Eigen::Vector3d d = planar(position(box) - start);
    const double along = d.dot(down);
    m["slides"] = along > 1e-3;
    m["pred_exact_slides"] = mu < tanTheta;
    m["pred_box_slides"] = mu * boxFactor(phi) < tanTheta;
    m["accel"] = 2.0 * along / (dt * dt * n * (n + 1.0));
    m["pred_exact_accel"]
        = std::max(0.0, kGravity * (std::sin(theta) - mu * std::cos(theta)));
    m["dir_err_deg"] = d.norm() > 1e-3 ? angleDeg(d, down) : 0.0;
    m["creep"] = planar(position(box) - *half).norm() / (0.5 * n * dt);
  };
  return s;
}

// A3: the incline tilts at 0.05 rad/s; onset angle of sliding.
inline Scene tilt(const Params& p, double dt)
{
  const double mu = param(p, "mu", 0.5);
  const double phi = rad(param(p, "phi", 0.0));
  const double exact = std::atan(mu);
  const double box = std::atan(mu * boxFactor(phi));
  const double rate = 0.05;
  const double start = std::min(exact, box) - rad(2.0);
  const Eigen::Vector3d down(std::cos(phi), std::sin(phi), 0.0);
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu);
  auto* body = addFree(
      *s.world,
      "box",
      boxShape(1.0, 1.0, 1.0),
      1000.0,
      pose({0.0, 0.0, 0.5 - kOverlap}),
      mu,
      mu);
  s.steps = steps((std::max(exact, box) + rad(3.0) - start) / rate, dt);
  auto* world = s.world.get();
  auto onset = std::make_shared<double>(kNaN);
  const auto angleAt = [=](int i) {
    return start + rate * dt * (i + 1);
  };
  s.preStep = [=](int i) {
    const double theta = angleAt(i);
    world->setGravity(
        kGravity
        * (std::sin(theta) * down
           - std::cos(theta) * Eigen::Vector3d::UnitZ()));
  };
  s.postStep = [=](int i) {
    if (body->getLinearVelocity().norm() < 5e-4)
      return true;
    *onset = angleAt(i);
    return false;
  };
  s.finish = [=](Metrics& m) {
    if (!std::isnan(*onset))
      m["onset_deg"] = deg(*onset);
    m["pred_exact_deg"] = deg(exact);
    m["pred_box_deg"] = deg(box);
  };
  return s;
}

// A4/A12/R8: 1 kg block on flat ground pushed by F = k mu m g at phi from
// world X. mu2 and fdir (first friction axis azimuth) make it anisotropic.
inline Scene push(const Params& p, double dt)
{
  const double mu = param(p, "mu", 0.5);
  const double mu2 = param(p, "mu2", mu);
  const double phi = rad(param(p, "phi", 0.0));
  const double k = param(p, "k", 1.05);
  const double axis = rad(param(p, "fdir", 0.0));
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu2);
  const Eigen::Vector3d start(0.0, 0.0, 0.1 - kOverlap);
  auto* box = addFree(
      *s.world, "box", boxShape(0.2, 0.2, 0.2), 1.0, pose(start), mu, mu2);
  if (p.count("fdir")) {
    box->getShapeNode(0)->getDynamicsAspect()->setFirstFrictionDirection(
        Eigen::Vector3d(std::cos(axis), std::sin(axis), 0.0));
  }
  const Eigen::Vector3d dir(std::cos(phi), std::sin(phi), 0.0);
  const Eigen::Vector3d force = k * mu * kGravity * dir;
  s.steps = steps(param(p, "T", 1.0), dt);
  auto* world = s.world.get();
  // Sums over sliding steps: |f_t|/(mu N), angle(f_t, -v_t), count.
  auto sums = std::make_shared<Eigen::Vector3d>(Eigen::Vector3d::Zero());
  s.preStep = [=](int) {
    box->addExtForce(force);
  };
  s.postStep = [=](int) {
    const Eigen::Vector3d v = planar(box->getLinearVelocity());
    const Eigen::Vector3d f = contactForce(*world, box);
    if (v.norm() > kSlideSpeed && f.z() > 0.0)
      *sums += Eigen::Vector3d(
          planar(f).norm() / (mu * f.z()), angleDeg(planar(f), -v), 1.0);
    return true;
  };
  const double n = s.steps;
  s.finish = [=](Metrics& m) {
    const Eigen::Vector3d d = planar(position(box) - start);
    // Static capacity along phi in units of mu m g: box (pyramid) and ellipse.
    const double inf = std::numeric_limits<double>::infinity();
    const double c = std::abs(std::cos(phi - axis));
    const double sn = std::abs(std::sin(phi - axis));
    const double alongY = sn > 1e-12 ? (mu2 > 0.0 ? mu2 / sn : 0.0) : inf;
    m["slides"] = d.norm() > 1e-3;
    m["accel"] = 2.0 * d.dot(dir) / (dt * dt * n * (n + 1.0));
    m["pred_exact_accel"] = std::max(0.0, (k - 1.0) * mu * kGravity);
    m["pred_box_cap"] = std::min(c > 1e-12 ? mu / c : inf, alongY) / mu;
    m["pred_exact_cap"]
        = alongY == 0.0
              ? 0.0
              : 1.0
                    / std::sqrt(
                        c * c
                        + (sn > 1e-12 ? sn * sn * mu * mu / (mu2 * mu2) : 0.0));
    m["pred_box_slides"] = k > m["pred_box_cap"];
    m["pred_exact_slides"] = k > m["pred_exact_cap"];
    if ((*sums)(2) > 0.0) {
      m["force_ratio"] = (*sums)(0) / (*sums)(2);
      m["dir_err_deg"] = (*sums)(1) / (*sums)(2);
    }
    m["vel_dir_err_deg"] = angleDeg(planar(box->getLinearVelocity()), dir);
    // Box law on a translating block: each axis slides on its own once the
    // push along it exceeds mu_i N (axis snapping, D App. B).
    const double fx = k * mu * c, fy = k * mu * sn;
    const Eigen::Vector3d accel(
        std::max(0.0, fx - mu), std::max(0.0, fy - mu2), 0.0);
    const double friction = std::hypot(std::min(fx, mu), std::min(fy, mu2));
    if (accel.norm() > 0.0) {
      m["pred_box_force_ratio"] = friction / mu;
      m["pred_box_vel_dir_err_deg"]
          = angleDeg(accel, Eigen::Vector3d(c, sn, 0.0));
    }
  };
  return s;
}

// A5: 1 kg block launched at v0 along phi with no applied force. The box law
// decelerates each axis by mu_i g on its own (D App. B).
inline Scene slide(const Params& p, double dt)
{
  const double mu = param(p, "mu", 0.5);
  const double mu2 = param(p, "mu2", mu);
  const double phi = rad(param(p, "phi", 0.0));
  const double v0 = param(p, "v0", 2.0);
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu2);
  const Eigen::Vector3d start(0.0, 0.0, 0.1 - kOverlap);
  auto* box = addFree(
      *s.world, "box", boxShape(0.2, 0.2, 0.2), 1.0, pose(start), mu, mu2);
  const Eigen::Vector3d dir(std::cos(phi), std::sin(phi), 0.0);
  const Eigen::Vector3d perp(-dir.y(), dir.x(), 0.0);
  freeJoint(box)->setLinearVelocity(v0 * dir);
  const double stopTime = v0 / (std::max(std::min(mu, mu2), 1e-3) * kGravity);
  s.steps = steps(std::min(param(p, "T", 5.0), stopTime + 0.3), dt);
  // lateral max, stop step, position at the stop (x, y)
  auto st = std::make_shared<Eigen::Vector4d>(0.0, -1.0, 0.0, 0.0);
  s.postStep = [=](int i) {
    const Eigen::Vector3d d = planar(position(box) - start);
    (*st)(0) = std::max((*st)(0), std::abs(d.dot(perp)));
    if ((*st)(1) < 0.0
        && box->getLinearVelocity().norm() < 0.1 * mu * kGravity * dt) {
      (*st)(1) = i + 1;
      st->tail<2>() = d.head<2>();
    }
    return true;
  };
  const int n = s.steps;
  s.finish = [=](Metrics& m) {
    const Eigen::Vector3d d = planar(position(box) - start);
    const double exact = discreteSlideDistance(v0, mu * kGravity, dt, n);
    const Eigen::Vector3d boxD(
        std::copysign(
            discreteSlideDistance(v0 * std::abs(dir.x()), mu * kGravity, dt, n),
            dir.x()),
        std::copysign(
            discreteSlideDistance(
                v0 * std::abs(dir.y()), mu2 * kGravity, dt, n),
            dir.y()),
        0.0);
    m["dist"] = d.norm();
    m["dist_ratio"] = d.norm() / exact;
    m["pred_box_dist_ratio"] = boxD.norm() / exact;
    m["dir_deg"] = deg(std::atan2(d.y(), d.x()));
    m["pred_exact_dir_deg"] = deg(phi);
    m["pred_box_dir_deg"] = deg(std::atan2(boxD.y(), boxD.x()));
    m["lateral"] = (*st)(0);
    m["pred_box_lateral"] = std::abs(boxD.dot(perp));
    // Stop time and the drift between the stop and the end exist only when
    // the block stopped inside the horizon (never for mu = 0).
    if ((*st)(1) >= 0.0 && (*st)(1) < n) {
      m["stop_time"] = (*st)(1) * dt;
      m["creep"] = (d.head<2>() - st->tail<2>()).norm() / ((n - (*st)(1)) * dt);
    }
  };
  return s;
}

// A6: flat disk (R 0.2, h 0.02) spinning at w0 about the vertical. References
// from the step's own contacts: exact tau = mu sum N_i r_i; box (both rows
// saturated) tau = mu sum N_i (|x_i| + |y_i|), the |cos| + |sin| factor.
inline Scene spin(const Params& p, double dt)
{
  const double mu = param(p, "mu", 0.5);
  const double radius = param(p, "R", 0.2);
  const double height = param(p, "h", 0.02);
  const double w0 = param(p, "w0", 20.0);
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu);
  const auto shape
      = std::make_shared<dart::dynamics::CylinderShape>(radius, height);
  const double mass = 1000.0 * kPi * radius * radius * height;
  auto* disk = addFree(
      *s.world,
      "disk",
      shape,
      mass,
      pose({0.0, 0.0, height / 2 - kOverlap}),
      mu,
      mu);
  freeJoint(disk)->setAngularVelocity(Eigen::Vector3d(0.0, 0.0, w0));
  const double izz = shape->computeInertia(mass)(2, 2);
  s.steps = steps(param(p, "T", 2.0), dt);
  auto* world = s.world.get();
  // previous w, ratio sum, ratio min, ratio max, box error max, count
  auto st = std::make_shared<std::vector<double>>(
      std::vector<double>{w0, 0.0, 1e300, -1e300, 0.0, 0.0});
  s.postStep = [=](int) {
    auto& v = *st;
    const double w = disk->getAngularVelocity().z();
    const double alpha = (w - v[0]) / dt;
    double exact = 0.0;
    double box = 0.0;
    const auto& result = world->getLastCollisionResult();
    for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
      const auto& c = result.getContact(i);
      const Eigen::Vector3d r = planar(c.point - position(disk));
      const double normal = std::abs(c.force.dot(c.normal));
      exact += mu * normal * r.norm();
      box += mu * normal * (std::abs(r.x()) + std::abs(r.y()));
    }
    if (std::abs(v[0]) > 2.0 && exact > 0.0) {
      const double ratio = -alpha * izz / (std::copysign(1.0, v[0]) * exact);
      v[1] += ratio;
      v[2] = std::min(v[2], ratio);
      v[3] = std::max(v[3], ratio);
      v[4] = std::max(v[4], std::abs(ratio * exact / box - 1.0));
      v[5] += 1.0;
    }
    v[0] = w;
    return std::abs(w) > 2.0;
  };
  s.finish = [=](Metrics& m) {
    const auto& v = *st;
    if (v[5] > 0) {
      m["alpha_ratio_mean"] = v[1] / v[5];
      m["alpha_ratio_min"] = v[2];
      m["alpha_ratio_max"] = v[3];
      m["box_err_max"] = v[4];
    }
    m["pred_box_ratio_mean_rim"] = 4.0 / kPi;
    m["pred_exact_ratio"] = 1.0;
  };
  return s;
}

// A7: sphere (R 0.25, 1 kg) launched at v0 along phi with spin w0 about the
// horizontal axis normal to the launch (w0 < 0 is backspin). v_roll =
// (5 v0 + 2 R w0)/7 is conserved exactly (D App. A); the box decays each slip
// component on its own, so an oblique launch drifts sideways.
inline Scene backspin(const Params& p, double dt)
{
  const double mu = param(p, "mu", 0.5);
  const double radius = param(p, "R", 0.25);
  const double v0 = param(p, "v0", 4.0);
  const double w0 = param(p, "w0", -200.0);
  const double phi = rad(param(p, "phi", 0.0));
  const Eigen::Vector3d dir(std::cos(phi), std::sin(phi), 0.0);
  const Eigen::Vector3d perp(-dir.y(), dir.x(), 0.0);
  const Eigen::Vector3d spinAxis = Eigen::Vector3d::UnitZ().cross(dir);
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu);
  const Eigen::Vector3d start(0.0, 0.0, radius - kOverlap);
  auto* ball = addFree(
      *s.world,
      "ball",
      std::make_shared<dart::dynamics::SphereShape>(radius),
      1.0,
      pose(start),
      mu,
      mu);
  freeJoint(ball)->setLinearVelocity(v0 * dir);
  freeJoint(ball)->setAngularVelocity(w0 * spinAxis);
  const double slip0 = v0 - radius * w0;
  const double decay = 3.5 * mu * kGravity * dt; // slip lost per step
  const double rollSteps = std::ceil(std::abs(slip0) / decay);
  s.steps = steps(param(p, "T", rollSteps * dt + 0.5), dt);
  auto st = std::make_shared<Eigen::Vector2d>(-1.0, 0.0); // roll step, lateral
  const auto slipOf = [=]() {
    return planar(
        ball->getLinearVelocity()
        + ball->getAngularVelocity().cross(-radius * Eigen::Vector3d::UnitZ()));
  };
  s.postStep = [=](int i) {
    if ((*st)(0) < 0.0 && slipOf().norm() < 0.01 * decay)
      (*st)(0) = i + 1;
    (*st)(1) = std::max(
        (*st)(1), std::abs(planar(position(ball) - start).dot(perp)));
    return true;
  };
  const int n = s.steps;
  s.finish = [=](Metrics& m) {
    const double vRoll = (5.0 * v0 + 2.0 * radius * w0) / 7.0;
    // Box: each world-axis slip component decays by 3.5 mu g h per step.
    Eigen::Vector2d v(v0 * dir.x(), v0 * dir.y());
    Eigen::Vector2d slip(slip0 * dir.x(), slip0 * dir.y());
    Eigen::Vector2d x = Eigen::Vector2d::Zero();
    double boxLateral = 0.0;
    for (int i = 0; i < n; ++i) {
      for (int a = 0; a < 2; ++a) {
        const double ds = std::abs(slip[a]) > decay
                              ? std::copysign(decay, slip[a])
                              : slip[a];
        v[a] -= ds / 3.5;
        slip[a] -= ds;
      }
      x += dt * v;
      boxLateral
          = std::max(boxLateral, std::abs(x.x() * perp.x() + x.y() * perp.y()));
    }
    m["v_roll"] = ball->getLinearVelocity().dot(dir);
    m["pred_v_roll"] = vRoll;
    m["v_roll_err"] = std::abs(m["v_roll"] - vRoll) / std::abs(vRoll);
    if ((*st)(0) >= 0.0)
      m["roll_step"] = (*st)(0);
    m["pred_roll_step"] = rollSteps;
    m["lateral"] = (*st)(1);
    m["pred_box_lateral"] = boxLateral;
  };
  return s;
}

// A8: sphere (shape 0) or cylinder (shape 1, axis Y) rolling at v; DART has no
// rolling resistance, so any speed loss is spurious drag.
inline Scene roll(const Params& p, double dt)
{
  const double mu = param(p, "mu", 1.0);
  const double v0 = param(p, "v", 1.0);
  const bool cylinder = param(p, "shape", 0.0) != 0.0;
  const double radius = 0.25;
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu);
  dart::dynamics::ShapePtr shape;
  Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
  if (cylinder) {
    shape = std::make_shared<dart::dynamics::CylinderShape>(radius, 0.2);
    r = rotation(kPi / 2, Eigen::Vector3d::UnitX());
  } else {
    shape = std::make_shared<dart::dynamics::SphereShape>(radius);
  }
  auto* body = addFree(
      *s.world,
      "roller",
      shape,
      10.0,
      pose({0.0, 0.0, radius - kOverlap}, r),
      mu,
      mu);
  freeJoint(body)->setLinearVelocity(Eigen::Vector3d(v0, 0.0, 0.0));
  freeJoint(body)->setAngularVelocity(Eigen::Vector3d(0.0, v0 / radius, 0.0));
  s.steps = steps(param(p, "T", 10.0), dt);
  // max slip, vertical speed square sum, count
  auto st = std::make_shared<Eigen::Vector3d>(Eigen::Vector3d::Zero());
  s.postStep = [=](int) {
    const Eigen::Vector3d v = body->getLinearVelocity();
    (*st)(0) = std::max(
        (*st)(0), std::abs(v.x() - radius * body->getAngularVelocity().y()));
    (*st)(1) += v.z() * v.z();
    (*st)(2) += 1.0;
    return true;
  };
  s.finish = [=](Metrics& m) {
    const double distance = position(body).x();
    m["speed_loss_per_m"]
        = (v0 - body->getLinearVelocity().x()) / (v0 * distance);
    m["slip_max"] = (*st)(0);
    m["vz_rms"] = std::sqrt((*st)(1) / (*st)(2));
  };
  return s;
}

// A10: 1 kg box on a static belt whose surface velocity is vb at beta in the
// friction frame; sync time vb/(mu g), box per axis (D App. B). The oracle
// compares signed velocities in the contact's friction frame, so a box carried
// the wrong way fails.
inline Scene conveyor(const Params& p, double dt)
{
  const double mu = param(p, "mu", 0.3);
  const double vb = param(p, "vb", 1.0);
  const double beta = rad(param(p, "beta", 0.0));
  const Eigen::Vector2d target(vb * std::cos(beta), vb * std::sin(beta));
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu);
  auto* box = addFree(
      *s.world,
      "box",
      boxShape(0.2, 0.2, 0.2),
      1.0,
      pose({0.0, 0.0, 0.1 - kOverlap}),
      mu,
      mu);
  s.world->getConstraintSolver()->addContactSurfaceHandler(
      std::make_shared<SurfaceVelocityHandler>(
          Eigen::Vector3d(0.0, target.x(), target.y())));
  s.steps = steps(param(p, "T", vb / (mu * kGravity) + 0.5), dt);
  // Velocity of body 1 relative to body 2 at the last step's first contact in
  // DART's default friction frame (n x t, t), t = n x z normalized (n x x for a
  // vertical normal), where a sticking contact reaches the surface velocity.
  auto* world = s.world.get();
  const auto slip = [=] {
    const auto& result = world->getLastCollisionResult();
    if (result.getNumContacts() == 0)
      return Eigen::Vector2d(kNaN, kNaN);
    const auto& c = result.getContact(0);
    Eigen::Vector3d t = c.normal.cross(Eigen::Vector3d::UnitZ());
    if (t.squaredNorm() < 1e-12)
      t = c.normal.cross(Eigen::Vector3d::UnitX());
    t.normalize();
    const double sign = c.getBodyNodePtr1().get() == box ? 1.0 : -1.0;
    const Eigen::Vector3d v = sign * box->getLinearVelocity();
    return Eigen::Vector2d(v.dot(c.normal.cross(t)), v.dot(t));
  };
  auto sync = std::make_shared<double>(-1.0);
  s.postStep = [=](int i) {
    if (*sync < 0.0 && (slip() - target).cwiseAbs().maxCoeff() < 1e-4)
      *sync = (i + 1) * dt;
    return true;
  };
  s.finish = [=](Metrics& m) {
    const double step = mu * kGravity * dt;
    if (*sync >= 0.0)
      m["sync_time"] = *sync;
    m["pred_exact_sync_time"] = std::ceil(vb / step) * dt;
    m["pred_box_sync_time"]
        = std::ceil(target.cwiseAbs().maxCoeff() / step) * dt;
    m["v_err"] = (slip() - target).norm();
  };
  return s;
}

// A11: two prismatic fingers squeeze a 1 kg, 0.1 m box against gravity with
// Fg, the gripper rolled by psi about the squeeze axis. FORCE fingers form a
// contact-only island; servo = 1 uses SERVO fingers limited to Fg (a mixed
// island). fdir = 1 fixes fdir1 to the pads (rotating with psi), which gives
// the box factor; DART's default basis keeps t1 along gravity.
inline Scene grasp(const Params& p, double dt)
{
  const double mu = param(p, "mu", 0.5);
  const double psi = rad(param(p, "psi", 0.0));
  const double fg = param(p, "Fg", 12.0);
  const bool servo = param(p, "servo", 0.0) != 0.0;
  const bool padFdir = param(p, "fdir", 1.0) != 0.0;
  Scene s;
  s.world = makeWorld(dt);
  auto* object = addFree(
      *s.world,
      "object",
      boxShape(0.1, 0.1, 0.1),
      1.0,
      pose(Eigen::Vector3d::Zero()),
      mu,
      mu);
  auto gripper = dart::dynamics::Skeleton::create("gripper");
  auto* base
      = gripper->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  base->getParentJoint()->setTransformFromParentBodyNode(
      pose(Eigen::Vector3d::Zero(), rotation(psi, Eigen::Vector3d::UnitY())));
  std::vector<std::pair<dart::dynamics::Joint*, double>> fingers;
  for (const double side : {-1.0, 1.0}) {
    dart::dynamics::PrismaticJoint::Properties joint;
    joint.mAxis = Eigen::Vector3d::UnitY();
    auto pair
        = base->createChildJointAndBodyNodePair<dart::dynamics::PrismaticJoint>(
            joint);
    pair.first->setTransformFromParentBodyNode(
        pose(Eigen::Vector3d(0.0, side * (0.06 - kOverlap), 0.0)));
    const auto pad = boxShape(0.06, 0.02, 0.06);
    auto* node = addShape(pair.second, pad, mu, mu);
    pair.second->setInertia(dart::dynamics::Inertia(
        0.1, Eigen::Vector3d::Zero(), pad->computeInertia(0.1)));
    if (padFdir) {
      node->getDynamicsAspect()->setFirstFrictionDirection(
          Eigen::Vector3d::UnitZ());
    }
    if (servo) {
      pair.first->setActuatorType(dart::dynamics::Joint::SERVO);
      pair.first->setForceUpperLimit(0, fg);
      pair.first->setForceLowerLimit(0, -fg);
    }
    fingers.emplace_back(pair.first, side);
  }
  s.world->addSkeleton(gripper);
  s.preStep = [=](int) {
    for (const auto& [joint, side] : fingers)
      joint->setCommand(0, servo ? -side * 0.05 : -side * fg);
  };
  const double time = param(p, "T", 0.5);
  s.steps = steps(time, dt);
  s.finish = [=](Metrics& m) {
    const double drop = -position(object).z();
    const double factor
        = padFdir ? std::max(std::abs(std::cos(psi)), std::abs(std::sin(psi)))
                  : 1.0;
    m["held"] = drop < 2e-3;
    m["drop"] = drop;
    m["pred_exact_Fg"] = kGravity / (2.0 * mu);
    m["pred_box_Fg"] = factor * kGravity / (2.0 * mu);
  };
  return s;
}

// A13: gz-physics slip-compliance replica: 1 kg box, mu 1, F pushes along the
// first (dir 0, world X) or second (dir 1, world Y) friction axis; steady v =
// slip F within gz's 1e-4.
inline Scene slipCompliance(const Params& p, double dt)
{
  const double slip = param(p, "slip", 0.1);
  const double force = param(p, "F", 1.0);
  const bool secondary = param(p, "dir", 0.0) != 0.0;
  const double mu = param(p, "mu", 1.0);
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu);
  auto* box = addFree(
      *s.world,
      "box",
      boxShape(0.2, 0.2, 0.2),
      1.0,
      pose({0.0, 0.0, 0.1 - kOverlap}),
      mu,
      mu);
  auto* dynamics = box->getShapeNode(0)->getDynamicsAspect();
  dynamics->setPrimarySlipCompliance(secondary ? 0.0 : slip);
  dynamics->setSecondarySlipCompliance(secondary ? slip : 0.0);
  const Eigen::Vector3d dir
      = secondary ? Eigen::Vector3d::UnitY() : Eigen::Vector3d::UnitX();
  s.preStep = [=](int) {
    box->addExtForce(force * dir);
  };
  s.steps = steps(param(p, "T", 1.0 + 8.0 * slip), dt);
  auto sum = std::make_shared<Eigen::Vector2d>(Eigen::Vector2d::Zero());
  const int tail = std::max(1, s.steps / 10);
  const int n = s.steps;
  s.postStep = [=](int i) {
    if (i >= n - tail)
      *sum += Eigen::Vector2d(box->getLinearVelocity().dot(dir), 1.0);
    return true;
  };
  s.finish = [=](Metrics& m) {
    m["v_steady"] = (*sum)(0) / (*sum)(1);
    m["pred_v"] = slip * force;
    m["v_err"] = std::abs(m["v_steady"] - slip * force);
  };
  return s;
}

// C1: Painleve tipping box (w .3 along X, d 1.2, h .6) launched at v0 along +X,
// a basis axis, so the box law is exact here. It tips iff mu > w/h; while it
// slides untipped the front normal share is (1 + mu h/w)/2.
inline Scene painleve(const Params& p, double dt)
{
  const double mu = param(p, "mu", 0.4);
  const double v0 = param(p, "v0", 4.0);
  const double w = 0.3, d = 1.2, h = 0.6;
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu);
  auto* box = addFree(
      *s.world,
      "box",
      boxShape(w, d, h),
      1000.0 * w * d * h,
      pose({0.0, 0.0, h / 2 - kOverlap}),
      mu,
      mu);
  freeJoint(box)->setLinearVelocity(Eigen::Vector3d(v0, 0.0, 0.0));
  s.steps = steps(param(p, "T", std::min(3.0, v0 / (mu * kGravity) + 0.5)), dt);
  auto* world = s.world.get();
  // max pitch, front share sum, count
  auto st = std::make_shared<Eigen::Vector3d>(Eigen::Vector3d::Zero());
  s.postStep = [=](int) {
    const Eigen::Vector3d up = box->getWorldTransform().linear().col(2);
    const double pitch = deg(std::acos(std::clamp(up.z(), -1.0, 1.0)));
    (*st)(0) = std::max((*st)(0), pitch);
    if (box->getLinearVelocity().x() > 0.5 && pitch < 1.0) {
      double front = 0.0;
      double total = 0.0;
      const auto& result = world->getLastCollisionResult();
      for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
        const auto& c = result.getContact(i);
        const double normal = std::abs(c.force.dot(c.normal));
        total += normal;
        if (c.point.x() > position(box).x())
          front += normal;
      }
      if (total > 0.0)
        *st += Eigen::Vector3d(0.0, front / total, 1.0);
    }
    return true;
  };
  s.finish = [=](Metrics& m) {
    const double share = (1.0 + mu * h / w) / 2.0;
    m["max_pitch_deg"] = (*st)(0);
    m["tipped"] = (*st)(0) > 5.0;
    m["pred_tips"] = mu > w / h;
    if ((*st)(2) > 0.0)
      m["front_share"] = (*st)(1) / (*st)(2);
    if (share < 1.0)
      m["pred_front_share"] = share;
  };
  return s;
}

// C2: plank (2 m x 0.4 x 0.04) between the floor (mu_f) and a fixed wall
// (mu_w) at alpha to the floor. The threshold is the angle where the limiting
// forces balance the moment about the center (thin limit: tan alpha* =
// (1 - mu_f mu_w)/(2 mu_f)), computed here for the actual corner contacts.
inline Scene ladder(const Params& p, double dt)
{
  const double muF = param(p, "muf", 0.5);
  const double muW = param(p, "muw", 0.0);
  const double alpha = rad(param(p, "alpha", 45.0));
  const double length = 2.0, thick = 0.04, width = 0.4;
  // Corner offsets from the center for a plank at angle a: the lowest bottom
  // corner (floor contact) and the wall-most top corner (wall contact at x=0).
  const auto corners = [=](double a) {
    const Eigen::Matrix3d r = rotation(a - kPi / 2, Eigen::Vector3d::UnitY());
    Eigen::Vector3d floor(0, 0, 1e300);
    Eigen::Vector3d wall(1e300, 0, 0);
    for (const double sx : {-1.0, 1.0}) {
      const Eigen::Vector3d bottom
          = r * Eigen::Vector3d(sx * thick / 2, 0.0, -length / 2);
      const Eigen::Vector3d top
          = r * Eigen::Vector3d(sx * thick / 2, 0.0, length / 2);
      if (bottom.z() < floor.z())
        floor = bottom;
      if (top.x() < wall.x())
        wall = top;
    }
    return std::make_pair(floor, wall);
  };
  // Net moment about the center under limiting friction (Nf = 1).
  const auto moment = [=](double a) {
    const auto [f, w] = corners(a);
    const double nw = muF;
    return f.z() * (-muF) - f.x() * 1.0 + w.z() * nw - w.x() * (muW * nw);
  };
  double lo = rad(5.0), hi = rad(85.0);
  for (int i = 0; i < 60; ++i) {
    const double mid = 0.5 * (lo + hi);
    (moment(mid) > 0.0) == (moment(hi) > 0.0) ? hi = mid : lo = mid;
  }
  const double critical = 0.5 * (lo + hi);
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, muF, muF);
  addStatic(
      *s.world,
      "wall",
      boxShape(0.2, 4.0, 4.0),
      pose({-0.1, 0.0, 2.0}),
      muW,
      muW);
  const auto [f, w] = corners(alpha);
  const Eigen::Vector3d center(-w.x() - kOverlap, 0.0, -f.z() - kOverlap);
  auto* plank = addFree(
      *s.world,
      "plank",
      boxShape(thick, width, length),
      1000.0 * thick * width * length,
      pose(center, rotation(alpha - kPi / 2, Eigen::Vector3d::UnitY())),
      std::max(muF, muW),
      std::max(muF, muW));
  s.steps = steps(param(p, "T", 2.0), dt);
  s.finish = [=](Metrics& m) {
    m["slid"] = (position(plank) - center).norm() > 0.02;
    m["pred_alpha_deg"] = deg(critical);
    m["pred_alpha_thin_deg"] = deg(std::atan((1.0 - muF * muW) / (2.0 * muF)));
  };
  return s;
}

// C5: rod (1 m) at tan(theta) = 2 to the floor whose lower end slides away
// from the top; mu > 4/3 is Painleve's paradox region (robustness only).
inline Scene rod(const Params& p, double dt)
{
  const double mu = param(p, "mu", 0.5);
  const double v0 = param(p, "v0", 1.0);
  const double theta = std::atan(2.0);
  const double length = 1.0, side = 0.05;
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu);
  // Rod axis from the bottom to the top: (-cos theta, 0, sin theta).
  const Eigen::Matrix3d r = rotation(theta - kPi / 2, Eigen::Vector3d::UnitY());
  const double lowest = std::min(
      (r * Eigen::Vector3d(side / 2, 0, -length / 2)).z(),
      (r * Eigen::Vector3d(-side / 2, 0, -length / 2)).z());
  auto* body = addFree(
      *s.world,
      "rod",
      boxShape(side, side, length),
      1000.0 * side * side * length,
      pose({0.0, 0.0, -lowest - kOverlap}, r),
      mu,
      mu);
  freeJoint(body)->setLinearVelocity(Eigen::Vector3d(v0, 0.0, 0.0));
  s.steps = steps(param(p, "T", 1.0), dt);
  s.finish = [=](Metrics& m) {
    m["final_height"] = position(body).z();
    m["final_speed"] = body->getLinearVelocity().norm();
  };
  return s;
}

#if FRICTION_EVAL_DART620
// C4/R6: semicircular arch (R 1, t/R 0.15) of n wedge voussoirs standing on
// the ground. mu* = 0.3657 for t/R = 0.15 (D's thrust-line search; it does not
// depend on n there): below it the arch must collapse.
inline Scene arch(const Params& p, double dt)
{
  const int n = static_cast<int>(param(p, "n", 10.0));
  const double mu = param(p, "mu", 0.8);
  const double radius = 1.0, thick = 0.15, depth = 0.3;
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu);
  auto starts = std::make_shared<std::vector<Eigen::Vector3d>>();
  std::vector<dart::dynamics::BodyNode*> stones;
  for (int k = 0; k < n; ++k) {
    std::vector<Eigen::Vector3d> vertices;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const double a : {k * kPi / n, (k + 1) * kPi / n}) {
      for (const double r : {radius - thick / 2, radius + thick / 2}) {
        for (const double y : {-depth / 2, depth / 2}) {
          vertices.emplace_back(r * std::cos(a), y, r * std::sin(a));
          center += vertices.back() / 8.0;
        }
      }
    }
    auto mesh = std::make_shared<dart::math::TriMeshd>();
    for (const auto& v : vertices)
      mesh->addVertex(v - center);
    const double area = 0.5 * (kPi / n) * radius * 2.0 * thick;
    stones.push_back(addFree(
        *s.world,
        "voussoir" + std::to_string(k),
        dart::dynamics::ConvexMeshShape::fromMesh(mesh),
        1000.0 * area * depth,
        pose(center),
        mu,
        mu));
    starts->push_back(center);
  }
  s.steps = steps(param(p, "T", 3.0), dt);
  auto collapse = std::make_shared<Eigen::Vector2d>(0.0, kNaN); // max, time
  s.postStep = [=](int i) {
    for (std::size_t k = 0; k < stones.size(); ++k) {
      const double d = (position(stones[k]) - (*starts)[k]).norm();
      (*collapse)(0) = std::max((*collapse)(0), d);
    }
    if (std::isnan((*collapse)(1)) && (*collapse)(0) > thick / 3)
      (*collapse)(1) = (i + 1) * dt;
    return true;
  };
  s.finish = [=](Metrics& m) {
    m["max_disp"] = (*collapse)(0);
    m["collapsed"] = (*collapse)(0) > thick / 3;
    if (!std::isnan((*collapse)(1)))
      m["collapse_time"] = (*collapse)(1);
    m["pred_mu_star"] = 0.3657;
  };
  return s;
}
#endif

//==============================================================================
// L3 robustness and P1 performance scenes
//==============================================================================

// Shared observer for many-body scenes: maximum displacement from the start.
inline std::function<void(Metrics&)> displacementReport(
    std::vector<dart::dynamics::BodyNode*> bodies, double threshold)
{
  std::vector<Eigen::Vector3d> starts;
  for (const auto* body : bodies)
    starts.push_back(position(body));
  return [=](Metrics& m) {
    double maxDisp = 0.0;
    double moved = 0.0;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
      const double d = (position(bodies[i]) - starts[i]).norm();
      maxDisp = std::max(maxDisp, d);
      moved += d > threshold;
    }
    m["max_disp"] = maxDisp;
    m["moved"] = moved;
  };
}

// R1: vertical stack of n 0.2 m boxes (8 kg each). R3 (ratio != 1): a heavy
// box of ratio x the mass on a light 1 kg box.
inline Scene stack(const Params& p, double dt)
{
  const int n = static_cast<int>(param(p, "n", 5.0));
  const double ratio = param(p, "ratio", 0.0);
  const double mu = param(p, "mu", 0.5);
  const double size = 0.2;
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu);
  std::vector<dart::dynamics::BodyNode*> boxes;
  const int count = ratio > 0.0 ? 2 : n;
  for (int i = 0; i < count; ++i) {
    const double mass = ratio > 0.0 ? (i == 0 ? 1.0 : ratio) : 8.0;
    boxes.push_back(addFree(
        *s.world,
        "box" + std::to_string(i),
        boxShape(size, size, size),
        mass,
        pose({0.0, 0.0, (i + 0.5) * size - (i + 1) * kOverlap}),
        mu,
        mu));
  }
  s.steps = steps(param(p, "T", 10.0), dt);
  auto* top = boxes.back();
  const Eigen::Vector3d topStart = position(top);
  auto rest = std::make_shared<double>(kNaN);
  auto* world = s.world.get();
  s.postStep = [=](int i) {
    if (!std::isnan(*rest))
      return true;
    for (std::size_t k = 0; k < world->getNumSkeletons(); ++k) {
      const auto skeleton = world->getSkeleton(k);
      if (skeleton->isMobile() && !skeleton->isResting())
        return true;
    }
    *rest = (i + 1) * dt;
    return true;
  };
  const auto report = displacementReport(boxes, 0.01);
  s.finish = [=](Metrics& m) {
    report(m);
    m["top_drift"] = planar(position(top) - topStart).norm();
    m["top_sink"] = topStart.z() - position(top).z();
    double speed = 0.0;
    for (const auto* box : boxes)
      speed = std::max(speed, box->getLinearVelocity().norm());
    m["speed_end"] = speed;
    if (!std::isnan(*rest))
      m["rest_time"] = *rest;
  };
  return s;
}

// R2: 2-D pyramid of rows x 0.2 m boxes with 0.02 m gaps (each box rests on
// two below).
inline Scene pyramid(const Params& p, double dt)
{
  const int rows = static_cast<int>(param(p, "rows", 10.0));
  const double mu = param(p, "mu", 0.5);
  const double size = 0.2, gap = 0.02;
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu);
  std::vector<dart::dynamics::BodyNode*> boxes;
  for (int row = 0; row < rows; ++row) {
    for (int i = 0; i < rows - row; ++i) {
      const double x = (i - 0.5 * (rows - row - 1)) * (size + gap);
      boxes.push_back(addFree(
          *s.world,
          "box" + std::to_string(row) + "_" + std::to_string(i),
          boxShape(size, size, size),
          8.0,
          pose({x, 0.0, (row + 0.5) * size - (row + 1) * kOverlap}),
          mu,
          mu));
    }
  }
  s.steps = steps(param(p, "T", 5.0), dt);
  s.finish = displacementReport(boxes, size / 2);
  return s;
}

// R5: four-level house of 26 cards (#3377 geometry: 0.03 x 0.45 x 1.0 cards at
// 0.23 rad, 3 mm initial overlap); proj = 1 fires a sphere at it after 1 s.
inline Scene cardHouse(const Params& p, double dt)
{
  const double mu = param(p, "mu", 0.8);
  const double angle = 0.23, thick = 0.03, width = 0.45, height = 1.0;
  const double overlap = 0.003, spacing = height - overlap;
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu);
  std::vector<dart::dynamics::BodyNode*> cards;
  const auto addCard = [&](const Eigen::Isometry3d& tf) {
    cards.push_back(addFree(
        *s.world,
        "card" + std::to_string(cards.size()),
        boxShape(thick, width, height),
        1000.0 * thick * width * height,
        tf,
        mu,
        mu));
  };
  const auto halfExtentZ = [&](const Eigen::Matrix3d& r) {
    return 0.5
           * (std::abs(r(2, 0)) * thick + std::abs(r(2, 1)) * width
              + std::abs(r(2, 2)) * height);
  };
  const double halfX
      = 0.5 * (height * std::sin(angle) + thick * std::cos(angle));
  const double frameHeight
      = 2.0 * halfExtentZ(rotation(angle, Eigen::Vector3d::UnitY()));
  double base = 0.0;
  for (int level = 0; level < 4; ++level) {
    const int frames = 4 - level;
    for (int f = 0; f < frames; ++f) {
      const double x = (f - 0.5 * (frames - 1)) * spacing;
      for (const double side : {-1.0, 1.0}) {
        const Eigen::Matrix3d r
            = rotation(-side * angle, Eigen::Vector3d::UnitY());
        addCard(pose(
            {x + side * (halfX - 0.5 * overlap),
             0.0,
             base + halfExtentZ(r) - overlap},
            r));
      }
    }
    if (level == 3)
      break;
    const double supportBase = base + frameHeight - overlap;
    for (int f = 0; f < frames - 1; ++f) {
      addCard(pose(
          {(f + 0.5 - 0.5 * (frames - 1)) * spacing,
           0.0,
           supportBase + 0.5 * thick - overlap},
          rotation(kPi / 2, Eigen::Vector3d::UnitY())));
    }
    base = supportBase + thick - overlap;
  }
  const auto report = displacementReport(cards, 0.2);
  dart::dynamics::BodyNode* ball = nullptr;
  if (param(p, "proj", 0.0) != 0.0) {
    ball = addFree(
        *s.world,
        "projectile",
        std::make_shared<dart::dynamics::SphereShape>(0.1),
        2.0,
        pose({-4.0, 0.0, 0.1 - kOverlap}),
        mu,
        mu);
  }
  const int launch = steps(1.0, dt);
  auto standing = std::make_shared<double>(kNaN);
  s.preStep = [=](int i) {
    if (ball && i == launch)
      freeJoint(ball)->setLinearVelocity(Eigen::Vector3d(6.0, 0.0, 0.0));
  };
  s.postStep = [=](int i) {
    if (i + 1 == launch) {
      Metrics m;
      report(m);
      *standing = m["max_disp"] < 0.05;
    }
    return true;
  };
  s.steps = steps(param(p, "T", 2.0), dt);
  s.finish = [=](Metrics& m) {
    report(m);
    if (!std::isnan(*standing))
      m["standing_1s"] = *standing;
  };
  return s;
}

// R9: differential-drive vehicle (sphere wheels on SERVO revolute joints,
// frictionless casters) turning with wheel speeds wl (y = +0.2) and wr.
inline Scene diffDrive(const Params& p, double dt)
{
  const double mu = param(p, "mu", 1.0);
  const double wl = param(p, "wl", 5.0);
  const double wr = param(p, "wr", 10.0);
  const double radius = 0.1, track = 0.4;
  Scene s;
  s.world = makeWorld(dt);
  addGround(*s.world, mu, mu);
  auto robot = dart::dynamics::Skeleton::create("robot");
  auto* chassis
      = robot->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;
  const auto body = boxShape(0.4, 0.3, 0.06);
  addShape(chassis, body, 0.0, 0.0);
  chassis->setInertia(dart::dynamics::Inertia(
      5.0, Eigen::Vector3d::Zero(), body->computeInertia(5.0)));
  for (const double x : {-0.15, 0.15}) {
    auto* caster = addShape(
        chassis, std::make_shared<dart::dynamics::SphereShape>(0.05), 0.0, 0.0);
    caster->setRelativeTranslation(Eigen::Vector3d(x, 0.0, -0.05));
  }
  std::vector<std::pair<dart::dynamics::Joint*, double>> wheels;
  for (const double side : {1.0, -1.0}) {
    dart::dynamics::RevoluteJoint::Properties joint;
    joint.mAxis = Eigen::Vector3d::UnitY();
    joint.mT_ParentBodyToJoint.translation()
        = Eigen::Vector3d(0.0, side * track / 2, 0.0);
    auto pair
        = chassis
              ->createChildJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
                  joint);
    const auto wheel = std::make_shared<dart::dynamics::SphereShape>(radius);
    addShape(pair.second, wheel, mu, mu);
    pair.second->setInertia(dart::dynamics::Inertia(
        0.5, Eigen::Vector3d::Zero(), wheel->computeInertia(0.5)));
    pair.first->setActuatorType(dart::dynamics::Joint::SERVO);
    pair.first->setForceUpperLimit(0, 20.0);
    pair.first->setForceLowerLimit(0, -20.0);
    wheels.emplace_back(pair.first, side > 0.0 ? wl : wr);
  }
  dart::dynamics::FreeJoint::setTransformOf(
      chassis, pose({0.0, 0.0, radius - kOverlap}));
  s.world->addSkeleton(robot);
  s.preStep = [=](int) {
    for (const auto& [joint, speed] : wheels)
      joint->setCommand(0, speed);
  };
  const double time = param(p, "T", 5.0);
  s.steps = steps(time, dt);
  const int settle = steps(2.0, dt);
  // last yaw, unwrapped yaw, unwrapped yaw after 2 s
  auto yaw = std::make_shared<Eigen::Vector3d>(0.0, 0.0, kNaN);
  s.postStep = [=](int i) {
    const Eigen::Matrix3d r = chassis->getWorldTransform().linear();
    const double y = std::atan2(r(1, 0), r(0, 0));
    double delta = y - (*yaw)(0);
    delta -= 2.0 * kPi * std::round(delta / (2.0 * kPi));
    (*yaw)(1) += delta;
    (*yaw)(0) = y;
    if (i + 1 == settle)
      (*yaw)(2) = (*yaw)(1);
    return true;
  };
  s.finish = [=](Metrics& m) {
    // The yaw rate is measured after the 2 s spin-up.
    if (!std::isnan((*yaw)(2)))
      m["yaw_rate"] = ((*yaw)(1) - (*yaw)(2)) / (time - 2.0);
    const double speed = planar(chassis->getLinearVelocity()).norm();
    m["pred_yaw_rate"] = radius * (wr - wl) / track;
    m["speed"] = speed;
    m["pred_speed"] = radius * (wl + wr) / 2.0;
  };
  return s;
}

// P1 S4/S5: contact_benchmark --generate-objects n: three lanes (sphere 0.5,
// unit box, cylinder 0.5 x 1) of unit-mass objects at 1.1 m spacing on a thin
// floor, default friction.
inline Scene generated(const Params& p, double dt)
{
  const int n = static_cast<int>(param(p, "n", 90.0));
  const double spacing = 1.1;
  const int rows = (n + 2) / 3;
  Scene s;
  s.world = makeWorld(dt);
  addStatic(
      *s.world,
      "ground",
      boxShape(
          std::max(10.0, (rows + 2.0) * spacing),
          std::max(10.0, 4.0 * spacing),
          0.002),
      pose({0.5 * (rows - 1) * spacing, 0.0, -0.001}),
      1.0,
      1.0);
  for (int i = 0; i < n; ++i) {
    const int lane = i % 3;
    dart::dynamics::ShapePtr shape;
    if (lane == 0)
      shape = std::make_shared<dart::dynamics::SphereShape>(0.5);
    else if (lane == 1)
      shape = boxShape(1.0, 1.0, 1.0);
    else
      shape = std::make_shared<dart::dynamics::CylinderShape>(0.5, 1.0);
    addFree(
        *s.world,
        "object" + std::to_string(i),
        shape,
        1.0,
        pose({(i / 3) * spacing, (lane - 1.0) * spacing, 0.5}),
        1.0,
        1.0);
  }
  s.steps = steps(param(p, "T", 0.3), dt);
  auto* world = s.world.get();
  s.finish = [=](Metrics& m) {
    double resting = 0.0;
    for (std::size_t k = 0; k < world->getNumSkeletons(); ++k)
      resting += world->getSkeleton(k)->isResting();
    m["resting"] = resting;
  };
  return s;
}

using SceneFactory = std::function<Scene(const Params&, double)>;

/// Scene registry; a scene's defaults can be overridden by any Params entry.
inline const std::map<std::string, SceneFactory>& scenes()
{
  const auto with = [](SceneFactory f, Params defaults) -> SceneFactory {
    return [=](const Params& p, double dt) {
      Params merged = defaults;
      for (const auto& [key, value] : p)
        merged[key] = value;
      return f(merged, dt);
    };
  };
  static const std::map<std::string, SceneFactory> registry
      = { {"A1", incline},
          {"A2", with(incline, {{"phi", 45.0}, {"mu", 0.36}})},
          {"A3", tilt},
          {"A4", push},
          {"A5", slide},
          {"A6", spin},
          {"A7", backspin},
          {"A8", roll},
          {"A10", conveyor},
          {"A11", grasp},
          {"A12", with(push, {{"mu", 1.0}, {"mu2", 0.5}, {"fdir", 0.0}})},
          {"A13", slipCompliance},
          {"C1", painleve},
          {"C2", ladder},
#if FRICTION_EVAL_DART620
          {"C4", arch},
          {"R6", with(arch, {{"n", 101.0}, {"mu", 0.8}})},
#endif
          {"C5", rod},
          {"R1", stack},
          {"R2", pyramid},
          {"R3", with(stack, {{"ratio", 100.0}})},
          {"R5", cardHouse},
          {"R9", diffDrive},
          {"P1", generated},
        };
  return registry;
}

/// Runs a scene to completion and returns its metrics.
inline Metrics run(Scene& scene)
{
  for (int i = 0; i < scene.steps; ++i) {
    if (scene.preStep)
      scene.preStep(i);
    scene.world->step();
    if (scene.postStep && !scene.postStep(i))
      break;
  }
  Metrics metrics;
  if (scene.finish)
    scene.finish(metrics);
  return metrics;
}

} // namespace friction_eval

#endif // DART_TOOLS_FRICTION_EVAL_FRICTION_SCENES_HPP_
