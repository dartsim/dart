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

#include "dart/collision/CollisionResult.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/common/Console.hpp"
#include "dart/common/Macros.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Inertia.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/SkelParser.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include <cmath>
#include <cstdlib>

using namespace std;
using namespace Eigen;
using namespace dart;

namespace {

struct SoftStabilityScene
{
  std::string uri;
  std::size_t minSoftBodies;
  std::size_t minPointMasses;
  std::size_t steps;
};

struct SoftSceneStats
{
  std::size_t softBodies = 0u;
  std::size_t pointMasses = 0u;
};

struct SoftStateSnapshot
{
  std::vector<double> values;
  std::size_t dofs = 0u;
  std::size_t softBodies = 0u;
  std::size_t pointMasses = 0u;
};

struct ContactMetrics
{
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  Eigen::Vector3d centerOfPressure = Eigen::Vector3d::Zero();
  std::size_t contacts = 0u;
  bool hasCenterOfPressure = false;
};

constexpr double kMaxPointMassPositionNorm = 1.0e4;
constexpr double kMaxPointMassVelocityNorm = 1.0e6;
constexpr double kChecksumRelativeTolerance = 1.0e-12;

template <class Derived>
void expectFinite(
    const Eigen::MatrixBase<Derived>& value, const std::string& context)
{
  EXPECT_FALSE(math::isNan(value)) << context << " contains NaN";
  EXPECT_FALSE(math::isInf(value)) << context << " contains Inf";
}

void expectFinite(double value, const std::string& context)
{
  EXPECT_TRUE(std::isfinite(value)) << context << " = " << value;
}

SoftSceneStats collectSoftSceneStats(const simulation::WorldPtr& world)
{
  SoftSceneStats stats;
  if (!world)
    return stats;

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const dynamics::SkeletonPtr skeleton = world->getSkeleton(i);
    if (!skeleton)
      continue;

    stats.softBodies += skeleton->getNumSoftBodyNodes();
    for (std::size_t j = 0; j < skeleton->getNumSoftBodyNodes(); ++j) {
      const dynamics::SoftBodyNode* softBody = skeleton->getSoftBodyNode(j);
      if (softBody != nullptr)
        stats.pointMasses += softBody->getNumPointMasses();
    }
  }

  return stats;
}

template <class Derived>
void appendStateValues(
    const Eigen::MatrixBase<Derived>& value, std::vector<double>& values)
{
  values.reserve(values.size() + static_cast<std::size_t>(value.size()));
  for (Eigen::Index i = 0; i < value.size(); ++i)
    values.push_back(value(i));
}

SoftStateSnapshot computeSoftStateSnapshot(const simulation::WorldPtr& world)
{
  SoftStateSnapshot snapshot;
  if (!world)
    return snapshot;

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const dynamics::SkeletonPtr skeleton = world->getSkeleton(i);
    if (!skeleton)
      continue;

    const Eigen::VectorXd positions = skeleton->getPositions();
    const Eigen::VectorXd velocities = skeleton->getVelocities();
    appendStateValues(positions, snapshot.values);
    appendStateValues(velocities, snapshot.values);
    snapshot.dofs += static_cast<std::size_t>(positions.size());

    snapshot.softBodies += skeleton->getNumSoftBodyNodes();
    for (std::size_t j = 0; j < skeleton->getNumSoftBodyNodes(); ++j) {
      const dynamics::SoftBodyNode* softBody = skeleton->getSoftBodyNode(j);
      if (softBody == nullptr)
        continue;

      for (std::size_t k = 0; k < softBody->getNumPointMasses(); ++k) {
        const dynamics::PointMass* pointMass = softBody->getPointMass(k);
        if (pointMass == nullptr)
          continue;

        appendStateValues(pointMass->getPositions(), snapshot.values);
        appendStateValues(pointMass->getVelocities(), snapshot.values);
        appendStateValues(pointMass->getWorldPosition(), snapshot.values);
        ++snapshot.pointMasses;
      }
    }
  }

  return snapshot;
}

void expectNearSnapshotValue(double lhs, double rhs, const std::string& context)
{
  const double scale = std::max({1.0, std::abs(lhs), std::abs(rhs)});
  EXPECT_NEAR(lhs, rhs, kChecksumRelativeTolerance * scale) << context;
}

void expectSnapshotsNear(
    const SoftStateSnapshot& lhs,
    const SoftStateSnapshot& rhs,
    const std::string& context)
{
  EXPECT_EQ(lhs.dofs, rhs.dofs) << context;
  EXPECT_EQ(lhs.softBodies, rhs.softBodies) << context;
  EXPECT_EQ(lhs.pointMasses, rhs.pointMasses) << context;
  ASSERT_EQ(lhs.values.size(), rhs.values.size()) << context;

  for (std::size_t i = 0; i < lhs.values.size(); ++i) {
    expectNearSnapshotValue(
        lhs.values[i],
        rhs.values[i],
        context + " state[" + std::to_string(i) + "]");
  }
}

void expectVectorNear(
    const Eigen::VectorXd& lhs,
    const Eigen::VectorXd& rhs,
    double tolerance,
    const std::string& context)
{
  ASSERT_EQ(lhs.size(), rhs.size()) << context;
  for (Eigen::Index i = 0; i < lhs.size(); ++i)
    EXPECT_NEAR(lhs[i], rhs[i], tolerance) << context << " index=" << i;
}

void expectMatrixNear(
    const Eigen::MatrixXd& lhs,
    const Eigen::MatrixXd& rhs,
    double tolerance,
    const std::string& context)
{
  ASSERT_EQ(lhs.rows(), rhs.rows()) << context;
  ASSERT_EQ(lhs.cols(), rhs.cols()) << context;
  for (Eigen::Index row = 0; row < lhs.rows(); ++row) {
    for (Eigen::Index col = 0; col < lhs.cols(); ++col) {
      EXPECT_NEAR(lhs(row, col), rhs(row, col), tolerance)
          << context << " row=" << row << " col=" << col;
    }
  }
}

Eigen::VectorXd projectPointMassGravity(
    const dynamics::SoftBodyNode* softBody, const Eigen::Vector3d& gravity)
{
  DART_ASSERT(softBody != nullptr);
  Eigen::Vector6d pointMassGravityWrench = Eigen::Vector6d::Zero();

  if (softBody->getGravityMode()) {
    const Eigen::Vector3d localGravity
        = softBody->getWorldTransform().linear().transpose() * gravity;

    for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
      const dynamics::PointMass* pointMass = softBody->getPointMass(i);
      DART_ASSERT(pointMass != nullptr);

      const Eigen::Vector3d pointForce = pointMass->getMass() * localGravity;
      pointMassGravityWrench.head<3>().noalias()
          += pointMass->getLocalPosition().cross(pointForce);
      pointMassGravityWrench.tail<3>().noalias() += pointForce;
    }
  }

  return -(softBody->getJacobian().transpose() * pointMassGravityWrench);
}

Eigen::VectorXd projectPointMassExternalForces(
    const dynamics::SkeletonPtr& skeleton,
    const dynamics::SoftBodyNode* softBody,
    const std::vector<Eigen::Vector3d>& localForces)
{
  DART_ASSERT(skeleton != nullptr);
  DART_ASSERT(softBody != nullptr);
  DART_ASSERT(localForces.size() == softBody->getNumPointMasses());

  Eigen::Vector6d pointMassExternalWrench = Eigen::Vector6d::Zero();
  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    const dynamics::PointMass* pointMass = softBody->getPointMass(i);
    DART_ASSERT(pointMass != nullptr);

    pointMassExternalWrench.head<3>().noalias()
        += pointMass->getLocalPosition().cross(localForces[i]);
    pointMassExternalWrench.tail<3>().noalias() += localForces[i];
  }

  const Eigen::VectorXd localProjection
      = softBody->getJacobian().transpose() * pointMassExternalWrench;
  const std::vector<std::size_t>& indices
      = softBody->getDependentGenCoordIndices();
  DART_ASSERT(
      localProjection.size() == static_cast<Eigen::Index>(indices.size()));

  Eigen::VectorXd projection = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  for (std::size_t i = 0; i < indices.size(); ++i)
    projection[indices[i]] = localProjection[static_cast<Eigen::Index>(i)];

  return projection;
}

Eigen::MatrixXd projectPointMassMassMatrix(
    const dynamics::SkeletonPtr& skeleton,
    const dynamics::SoftBodyNode* softBody)
{
  DART_ASSERT(skeleton != nullptr);
  DART_ASSERT(softBody != nullptr);

  const std::vector<std::size_t>& indices
      = softBody->getDependentGenCoordIndices();
  Eigen::MatrixXd localContribution
      = Eigen::MatrixXd::Zero(indices.size(), indices.size());

  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    const dynamics::PointMass* pointMass = softBody->getPointMass(i);
    DART_ASSERT(pointMass != nullptr);

    Eigen::Isometry3d pointTransform = Eigen::Isometry3d::Identity();
    pointTransform.translation() = pointMass->getLocalPosition();
    const Eigen::Matrix<double, 3, Eigen::Dynamic> pointJacobian
        = math::AdInvTJac(pointTransform, softBody->getJacobian())
              .bottomRows<3>();

    localContribution.noalias()
        += pointMass->getMass() * pointJacobian.transpose() * pointJacobian;
  }

  Eigen::MatrixXd contribution
      = Eigen::MatrixXd::Zero(skeleton->getNumDofs(), skeleton->getNumDofs());
  for (std::size_t row = 0; row < indices.size(); ++row) {
    for (std::size_t col = 0; col < indices.size(); ++col)
      contribution(indices[row], indices[col]) = localContribution(row, col);
  }

  return contribution;
}

void expectFiniteSoftBodyState(
    const dynamics::SoftBodyNode* softBody, const std::string& context)
{
  ASSERT_TRUE(softBody != nullptr) << context;

  expectFinite(softBody->getVertexSpringStiffness(), context + " vertex k");
  expectFinite(softBody->getEdgeSpringStiffness(), context + " edge k");
  expectFinite(softBody->getDampingCoefficient(), context + " damping");
  EXPECT_GE(softBody->getVertexSpringStiffness(), 0.0) << context;
  EXPECT_GE(softBody->getEdgeSpringStiffness(), 0.0) << context;
  EXPECT_GE(softBody->getDampingCoefficient(), 0.0) << context;

  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    const dynamics::PointMass* pointMass = softBody->getPointMass(i);
    const std::string pointContext
        = context + " point_mass=" + std::to_string(i);
    ASSERT_TRUE(pointMass != nullptr) << pointContext;

    expectFinite(pointMass->getPositions(), pointContext + " positions");
    expectFinite(pointMass->getVelocities(), pointContext + " velocities");
    expectFinite(
        pointMass->getAccelerations(), pointContext + " accelerations");
    expectFinite(pointMass->getForces(), pointContext + " forces");
    expectFinite(pointMass->getRestingPosition(), pointContext + " rest");
    expectFinite(pointMass->getLocalPosition(), pointContext + " local");
    expectFinite(pointMass->getWorldPosition(), pointContext + " world");
    expectFinite(pointMass->getBodyVelocity(), pointContext + " body velocity");
    expectFinite(
        pointMass->getWorldVelocity(), pointContext + " world velocity");
    expectFinite(
        pointMass->getBodyAcceleration(), pointContext + " body acceleration");
    expectFinite(
        pointMass->getWorldAcceleration(),
        pointContext + " world acceleration");

    EXPECT_LT(pointMass->getWorldPosition().norm(), kMaxPointMassPositionNorm)
        << pointContext;
    EXPECT_LT(pointMass->getWorldVelocity().norm(), kMaxPointMassVelocityNorm)
        << pointContext;
  }
}

void expectFiniteWorldState(
    const simulation::WorldPtr& world,
    const SoftStabilityScene& scene,
    const std::string& detectorName,
    std::size_t threads,
    std::size_t step)
{
  ASSERT_TRUE(world != nullptr) << scene.uri;

  const std::string worldContext = scene.uri + " detector=" + detectorName
                                   + " threads=" + std::to_string(threads)
                                   + " step=" + std::to_string(step);
  expectFinite(world->getTime(), worldContext + " time");
  EXPECT_LT(std::abs(world->getTime()), 1.0e4) << worldContext;

  const SoftSceneStats stats = collectSoftSceneStats(world);
  EXPECT_GE(stats.softBodies, scene.minSoftBodies) << worldContext;
  EXPECT_GE(stats.pointMasses, scene.minPointMasses) << worldContext;

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const dynamics::SkeletonPtr skeleton = world->getSkeleton(i);
    const std::string skeletonContext
        = worldContext + " skeleton=" + std::to_string(i);
    ASSERT_TRUE(skeleton != nullptr) << skeletonContext;

    expectFinite(skeleton->getPositions(), skeletonContext + " positions");
    expectFinite(skeleton->getVelocities(), skeletonContext + " velocities");
    expectFinite(
        skeleton->getAccelerations(), skeletonContext + " accelerations");
    expectFinite(skeleton->getForces(), skeletonContext + " forces");

    for (std::size_t j = 0; j < skeleton->getNumSoftBodyNodes(); ++j) {
      const dynamics::SoftBodyNode* softBody = skeleton->getSoftBodyNode(j);
      const std::string softBodyContext
          = skeletonContext + " soft_body=" + std::to_string(j);
      expectFiniteSoftBodyState(softBody, softBodyContext);
    }
  }
}

dynamics::SoftBodyNode* firstSoftBody(const simulation::WorldPtr& world)
{
  if (!world)
    return nullptr;

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const dynamics::SkeletonPtr skeleton = world->getSkeleton(i);
    if (!skeleton)
      continue;

    if (skeleton->getNumSoftBodyNodes() > 0u)
      return skeleton->getSoftBodyNode(0);
  }

  return nullptr;
}

class ScopedEnvironmentVariable
{
public:
  ScopedEnvironmentVariable(const char* name, const char* value) : mName(name)
  {
    if (const char* original = std::getenv(name)) {
      mHadOriginal = true;
      mOriginal = original;
    }
    set(value);
  }

  ~ScopedEnvironmentVariable()
  {
    if (mHadOriginal)
      set(mOriginal.c_str());
    else
      clear();
  }

private:
  void set(const char* value)
  {
#ifdef _WIN32
    _putenv_s(mName.c_str(), value);
#else
    setenv(mName.c_str(), value, 1);
#endif
  }

  void clear()
  {
#ifdef _WIN32
    _putenv_s(mName.c_str(), "");
#else
    unsetenv(mName.c_str());
#endif
  }

  std::string mName;
  std::string mOriginal;
  bool mHadOriginal{false};
};

struct SoftFaceRestScene
{
  simulation::WorldPtr world;
  dynamics::SoftBodyNode* softBody{nullptr};
  dynamics::BodyNode* sphereBody{nullptr};
  double sphereRadius{0.15};
};

SoftFaceRestScene makeSoftFaceRestScene(bool enableFaceInteriorContacts)
{
  ScopedEnvironmentVariable environment(
      "DART_SOFT_FACE_INTERIOR_CONTACTS",
      enableFaceInteriorContacts ? "1" : "0");

  SoftFaceRestScene scene;
  scene.world = simulation::World::create();
  scene.world->setTimeStep(0.001);
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  scene.world->setCollisionDetector(simulation::CollisionDetectorType::Dart);

  auto softSkeleton = dynamics::Skeleton::create("coarse_soft_support");
  const auto softProperties = dynamics::SoftBodyNodeHelper::makeBoxProperties(
      Eigen::Vector3d(1.0, 1.0, 0.2),
      Eigen::Isometry3d::Identity(),
      1.0,
      1000.0,
      1000.0,
      10.0);
  const dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties("coarse_soft_support_body"));
  const dynamics::SoftBodyNode::Properties softBodyProperties(
      bodyProperties, softProperties);
  auto softPair = softSkeleton->createJointAndBodyNodePair<
      dynamics::WeldJoint,
      dynamics::SoftBodyNode>(
      nullptr, dynamics::WeldJoint::Properties(), softBodyProperties);
  scene.softBody = softPair.second;
  softSkeleton->setMobile(false);
  scene.world->addSkeleton(softSkeleton);

  auto sphereSkeleton = dynamics::Skeleton::create("heavy_sphere");
  auto spherePair
      = sphereSkeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  scene.sphereBody = spherePair.second;
  const auto sphereShape
      = std::make_shared<dynamics::SphereShape>(scene.sphereRadius);
  scene.sphereBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(sphereShape);

  dynamics::Inertia inertia;
  inertia.setMass(5.0);
  inertia.setMoment(sphereShape->computeInertia(inertia.getMass()));
  scene.sphereBody->setInertia(inertia);

  Eigen::Isometry3d sphereTransform = Eigen::Isometry3d::Identity();
  sphereTransform.translation() = Eigen::Vector3d(-0.25, -0.25, 0.22);
  dynamics::FreeJoint::setTransformOf(spherePair.first, sphereTransform);
  scene.world->addSkeleton(sphereSkeleton);
  return scene;
}

double softBodyLowerBound(const dynamics::SoftBodyNode* softBody)
{
  double lowerBound = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0u; i < softBody->getNumPointMasses(); ++i) {
    lowerBound = std::min(
        lowerBound, softBody->getPointMass(i)->getWorldPosition().z());
  }
  return lowerBound;
}

void enableAdaptiveContactActivation(
    const simulation::WorldPtr& world,
    std::size_t ringCount = 2u,
    std::size_t lingerSteps = 8u,
    double velocityTolerance = 1.0e-4,
    double positionTolerance = 1.0e-4)
{
  ASSERT_TRUE(world != nullptr);
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const dynamics::SkeletonPtr skeleton = world->getSkeleton(i);
    if (!skeleton)
      continue;

    for (std::size_t j = 0; j < skeleton->getNumSoftBodyNodes(); ++j) {
      dynamics::SoftBodyNode* softBody = skeleton->getSoftBodyNode(j);
      ASSERT_TRUE(softBody != nullptr);
      softBody->setAdaptiveContactActivationRingCount(ringCount);
      softBody->setAdaptiveContactActivationLingerSteps(lingerSteps);
      softBody->setAdaptiveContactActivationVelocityTolerance(
          velocityTolerance);
      softBody->setAdaptiveContactActivationPositionTolerance(
          positionTolerance);
      softBody->setAdaptiveContactActivationEnabled(true);
    }
  }
}

std::size_t countPointMasses(const simulation::WorldPtr& world)
{
  return collectSoftSceneStats(world).pointMasses;
}

std::size_t countActivePointMasses(const simulation::WorldPtr& world)
{
  std::size_t active = 0u;
  if (!world)
    return active;

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const dynamics::SkeletonPtr skeleton = world->getSkeleton(i);
    if (!skeleton)
      continue;

    for (std::size_t j = 0; j < skeleton->getNumSoftBodyNodes(); ++j) {
      const dynamics::SoftBodyNode* softBody = skeleton->getSoftBodyNode(j);
      if (softBody != nullptr)
        active += softBody->getNumActivePointMasses();
    }
  }

  return active;
}

std::vector<double> collectSkeletonPositionsAndVelocities(
    const simulation::WorldPtr& world)
{
  std::vector<double> values;
  if (!world)
    return values;

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const dynamics::SkeletonPtr skeleton = world->getSkeleton(i);
    if (!skeleton)
      continue;

    appendStateValues(skeleton->getPositions(), values);
    appendStateValues(skeleton->getVelocities(), values);
  }

  return values;
}

double maxAbsDifference(
    const std::vector<double>& lhs, const std::vector<double>& rhs)
{
  DART_ASSERT(lhs.size() == rhs.size());
  double maxDifference = 0.0;
  for (std::size_t i = 0; i < lhs.size(); ++i)
    maxDifference = std::max(maxDifference, std::abs(lhs[i] - rhs[i]));

  return maxDifference;
}

void configureDetector(
    const simulation::WorldPtr& world, bool useNativeDetector)
{
  DART_ASSERT(world != nullptr);
  if (useNativeDetector)
    world->setCollisionDetector(simulation::CollisionDetectorType::Dart);
}

double computeSoftMechanicalEnergy(
    const dynamics::SkeletonPtr& skeleton, const Eigen::Vector3d& gravity)
{
  DART_ASSERT(skeleton != nullptr);

  // Skeleton energy covers the rigid parent bodies and joints. Point masses
  // are not Skeleton DOFs in DART 6, so add their kinetic, gravitational, and
  // spring energies explicitly through the public soft-body API.
  double energy
      = skeleton->computeKineticEnergy() + skeleton->computePotentialEnergy();
  for (std::size_t i = 0; i < skeleton->getNumSoftBodyNodes(); ++i) {
    const dynamics::SoftBodyNode* softBody = skeleton->getSoftBodyNode(i);
    DART_ASSERT(softBody != nullptr);
    const double vertexStiffness = softBody->getVertexSpringStiffness();
    const double edgeStiffness = softBody->getEdgeSpringStiffness();

    for (std::size_t j = 0; j < softBody->getNumPointMasses(); ++j) {
      const dynamics::PointMass* pointMass = softBody->getPointMass(j);
      DART_ASSERT(pointMass != nullptr);
      energy += 0.5 * pointMass->getMass()
                * pointMass->getWorldVelocity().squaredNorm();
      if (softBody->getGravityMode())
        energy -= pointMass->getMass()
                  * gravity.dot(pointMass->getWorldPosition());
      energy += 0.5 * vertexStiffness * pointMass->getPositions().squaredNorm();

      for (std::size_t k = 0; k < pointMass->getNumConnectedPointMasses();
           ++k) {
        const dynamics::PointMass* neighbor
            = pointMass->getConnectedPointMass(k);
        DART_ASSERT(neighbor != nullptr);
        if (neighbor->getIndexInSoftBodyNode() <= j)
          continue;
        energy += 0.5 * edgeStiffness
                  * (pointMass->getPositions() - neighbor->getPositions())
                        .squaredNorm();
      }
    }
  }

  return energy;
}

double computeSoftSystemMass(const dynamics::SkeletonPtr& skeleton)
{
  DART_ASSERT(skeleton != nullptr);
  double mass = 0.0;
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i)
    mass += skeleton->getBodyNode(i)->getInertia().getMass();

  for (std::size_t i = 0; i < skeleton->getNumSoftBodyNodes(); ++i) {
    const dynamics::SoftBodyNode* softBody = skeleton->getSoftBodyNode(i);
    DART_ASSERT(softBody != nullptr);
    for (std::size_t j = 0; j < softBody->getNumPointMasses(); ++j)
      mass += softBody->getPointMass(j)->getMass();
  }

  return mass;
}

ContactMetrics computeContactMetrics(
    const simulation::WorldPtr& world,
    const dynamics::SkeletonPtr& targetSkeleton)
{
  DART_ASSERT(world != nullptr);
  DART_ASSERT(targetSkeleton != nullptr);

  ContactMetrics metrics;
  Eigen::Vector3d weightedContactPoint = Eigen::Vector3d::Zero();
  double verticalWeight = 0.0;
  const auto& result = world->getLastCollisionResult();
  for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
    const collision::Contact& contact = result.getContact(i);
    const dynamics::ConstBodyNodePtr body1 = contact.getBodyNodePtr1();
    const dynamics::ConstBodyNodePtr body2 = contact.getBodyNodePtr2();
    const bool targetIsBody1
        = body1 && body1->getSkeleton().get() == targetSkeleton.get();
    const bool targetIsBody2
        = body2 && body2->getSkeleton().get() == targetSkeleton.get();
    if (targetIsBody1 == targetIsBody2)
      continue;

    const Eigen::Vector3d forceOnTarget
        = targetIsBody1 ? contact.force : -contact.force;
    metrics.force += forceOnTarget;
    ++metrics.contacts;

    const double upwardForce = std::max(0.0, forceOnTarget.y());
    weightedContactPoint += upwardForce * contact.point;
    verticalWeight += upwardForce;
  }

  if (verticalWeight > 1.0e-12) {
    metrics.centerOfPressure = weightedContactPoint / verticalWeight;
    metrics.hasCenterOfPressure = true;
  }
  return metrics;
}

} // namespace

//==============================================================================
class SoftDynamicsTest : public ::testing::Test
{
public:
  // Get mass matrix of _skel using Jacobians and inertias of each body
  // in _skel.
  MatrixXd getMassMatrix(dynamics::SkeletonPtr _skel);

  // Get augmented mass matrix of _skel using Jacobians and inertias of
  // each body in _skel.
  MatrixXd getAugMassMatrix(dynamics::SkeletonPtr _skel);

  // TODO(JS): Not implemented yet.
  // Compare velocities computed by recursive method, Jacobian, and finite
  // difference.
  // void compareVelocities(const std::string& _fileName) {}

  // TODO(JS): Not implemented yet.
  // Compare accelerations computed by recursive method, Jacobian, and finite
  // difference.
  // void compareAccelerations(const std::string& _fileName) {}
};

//==============================================================================
MatrixXd SoftDynamicsTest::getMassMatrix(dynamics::SkeletonPtr _skel)
{
  int skelDof = _skel->getNumDofs();

  MatrixXd skelM = MatrixXd::Zero(skelDof, skelDof); // Mass matrix of skeleton
  MatrixXd M;                                        // Body mass
  Eigen::Matrix6d I;                                 // Body inertia
  math::Jacobian J;                                  // Body Jacobian

  for (std::size_t i = 0; i < _skel->getNumBodyNodes(); ++i) {
    dynamics::BodyNode* body = _skel->getBodyNode(i);

    int dof = body->getNumDependentGenCoords();
    I = body->getSpatialInertia();
    J = body->getJacobian();

    EXPECT_EQ(I.rows(), 6);
    EXPECT_EQ(I.cols(), 6);
    EXPECT_EQ(J.rows(), 6);
    EXPECT_EQ(J.cols(), dof);

    M = J.transpose() * I * J; // (dof x dof) matrix

    for (int j = 0; j < dof; ++j) {
      int jIdx = body->getDependentGenCoordIndex(j);

      for (int k = 0; k < dof; ++k) {
        int kIdx = body->getDependentGenCoordIndex(k);

        skelM(jIdx, kIdx) += M(j, k);
      }
    }

    if (const dynamics::SoftBodyNode* softBody = body->asSoftBodyNode())
      skelM += projectPointMassMassMatrix(_skel, softBody);
  }

  return skelM;
}

//==============================================================================
MatrixXd SoftDynamicsTest::getAugMassMatrix(dynamics::SkeletonPtr _skel)
{
  int dof = _skel->getNumDofs();
  double dt = _skel->getTimeStep();

  MatrixXd M = getMassMatrix(_skel);
  MatrixXd D = MatrixXd::Zero(dof, dof);
  MatrixXd K = MatrixXd::Zero(dof, dof);
  MatrixXd AugM;

  // Compute diagonal matrices of joint damping and joint stiffness
  for (std::size_t i = 0; i < _skel->getNumBodyNodes(); ++i) {
    dynamics::BodyNode* body = _skel->getBodyNode(i);
    dynamics::Joint* joint = body->getParentJoint();

    EXPECT_TRUE(body != nullptr);
    EXPECT_TRUE(joint != nullptr);

    int dof = joint->getNumDofs();

    for (int j = 0; j < dof; ++j) {
      int idx = joint->getIndexInSkeleton(j);

      D(idx, idx) = joint->getDampingCoefficient(j);
      K(idx, idx) = joint->getSpringStiffness(j);
    }
  }

  //  dynamics::SkeletonPtr softSkel
  //      = dynamic_cast<dynamics::SkeletonPtr>(_skel);

  //  if (softSkel != nullptr)
  //  {
  //    for (int i = 0; i < softSkel->getNumSoftBodyNodes(); ++i)
  //    {
  //      dynamics::SoftBodyNode* softBody = softSkel->getSoftBodyNode(i);

  //      for (int j = 0; j < softBody->getNumPointMasses(); ++j)
  //      {
  //        dynamics::PointMass* pm = softBody->getPointMass(j);

  //        int dof = 3;

  //        for (int k = 0; k < dof; ++k)
  //        {
  //          int idx = pm->getIndexInSkeleton(k);

  //          D(idx, idx) = softBody->getDampingCoefficient();
  //          K(idx, idx) = softBody->getVertexSpringStiffness();
  //        }
  //      }
  //    }
  //  }

  AugM = M + (dt * D) + (dt * dt * K);

  return AugM;
}

//==============================================================================
TEST_F(SoftDynamicsTest, representativeEquationMatrixAndVectorChecks)
{
  struct EquationScene
  {
    std::string uri;
    bool expectMobileRigidSkeleton;
    bool expectMobileSoftSkeleton;
    bool expectRigidSkeleton;
  };

  const std::array<EquationScene, 3> scenes = {{
      {"dart://sample/skel/test/double_pendulum.skel", true, false, true},
      {"dart://sample/skel/test/test_drop_box.skel", false, true, true},
      {"dart://sample/skel/softBodies.skel", false, true, true},
  }};

  for (const EquationScene& scene : scenes) {
    simulation::WorldPtr world = utils::SkelParser::readWorld(scene.uri);
    ASSERT_TRUE(world != nullptr) << scene.uri;

    bool sawMobileRigidSkeleton = false;
    bool sawMobileSoftSkeleton = false;
    bool sawRigidSkeleton = false;

    for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
      const dynamics::SkeletonPtr skeleton = world->getSkeleton(i);
      ASSERT_TRUE(skeleton != nullptr) << scene.uri << " skeleton=" << i;

      const bool hasSoftBody = skeleton->getNumSoftBodyNodes() > 0u;
      const bool hasRigidBody
          = skeleton->getNumBodyNodes() > skeleton->getNumSoftBodyNodes();
      sawRigidSkeleton = sawRigidSkeleton || hasRigidBody;

      const std::string context = scene.uri + " skeleton=" + std::to_string(i)
                                  + " name=" + skeleton->getName();
      const std::size_t dof = skeleton->getNumDofs();
      if (dof == 0u)
        continue;

      sawMobileSoftSkeleton = sawMobileSoftSkeleton || hasSoftBody;
      sawMobileRigidSkeleton = sawMobileRigidSkeleton || !hasSoftBody;

      const Eigen::MatrixXd mass = skeleton->getMassMatrix();
      const Eigen::MatrixXd expectedMass = getMassMatrix(skeleton);
      const Eigen::MatrixXd invMass = skeleton->getInvMassMatrix();
      const Eigen::MatrixXd augMass = skeleton->getAugMassMatrix();
      const Eigen::MatrixXd expectedAugMass = getAugMassMatrix(skeleton);
      const Eigen::MatrixXd invAugMass = skeleton->getInvAugMassMatrix();
      const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(dof, dof);

      expectMatrixNear(mass, expectedMass, 1.0e-8, context + " mass");
      expectMatrixNear(
          augMass, expectedAugMass, 1.0e-8, context + " augmented mass");
      expectMatrixNear(
          mass * invMass, identity, 1.0e-8, context + " mass inverse left");
      expectMatrixNear(
          invMass * mass, identity, 1.0e-8, context + " mass inverse right");
      expectMatrixNear(
          augMass * invAugMass,
          identity,
          1.0e-8,
          context + " augmented inverse left");
      expectMatrixNear(
          invAugMass * augMass,
          identity,
          1.0e-8,
          context + " augmented inverse right");

      const Eigen::VectorXd coriolis = skeleton->getCoriolisForces();
      const Eigen::VectorXd combined = skeleton->getCoriolisAndGravityForces();
      const Eigen::Vector3d originalGravity = skeleton->getGravity();
      const Eigen::VectorXd originalForces = skeleton->getForces();
      const Eigen::VectorXd originalAccelerations
          = skeleton->getAccelerations();

      skeleton->resetGeneralizedForces();
      skeleton->clearExternalForces();
      skeleton->setAccelerations(Eigen::VectorXd::Zero(dof));

      skeleton->setGravity(Eigen::Vector3d::Zero());
      skeleton->computeInverseDynamics(false, false, false);
      const Eigen::VectorXd coriolisFromInverseDynamics = skeleton->getForces();

      skeleton->setGravity(originalGravity);
      skeleton->computeInverseDynamics(false, false, false);
      const Eigen::VectorXd combinedFromInverseDynamics = skeleton->getForces();

      expectVectorNear(
          coriolis,
          coriolisFromInverseDynamics,
          1.0e-8,
          context + " coriolis inverse dynamics");
      expectVectorNear(
          combined,
          combinedFromInverseDynamics,
          1.0e-8,
          context + " combined inverse dynamics");

      skeleton->setGravity(originalGravity);
      skeleton->setForces(originalForces);
      skeleton->setAccelerations(originalAccelerations);
    }

    EXPECT_EQ(scene.expectMobileRigidSkeleton, sawMobileRigidSkeleton)
        << scene.uri;
    EXPECT_EQ(scene.expectMobileSoftSkeleton, sawMobileSoftSkeleton)
        << scene.uri;
    EXPECT_EQ(scene.expectRigidSkeleton, sawRigidSkeleton) << scene.uri;
  }
}

//==============================================================================
TEST_F(SoftDynamicsTest, finiteStateForRepresentativeSoftScenes)
{
  struct DetectorConfig
  {
    std::string name;
    bool useNativeDetector;
  };

  const std::array<SoftStabilityScene, 7> scenes = {{
      {"dart://sample/skel/test/test_drop_box.skel", 1u, 26u, 30u},
      {"dart://sample/skel/test/test_drop_low_stiffness.skel", 1u, 26u, 30u},
      {"dart://sample/skel/test/test_double_pendulum.skel", 2u, 52u, 30u},
      {"dart://sample/skel/test/test_adaptive_deformable.skel", 1u, 12u, 30u},
      {"dart://sample/skel/soft_cubes.skel", 2u, 52u, 30u},
      {"dart://sample/skel/softBodies.skel", 5u, 290u, 30u},
      {"dart://sample/skel/soft_open_chain.skel", 5u, 120u, 30u},
  }};
  const std::array<DetectorConfig, 2> detectorConfigs = {{
      {"default", false},
      {"dart", true},
  }};
  const std::array<std::size_t, 2> threadCounts = {{1u, 4u}};

  for (const DetectorConfig& detectorConfig : detectorConfigs) {
    std::vector<SoftStateSnapshot> finalSnapshots;
    finalSnapshots.reserve(threadCounts.size());

    for (const SoftStabilityScene& scene : scenes) {
      finalSnapshots.clear();

      for (const std::size_t threads : threadCounts) {
        simulation::WorldPtr world = utils::SkelParser::readWorld(scene.uri);
        ASSERT_TRUE(world != nullptr) << scene.uri;
        if (detectorConfig.useNativeDetector)
          world->setCollisionDetector(simulation::CollisionDetectorType::Dart);
        world->setNumSimulationThreads(threads);

        expectFiniteWorldState(world, scene, detectorConfig.name, threads, 0u);

        for (std::size_t step = 1u; step <= scene.steps; ++step) {
          world->step();

          if (step == scene.steps || step % 10u == 0u) {
            expectFiniteWorldState(
                world, scene, detectorConfig.name, threads, step);
          }
        }

        finalSnapshots.push_back(computeSoftStateSnapshot(world));
      }

      ASSERT_EQ(finalSnapshots.size(), threadCounts.size())
          << detectorConfig.name << " " << scene.uri;
      expectSnapshotsNear(
          finalSnapshots[0],
          finalSnapshots[1],
          scene.uri + " detector=" + detectorConfig.name
              + " threads=1-vs-4 final state");
    }
  }
}

//==============================================================================
TEST_F(SoftDynamicsTest, softFaceInteriorContactPreventsSphereTunneling)
{
  SoftFaceRestScene enabled = makeSoftFaceRestScene(true);
  SoftFaceRestScene disabled = makeSoftFaceRestScene(false);
  ASSERT_TRUE(enabled.world != nullptr);
  ASSERT_TRUE(disabled.world != nullptr);
  ASSERT_TRUE(enabled.softBody != nullptr);
  ASSERT_TRUE(disabled.softBody != nullptr);
  ASSERT_TRUE(enabled.sphereBody != nullptr);
  ASSERT_TRUE(disabled.sphereBody != nullptr);

  const auto enabledDetector
      = std::dynamic_pointer_cast<collision::DARTCollisionDetector>(
          enabled.world->getCollisionDetector());
  const auto disabledDetector
      = std::dynamic_pointer_cast<collision::DARTCollisionDetector>(
          disabled.world->getCollisionDetector());
  ASSERT_TRUE(enabledDetector != nullptr);
  ASSERT_TRUE(disabledDetector != nullptr);
  ASSERT_TRUE(enabledDetector->getSoftFaceInteriorContactsEnabled());
  ASSERT_FALSE(disabledDetector->getSoftFaceInteriorContactsEnabled());

  bool sawEnabledContact = false;
  for (std::size_t step = 1u; step <= 500u; ++step) {
    enabled.world->step();
    disabled.world->step();

    const Eigen::Vector3d enabledPosition
        = enabled.sphereBody->getWorldTransform().translation();
    expectFinite(enabledPosition, "enabled sphere position");
    expectFinite(
        enabled.sphereBody->getLinearVelocity(), "enabled sphere velocity");
    const double enabledLowerBound = softBodyLowerBound(enabled.softBody);
    expectFinite(enabledLowerBound, "enabled soft lower bound");
    ASSERT_GT(
        enabledPosition.z() - enabled.sphereRadius, enabledLowerBound - 1.0e-6)
        << "step=" << step;

    sawEnabledContact
        = sawEnabledContact
          || enabled.world->getLastCollisionResult().getNumContacts() > 0u;
    if (step % 50u == 0u) {
      expectFiniteSoftBodyState(
          enabled.softBody,
          "enabled soft face support step=" + std::to_string(step));
    }
  }
  EXPECT_TRUE(sawEnabledContact);

  const double disabledSphereBottom
      = disabled.sphereBody->getWorldTransform().translation().z()
        - disabled.sphereRadius;
  const double disabledLowerBound = softBodyLowerBound(disabled.softBody);
  expectFinite(disabledSphereBottom, "disabled sphere bottom");
  expectFinite(disabledLowerBound, "disabled soft lower bound");

  // The disabled lane is a diagnostic control, not a behavioral contract:
  // other vertex contacts may happen to catch a changed scene in the future.
  // Record the observed outcome without requiring that it tunnel.
  RecordProperty("disabled_sphere_bottom", disabledSphereBottom);
  RecordProperty("disabled_soft_lower_bound", disabledLowerBound);
  RecordProperty(
      "disabled_sphere_sank_through",
      disabledSphereBottom < disabledLowerBound ? "true" : "false");
}

//==============================================================================
TEST_F(SoftDynamicsTest, adaptiveContactActivationDefaultOffMatchesControlWorld)
{
  simulation::WorldPtr lhs
      = utils::SkelParser::readWorld("dart://sample/skel/soft_cubes.skel");
  simulation::WorldPtr rhs
      = utils::SkelParser::readWorld("dart://sample/skel/soft_cubes.skel");
  ASSERT_TRUE(lhs != nullptr);
  ASSERT_TRUE(rhs != nullptr);

  lhs->setCollisionDetector(simulation::CollisionDetectorType::Dart);
  rhs->setCollisionDetector(simulation::CollisionDetectorType::Dart);
  lhs->setNumSimulationThreads(1u);
  rhs->setNumSimulationThreads(1u);

  for (std::size_t step = 0u; step < 120u; ++step) {
    lhs->step();
    rhs->step();
  }

  expectSnapshotsNear(
      computeSoftStateSnapshot(lhs),
      computeSoftStateSnapshot(rhs),
      "adaptive activation default-off control equivalence");
}

//==============================================================================
TEST_F(SoftDynamicsTest, adaptiveContactActivationMatchesSoftDropBehavior)
{
  simulation::WorldPtr activationOff
      = utils::SkelParser::readWorld("dart://sample/skel/soft_cubes.skel");
  simulation::WorldPtr activationOn
      = utils::SkelParser::readWorld("dart://sample/skel/soft_cubes.skel");
  ASSERT_TRUE(activationOff != nullptr);
  ASSERT_TRUE(activationOn != nullptr);

  activationOff->setCollisionDetector(simulation::CollisionDetectorType::Dart);
  activationOn->setCollisionDetector(simulation::CollisionDetectorType::Dart);
  enableAdaptiveContactActivation(activationOn);

  for (std::size_t step = 0u; step < 200u; ++step) {
    activationOff->step();
    activationOn->step();
  }

  const std::vector<double> offState
      = collectSkeletonPositionsAndVelocities(activationOff);
  const std::vector<double> onState
      = collectSkeletonPositionsAndVelocities(activationOn);
  ASSERT_EQ(offState.size(), onState.size());
  EXPECT_LT(maxAbsDifference(offState, onState), 0.75);
  EXPECT_GT(countActivePointMasses(activationOn), 0u);
  EXPECT_LT(
      countActivePointMasses(activationOn), countPointMasses(activationOn));
}

//==============================================================================
TEST_F(
    SoftDynamicsTest,
    adaptiveContactActivationRestGatedLingerFreezesQuietPoints)
{
  simulation::WorldPtr world = utils::SkelParser::readWorld(
      "dart://sample/skel/test/test_drop_box.skel");
  ASSERT_TRUE(world != nullptr);
  world->setGravity(Eigen::Vector3d::Zero());
  world->setCollisionDetector(simulation::CollisionDetectorType::Dart);

  dynamics::SoftBodyNode* softBody = firstSoftBody(world);
  ASSERT_TRUE(softBody != nullptr);
  ASSERT_GT(softBody->getNumPointMasses(), 2u);
  softBody->setVertexSpringStiffness(0.0);
  softBody->setEdgeSpringStiffness(0.0);
  softBody->setDampingCoefficient(0.0);
  softBody->getPointMass(0)->setVelocities(Eigen::Vector3d(0.1, 0.0, 0.0));

  enableAdaptiveContactActivation(world, 0u, 0u, 1.0e-3, 1.0e-9);
  world->step();
  world->step();

  EXPECT_GT(softBody->getNumActivePointMasses(), 0u);
  EXPECT_LT(softBody->getNumActivePointMasses(), softBody->getNumPointMasses());
  EXPECT_GT(softBody->getPointMass(0)->getVelocities().norm(), 1.0e-3);
  EXPECT_NEAR(softBody->getPointMass(1)->getVelocities().norm(), 0.0, 1.0e-12);
}

//==============================================================================
TEST_F(SoftDynamicsTest, adaptiveContactActivationReactivatesAfterRigidApproach)
{
  simulation::WorldPtr world
      = utils::SkelParser::readWorld("dart://sample/skel/soft_cubes.skel");
  ASSERT_TRUE(world != nullptr);
  world->setCollisionDetector(simulation::CollisionDetectorType::Dart);
  enableAdaptiveContactActivation(world, 2u, 0u);

  bool sawAllFrozen = false;
  bool sawReactivation = false;
  for (std::size_t step = 0u; step < 220u; ++step) {
    world->step();
    const std::size_t active = countActivePointMasses(world);
    if (active == 0u)
      sawAllFrozen = true;
    if (sawAllFrozen && active > 0u && active < countPointMasses(world)) {
      sawReactivation = true;
      break;
    }
  }

  EXPECT_TRUE(sawAllFrozen);
  EXPECT_TRUE(sawReactivation);
}

//==============================================================================
TEST_F(SoftDynamicsTest, adaptiveContactActivationAllFrozenPointsRideRigidly)
{
  simulation::WorldPtr world = utils::SkelParser::readWorld(
      "dart://sample/skel/test/test_adaptive_deformable.skel");
  ASSERT_TRUE(world != nullptr);
  world->setCollisionDetector(simulation::CollisionDetectorType::Dart);
  enableAdaptiveContactActivation(world, 2u, 0u);

  for (std::size_t step = 0u; step < 40u; ++step)
    world->step();

  dynamics::SoftBodyNode* softBody = firstSoftBody(world);
  ASSERT_TRUE(softBody != nullptr);
  ASSERT_EQ(softBody->getNumActivePointMasses(), 0u);
  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    const dynamics::PointMass* pointMass = softBody->getPointMass(i);
    ASSERT_TRUE(pointMass != nullptr);
    EXPECT_NEAR(pointMass->getPositions().norm(), 0.0, 1.0e-12);
    EXPECT_NEAR(pointMass->getVelocities().norm(), 0.0, 1.0e-12);
    const Eigen::Vector3d expectedWorld
        = softBody->getWorldTransform() * pointMass->getRestingPosition();
    EXPECT_NEAR(
        (pointMass->getWorldPosition() - expectedWorld).norm(), 0.0, 1.0e-12)
        << "all-frozen point rides parent transform";
  }
}

//==============================================================================
TEST_F(SoftDynamicsTest, adaptiveContactActivationKeepsPublicMatricesAllActive)
{
  simulation::WorldPtr activationOff = utils::SkelParser::readWorld(
      "dart://sample/skel/test/test_drop_box.skel");
  simulation::WorldPtr activationOn = utils::SkelParser::readWorld(
      "dart://sample/skel/test/test_drop_box.skel");
  ASSERT_TRUE(activationOff != nullptr);
  ASSERT_TRUE(activationOn != nullptr);
  enableAdaptiveContactActivation(activationOn);

  const dynamics::SkeletonPtr offSkeleton
      = activationOff->getSkeleton("skeleton 1");
  const dynamics::SkeletonPtr onSkeleton
      = activationOn->getSkeleton("skeleton 1");
  ASSERT_TRUE(offSkeleton != nullptr);
  ASSERT_TRUE(onSkeleton != nullptr);

  expectMatrixNear(
      offSkeleton->getMassMatrix(),
      onSkeleton->getMassMatrix(),
      1.0e-12,
      "activation on mass matrix");
  expectMatrixNear(
      offSkeleton->getAugMassMatrix(),
      onSkeleton->getAugMassMatrix(),
      1.0e-12,
      "activation on augmented mass matrix");
  expectMatrixNear(
      offSkeleton->getInvMassMatrix(),
      onSkeleton->getInvMassMatrix(),
      1.0e-12,
      "activation on inverse mass matrix");
  expectMatrixNear(
      offSkeleton->getInvAugMassMatrix(),
      onSkeleton->getInvAugMassMatrix(),
      1.0e-12,
      "activation on inverse augmented mass matrix");

  // The all-active guarantee must survive frozen points: soft trees compute
  // inverse matrices from the public (all-active) mass matrices, so the
  // identity products discriminate against any activity-dependent inverse.
  dynamics::SoftBodyNode* softBody = onSkeleton->getSoftBodyNode(0);
  ASSERT_TRUE(softBody != nullptr);
  for (std::size_t i = 0; i < 400; ++i) {
    activationOn->step();
    if (softBody->getNumActivePointMasses() < softBody->getNumPointMasses())
      break;
  }
  ASSERT_LT(softBody->getNumActivePointMasses(), softBody->getNumPointMasses())
      << "scene never froze a point; frozen-state matrix check not exercised";

  const Eigen::MatrixXd frozenM = onSkeleton->getMassMatrix();
  const Eigen::MatrixXd frozenInvM = onSkeleton->getInvMassMatrix();
  const Eigen::MatrixXd frozenAugM = onSkeleton->getAugMassMatrix();
  const Eigen::MatrixXd frozenInvAugM = onSkeleton->getInvAugMassMatrix();
  const Eigen::MatrixXd identity
      = Eigen::MatrixXd::Identity(frozenM.rows(), frozenM.cols());
  expectMatrixNear(
      frozenM * frozenInvM,
      identity,
      1.0e-9,
      "frozen-state mass/inverse identity");
  expectMatrixNear(
      frozenAugM * frozenInvAugM,
      identity,
      1.0e-9,
      "frozen-state augmented mass/inverse identity");
}

//==============================================================================
TEST_F(
    SoftDynamicsTest,
    adaptiveContactActivationCloneAndStateRestoreResetTransientState)
{
  simulation::WorldPtr world = utils::SkelParser::readWorld(
      "dart://sample/skel/test/test_drop_box.skel");
  ASSERT_TRUE(world != nullptr);
  enableAdaptiveContactActivation(world, 1u, 3u, 2.0e-4, 3.0e-4);

  dynamics::SoftBodyNode* softBody = firstSoftBody(world);
  ASSERT_TRUE(softBody != nullptr);
  dynamics::SkeletonPtr skeleton = softBody->getSkeleton();
  ASSERT_TRUE(skeleton != nullptr);

  dynamics::SkeletonPtr clone = skeleton->cloneSkeleton();
  ASSERT_TRUE(clone != nullptr);
  ASSERT_EQ(clone->getNumSoftBodyNodes(), 1u);
  const dynamics::SoftBodyNode* clonedSoftBody = clone->getSoftBodyNode(0);
  ASSERT_TRUE(clonedSoftBody != nullptr);
  EXPECT_TRUE(clonedSoftBody->isAdaptiveContactActivationEnabled());
  EXPECT_EQ(clonedSoftBody->getAdaptiveContactActivationRingCount(), 1u);
  EXPECT_EQ(clonedSoftBody->getAdaptiveContactActivationLingerSteps(), 3u);
  EXPECT_DOUBLE_EQ(
      clonedSoftBody->getAdaptiveContactActivationVelocityTolerance(), 2.0e-4);
  EXPECT_DOUBLE_EQ(
      clonedSoftBody->getAdaptiveContactActivationPositionTolerance(), 3.0e-4);
  EXPECT_EQ(
      clonedSoftBody->getNumActivePointMasses(),
      clonedSoftBody->getNumPointMasses());

  enableAdaptiveContactActivation(world, 1u, 0u);
  for (std::size_t step = 0u; step < 4u; ++step)
    world->step();
  EXPECT_EQ(softBody->getNumActivePointMasses(), 0u);

  dynamics::SoftBodyNode::AspectState restoredState;
  restoredState.mPointStates.resize(softBody->getNumPointMasses());
  restoredState.mPointStates[0].mPositions[0] = 1.0e-5;
  softBody->setAspectState(restoredState);
  // A state restore resets activation bookkeeping to all-active immediately
  // so instrumentation never reports stale pre-restore counts.
  EXPECT_EQ(softBody->getNumActivePointMasses(), softBody->getNumPointMasses());
  world->step();
  EXPECT_EQ(softBody->getNumActivePointMasses(), softBody->getNumPointMasses());
}

//==============================================================================
TEST_F(SoftDynamicsTest, adaptiveContactActivationDeterministicAcrossThreads)
{
  std::vector<SoftStateSnapshot> snapshots;
  for (const std::size_t threads : {1u, 1u, 4u}) {
    simulation::WorldPtr world
        = utils::SkelParser::readWorld("dart://sample/skel/soft_cubes.skel");
    ASSERT_TRUE(world != nullptr);
    world->setCollisionDetector(simulation::CollisionDetectorType::Dart);
    world->setNumSimulationThreads(threads);
    enableAdaptiveContactActivation(world);

    for (std::size_t step = 0u; step < 200u; ++step)
      world->step();

    snapshots.push_back(computeSoftStateSnapshot(world));
  }

  ASSERT_EQ(snapshots.size(), 3u);
  expectSnapshotsNear(
      snapshots[0], snapshots[1], "activation repeat determinism threads=1");
  expectSnapshotsNear(
      snapshots[0], snapshots[2], "activation determinism threads=1-vs-4");
}

//==============================================================================
TEST_F(SoftDynamicsTest, softMechanicalEnergyGrowthIsBounded)
{
  struct DetectorConfig
  {
    const char* name;
    bool useNativeDetector;
  };
  const std::array<DetectorConfig, 2> detectors
      = {{{"default", false}, {"dart", true}}};

  for (const DetectorConfig& detector : detectors) {
    simulation::WorldPtr freeWorld = utils::SkelParser::readWorld(
        "dart://sample/skel/test/test_drop_low_stiffness.skel");
    ASSERT_TRUE(freeWorld != nullptr) << detector.name;
    freeWorld->removeSkeleton(freeWorld->getSkeleton("ground skeleton"));
    freeWorld->setGravity(Eigen::Vector3d::Zero());
    configureDetector(freeWorld, detector.useNativeDetector);

    const dynamics::SkeletonPtr freeSoft = freeWorld->getSkeleton("skeleton 1");
    ASSERT_TRUE(freeSoft != nullptr) << detector.name;
    dynamics::SoftBodyNode* freeSoftBody = freeSoft->getSoftBodyNode(0);
    ASSERT_TRUE(freeSoftBody != nullptr) << detector.name;
    ASSERT_GT(freeSoftBody->getNumPointMasses(), 1u) << detector.name;
    // The fixture ships with ke=0; a nonzero edge stiffness keeps the
    // edge-spring energy term exercised instead of multiplying to zero.
    freeSoftBody->setEdgeSpringStiffness(5.0);
    freeSoftBody->getPointMass(0)->setPositions(
        Eigen::Vector3d(0.02, 0.0, 0.0));
    freeSoftBody->getPointMass(1)->setPositions(
        Eigen::Vector3d(-0.01, 0.015, 0.0));

    const double initialEnergy
        = computeSoftMechanicalEnergy(freeSoft, freeWorld->getGravity());
    expectFinite(initialEnergy, std::string(detector.name) + " initial energy");
    ASSERT_GT(initialEnergy, 1.0e-8) << detector.name;
    double previousEnergy = initialEnergy;
    double maxEnergyIncrease = 0.0;
    for (std::size_t step = 0; step < 500u; ++step) {
      freeWorld->step();
      EXPECT_EQ(freeWorld->getLastCollisionResult().getNumContacts(), 0u)
          << detector.name << " step=" << step;
      const double energy
          = computeSoftMechanicalEnergy(freeSoft, freeWorld->getGravity());
      expectFinite(
          energy,
          std::string(detector.name)
              + " free energy step=" + std::to_string(step));
      maxEnergyIncrease = std::max(maxEnergyIncrease, energy - previousEnergy);
      previousEnergy = energy;
    }

    // One nanjoule per step (scaled only for energies above one joule) absorbs
    // floating-point summation noise while rejecting integrator energy growth.
    const double freeStepTolerance
        = 1.0e-9 * std::max(1.0, std::abs(initialEnergy));
    EXPECT_LE(maxEnergyIncrease, freeStepTolerance) << detector.name;
    EXPECT_LT(previousEnergy, initialEnergy) << detector.name;

    simulation::WorldPtr contactWorld = utils::SkelParser::readWorld(
        "dart://sample/skel/test/test_drop_low_stiffness.skel");
    ASSERT_TRUE(contactWorld != nullptr) << detector.name;
    configureDetector(contactWorld, detector.useNativeDetector);
    const dynamics::SkeletonPtr contactSoft
        = contactWorld->getSkeleton("skeleton 1");
    ASSERT_TRUE(contactSoft != nullptr) << detector.name;

    previousEnergy
        = computeSoftMechanicalEnergy(contactSoft, contactWorld->getGravity());
    maxEnergyIncrease = 0.0;
    bool sawContact = false;
    for (std::size_t step = 0; step < 800u; ++step) {
      contactWorld->step();
      sawContact
          = sawContact
            || contactWorld->getLastCollisionResult().getNumContacts() > 0u;
      const double energy = computeSoftMechanicalEnergy(
          contactSoft, contactWorld->getGravity());
      expectFinite(
          energy,
          std::string(detector.name)
              + " contact energy step=" + std::to_string(step));
      maxEnergyIncrease = std::max(maxEnergyIncrease, energy - previousEnergy);
      previousEnergy = energy;
    }

    ASSERT_TRUE(sawContact) << detector.name;
    // Contact impulses may inject correction energy. Limiting one-step growth
    // to the system weight times 1 cm permits that impulse-scale correction
    // while rejecting macroscopic energy spikes or runaway contact response.
    const double contactStepBound = computeSoftSystemMass(contactSoft)
                                    * std::abs(contactWorld->getGravity().y())
                                    * 0.01;
    EXPECT_LE(maxEnergyIncrease, contactStepBound) << detector.name;
  }
}

//==============================================================================
TEST_F(SoftDynamicsTest, restingSoftContactForceAndCenterOfPressureAreSmooth)
{
  struct DetectorConfig
  {
    const char* name;
    bool useNativeDetector;
  };
  const std::array<DetectorConfig, 2> detectors
      = {{{"default", false}, {"dart", true}}};

  for (const DetectorConfig& detector : detectors) {
    for (const bool adaptive : {false, true}) {
      simulation::WorldPtr world = utils::SkelParser::readWorld(
          "dart://sample/skel/test/test_drop_low_stiffness.skel");
      ASSERT_TRUE(world != nullptr) << detector.name;
      configureDetector(world, detector.useNativeDetector);
      if (adaptive)
        enableAdaptiveContactActivation(world);

      const dynamics::SkeletonPtr softSkeleton
          = world->getSkeleton("skeleton 1");
      ASSERT_TRUE(softSkeleton != nullptr) << detector.name;
      for (std::size_t step = 0; step < 3000u; ++step)
        world->step();

      const std::string context = std::string(detector.name)
                                  + (adaptive ? " adaptive" : " all-active");
      const double weight = computeSoftSystemMass(softSkeleton)
                            * std::abs(world->getGravity().y());
      double minVerticalForce = std::numeric_limits<double>::infinity();
      double maxVerticalForce = 0.0;
      double maxCopDisplacement = 0.0;
      Eigen::Vector3d previousCop = Eigen::Vector3d::Zero();
      bool havePreviousCop = false;
      std::size_t sampledContacts = 0u;

      for (std::size_t step = 0; step < 300u; ++step) {
        world->step();
        const ContactMetrics metrics
            = computeContactMetrics(world, softSkeleton);
        expectFinite(metrics.force, context + " contact force");
        ASSERT_TRUE(metrics.hasCenterOfPressure) << context << " step=" << step;
        expectFinite(metrics.centerOfPressure, context + " center of pressure");
        sampledContacts += metrics.contacts;
        minVerticalForce = std::min(minVerticalForce, metrics.force.y());
        maxVerticalForce = std::max(maxVerticalForce, metrics.force.y());
        if (havePreviousCop) {
          const Eigen::Vector2d displacement(
              metrics.centerOfPressure.x() - previousCop.x(),
              metrics.centerOfPressure.z() - previousCop.z());
          maxCopDisplacement
              = std::max(maxCopDisplacement, displacement.norm());
        }
        previousCop = metrics.centerOfPressure;
        havePreviousCop = true;
      }

      ASSERT_GT(sampledContacts, 0u) << context;
      // The native manifold is held to +/-25% of weight and 2 cm CoP motion.
      // Legacy FCL needs a 0.25x-3x force band and 11 cm CoP bound for its
      // larger contact-point churn. Both still reject lost support, impulses
      // above 3x weight, and a one-step jump across the 20 cm footprint; each
      // backend uses the same bounds with adaptive activation on and off.
      const double minForceFactor = detector.useNativeDetector ? 0.75 : 0.25;
      const double maxForceFactor = detector.useNativeDetector ? 1.25 : 3.0;
      const double copDisplacementBound
          = detector.useNativeDetector ? 0.02 : 0.11;
      EXPECT_GE(minVerticalForce, minForceFactor * weight) << context;
      EXPECT_LE(maxVerticalForce, maxForceFactor * weight) << context;
      EXPECT_LE(maxCopDisplacement, copDisplacementBound) << context;
    }
  }
}

//==============================================================================
TEST_F(SoftDynamicsTest, softContactForcesAreRobustToLcpInitialPoint)
{
  struct DetectorConfig
  {
    const char* name;
    bool useNativeDetector;
  };
  const std::array<DetectorConfig, 2> detectors
      = {{{"default", false}, {"dart", true}}};

  for (const DetectorConfig& detector : detectors) {
    simulation::WorldPtr continuousWorld = utils::SkelParser::readWorld(
        "dart://sample/skel/test/test_drop_low_stiffness.skel");
    ASSERT_TRUE(continuousWorld != nullptr) << detector.name;
    configureDetector(continuousWorld, detector.useNativeDetector);
    for (std::size_t step = 0; step < 3000u; ++step)
      continuousWorld->step();

    simulation::WorldPtr restoredWorld = utils::SkelParser::readWorld(
        "dart://sample/skel/test/test_drop_low_stiffness.skel");
    ASSERT_TRUE(restoredWorld != nullptr) << detector.name;
    configureDetector(restoredWorld, detector.useNativeDetector);
    restoredWorld->reset();
    ASSERT_EQ(
        restoredWorld->getNumSkeletons(), continuousWorld->getNumSkeletons())
        << detector.name;
    for (std::size_t i = 0; i < continuousWorld->getNumSkeletons(); ++i) {
      restoredWorld->getSkeleton(i)->setState(
          continuousWorld->getSkeleton(i)->getState());
    }

    const dynamics::SkeletonPtr continuousSoft
        = continuousWorld->getSkeleton("skeleton 1");
    const dynamics::SkeletonPtr restoredSoft
        = restoredWorld->getSkeleton("skeleton 1");
    ASSERT_TRUE(continuousSoft != nullptr) << detector.name;
    ASSERT_TRUE(restoredSoft != nullptr) << detector.name;

    // Let the restored world rebuild contact/LCP caches from the same public
    // state before comparing its steady window with uninterrupted simulation.
    for (std::size_t step = 0; step < 300u; ++step) {
      continuousWorld->step();
      restoredWorld->step();
    }

    double continuousForceMagnitudeSum = 0.0;
    double restoredForceMagnitudeSum = 0.0;
    for (std::size_t step = 0; step < 300u; ++step) {
      continuousWorld->step();
      restoredWorld->step();
      const ContactMetrics continuousMetrics
          = computeContactMetrics(continuousWorld, continuousSoft);
      const ContactMetrics restoredMetrics
          = computeContactMetrics(restoredWorld, restoredSoft);
      ASSERT_GT(continuousMetrics.contacts, 0u)
          << detector.name << " continuous step=" << step;
      ASSERT_GT(restoredMetrics.contacts, 0u)
          << detector.name << " restored step=" << step;
      continuousForceMagnitudeSum += continuousMetrics.force.norm();
      restoredForceMagnitudeSum += restoredMetrics.force.norm();
    }

    const double continuousMeanForce = continuousForceMagnitudeSum / 300.0;
    const double restoredMeanForce = restoredForceMagnitudeSum / 300.0;
    const double weight = computeSoftSystemMass(continuousSoft)
                          * std::abs(continuousWorld->getGravity().y());
    // Five percent of weight permits cache-rebuild transients while rejecting
    // a warm-start history that materially changes steady contact load.
    EXPECT_NEAR(continuousMeanForce, restoredMeanForce, 0.05 * weight)
        << detector.name;
  }
}

//==============================================================================
TEST_F(SoftDynamicsTest, pointMassGravityContributesToGeneralizedForceVectors)
{
  simulation::WorldPtr world = utils::SkelParser::readWorld(
      "dart://sample/skel/test/test_drop_box.skel");
  ASSERT_TRUE(world != nullptr);

  const dynamics::SkeletonPtr skeleton = world->getSkeleton("skeleton 1");
  ASSERT_TRUE(skeleton != nullptr);
  ASSERT_EQ(skeleton->getNumSoftBodyNodes(), 1u);

  dynamics::SoftBodyNode* softBody = skeleton->getSoftBodyNode(0);
  ASSERT_TRUE(softBody != nullptr);
  ASSERT_TRUE(softBody->getGravityMode());
  ASSERT_GT(softBody->getNumPointMasses(), 0u);

  const Eigen::VectorXd expectedPointMassGravity
      = projectPointMassGravity(softBody, world->getGravity());
  const Eigen::MatrixXd expectedPointMassMass
      = projectPointMassMassMatrix(skeleton, softBody);
  ASSERT_GT(expectedPointMassGravity.norm(), 1.0e-9);
  ASSERT_GT(expectedPointMassMass.norm(), 1.0e-9);
  ASSERT_LT(skeleton->getVelocities().norm(), 1.0e-12);

  const Eigen::VectorXd withOriginalPointMasses = skeleton->getGravityForces();
  const Eigen::VectorXd withOriginalPointMassesCg
      = skeleton->getCoriolisAndGravityForces();
  const Eigen::MatrixXd withOriginalPointMassesM = skeleton->getMassMatrix();
  const Eigen::MatrixXd withOriginalPointMassesAugM
      = skeleton->getAugMassMatrix();
  const Eigen::MatrixXd withOriginalPointMassesInvM
      = skeleton->getInvMassMatrix();
  const Eigen::MatrixXd withOriginalPointMassesInvAugM
      = skeleton->getInvAugMassMatrix();

  const double skeletonMassBeforeHalving = skeleton->getMass();
  const double softBodyMassBeforeHalving = softBody->getMass();
  double halvedPointMassSum = 0.0;
  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    dynamics::PointMass* pointMass = softBody->getPointMass(i);
    ASSERT_TRUE(pointMass != nullptr);
    halvedPointMassSum += 0.5 * pointMass->getMass();
    pointMass->setMass(0.5 * pointMass->getMass());
  }

  // SoftBodyNode::getMass() shadows the non-virtual BodyNode::getMass(), so
  // Skeleton-level totals are rigid-only by construction and must not move
  // when point-mass masses change, while the soft body's own flesh-inclusive
  // mass reflects the halving. Point-mass gravity reaches the skeleton
  // through the aggregation paths verified below, not through the cached
  // skeleton total.
  EXPECT_NEAR(skeleton->getMass(), skeletonMassBeforeHalving, 1e-12);
  EXPECT_NEAR(
      softBody->getMass(),
      softBodyMassBeforeHalving - halvedPointMassSum,
      1e-12);

  const Eigen::VectorXd withHalfPointMasses = skeleton->getGravityForces();
  const Eigen::VectorXd withHalfPointMassesCg
      = skeleton->getCoriolisAndGravityForces();
  const Eigen::MatrixXd withHalfPointMassesM = skeleton->getMassMatrix();
  const Eigen::MatrixXd withHalfPointMassesAugM = skeleton->getAugMassMatrix();
  const Eigen::MatrixXd withHalfPointMassesInvM = skeleton->getInvMassMatrix();
  const Eigen::MatrixXd withHalfPointMassesInvAugM
      = skeleton->getInvAugMassMatrix();
  const Eigen::VectorXd actualPointMassDelta
      = withOriginalPointMasses - withHalfPointMasses;
  const Eigen::VectorXd actualPointMassCgDelta
      = withOriginalPointMassesCg - withHalfPointMassesCg;
  const Eigen::MatrixXd actualPointMassMassDelta
      = withOriginalPointMassesM - withHalfPointMassesM;
  const Eigen::MatrixXd actualPointMassAugMassDelta
      = withOriginalPointMassesAugM - withHalfPointMassesAugM;

  expectVectorNear(
      actualPointMassDelta,
      0.5 * expectedPointMassGravity,
      1.0e-10,
      "soft point-mass gravity projection");
  expectVectorNear(
      actualPointMassCgDelta,
      0.5 * expectedPointMassGravity,
      1.0e-10,
      "soft point-mass combined gravity projection");
  expectMatrixNear(
      actualPointMassMassDelta,
      0.5 * expectedPointMassMass,
      1.0e-10,
      "soft point-mass mass matrix projection");
  expectMatrixNear(
      actualPointMassAugMassDelta,
      0.5 * expectedPointMassMass,
      1.0e-10,
      "soft point-mass augmented mass matrix projection");

  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(
      skeleton->getNumDofs(), skeleton->getNumDofs());
  expectMatrixNear(
      withOriginalPointMassesM * withOriginalPointMassesInvM,
      identity,
      1.0e-10,
      "soft point-mass mass inverse left identity");
  expectMatrixNear(
      withOriginalPointMassesInvM * withOriginalPointMassesM,
      identity,
      1.0e-10,
      "soft point-mass mass inverse right identity");
  expectMatrixNear(
      withHalfPointMassesM * withHalfPointMassesInvM,
      identity,
      1.0e-10,
      "soft point-mass half mass inverse left identity");
  expectMatrixNear(
      withHalfPointMassesInvM * withHalfPointMassesM,
      identity,
      1.0e-10,
      "soft point-mass half mass inverse right identity");
  expectMatrixNear(
      withOriginalPointMassesAugM * withOriginalPointMassesInvAugM,
      identity,
      1.0e-10,
      "soft point-mass augmented inverse left identity");
  expectMatrixNear(
      withOriginalPointMassesInvAugM * withOriginalPointMassesAugM,
      identity,
      1.0e-10,
      "soft point-mass augmented inverse right identity");
  expectMatrixNear(
      withHalfPointMassesAugM * withHalfPointMassesInvAugM,
      identity,
      1.0e-10,
      "soft point-mass half augmented inverse left identity");
  expectMatrixNear(
      withHalfPointMassesInvAugM * withHalfPointMassesAugM,
      identity,
      1.0e-10,
      "soft point-mass half augmented inverse right identity");

  std::vector<Eigen::Vector3d> localExternalForces;
  localExternalForces.reserve(softBody->getNumPointMasses());
  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    dynamics::PointMass* pointMass = softBody->getPointMass(i);
    ASSERT_TRUE(pointMass != nullptr);

    const double scale = static_cast<double>(i + 1u);
    const Eigen::Vector3d localForce(0.01 * scale, -0.02 * scale, 0.03 * scale);
    pointMass->addExtForce(localForce, true);
    localExternalForces.push_back(localForce);
  }

  expectVectorNear(
      skeleton->getExternalForces(),
      projectPointMassExternalForces(skeleton, softBody, localExternalForces),
      1.0e-10,
      "soft point-mass external force projection");

  skeleton->clearExternalForces();
  expectVectorNear(
      skeleton->getExternalForces(),
      Eigen::VectorXd::Zero(skeleton->getNumDofs()),
      1.0e-12,
      "soft point-mass external force clear");
}
