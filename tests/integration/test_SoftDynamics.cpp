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

#include "dart/common/Console.hpp"
#include "dart/common/Macros.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/math/Constants.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/SkelParser.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <string>
#include <vector>

#include <cmath>

using namespace std;
using namespace Eigen;
using namespace dart;

//==============================================================================
/// Returns true if the two matrices are equal within the given bound
template <class MATRIX>
bool equals(
    const DenseBase<MATRIX>& A, const DenseBase<MATRIX>& B, double tol = 1e-5)
{
  // Get the matrix sizes and sanity check the call
  const std::size_t n1 = A.cols(), m1 = A.rows();
  const std::size_t n2 = B.cols(), m2 = B.rows();
  if (m1 != m2 || n1 != n2)
    return false;

  // Check each index
  for (std::size_t i = 0; i < m1; i++) {
    for (std::size_t j = 0; j < n1; j++) {
      if (std::isnan(A(i, j)) ^ std::isnan(B(i, j)))
        return false;
      else if (std::abs(A(i, j) - B(i, j)) > tol)
        return false;
    }
  }

  // If no problems, the two matrices are equal
  return true;
}

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
    std::size_t threads,
    std::size_t step)
{
  ASSERT_TRUE(world != nullptr) << scene.uri;

  const std::string worldContext = scene.uri
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

} // namespace

//==============================================================================
class SoftDynamicsTest : public ::testing::Test
{
public:
  // Get Skel file list to test.
  const std::vector<std::string>& getList();

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

  // Compare dynamics terms in equations of motion such as mass matrix, mass
  // inverse matrix, Coriolis force vector, gravity force vector, and external
  // force vector.
  void compareEquationsOfMotion(const std::string& _fileName);

protected:
  // Sets up the test fixture.
  void SetUp() override;

  // Skel file list.
  std::vector<std::string> list;
};

//==============================================================================
void SoftDynamicsTest::SetUp()
{
  list.push_back("dart://sample/skel/test/test_drop_box.skel");
}

//==============================================================================
const std::vector<std::string>& SoftDynamicsTest::getList()
{
  return list;
}

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
void SoftDynamicsTest::compareEquationsOfMotion(const std::string& _fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#if DART_BUILD_MODE_DEBUG
  std::size_t nRandomItr = 1;
#else
  std::size_t nRandomItr = 1;
#endif

  // Lower and upper bound of configuration for system
  double lb = -1.5 * constantsd::pi();
  double ub = 1.5 * constantsd::pi();

  // Lower and upper bound of joint damping and stiffness
  double lbD = 0.0;
  double ubD = 10.0;
  double lbK = 0.0;
  double ubK = 10.0;

  simulation::WorldPtr myWorld;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = utils::SkelParser::readWorld(_fileName);
  EXPECT_TRUE(myWorld != nullptr);

  for (std::size_t i = 0; i < myWorld->getNumSkeletons(); ++i) {
    dynamics::SkeletonPtr softSkel = myWorld->getSkeleton(i);

    int dof = softSkel->getNumDofs();
    //    int nBodyNodes     = skel->getNumBodyNodes();
    int nSoftBodyNodes = 0;
    if (softSkel != nullptr)
      nSoftBodyNodes = softSkel->getNumSoftBodyNodes();

    if (dof == 0) {
      dtmsg << "Skeleton [" << softSkel->getName()
            << "] is skipped since it has "
            << "0 DOF." << endl;
      continue;
    }

    for (std::size_t j = 0; j < nRandomItr; ++j) {
      // Random joint stiffness and damping coefficient
      for (std::size_t k = 0; k < softSkel->getNumBodyNodes(); ++k) {
        BodyNode* body = softSkel->getBodyNode(k);
        Joint* joint = body->getParentJoint();
        int localDof = joint->getNumDofs();

        for (int l = 0; l < localDof; ++l) {
          joint->setDampingCoefficient(l, Random::uniform(lbD, ubD));
          joint->setSpringStiffness(l, Random::uniform(lbK, ubK));

          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          joint->setRestPosition(l, Random::uniform(lbRP, ubRP));
        }
      }

      // Set random states
      Skeleton::Configuration x = softSkel->getConfiguration();
      for (auto k = 0u; k < softSkel->getNumDofs(); ++k) {
        x.mPositions[k] = Random::uniform(lb, ub);
        x.mVelocities[k] = Random::uniform(lb, ub);
      }
      softSkel->setConfiguration(x);

      //------------------------ Mass Matrix Test ----------------------------
      // Get matrices
      MatrixXd M = softSkel->getMassMatrix();
      MatrixXd M2 = getMassMatrix(softSkel);
      MatrixXd InvM = softSkel->getInvMassMatrix();
      MatrixXd M_InvM = M * InvM;
      MatrixXd InvM_M = InvM * M;

      MatrixXd AugM = softSkel->getAugMassMatrix();
      MatrixXd AugM2 = getAugMassMatrix(softSkel);
      MatrixXd InvAugM = softSkel->getInvAugMassMatrix();
      MatrixXd AugM_InvAugM = AugM * InvAugM;
      MatrixXd InvAugM_AugM = InvAugM * AugM;

      MatrixXd I = MatrixXd::Identity(dof, dof);

      // Check if the number of generalized coordinates and dimension of mass
      // matrix are same.
      EXPECT_EQ(M.rows(), dof);
      EXPECT_EQ(M.cols(), dof);

      // Check mass matrix
      EXPECT_TRUE(equals(M, M2, 1e-6));
      if (!equals(M, M2, 1e-6)) {
        cout << "M :" << endl << M << endl << endl;
        cout << "M2:" << endl << M2 << endl << endl;
      }

      // Check augmented mass matrix
      EXPECT_TRUE(equals(AugM, AugM2, 1e-6));
      if (!equals(AugM, AugM2, 1e-6)) {
        cout << "AugM :" << endl << AugM << endl << endl;
        cout << "AugM2:" << endl << AugM2 << endl << endl;
      }

      // Check if both of (M * InvM) and (InvM * M) are identity.
      EXPECT_TRUE(equals(M_InvM, I, 1e-6));
      if (!equals(M_InvM, I, 1e-6)) {
        cout << "InvM  :" << endl << InvM << endl << endl;
      }
      EXPECT_TRUE(equals(InvM_M, I, 1e-6));
      if (!equals(InvM_M, I, 1e-6)) {
        cout << "InvM_M:" << endl << InvM_M << endl << endl;
      }

      // Check if both of (M * InvM) and (InvM * M) are identity.
      EXPECT_TRUE(equals(AugM_InvAugM, I, 1e-6));
      if (!equals(AugM_InvAugM, I, 1e-6)) {
        cout << "InvAugM  :" << endl << InvAugM << endl << endl;
        cout << "InvAugM2  :" << endl << AugM.inverse() << endl << endl;
        cout << "AugM_InvAugM  :" << endl << AugM_InvAugM << endl << endl;
      }
      EXPECT_TRUE(equals(InvAugM_AugM, I, 1e-6));
      if (!equals(InvAugM_AugM, I, 1e-6)) {
        cout << "InvAugM_AugM:" << endl << InvAugM_AugM << endl << endl;
      }

      //------- Coriolis Force Vector and Combined Force Vector Tests --------
      // Get C1, Coriolis force vector using recursive method
      VectorXd C = softSkel->getCoriolisForces();
      VectorXd Cg = softSkel->getCoriolisAndGravityForces();

      // Get C2, Coriolis force vector using inverse dynamics algorithm
      Vector3d oldGravity = softSkel->getGravity();
      VectorXd oldTau = softSkel->getForces();
      VectorXd oldDdq = softSkel->getAccelerations();
      // TODO(JS): Save external forces of body nodes
      vector<double> oldKv(nSoftBodyNodes, 0.0);
      vector<double> oldKe(nSoftBodyNodes, 0.0);
      vector<double> oldD(nSoftBodyNodes, 0.0);
      for (int k = 0; k < nSoftBodyNodes; ++k) {
        DART_ASSERT(softSkel != nullptr);
        dynamics::SoftBodyNode* sbn = softSkel->getSoftBodyNode(k);
        oldKv[k] = sbn->getVertexSpringStiffness();
        oldKe[k] = sbn->getEdgeSpringStiffness();
        oldD[k] = sbn->getDampingCoefficient();
      }

      softSkel->resetGeneralizedForces();
      softSkel->clearExternalForces();
      softSkel->setAccelerations(VectorXd::Zero(dof));
      for (int k = 0; k < nSoftBodyNodes; ++k) {
        DART_ASSERT(softSkel != nullptr);
        dynamics::SoftBodyNode* sbn = softSkel->getSoftBodyNode(k);
        sbn->setVertexSpringStiffness(0.0);
        sbn->setEdgeSpringStiffness(0.0);
        sbn->setDampingCoefficient(0.0);
      }

      EXPECT_TRUE(softSkel->getForces() == VectorXd::Zero(dof));
      EXPECT_TRUE(softSkel->getExternalForces() == VectorXd::Zero(dof));
      EXPECT_TRUE(softSkel->getAccelerations() == VectorXd::Zero(dof));

      softSkel->setGravity(Vector3d::Zero());
      EXPECT_TRUE(softSkel->getGravity() == Vector3d::Zero());
      softSkel->computeInverseDynamics(false, false);
      VectorXd C2 = softSkel->getForces();

      softSkel->setGravity(oldGravity);
      EXPECT_TRUE(softSkel->getGravity() == oldGravity);
      softSkel->computeInverseDynamics(false, false);
      VectorXd Cg2 = softSkel->getForces();

      EXPECT_TRUE(equals(C, C2, 1e-6));
      if (!equals(C, C2, 1e-6)) {
        cout << "C :" << C.transpose() << endl;
        cout << "C2:" << C2.transpose() << endl;
      }

      EXPECT_TRUE(equals(Cg, Cg2, 1e-6));
      if (!equals(Cg, Cg2, 1e-6)) {
        cout << "Cg :" << Cg.transpose() << endl;
        cout << "Cg2:" << Cg2.transpose() << endl;
      }

      softSkel->setForces(oldTau);
      softSkel->setAccelerations(oldDdq);
      // TODO(JS): Restore external forces of body nodes
    }
  }
}

//==============================================================================
TEST_F(SoftDynamicsTest, compareEquationsOfMotion)
{
  // TODO(JS): Equations of motion for softbody skeleton is not done yet

  //  for (int i = 0; i < getList().size(); ++i)
  //  {
  // #if DART_BUILD_MODE_DEBUG
  //    dtdbg << getList()[i] << std::endl;
  // #endif
  //    compareEquationsOfMotion(getList()[i]);
  //  }
}

//==============================================================================
TEST_F(SoftDynamicsTest, finiteStateForRepresentativeSoftScenes)
{
  const std::array<SoftStabilityScene, 7> scenes = {{
      {"dart://sample/skel/test/test_drop_box.skel", 1u, 26u, 30u},
      {"dart://sample/skel/test/test_drop_low_stiffness.skel", 1u, 26u, 30u},
      {"dart://sample/skel/test/test_double_pendulum.skel", 2u, 52u, 30u},
      {"dart://sample/skel/test/test_adaptive_deformable.skel", 1u, 12u, 30u},
      {"dart://sample/skel/soft_cubes.skel", 2u, 52u, 30u},
      {"dart://sample/skel/softBodies.skel", 5u, 290u, 30u},
      {"dart://sample/skel/soft_open_chain.skel", 5u, 120u, 30u},
  }};
  const std::array<std::size_t, 2> threadCounts = {{1u, 4u}};

  for (const SoftStabilityScene& scene : scenes) {
    std::vector<SoftStateSnapshot> finalSnapshots;
    finalSnapshots.reserve(threadCounts.size());

    for (const std::size_t threads : threadCounts) {
      simulation::WorldPtr world = utils::SkelParser::readWorld(scene.uri);
      ASSERT_TRUE(world != nullptr) << scene.uri;
      world->setNumSimulationThreads(threads);

      expectFiniteWorldState(world, scene, threads, 0u);

      for (std::size_t step = 1u; step <= scene.steps; ++step) {
        world->step();

        if (step == scene.steps || step % 10u == 0u)
          expectFiniteWorldState(world, scene, threads, step);
      }

      finalSnapshots.push_back(computeSoftStateSnapshot(world));
    }

    ASSERT_EQ(finalSnapshots.size(), threadCounts.size()) << scene.uri;
    expectSnapshotsNear(
        finalSnapshots[0],
        finalSnapshots[1],
        scene.uri + " threads=1-vs-4 final state");
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

  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    dynamics::PointMass* pointMass = softBody->getPointMass(i);
    ASSERT_TRUE(pointMass != nullptr);
    pointMass->setMass(0.5 * pointMass->getMass());
  }

  const Eigen::VectorXd withHalfPointMasses = skeleton->getGravityForces();
  const Eigen::VectorXd withHalfPointMassesCg
      = skeleton->getCoriolisAndGravityForces();
  const Eigen::MatrixXd withHalfPointMassesM = skeleton->getMassMatrix();
  const Eigen::VectorXd actualPointMassDelta
      = withOriginalPointMasses - withHalfPointMasses;
  const Eigen::VectorXd actualPointMassCgDelta
      = withOriginalPointMassesCg - withHalfPointMassesCg;
  const Eigen::MatrixXd actualPointMassMassDelta
      = withOriginalPointMassesM - withHalfPointMassesM;

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
}
