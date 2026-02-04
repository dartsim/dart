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

#include "dart/collision/collision_result.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/joint.hpp"
#include "dart/dynamics/point_mass.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/dynamics/soft_body_node.hpp"
#include "dart/dynamics/weld_joint.hpp"
#include "dart/io/read.hpp"
#include "dart/math/constants.hpp"
#include "dart/simulation/world.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

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
  if (m1 != m2 || n1 != n2) {
    return false;
  }

  // Check each index
  for (std::size_t i = 0; i < m1; i++) {
    for (std::size_t j = 0; j < n1; j++) {
      if (std::isnan(A(i, j)) ^ std::isnan(B(i, j))) {
        return false;
      } else if (std::abs(A(i, j) - B(i, j)) > tol) {
        return false;
      }
    }
  }

  // If no problems, the two matrices are equal
  return true;
}

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
  void compareEquationsOfMotion(const std::string& fileName);

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
void SoftDynamicsTest::compareEquationsOfMotion(const std::string& fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#if !defined(NDEBUG)
  std::size_t nRandomItr = 1;
#else
  std::size_t nRandomItr = 1;
#endif

  // Lower and upper bound of configuration for system
  double lb = -1.5 * pi;
  double ub = 1.5 * pi;

  // Lower and upper bound of joint damping and stiffness
  double lbD = 0.0;
  double ubD = 10.0;
  double lbK = 0.0;
  double ubK = 10.0;

  simulation::WorldPtr myWorld;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = dart::io::readWorld(common::Uri(fileName));
  EXPECT_TRUE(myWorld != nullptr);

  for (std::size_t i = 0; i < myWorld->getNumSkeletons(); ++i) {
    dynamics::SkeletonPtr softSkel = myWorld->getSkeleton(i);

    int dof = softSkel->getNumDofs();
    //    int nBodyNodes     = skel->getNumBodyNodes();
    int nSoftBodyNodes = 0;
    if (softSkel != nullptr) {
      nSoftBodyNodes = softSkel->getNumSoftBodyNodes();
    }

    if (dof == 0) {
      DART_INFO(
          "Skeleton [{}] is skipped since it has 0 DOF.", softSkel->getName());
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
  // #if !defined(NDEBUG)
  //    DART_DEBUG("{}", getList()[i]);
  // #endif
  //    compareEquationsOfMotion(getList()[i]);
  //  }
}

//==============================================================================
// Test that negative damping coefficient is clamped to zero with a warning
// (rather than crashing via assertion). This ensures robustness when loading
// models with invalid parameters.
//==============================================================================
TEST(SoftBodyNode, NegativeDampingClampedToZero)
{
  using namespace dart::dynamics;
  using namespace dart::io;

  // Load a skeleton with soft body nodes
  auto world
      = readWorld(common::Uri("dart://sample/skel/test/test_drop_box.skel"));
  ASSERT_TRUE(world != nullptr);

  SkeletonPtr skel = nullptr;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    if (world->getSkeleton(i)->getNumSoftBodyNodes() > 0) {
      skel = world->getSkeleton(i);
      break;
    }
  }
  ASSERT_TRUE(skel != nullptr);
  ASSERT_GT(skel->getNumSoftBodyNodes(), 0);

  SoftBodyNode* sbn = skel->getSoftBodyNode(0);
  ASSERT_TRUE(sbn != nullptr);

  // Set negative damping - should be clamped to 0
  sbn->setDampingCoefficient(-5.0);
  EXPECT_EQ(sbn->getDampingCoefficient(), 0.0);

  // Positive damping should work normally
  sbn->setDampingCoefficient(10.0);
  EXPECT_EQ(sbn->getDampingCoefficient(), 10.0);

  // Zero damping should work
  sbn->setDampingCoefficient(0.0);
  EXPECT_EQ(sbn->getDampingCoefficient(), 0.0);
}

//==============================================================================
// Test that NaN and Inf damping coefficients are clamped to zero with a
// warning. This preserves the invariant from the original DART_ASSERT(d >= 0.0)
// which would reject NaN (since NaN >= 0.0 is false).
//==============================================================================
TEST(SoftBodyNode, NaNDampingClampedToZero)
{
  using namespace dart::dynamics;
  using namespace dart::io;

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  // Load a skeleton with soft body nodes
  auto world
      = readWorld(common::Uri("dart://sample/skel/test/test_drop_box.skel"));
  ASSERT_TRUE(world != nullptr);

  SkeletonPtr skel = nullptr;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    if (world->getSkeleton(i)->getNumSoftBodyNodes() > 0) {
      skel = world->getSkeleton(i);
      break;
    }
  }
  ASSERT_TRUE(skel != nullptr);
  ASSERT_GT(skel->getNumSoftBodyNodes(), 0);

  SoftBodyNode* sbn = skel->getSoftBodyNode(0);
  ASSERT_TRUE(sbn != nullptr);

  // NaN damping should be clamped to 0
  sbn->setDampingCoefficient(nan);
  EXPECT_EQ(sbn->getDampingCoefficient(), 0.0);

  // +Inf damping should be clamped to 0
  sbn->setDampingCoefficient(inf);
  EXPECT_EQ(sbn->getDampingCoefficient(), 0.0);

  // -Inf damping should be clamped to 0
  sbn->setDampingCoefficient(-inf);
  EXPECT_EQ(sbn->getDampingCoefficient(), 0.0);

  // Valid value should still work
  sbn->setDampingCoefficient(5.0);
  EXPECT_EQ(sbn->getDampingCoefficient(), 5.0);
}

TEST(SoftDynamics, StepSoftBodyExercisesPhysicsPaths)
{
  auto world = io::readWorld("dart://sample/skel/softBodies.skel");
  ASSERT_NE(world, nullptr);

  dynamics::SoftBodyNode* softBody = nullptr;
  dynamics::SkeletonPtr softSkel;
  for (std::size_t s = 0; s < world->getNumSkeletons(); ++s) {
    auto skel = world->getSkeleton(s);
    if (skel->getNumSoftBodyNodes() > 0) {
      softBody = skel->getSoftBodyNode(0);
      softSkel = skel;
      break;
    }
  }
  ASSERT_NE(softBody, nullptr);
  EXPECT_GT(softBody->getNumPointMasses(), 0u);

  for (int i = 0; i < 20; ++i) {
    world->step();
  }

  EXPECT_TRUE(softSkel->getPositions().array().isFinite().all());
  EXPECT_TRUE(softSkel->getVelocities().array().isFinite().all());
}

TEST(SoftDynamics, GravityToggleAndExternalForce)
{
  auto world = io::readWorld("dart://sample/skel/softBodies.skel");
  ASSERT_NE(world, nullptr);

  dynamics::SoftBodyNode* softBody = nullptr;
  dynamics::SkeletonPtr softSkel;
  for (std::size_t s = 0; s < world->getNumSkeletons(); ++s) {
    auto skel = world->getSkeleton(s);
    if (skel->getNumSoftBodyNodes() > 0) {
      softBody = skel->getSoftBodyNode(0);
      softSkel = skel;
      break;
    }
  }
  ASSERT_NE(softBody, nullptr);

  softBody->setGravityMode(false);
  world->step();
  EXPECT_TRUE(softSkel->getPositions().array().isFinite().all());

  softBody->setGravityMode(true);
  softBody->setExtForce(Eigen::Vector3d(0.0, 0.0, 1.0));
  world->step();
  EXPECT_TRUE(softSkel->getPositions().array().isFinite().all());

  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    auto* pm = softBody->getPointMass(i);
    pm->addExtForce(Eigen::Vector3d(0.0, 0.0, 0.1));
  }
  world->step();
  EXPECT_TRUE(softSkel->getPositions().array().isFinite().all());
}

TEST(SoftDynamics, SoftBodyContactAndInternalForces)
{
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);

  auto ground = dynamics::Skeleton::create("ground");
  auto groundPair = ground->createJointAndBodyNodePair<dynamics::WeldJoint>();
  auto* groundBody = groundPair.second;
  auto groundShape
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(5.0, 5.0, 0.2));
  groundBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(groundShape);
  Eigen::Isometry3d groundTf = Eigen::Isometry3d::Identity();
  groundTf.translation().z() = -0.1;
  groundPair.first->setTransformFromParentBodyNode(groundTf);
  world->addSkeleton(ground);

  auto softSkel = dynamics::Skeleton::create("soft");
  auto softPair = softSkel->createJointAndBodyNodePair<
      dynamics::FreeJoint,
      dynamics::SoftBodyNode>();
  auto* softBody = softPair.second;

  dynamics::SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d(0.5, 0.5, 0.5),
      Eigen::Isometry3d::Identity(),
      2.0,
      50.0,
      20.0,
      0.1);
  softBody->setVertexSpringStiffness(30.0);
  softBody->setEdgeSpringStiffness(15.0);
  softBody->setDampingCoefficient(0.5);

  Eigen::Isometry3d softTf = Eigen::Isometry3d::Identity();
  softTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.05);
  dynamics::FreeJoint::setTransformOf(
      softSkel.get(), softTf, dynamics::Frame::World(), true);
  world->addSkeleton(softSkel);

  for (int i = 0; i < 5; ++i) {
    world->step();
  }

  const auto& result = world->getLastCollisionResult();
  EXPECT_GT(result.getNumContacts(), 0u);

  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);
  EXPECT_TRUE(pm->getForces().allFinite());
  EXPECT_TRUE(pm->getConstraintImpulses().allFinite());
  EXPECT_TRUE(pm->getBodyVelocityChange().allFinite());
}

TEST(SoftDynamics, MultipleSoftSkeletonsInteract)
{
  auto world = io::readWorld("dart://sample/skel/soft_cubes.skel");
  ASSERT_NE(world, nullptr);

  for (int i = 0; i < 30; ++i) {
    world->step();
  }

  for (std::size_t s = 0; s < world->getNumSkeletons(); ++s) {
    auto skel = world->getSkeleton(s);
    EXPECT_TRUE(skel->getPositions().array().isFinite().all());
  }
}

TEST(SoftDynamics, WorldConfigurationAndSkeletonManagement)
{
  auto world = simulation::World::create();
  ASSERT_NE(world, nullptr);

  world->setTimeStep(0.0025);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.0025);

  const Eigen::Vector3d gravity(0.0, -1.0, -9.81);
  world->setGravity(gravity);
  EXPECT_TRUE(world->getGravity().isApprox(gravity));

  auto skeleton = dynamics::Skeleton::create("world-skeleton");
  skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  EXPECT_EQ(world->getNumSkeletons(), 0u);
  world->addSkeleton(skeleton);
  EXPECT_EQ(world->getNumSkeletons(), 1u);

  world->step();
  EXPECT_GT(world->getTime(), 0.0);

  world->reset();
  EXPECT_DOUBLE_EQ(world->getTime(), 0.0);

  world->removeSkeleton(skeleton);
  EXPECT_EQ(world->getNumSkeletons(), 0u);
}

TEST(SoftDynamics, InverseDynamicsAggregation)
{
  using namespace dart::dynamics;
  using namespace dart::simulation;

  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  auto skel = Skeleton::create("soft_inv");
  auto pair = skel->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d(0.3, 0.3, 0.3),
      Eigen::Isometry3d::Identity(),
      1.0,
      8.0,
      4.0,
      0.05);

  world->addSkeleton(skel);

  // Set some velocities so computeInverseDynamics has non-trivial input
  skel->setVelocities(Eigen::VectorXd::Constant(skel->getNumDofs(), 0.1));

  // computeInverseDynamics calls aggregateGravityForceVector and
  // aggregateExternalForces
  skel->computeInverseDynamics();

  // Verify gravity and external force vectors are finite
  EXPECT_TRUE(skel->getGravityForces().allFinite());
  EXPECT_TRUE(skel->getExternalForces().allFinite());
  EXPECT_TRUE(skel->getCoriolisAndGravityForces().allFinite());
}
