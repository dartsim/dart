#ifndef DART_TESTS_INTEGRATION_DYNAMICS_DYNAMICS_TEST_FIXTURE_HPP_
#define DART_TESTS_INTEGRATION_DYNAMICS_DYNAMICS_TEST_FIXTURE_HPP_

#include "helpers/gtest_utils.hpp"

#include "dart/common/uri.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/dynamics/simple_frame.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <Eigen/Dense>

#include <vector>

class DynamicsTest : public ::testing::Test
{
public:
  // Get Skel file URI to test.
  const std::vector<dart::common::Uri>& getList() const;

  // Get reference frames
  const std::vector<dart::dynamics::SimpleFrame*>& getRefFrames() const;

  // Randomize the properties of all the reference frames
  void randomizeRefFrames();

  // Get mass matrix of _skel using Jacobians and inertias of each body
  // in _skel.
  Eigen::MatrixXd getMassMatrix(dart::dynamics::SkeletonPtr _skel);

  // Get augmented mass matrix of _skel using Jacobians and inertias of
  // each body in _skel.
  Eigen::MatrixXd getAugMassMatrix(dart::dynamics::SkeletonPtr _skel);

  // Compare velocities computed by recursive method, Jacobian, and finite
  // difference.
  void testJacobians(const dart::common::Uri& uri);

  // Compare velocities and accelerations with actual values and approximates
  // using finite difference method.
  void testFiniteDifferenceGeneralizedCoordinates(const dart::common::Uri& uri);

  // Compare spatial velocities computed by forward kinematics and finite
  // difference.
  void testFiniteDifferenceBodyNodeVelocity(const dart::common::Uri& uri);

  // Compare accelerations computed by recursive method, Jacobian, and finite
  // difference.
  void testFiniteDifferenceBodyNodeAcceleration(const dart::common::Uri& uri);

  // Test if the recursive forward kinematics algorithm computes
  // transformations, spatial velocities, and spatial accelerations correctly.
  void testForwardKinematics(const dart::common::Uri& uri);

  // Test inverse dynamics with various joint forces.
  void testInverseDynamics(const dart::common::Uri& uri);

  // Compare dynamics terms in equations of motion such as mass matrix, mass
  // inverse matrix, Coriolis force vector, gravity force vector, and external
  // force vector.
  void compareEquationsOfMotion(const dart::common::Uri& uri);

  // Test skeleton's COM and its related quantities.
  void testCenterOfMass(const dart::common::Uri& uri);

  // Test if the com acceleration is equal to the gravity
  void testCenterOfMassFreeFall(const dart::common::Uri& uri);

  // Test constraint impulses
  void testConstraintImpulse(const dart::common::Uri& uri);

  // Test impulse based dynamics
  void testImpulseBasedDynamics(const dart::common::Uri& uri);

protected:
  // Sets up the test fixture.
  void SetUp() override;
  void TearDown() override;

  // Skel file list.
  std::vector<dart::common::Uri> fileList;

  std::vector<dart::dynamics::SimpleFrame*> refFrames;
};

#endif // DART_TESTS_INTEGRATION_DYNAMICS_DYNAMICS_TEST_FIXTURE_HPP_
