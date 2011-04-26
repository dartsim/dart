#ifndef SRC_UNITTESTS_TEST_UTILS_H
#define SRC_UNITTESTS_TEST_UTILS_H

#include "utils/rotation_conversion.h"
#include "utils/utils.h"

TEST(UTILS, ROTATION_CONVERSION) {
  using namespace utils::rot_conv;
  
  // Create Initial ExpMap
  Vector3d axis(2.0, 1.0, 1.0);
  axis.normalize();
  double angle = 1.2;
  EXPECT_DOUBLE_EQ(axis.norm(), 1.0);
  Vector3d expmap = axis * angle;


  // Test conversion between expmap and quaternion
  Quaterniond q = exp_to_quat(expmap);
  Vector3d expmap2 = quat_to_exp(q);

  EXPECT_NEAR((expmap - expmap2).norm(), 0.0, EPSILON)
    << "Orig: " << expmap << " Reconstructed: " << expmap2;
  
  // Test conversion between matrix and euler
  Matrix3d m = quat_to_matrix(q);
  Vector3d e = matrix_to_euler(m, XYZ);
  Matrix3d m2 = euler_to_matrix(e, XYZ);

  EXPECT_NEAR((m - m2).norm(), 0.0, EPSILON)
    << "Orig: " << m << " Reconstructed: " << m2;
}

TEST(UTILS, UTILS) {
  // Test CR Matrix
  EXPECT_DOUBLE_EQ(utils::CR(0, 1), -1.0);

  // Test randomize function
  double x = utils::random(0.0, 2.0);
  EXPECT_LT(0.0, x);
  EXPECT_LT(x, 2.0);

  // Test transform
  Matrix4d M;
  M << 1.0, 0.0, 0.0, 3.0,
    0.0, 1.0, 0.0, 2.0,
    0.0, 0.0, 1.0, 1.0,
    0.0, 0.0, 0.0, 1.0;
  Vector3d pt(1.0, 0.5, 1.0);
  Vector3d result = utils::transform(M, pt);
  Vector3d expected(4.0, 2.5, 2.0);
  EXPECT_NEAR( (result - expected).norm(), 0.0, EPSILON)
    << "result = " << result << " expected = " << expected;
  
}


#endif
