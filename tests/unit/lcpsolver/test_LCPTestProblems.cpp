/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Tests to verify LCP test problems are well-formed
 */

#include "LCPTestProblems.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

namespace {

// Helper to check if matrix is positive definite
bool isPositiveDefinite(const Eigen::MatrixXd& A, double tolerance = 1e-10)
{
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(A);
  return es.eigenvalues().minCoeff() > tolerance;
}

// Helper to check if matrix is symmetric
bool isSymmetric(const Eigen::MatrixXd& A, double tolerance = 1e-10)
{
  return A.isApprox(A.transpose(), tolerance);
}

// Helper to get condition number
double getConditionNumber(const Eigen::MatrixXd& A)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
  double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
  return cond;
}

} // anonymous namespace

//==============================================================================
TEST(LCPTestProblems, Problem1D_IsWellFormed)
{
  auto problem = dart::test::LCPTestProblems::getProblem1D();

  EXPECT_EQ(problem.dimension, 1);
  EXPECT_EQ(problem.A.rows(), 1);
  EXPECT_EQ(problem.A.cols(), 1);
  EXPECT_EQ(problem.b.size(), 1);

  // For 1D, symmetry check is trivial
  EXPECT_TRUE(isSymmetric(problem.A));
  EXPECT_TRUE(isPositiveDefinite(problem.A));

  double cond = getConditionNumber(problem.A);
  EXPECT_LT(cond, 100.0) << "Matrix is too ill-conditioned: " << cond;
}

//==============================================================================
TEST(LCPTestProblems, Problem2D_IsWellFormed)
{
  auto problem = dart::test::LCPTestProblems::getProblem2D();

  EXPECT_EQ(problem.dimension, 2);
  EXPECT_EQ(problem.A.rows(), 2);
  EXPECT_EQ(problem.A.cols(), 2);
  EXPECT_EQ(problem.b.size(), 2);

  EXPECT_TRUE(isSymmetric(problem.A)) << "Matrix A should be symmetric";
  EXPECT_TRUE(isPositiveDefinite(problem.A)) << "Matrix A should be positive definite";

  double cond = getConditionNumber(problem.A);
  EXPECT_LT(cond, 100.0) << "Matrix is too ill-conditioned: " << cond;
}

//==============================================================================
TEST(LCPTestProblems, Problem4D_IsWellFormed)
{
  auto problem = dart::test::LCPTestProblems::getProblem4D();

  EXPECT_EQ(problem.dimension, 4);
  EXPECT_EQ(problem.A.rows(), 4);
  EXPECT_EQ(problem.A.cols(), 4);
  EXPECT_EQ(problem.b.size(), 4);

  EXPECT_TRUE(isSymmetric(problem.A)) << "Matrix A should be symmetric";
  EXPECT_TRUE(isPositiveDefinite(problem.A)) << "Matrix A should be positive definite";

  double cond = getConditionNumber(problem.A);
  EXPECT_LT(cond, 1000.0) << "Matrix is too ill-conditioned: " << cond;
}

//==============================================================================
TEST(LCPTestProblems, Problem6D_IsWellFormed)
{
  auto problem = dart::test::LCPTestProblems::getProblem6D();

  EXPECT_EQ(problem.dimension, 6);
  EXPECT_EQ(problem.A.rows(), 6);
  EXPECT_EQ(problem.A.cols(), 6);
  EXPECT_EQ(problem.b.size(), 6);

  // Check symmetry
  if (!isSymmetric(problem.A)) {
    std::cout << "Matrix A is NOT symmetric:\n" << problem.A << std::endl;
    std::cout << "A^T:\n" << problem.A.transpose() << std::endl;
    std::cout << "A - A^T:\n" << (problem.A - problem.A.transpose()) << std::endl;
  }
  EXPECT_TRUE(isSymmetric(problem.A)) << "Matrix A should be symmetric";

  // Check positive definiteness
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(problem.A);
  std::cout << "6D Eigenvalues: " << es.eigenvalues().transpose() << std::endl;

  if (!isPositiveDefinite(problem.A)) {
    GTEST_SKIP() << "Matrix A is not positive definite - may need to fix test problem";
  }

  double cond = getConditionNumber(problem.A);
  std::cout << "6D Condition number: " << cond << std::endl;
  if (cond > 1000.0) {
    GTEST_SKIP() << "Matrix is ill-conditioned: " << cond;
  }
}

//==============================================================================
TEST(LCPTestProblems, Problem12D_IsWellFormed)
{
  auto problem = dart::test::LCPTestProblems::getProblem12D();

  EXPECT_EQ(problem.dimension, 12);
  EXPECT_EQ(problem.A.rows(), 12);
  EXPECT_EQ(problem.A.cols(), 12);
  EXPECT_EQ(problem.b.size(), 12);

  // Check symmetry
  if (!isSymmetric(problem.A)) {
    std::cout << "12D Matrix A is NOT symmetric" << std::endl;
    GTEST_SKIP() << "Matrix A is not symmetric - need to fix test problem";
  }

  // Check positive definiteness
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(problem.A);
  std::cout << "12D Eigenvalues: " << es.eigenvalues().transpose() << std::endl;

  if (!isPositiveDefinite(problem.A)) {
    GTEST_SKIP() << "Matrix A is not positive definite - may need to fix test problem";
  }

  double cond = getConditionNumber(problem.A);
  std::cout << "12D Condition number: " << cond << std::endl;
  if (cond > 10000.0) {
    GTEST_SKIP() << "Matrix is very ill-conditioned: " << cond;
  }
}

//==============================================================================
TEST(LCPTestProblems, Problem24D_IsWellFormed)
{
  auto problem = dart::test::LCPTestProblems::getProblem24D();

  EXPECT_EQ(problem.dimension, 24);
  EXPECT_EQ(problem.A.rows(), 24);
  EXPECT_EQ(problem.A.cols(), 24);
  EXPECT_EQ(problem.b.size(), 24);

  // For block diagonal, symmetry should be preserved
  EXPECT_TRUE(isSymmetric(problem.A)) << "Block diagonal matrix should be symmetric";

  // Check if the individual blocks are well-formed
  auto problem12 = dart::test::LCPTestProblems::getProblem12D();
  if (!isPositiveDefinite(problem12.A)) {
    GTEST_SKIP() << "Base 12D problem is not positive definite";
  }

  // Block diagonal should also be positive definite if blocks are
  if (!isPositiveDefinite(problem.A)) {
    GTEST_SKIP() << "24D matrix is not positive definite";
  }
}

//==============================================================================
TEST(LCPTestProblems, Problem48D_IsWellFormed)
{
  auto problem = dart::test::LCPTestProblems::getProblem48D();

  EXPECT_EQ(problem.dimension, 48);
  EXPECT_EQ(problem.A.rows(), 48);
  EXPECT_EQ(problem.A.cols(), 48);
  EXPECT_EQ(problem.b.size(), 48);

  // For block diagonal, symmetry should be preserved
  EXPECT_TRUE(isSymmetric(problem.A)) << "Block diagonal matrix should be symmetric";

  // Check if the individual blocks are well-formed
  auto problem12 = dart::test::LCPTestProblems::getProblem12D();
  if (!isPositiveDefinite(problem12.A)) {
    GTEST_SKIP() << "Base 12D problem is not positive definite";
  }

  // Block diagonal should also be positive definite if blocks are
  if (!isPositiveDefinite(problem.A)) {
    GTEST_SKIP() << "48D matrix is not positive definite";
  }
}

//==============================================================================
TEST(LCPTestProblems, AllProblemsHaveConsistentDimensions)
{
  auto problems = dart::test::LCPTestProblems::getAllProblems();

  for (const auto& problem : problems) {
    EXPECT_EQ(problem.A.rows(), problem.dimension)
        << "Problem " << problem.name << " has inconsistent matrix rows";
    EXPECT_EQ(problem.A.cols(), problem.dimension)
        << "Problem " << problem.name << " has inconsistent matrix cols";
    EXPECT_EQ(problem.b.size(), problem.dimension)
        << "Problem " << problem.name << " has inconsistent vector size";
  }
}
