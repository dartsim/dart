/*
 * Copyright (c) 2015, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include <dart/collision/dart/dart.hpp>

#include <gtest/gtest.h>

#include "TestHelpers.hpp"

using namespace dart;

//==============================================================================
Eigen::Vector3d makeRandomVectorDifferentFrom(
    const Eigen::Vector3d& point1,
    const Eigen::Vector3d& point2 = Eigen::Vector3d::Zero(),
    const double randomTrials = 1000)
{
  Eigen::Vector3d vec = Eigen::Vector3d::Random();

  for (auto i = 0u; i < randomTrials; ++i)
  {
    if (vec.isApprox(point1) || vec.isApprox(point2))
      vec.setRandom();
  }

  return vec;
}

//==============================================================================
void makeRandomTriangle(
    Eigen::Vector3d& a1, Eigen::Vector3d& a2, Eigen::Vector3d& a3)
{
  a1.setRandom();
  a2 = makeRandomVectorDifferentFrom(a1);
  a3 = makeRandomVectorDifferentFrom(a1, a2);
}

//==============================================================================
Eigen::Vector3d makePointOnPlane(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3)
{
  const Eigen::Vector2d coeffs = Eigen::Vector2d::Random();

  const Eigen::Vector3d p21 = p2 - p1;
  const Eigen::Vector3d p31 = p3 - p1;

  return p1 + coeffs[0] * p21 + coeffs[1] * p31;
}

//==============================================================================
Eigen::Vector3d makePointWithinTriangle(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3)
{
  Eigen::Vector3d coeffs = Eigen::Vector3d::Random();
  coeffs = coeffs.cwiseAbs();
  coeffs = coeffs / coeffs.sum();

  return coeffs[0] * p1 + coeffs[1] * p2 + coeffs[2] * p3;
}

//==============================================================================
Eigen::Vector3d makePointStrictlyWithinTriangle(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3)
{
  Eigen::Vector3d coeffs = Eigen::Vector3d::Random();
  coeffs = coeffs.cwiseAbs();
  coeffs = coeffs / (coeffs.sum());
  coeffs += Eigen::Vector3d::Constant(0.05);
  coeffs = coeffs / (coeffs.sum());

  return coeffs[0] * p1 + coeffs[1] * p2 + coeffs[2] * p3;
}

//==============================================================================
Eigen::Vector3d makePointOnEdgeOfTriangle(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3)
{
  const int index1 = math::random(0, 2);
  const int index2 = index1 + 1 == 3 ? 0 : index1 + 1;

  const std::vector<Eigen::Vector3d> points{p1, p2, p3};

  const Eigen::Vector3d vec = points[index2] - points[index1];
  const auto t = math::random(0.0, vec.norm());

  return points[index1] + t * vec;
}

//==============================================================================
Eigen::Vector3d makePointStrictlyOnEdgeOfTriangle(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3)
{
  const int index1 = math::random(0, 2);
  const int index2 = index1 + 1 == 3 ? 0 : index1 + 1;

  const std::vector<Eigen::Vector3d> points{p1, p2, p3};

  const Eigen::Vector3d vec = points[index2] - points[index1];
  const auto margin = vec.norm() * 0.01;
  const auto t = math::random(margin, vec.norm() - margin);

  return points[index1] + t * vec;
}

//==============================================================================
Eigen::Vector3d makePointOnVertexOfTriangle(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3)
{
  const int index = math::random(0, 2);
  const std::vector<Eigen::Vector3d> points{p1, p2, p3};

  return points[index];
}

//==============================================================================
Eigen::Vector3d makePointAbovePlane(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3)
{
  const Eigen::Vector3d p21 = p2 - p1;
  const Eigen::Vector3d p32 = p3 - p2;
  const Eigen::Vector3d n = p21.cross(p32).normalized();

  const auto range = 10.0;
  Eigen::Vector3d random = Eigen::Vector3d::Random();
  random.normalize();
  random = range * random;

  const Eigen::Vector3d center = (p1 + p2 + p3) / 3.0;

  return center + range * n + random;
}

//==============================================================================
Eigen::Vector3d makePointStrictlyAbovePlane(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3)
{
  const Eigen::Vector3d p21 = p2 - p1;
  const Eigen::Vector3d p32 = p3 - p2;
  const Eigen::Vector3d n = p21.cross(p32).normalized();

  const auto range = 1.0;
  const auto margin = range * 0.01;
  Eigen::Vector3d random = Eigen::Vector3d::Random();
  random.normalize();
  random = (range - margin) * random;

  const Eigen::Vector3d center = (p1 + p2 + p3) / 3.0;

  return center + range * n + random;
}

//==============================================================================
//int makeTriangleAndTriangleOneVertexIsOnTheOtherPlane(
//    Eigen::Vector3d& a1, Eigen::Vector3d& a2, Eigen::Vector3d& a3,
//    Eigen::Vector3d& b1, Eigen::Vector3d& b2, Eigen::Vector3d& b3,
//    Eigen::Vector3d& contact1)
//{
//  makeRandomTriangle(a1, a2, a3);

//  b1 = makePointOnPlane(a1, a2, a3);
//  b2 = makePointAbovePlane(a1, a2, a3);
//  b3 = makePointAbovePlane(a1, a2, a3);

//  contact1 = b1;

//  return 1;
//}

//==============================================================================
void makeTriangleAndTriangleOneVertexIsWithinOtherTriangle(
    Eigen::Vector3d& a1, Eigen::Vector3d& a2, Eigen::Vector3d& a3,
    Eigen::Vector3d& b1, Eigen::Vector3d& b2, Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1)
{
  makeRandomTriangle(a1, a2, a3);

  b1 = makePointStrictlyWithinTriangle(a1, a2, a3);
  b2 = makePointStrictlyAbovePlane(a1, a2, a3);
  b3 = makePointStrictlyAbovePlane(a1, a2, a3);

#ifndef NDEBUG
  Eigen::Vector3d a21 = a2 - a1;
  Eigen::Vector3d a32 = a3 - a2;
  Eigen::Vector3d n = a21.cross(a32);
  auto dot1 = n.dot(a1);
  auto dot2 = n.dot(a2);
  auto dot3 = n.dot(a3);
  auto dot4 = n.dot(b1);
  assert(std::abs(dot1 - dot2) < 1e-6);
  assert(std::abs(dot2 - dot3) < 1e-6);
  assert(std::abs(dot3 - dot4) < 1e-6);
#endif

  contact1 = b1;
}

//==============================================================================
TEST(CollideTriangleTriangle, Basic)
{
#ifdef NDEBUG
  const auto numTests = 5e+4;
#else
  const auto numTests = 1e+2;
#endif

  std::vector<Eigen::Vector3d> a1(numTests);
  std::vector<Eigen::Vector3d> a2(numTests);
  std::vector<Eigen::Vector3d> a3(numTests);
  std::vector<Eigen::Vector3d> b1(numTests);
  std::vector<Eigen::Vector3d> b2(numTests);
  std::vector<Eigen::Vector3d> b3(numTests);
  std::vector<Eigen::Vector3d> contact1(numTests);
  std::vector<Eigen::Vector3d> contact2(numTests);
  std::vector<Eigen::Vector3d> contact3(numTests);
  std::vector<Eigen::Vector3d> expectedPoint1(numTests);
  std::vector<Eigen::Vector3d> expectedPoint2(numTests);
  std::vector<Eigen::Vector3d> expectedPoint3(numTests);

  for (auto i = 0; i < numTests; ++i)
  {
    makeTriangleAndTriangleOneVertexIsWithinOtherTriangle(
          a1[i], a2[i], a3[i], b1[i], b2[i], b3[i], expectedPoint1[i]);
  }

  common::Timer t;

  t.start();
  for (auto i = 0; i < numTests; ++i)
  {
    const auto numContacts = collision::collideTriangleTriangle(
          a1[i], a2[i], a3[i], b1[i], b2[i], b3[i],
          contact1[i], contact2[i], contact3[i]);

    EXPECT_TRUE(numContacts == 1);
    EXPECT_TRUE(contact1[i].isApprox(expectedPoint1[i]));
  }
  t.stop();
  std::cout << "total time (sec): " << t.getLastElapsedTime() << std::endl;
}

//==============================================================================
TEST(CollideTriangleTriangle, EdgeContact)
{
  Eigen::Vector3d P1(0, 0, 0);
  Eigen::Vector3d P2(2, 0, 0);
  Eigen::Vector3d P3(0, 2, 0);

  Eigen::Vector3d Q1(0, 0, 0);
  Eigen::Vector3d Q2(1, 0, 0);
  Eigen::Vector3d Q3(0, 0, 1);

  Eigen::Vector3d contact_points[3];

  auto res = collision::collideTriangleTriangle(
        P1, P2, P3, Q1, Q2, Q3,
        contact_points[0], contact_points[1], contact_points[2]);

  EXPECT_TRUE(res == 2);
  EXPECT_TRUE(contact_points[0].isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(contact_points[1].isApprox(Eigen::Vector3d(1, 0, 0)));
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
