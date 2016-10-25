/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/collision/dart/CollideTriangleTriangle.hpp"

#include "dart/math/Constants.hpp"

namespace dart {
namespace collision {

#define DART_TRIANGLE_TRIANGLE_EPS 1e-6

namespace v1 {

//==============================================================================
inline bool checkColinearPointAndLine(const Eigen::Vector3d& A1,
                                      const Eigen::Vector3d& B1,
                                      const Eigen::Vector3d& B2)
{
  // We assume that A1, B1, B2 are on the same line (colinear), and check if
  // A1 is placed between B1 and B2.

  const Eigen::Vector3d B12 = B2 - B1;

#ifndef NDEBUG
  // Check if A1, B1, B3 are in a same line
  Eigen::Vector3d A1B1 = B1 - A1;
  Eigen::Vector3d A1B2 = B2 - A1;

  Eigen::Vector3d cross1 = B12.cross(A1B1);
  Eigen::Vector3d cross2 = B12.cross(A1B2);

  assert(cross1.norm() < DART_TRIANGLE_TRIANGLE_EPS);
  assert(cross2.norm() < DART_TRIANGLE_TRIANGLE_EPS);
#endif

  // Check if A1 is left side of the line of B1B2:
  // A1 (non-zero distance) B1-------------B2
  const Eigen::Vector3d B1A1 = A1 - B1;
  const auto dot = (B1A1).dot(B12);
  if (dot < 0.0)
    return false;

  // Check if A1 is right side of the line of B1B2:
  // B1-------------B2 (non-zero distance) A1
  const auto squaredLength = B12.squaredNorm();
  if (dot > squaredLength)
    return false;

  // Otherwise A1 is on the line of B1B2:
  // B1-------A1--------------B2
  return true;
}

//==============================================================================
inline int checkColinearPointAndLine(const Eigen::Vector3d& A1,
                                     const Eigen::Vector3d& B1,
                                     const Eigen::Vector3d& B2,
                                     Eigen::Vector3d* contacts)
{
  // We assume that A1, B1, B2 are on the same line (colinear), and check if
  // A1 is placed between B1 and B2.

  const Eigen::Vector3d B12 = B2 - B1;

#ifndef NDEBUG
  // Check if A1, B1, B3 are in a same line
  Eigen::Vector3d A1B1 = B1 - A1;
  Eigen::Vector3d A1B2 = B2 - A1;

  Eigen::Vector3d cross1 = B12.cross(A1B1);
  Eigen::Vector3d cross2 = B12.cross(A1B2);

  assert(cross1.norm() < DART_TRIANGLE_TRIANGLE_EPS);
  assert(cross2.norm() < DART_TRIANGLE_TRIANGLE_EPS);
#endif

  // Check if A1 is left side of the line of B1B2:
  // A1 (non-zero distance) B1-------------B2
  const Eigen::Vector3d B1A1 = A1 - B1;
  const auto dot = (B1A1).dot(B12);
  if (dot < 0.0)
    return 0;

  // Check if A1 is right side of the line of B1B2:
  // B1-------------B2 (non-zero distance) A1
  const auto squaredLength = B12.squaredNorm();
  if (dot > squaredLength)
    return 0;

  // Otherwise A1 is on the line of B1B2:
  // B1-------A1--------------B2
  contacts[0] = A1;

  return 1;
}

//==============================================================================
inline int checkColinearLines(const double daMin,
                              const double daMax,
                              const double dbMin,
                              const double dbMax)
{
  // We assume that AMin, AMax, BMin, and BMax are on the same line (colinear),
  // and check if there exists intersection between line AMin-AMax and
  // BMin-BMax. Note that AMin <= AMax and BMin <= BMax as the variable names
  // say.

  if (dbMin < daMin)
  {
    if (dbMax < daMin)
      return false;
    else if (dbMax < daMax)
      return true;
    else
      return true;
  }
  else if (dbMin < daMax)
  {
    if (dbMax < daMax)
      return true;
    else
      return true;
  }
  else
  {
    return false;
  }
}

//==============================================================================
inline int checkColinearLines(const Eigen::Vector3d& AMin,
                              const Eigen::Vector3d& AMax,
                              const Eigen::Vector3d& BMin,
                              const Eigen::Vector3d& BMax,
                              const double daMin,
                              const double daMax,
                              const double dbMin,
                              const double dbMax,
                              Eigen::Vector3d* contacts)
{
  // We assume that AMin, AMax, BMin, and BMax are on the same line (colinear),
  // and check if there exists intersection between line AMin-AMax and
  // BMin-BMax. Note that AMin <= AMax and BMin <= BMax as the variable names
  // say.

  if (dbMin < daMin)
  {
    if (dbMax < daMin)
    {
      return 0;
    }
    else if (dbMax < daMax)
    {
      // collision: (aMin, bMax)

      if (dbMax - daMin < DART_TRIANGLE_TRIANGLE_EPS)
      {
        contacts[0] = AMin;  // or bMax or 0.5*(aMin+bMax)
        return 1;
      }

      contacts[0] = AMin;
      contacts[1] = BMax;
      return 2;
    }
    else
    {
      // collision: (aMin, aMax)
      contacts[0] = AMin;
      contacts[1] = AMax;
      return 2;
    }
  }
  else if (dbMin < daMax)
  {
    if (dbMax < daMax)
    {
      // collision: (bMin, bMax)
      contacts[0] = BMin;
      contacts[1] = BMax;
      return 2;
    }
    else
    {
      // collision: (bMin, aMax)

      if (daMax - dbMin < DART_TRIANGLE_TRIANGLE_EPS)
      {
        contacts[0] = AMax; // or bMin or 0.5*(aMax+bMin)
        return 1;
      }

      contacts[0] = BMin;
      contacts[1] = AMax;
      return 2;
    }
  }
  else
  {
    return 0;
  }
}

//==============================================================================
inline bool checkColinearLines(const Eigen::Vector3d& A1,
                              const Eigen::Vector3d& A2,
                              const Eigen::Vector3d& B1,
                              const Eigen::Vector3d& B2)
{
  const Eigen::Vector3d A12 = A2 - A1;

  const auto da1 = A12.dot(A1);
  const auto da2 = A12.dot(A2);
  const auto db1 = A12.dot(B1);
  const auto db2 = A12.dot(B2);

  if (da1 < da2)
  {
    if (db1 < db2)
      return checkColinearLines(da1, da2, db1, db2);
    else
      return checkColinearLines(da1, da2, db2, db1);
  }
  else
  {
    if (db1 < db2)
      return checkColinearLines(da2, da1, db1, db2);
    else
      return checkColinearLines(da2, da1, db2, db1);
  }
}

//==============================================================================
inline int checkColinearLines(const Eigen::Vector3d& A1,
                              const Eigen::Vector3d& A2,
                              const Eigen::Vector3d& B1,
                              const Eigen::Vector3d& B2,
                              Eigen::Vector3d* contacts)
{
  const Eigen::Vector3d A12 = A2 - A1;

  const auto da1 = A12.dot(A1);
  const auto da2 = A12.dot(A2);
  const auto db1 = A12.dot(B1);
  const auto db2 = A12.dot(B2);

  if (da1 < da2)
  {
    if (db1 < db2)
      return checkColinearLines(A1, A2, B1, B2, da1, da2, db1, db2, contacts);
    else
      return checkColinearLines(A1, A2, B2, B1, da1, da2, db2, db1, contacts);
  }
  else
  {
    if (db1 < db2)
      return checkColinearLines(A2, A1, B1, B2, da2, da1, db1, db2, contacts);
    else
      return checkColinearLines(A2, A1, B2, B1, da2, da1, db2, db1, contacts);
  }
}

//==============================================================================
inline void computePointOnPlane(const Eigen::Vector3d& B2,
                                const Eigen::Vector3d& B3,
                                const Eigen::Vector3d& N1,
                                const double db2,
                                Eigen::Vector3d& B23OnPlaneA)
{
  const Eigen::Vector3d B23 = B3 - B2;
  B23OnPlaneA = B2;
  B23OnPlaneA.noalias() -= (db2 / N1.dot(B23)) * B23;
}

//==============================================================================
inline bool case1AndCase1(const Eigen::Vector3d& A1,
                          const Eigen::Vector3d& B1)
{
  if (A1 == B1) // TODO(JS): ((a1 - b1).squaredNorm() < eps*eps) ?
    return true;

  return false;
}

//==============================================================================
inline int case1AndCase1(const Eigen::Vector3d& A1,
                         const Eigen::Vector3d& B1,
                         Eigen::Vector3d* contacts)
{
  //             _______________________
  //            |                       |
  //            |                       |
  //            |                       |
  //            |                       |_____
  //            |                       |    /
  //            |         ____          |   /
  //            |        |   /          |  /
  //            |        |  /           | /
  //            |        | /            |/
  //            |________|/_____________| ________\ Axis
  //           /   A1 /|  B1           /|         /
  //          /      / |              / |
  //         /      /  |             /  |
  //        /      /___|            /   |
  //       /                       /    |
  //      /_______________________/     |
  //   Plane A  |                       |
  //            |                       |
  //            |_______________________| Plane B
  //
  // We now check if A1 and B1 intersect.

  if (A1 == B1) // TODO(JS): ((a1 - b1).squaredNorm() < eps*eps) ?
  {
    contacts[0] = A1;
    return 1;
  }

  return 0;
}

//==============================================================================
inline bool case1AndCase2(const Eigen::Vector3d& A1,
                          const Eigen::Vector3d& B1,
                          const Eigen::Vector3d& B2)
{
  return checkColinearPointAndLine(A1, B1, B2);
}

//==============================================================================
inline int case1AndCase2(const Eigen::Vector3d& A1,
                         const Eigen::Vector3d& B1,
                         const Eigen::Vector3d& B2,
                         Eigen::Vector3d* contacts)
{
  //             _______________________
  //            |                       |
  //            |                       |
  //            |        B3             |
  //            |        |\             |_____
  //            |        | \            |    /
  //            |        |  \           |   /
  //            |        |   \          |  /
  //            |        |    \         | /
  //            |        | B1  \ B2     |/
  //            |________|______\_______| ________\ Axis
  //           /   A1 /|               /|         /
  //          /      / |              / |
  //         /      /  |             /  |
  //        /      /___|            /   |
  //       /                       /    |
  //      /_______________________/     |
  //   Plane A  |                       |
  //            |                       |
  //            |_______________________| Plane B
  //
  // We now check if A1 and B1-B2 intersect.

  return checkColinearPointAndLine(A1, B1, B2, contacts);
}

//==============================================================================
inline bool case1AndCase3(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
    const double db2)
{
  Eigen::Vector3d B23OnAxis;
  computePointOnPlane(B2, B3, N1, db2, B23OnAxis);

  return checkColinearPointAndLine(A1, B1, B23OnAxis);
}

//==============================================================================
inline int case1AndCase3(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
    const double db2,
    Eigen::Vector3d* contacts)
{
  //             _______________________
  //            |                       |
  //            |                       |
  //            |        B2             |
  //            |        |\             |_____
  //            |        | \            |    /
  //            |        |  \           |   /
  //            |        |   \          |  /
  //            |        |    \         | /
  //            |        | B23OnAxis    |/
  //            |________|______\_______| ________\ Axis
  //           /   A1 /| :      .  B1  /|         /
  //          /      / | :     .      / |
  //         /      /  | :    .      /  |
  //        /      /___| :   .      /   |
  //       /             :  .      /    |
  //      /______________:________/     |
  //   Plane A  |        |/             |
  //            |      B3               |
  //            |_______________________| Plane B
  //
  // We now check if A1 and B1-B23OnAxis intersect.

  Eigen::Vector3d B23OnAxis;
  computePointOnPlane(B2, B3, N1, db2, B23OnAxis);

  return checkColinearPointAndLine(A1, B1, B23OnAxis, contacts);
}

//==============================================================================
inline bool case1AndCase4(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
    const double db1)
{
  Eigen::Vector3d B12OnPlaneA;
  Eigen::Vector3d B13OnPlaneA;
  computePointOnPlane(B1, B2, N1, db1, B12OnPlaneA);
  computePointOnPlane(B1, B3, N1, db1, B13OnPlaneA);

#ifndef NDEBUG
  auto tmp1 = N1.dot(B12OnPlaneA);
  auto tmp2 = N1.dot(B13OnPlaneA);
  auto tmp3 = N1.dot(A1);

  assert(std::abs(tmp1 - tmp2) < DART_TRIANGLE_TRIANGLE_EPS);
  assert(std::abs(tmp2 - tmp3) < DART_TRIANGLE_TRIANGLE_EPS);
#endif

  return checkColinearPointAndLine(A1, B12OnPlaneA, B13OnPlaneA);
}

//==============================================================================
inline int case1AndCase4(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
    const double db1,
    Eigen::Vector3d* contacts)
{
  //             _______________________
  //            |                       |
  //            |                       |
  //            |        B1             |
  //            |        |\             |_____
  //            |        | \            |    /
  //            |        |  \           |   /
  //            |        |   \          |  /
  //            |        |    \         | /
  //            |     A1 | B12OnAxis    |/
  //            |________|______\_______| ________\ Axis
  //           /      /| :       . B13OnAxis      /
  //          /      / | :        .   / |
  //         /      /  | :         . /  |
  //        /      /___| :          /   |
  //       /             :         / \  |
  //      /______________:________/   \ |
  //   Plane A  |        |_____________\| B3
  //            |      B2               |
  //            |_______________________| Plane B
  //
  // We now check if A1 and B12OnAxis-B23OnAxis intersect.

  Eigen::Vector3d B12OnPlaneA;
  Eigen::Vector3d B13OnPlaneA;
  computePointOnPlane(B1, B2, N1, db1, B12OnPlaneA);
  computePointOnPlane(B1, B3, N1, db1, B13OnPlaneA);

#ifndef NDEBUG
  auto tmp1 = N1.dot(B12OnPlaneA);
  auto tmp2 = N1.dot(B13OnPlaneA);
  auto tmp3 = N1.dot(A1);

  assert(std::abs(tmp1 - tmp2) < DART_TRIANGLE_TRIANGLE_EPS);
  assert(std::abs(tmp2 - tmp3) < DART_TRIANGLE_TRIANGLE_EPS);
#endif

  return checkColinearPointAndLine(A1, B12OnPlaneA, B13OnPlaneA, contacts);
}

//==============================================================================
inline bool case2AndCase2(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2)
{
  return checkColinearLines(A1, A2, B1, B2);
}

//==============================================================================
inline int case2AndCase2(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    Eigen::Vector3d* contacts)
{
  return checkColinearLines(A1, A2, B1, B2, contacts);
}

//==============================================================================
inline bool case2AndCase3(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
    const double db2)
{
  Eigen::Vector3d b32OnPlaneA;
  computePointOnPlane(B2, B3, N1, db2, b32OnPlaneA);

  return checkColinearLines(A1, A2, B1, b32OnPlaneA);
}

//==============================================================================
inline int case2AndCase3(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
    const double db2,
    Eigen::Vector3d* contacts)
{
  Eigen::Vector3d b32OnPlaneA;
  computePointOnPlane(B2, B3, N1, db2, b32OnPlaneA);

  return checkColinearLines(A1, A2, B1, b32OnPlaneA, contacts);
}

//==============================================================================
inline bool case2AndCase4(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
    const double db1)
{
  Eigen::Vector3d b21OnPlaneA;
  Eigen::Vector3d b31OnPlaneA;
  computePointOnPlane(B1, B2, N1, db1, b21OnPlaneA);
  computePointOnPlane(B1, B3, N1, db1, b31OnPlaneA);

  return checkColinearLines(A1, A2, b21OnPlaneA, b31OnPlaneA);
}

//==============================================================================
inline int case2AndCase4(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
    const double db1,
    Eigen::Vector3d* contacts)
{
  Eigen::Vector3d b21OnPlaneA;
  Eigen::Vector3d b31OnPlaneA;
  computePointOnPlane(B1, B2, N1, db1, b21OnPlaneA);
  computePointOnPlane(B1, B3, N1, db1, b31OnPlaneA);

  return checkColinearLines(A1, A2, b21OnPlaneA, b31OnPlaneA, contacts);
}

//==============================================================================
inline bool case3AndCase3(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& A3,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
    const Eigen::Vector3d& N2,
    const double da2, const double db2)
{
  Eigen::Vector3d A23OnPlaneB;
  computePointOnPlane(A2, A3, N2, da2, A23OnPlaneB);

  Eigen::Vector3d B23OnPlaneA;
  computePointOnPlane(B2, B3, N1, db2, B23OnPlaneA);

  return checkColinearLines(A1, A23OnPlaneB, B1, B23OnPlaneA);
}

//==============================================================================
inline int case3AndCase3(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& A3,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
    const Eigen::Vector3d& N2,
    const double da2, const double db2,
    Eigen::Vector3d* contacts)
{
  //             _______________________
  //            |                       |
  //            |                       |
  //            |        B2             |
  //            |        |\             |_____
  //            |        | \            |    /
  //            |     A2 |  \           |   /
  //            |     .: |   \          |  /
  //            |    . : |    \         | /
  //         A23OnAxis : | B23OnAxis    |/
  //            |______:_|______\_______| ________\ Axis
  //           /A1 \   | :      / B1   /|         /
  //          /     \  | :     /      / |
  //         /       \ | :    /      /  |
  //        /         \| :   /      /   |
  //       /          A3 :  /      /    |
  //      /______________:_/______/     |
  //   Plane A  |        |/             |
  //            |        B3             |
  //            |_______________________| Plane B
  //
  // We now check if A1-A23OnAxis and B1-B23OnAxis intersect.

  Eigen::Vector3d A23OnPlaneB;
  computePointOnPlane(A2, A3, N2, da2, A23OnPlaneB);

  Eigen::Vector3d B23OnPlaneA;
  computePointOnPlane(B2, B3, N1, db2, B23OnPlaneA);

  return checkColinearLines(A1, A23OnPlaneB, B1, B23OnPlaneA, contacts);
}

//==============================================================================
inline bool case3AndCase4(
    const Eigen::Vector3d& A1, // 0
    const Eigen::Vector3d& A2, // +/-
    const Eigen::Vector3d& A3, // -/+
    const Eigen::Vector3d& B1, // +/-
    const Eigen::Vector3d& B2, // -/+
    const Eigen::Vector3d& B3, // -/+
    const Eigen::Vector3d& N1,
    const Eigen::Vector3d& N2,
    const double da2,
    const double db1)
{
  Eigen::Vector3d A23OnPlaneB;
  computePointOnPlane(A2, A3, N2, da2, A23OnPlaneB);

  Eigen::Vector3d B12OnPlaneA;
  Eigen::Vector3d B13OnPlaneA;
  computePointOnPlane(B1, B2, N1, db1, B12OnPlaneA);
  computePointOnPlane(B1, B3, N1, db1, B13OnPlaneA);

  return checkColinearLines(A1, A23OnPlaneB, B12OnPlaneA, B13OnPlaneA);
}

//==============================================================================
inline int case3AndCase4(
    const Eigen::Vector3d& A1, // 0
    const Eigen::Vector3d& A2, // +/-
    const Eigen::Vector3d& A3, // -/+
    const Eigen::Vector3d& B1, // +/-
    const Eigen::Vector3d& B2, // -/+
    const Eigen::Vector3d& B3, // -/+
    const Eigen::Vector3d& N1,
    const Eigen::Vector3d& N2,
    const double da2,
    const double db1,
    Eigen::Vector3d* contacts)
{
  //             _______________________
  //            |                       |
  //            |                       |
  //            |        B1             |
  //            |        |\             |_____
  //            |        | \            |    /
  //            |     A2 |  \           |   /
  //            |     .: |   \          |  /
  //            |    . : |    \         | /
  //         A23OnAxis : | B12OnAxis    |/
  //            |______:_|______\_______| ________\ Axis
  //           /A1 \   | :       . B13OnAxis      /
  //          /     \  | :        .   / |
  //         /       \ | :         . /  |
  //        /         \| :          /   |
  //       /          A3 :         / \  |
  //      /______________:________/   \ |
  //   Plane A  |        |_____________\| B3
  //            |      B2               |
  //            |_______________________| Plane B
  //
  // We now check if A1-A23OnAxis and B12OnAxis-B23OnAxis intersect.

  Eigen::Vector3d A23OnPlaneB;
  computePointOnPlane(A2, A3, N2, da2, A23OnPlaneB);

  Eigen::Vector3d B12OnPlaneA;
  Eigen::Vector3d B13OnPlaneA;
  computePointOnPlane(B1, B2, N1, db1, B12OnPlaneA);
  computePointOnPlane(B1, B3, N1, db1, B13OnPlaneA);

  return checkColinearLines(A1, A23OnPlaneB, B12OnPlaneA, B13OnPlaneA, contacts);
}

//==============================================================================
inline bool case4AndCase4(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& A3,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
    const Eigen::Vector3d& N2,
    const double da1,
    const double db1)
{
  Eigen::Vector3d A12OnAxis;
  Eigen::Vector3d A13OnAxis;
  computePointOnPlane(A1, A2, N2, da1, A12OnAxis);
  computePointOnPlane(A1, A3, N2, da1, A13OnAxis);

  Eigen::Vector3d B12OnAxis;
  Eigen::Vector3d B13OnAxis;
  computePointOnPlane(B1, B2, N1, db1, B12OnAxis);
  computePointOnPlane(B1, B3, N1, db1, B13OnAxis);

  return checkColinearLines(A12OnAxis, A13OnAxis, B12OnAxis, B13OnAxis);
}

//==============================================================================
inline int case4AndCase4(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& A3,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
    const Eigen::Vector3d& N2,
    const double da1,
    const double db1,
    Eigen::Vector3d* contacts)
{
  //             _______________________
  //            |                       |
  //            |                       |
  //            |        B1             |
  //            |        |\             |_____
  //            |        | \            |    /
  //            |     A1 |  \           |   /
  //            |     .: |   \          |  /
  //            |    . : |    \         | /
  //       A13OnAxis   : | B12OnAxis    |/
  //            |______:_|______\_______| ________\ Axis
  //  A12OnAxis   /    | :       . B13OnAxis      /
  //          /  /     | :        .   / |
  //         /  /      | :         . /  |
  //        /  /_______| :          /   |
  //       / A2       A3 :         / \  |
  //      /______________:________/   \ |
  //   Plane A  |        |_____________\| B3
  //            |      B2               |
  //            |_______________________| Plane B
  //
  // We now check if A12OnAxis-A13OnAxis and B12OnAxis-B23OnAxis intersect.

  Eigen::Vector3d A12OnAxis;
  Eigen::Vector3d A13OnAxis;
  computePointOnPlane(A1, A2, N2, da1, A12OnAxis);
  computePointOnPlane(A1, A3, N2, da1, A13OnAxis);

  Eigen::Vector3d B12OnAxis;
  Eigen::Vector3d B13OnAxis;
  computePointOnPlane(B1, B2, N1, db1, B12OnAxis);
  computePointOnPlane(B1, B3, N1, db1, B13OnAxis);

  return checkColinearLines(
        A12OnAxis, A13OnAxis, B12OnAxis, B13OnAxis, contacts);
}

//==============================================================================
inline bool case1And(const Eigen::Vector3d& A1,
                    const Eigen::Vector3d& B1,
                    const Eigen::Vector3d& B2,
                    const Eigen::Vector3d& B3,
                    const Eigen::Vector3d& N1,
                    const double db1, const double db2, const double db3)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(A1, B3, B1, B2, N1, db3);
      // (+, +, 0): Case 1
      else
        return case1AndCase1(A1, B3);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(A1, B2, B3, B1, N1, db2);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(A1, B1, B2, B3, N1, db1);
      // (+, -, 0): Case 3
      else
        return case1AndCase3(A1, B3, B1, B2, N1, db1);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(A1, B2);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(A1, B2, B3, B1, N1, db3);
      // (+, 0, 0): Case 2
      else
        return case1AndCase2(A1, B2, B3);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(A1, B1, B2, B3, N1, db1);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(A1, B2, B3, B1, N1, db2);
      // (-, +, 0): Case 3
      else
        return case1AndCase3(A1, B3, B1, B2, N1, db1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(A1, B3, B1, B2, N1, db3);
      // (-, -, 0): Case 1
      else
        return case1AndCase1(A1, B3);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(A1, B2, B3, B1, N1, db3);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(A1, B2);
      // (-, 0, 0): Case 2
      else
        return case1AndCase2(A1, B2, B3);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(A1, B1);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(A1, B1, B2, B3, N1, db2);
      // (0, +, 0): Case 2
      else
        return case1AndCase2(A1, B3, B1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(A1, B1, B2, B3, N1, db2);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(A1, B1);
      // (0, -, 0): Case 2
      else
        return case1AndCase2(A1, B3, B1);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(A1, B1, B2);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(A1, B1, B2);
      // (0, 0, 0): Coplanar case
      else
      {
        assert(false);
        return 0;
      }
    }
  }
}

//==============================================================================
inline int case1And(const Eigen::Vector3d& A1,
                    const Eigen::Vector3d& B1,
                    const Eigen::Vector3d& B2,
                    const Eigen::Vector3d& B3,
                    const Eigen::Vector3d& N1,
                    const double db1, const double db2, const double db3,
                    Eigen::Vector3d* contacts)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(A1, B3, B1, B2, N1, db3, contacts);
      // (+, +, 0): Case 1
      else
        return case1AndCase1(A1, B3, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(A1, B2, B3, B1, N1, db2, contacts);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(A1, B1, B2, B3, N1, db1, contacts);
      // (+, -, 0): Case 3
      else
        return case1AndCase3(A1, B3, B1, B2, N1, db1, contacts);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(A1, B2, contacts);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(A1, B2, B3, B1, N1, db3, contacts);
      // (+, 0, 0): Case 2
      else
        return case1AndCase2(A1, B2, B3, contacts);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(A1, B1, B2, B3, N1, db1, contacts);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(A1, B2, B3, B1, N1, db2, contacts);
      // (-, +, 0): Case 3
      else
        return case1AndCase3(A1, B3, B1, B2, N1, db1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(A1, B3, B1, B2, N1, db3, contacts);
      // (-, -, 0): Case 1
      else
        return case1AndCase1(A1, B3, contacts);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(A1, B2, B3, B1, N1, db3, contacts);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(A1, B2, contacts);
      // (-, 0, 0): Case 2
      else
        return case1AndCase2(A1, B2, B3, contacts);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(A1, B1, contacts);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(A1, B1, B2, B3, N1, db2, contacts);
      // (0, +, 0): Case 2
      else
        return case1AndCase2(A1, B3, B1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(A1, B1, B2, B3, N1, db2, contacts);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(A1, B1, contacts);
      // (0, -, 0): Case 2
      else
        return case1AndCase2(A1, B3, B1, contacts);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(A1, B1, B2, contacts);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(A1, B1, B2, contacts);
      // (0, 0, 0): Coplanar case
      else
      {
        assert(false);
        return 0;
      }
    }
  }
}

//==============================================================================
inline bool case2And(const Eigen::Vector3d& A1,
                     const Eigen::Vector3d& A2,
                     const Eigen::Vector3d& B1,
                     const Eigen::Vector3d& B2,
                     const Eigen::Vector3d& B3,
                     const Eigen::Vector3d& N1,
                     const double db1, const double db2, const double db3)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(A1, A2, B3, B1, B2, N1, db3);
      // (+, +, 0): Case 1
      else
        return case1AndCase2(B3, A1, A2);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(A1, A2, B2, B3, B1, N1, db2);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(A1, A2, B1, B2, B3, N1, db1);
      // (+, -, 0): Case 3
      else
        return case2AndCase3(A1, A2, B3, B1, B2, N1, db1);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(B2, A1, A2);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(A1, A2, B2, B3, B1, N1, db3);
      // (+, 0, 0): Case 2
      else
        return case2AndCase2(A1, A2, B2, B3);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(A1, A2, B1, B2, B3, N1, db1);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(A1, A2, B2, B3, B1, N1, db2);
      // (-, +, 0): Case 3
      else
        return case2AndCase3(A1, A2, B3, B1, B2, N1, db1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(A1, A2, B3, B1, B2, N1, db3);
      // (-, -, 0): Case 1
      else
        return case1AndCase2(B3, A1, A2);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(A1, A2, B2, B3, B1, N1, db3);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(B2, A1, A2);
      // (-, 0, 0): Case 2
      else
        return case2AndCase2(A1, A2, B2, B3);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(B1, A1, A2);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(A1, A2, B1, B2, B3, N1, db2);
      // (0, +, 0): Case 2
      else
        return case2AndCase2(A1, A2, B3, B1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(A1, A2, B1, B2, B3, N1, db2);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(B1, A1, A2);
      // (0, -, 0): Case 2
      else
        return case2AndCase2(A1, A2, B3, B1);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase2(A1, A2, B1, B2);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase2(A1, A2, B1, B2);
      // (0, 0, 0): Coplanar case
      else
      {
        assert(false);
        return 0;
      }
    }
  }
}

//==============================================================================
inline int case2And(const Eigen::Vector3d& A1,
                    const Eigen::Vector3d& A2,
                    const Eigen::Vector3d& B1,
                    const Eigen::Vector3d& B2,
                    const Eigen::Vector3d& B3,
                    Eigen::Vector3d* contacts,
                    const Eigen::Vector3d& N1,
                    const double db1, const double db2, const double db3)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(A1, A2, B3, B1, B2, N1, db3, contacts);
      // (+, +, 0): Case 1
      else
        return case1AndCase2(B3, A1, A2, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(A1, A2, B2, B3, B1, N1, db2, contacts);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(A1, A2, B1, B2, B3, N1, db1, contacts);
      // (+, -, 0): Case 3
      else
        return case2AndCase3(A1, A2, B3, B1, B2, N1, db1, contacts);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(B2, A1, A2, contacts);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(A1, A2, B2, B3, B1, N1, db3, contacts);
      // (+, 0, 0): Case 2
      else
        return case2AndCase2(A1, A2, B2, B3, contacts);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(A1, A2, B1, B2, B3, N1, db1, contacts);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(A1, A2, B2, B3, B1, N1, db2, contacts);
      // (-, +, 0): Case 3
      else
        return case2AndCase3(A1, A2, B3, B1, B2, N1, db1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(A1, A2, B3, B1, B2, N1, db3, contacts);
      // (-, -, 0): Case 1
      else
        return case1AndCase2(B3, A1, A2, contacts);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(A1, A2, B2, B3, B1, N1, db3, contacts);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(B2, A1, A2, contacts);
      // (-, 0, 0): Case 2
      else
        return case2AndCase2(A1, A2, B2, B3, contacts);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(B1, A1, A2, contacts);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(A1, A2, B1, B2, B3, N1, db2, contacts);
      // (0, +, 0): Case 2
      else
        return case2AndCase2(A1, A2, B3, B1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(A1, A2, B1, B2, B3, N1, db2, contacts);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(B1, A1, A2, contacts);
      // (0, -, 0): Case 2
      else
        return case2AndCase2(A1, A2, B3, B1, contacts);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase2(A1, A2, B1, B2, contacts);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase2(A1, A2, B1, B2, contacts);
      // (0, 0, 0): Coplanar case
      else
      {
        assert(false);
        return 0;
      }
    }
  }
}

//==============================================================================
inline bool case3And(const Eigen::Vector3d& A1,
                    const Eigen::Vector3d& A2,
                    const Eigen::Vector3d& A3,
                    const Eigen::Vector3d& B1,
                    const Eigen::Vector3d& B2,
                    const Eigen::Vector3d& B3,
                    const Eigen::Vector3d& N1,
                    const Eigen::Vector3d& N2,
                    const double da2,
                    const double db1, const double db2, const double db3)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(A1, A2, A3, B3, B1, B2, N1, N2, da2, db3);
      // (+, +, 0): Case 1
      else
        return case1AndCase3(B3, A1, A2, A3, N2, da2);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(A1, A2, A3, B2, B3, B1, N1, N2, da2, db2);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(A1, A2, A3, B1, B2, B3, N1, N2, da2, db1);
      // (+, -, 0): Case 3
      else
        return case3AndCase3(A1, A2, A3, B3, B1, B2, N2, N1, da2, db1);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(B2, A1, A2, A3, N2, da2);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(A1, A2, A3, B2, B3, B1, N2, N1, da2, db3);
      // (+, 0, 0): Case 2
      else
        return case2AndCase3(B2, B3, A1, A2, A3, N2, da2);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(A1, A2, A3, B1, B2, B3, N1, N2, da2, db1);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(A1, A2, A3, B2, B3, B1, N1, N2, da2, db2);
      // (-, +, 0): Case 3
      else
        return case3AndCase3(A1, A2, A3, B3, B1, B2, N2, N1, da2, db1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(A1, A2, A3, B3, B1, B2, N1, N2, da2, db3);
      // (-, -, 0): Case 1
      else
        return case1AndCase3(B3, A1, A2, A3, N2, da2);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(A1, A2, A3, B2, B3, B1, N2, N1, da2, db3);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(B2, A1, A2, A3, N2, da2);
      // (-, 0, 0): Case 2
      else
        return case2AndCase3(B2, B3, A1, A2, A3, N2, da2);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(B1, A1, A2, A3, N2, da2);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(A1, A2, A3, B1, B2, B3, N2, N1, da2, db2);
      // (0, +, 0): Case 2
      else
        return case2AndCase3(B3, B1, A1, A2, A3, N2, da2);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(A1, A2, A3, B1, B2, B3, N2, N1, da2, db2);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(B1, A1, A2, A3, N2, da2);
      // (0, -, 0): Case 2
      else
        return case2AndCase3(B3, B1, A1, A2, A3, N2, da2);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(B1, B2, A1, A2, A3, N2, da2);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(B1, B2, A1, A2, A3, N2, da2);
      // (0, 0, 0): Coplanar case
      else
      {
        assert(false);
        return false;
      }
    }
  }
}

//==============================================================================
inline int case3And(const Eigen::Vector3d& A1,
                    const Eigen::Vector3d& A2,
                    const Eigen::Vector3d& A3,
                    const Eigen::Vector3d& B1,
                    const Eigen::Vector3d& B2,
                    const Eigen::Vector3d& B3,
                    Eigen::Vector3d* contacts,
                    const Eigen::Vector3d& N1,
                    const Eigen::Vector3d& N2,
                    const double da2,
                    const double db1, const double db2, const double db3)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(A1, A2, A3, B3, B1, B2, N1, N2, da2, db3, contacts);
      // (+, +, 0): Case 1
      else
        return case1AndCase3(B3, A1, A2, A3, N2, da2, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(A1, A2, A3, B2, B3, B1, N1, N2, da2, db2, contacts);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(A1, A2, A3, B1, B2, B3, N1, N2, da2, db1, contacts);
      // (+, -, 0): Case 3
      else
        return case3AndCase3(A1, A2, A3, B3, B1, B2, N2, N1, da2, db1, contacts);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(B2, A1, A2, A3, N2, da2, contacts);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(A1, A2, A3, B2, B3, B1, N2, N1, da2, db3, contacts);
      // (+, 0, 0): Case 2
      else
        return case2AndCase3(B2, B3, A1, A2, A3, N2, da2, contacts);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(A1, A2, A3, B1, B2, B3, N1, N2, da2, db1, contacts);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(A1, A2, A3, B2, B3, B1, N1, N2, da2, db2, contacts);
      // (-, +, 0): Case 3
      else
        return case3AndCase3(A1, A2, A3, B3, B1, B2, N2, N1, da2, db1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(A1, A2, A3, B3, B1, B2, N1, N2, da2, db3, contacts);
      // (-, -, 0): Case 1
      else
        return case1AndCase3(B3, A1, A2, A3, N2, da2, contacts);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(A1, A2, A3, B2, B3, B1, N2, N1, da2, db3, contacts);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(B2, A1, A2, A3, N2, da2, contacts);
      // (-, 0, 0): Case 2
      else
        return case2AndCase3(B2, B3, A1, A2, A3, N2, da2, contacts);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(B1, A1, A2, A3, N2, da2, contacts);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(A1, A2, A3, B1, B2, B3, N2, N1, da2, db2, contacts);
      // (0, +, 0): Case 2
      else
        return case2AndCase3(B3, B1, A1, A2, A3, N2, da2, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(A1, A2, A3, B1, B2, B3, N2, N1, da2, db2, contacts);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(B1, A1, A2, A3, N2, da2, contacts);
      // (0, -, 0): Case 2
      else
        return case2AndCase3(B3, B1, A1, A2, A3, N2, da2, contacts);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(B1, B2, A1, A2, A3, N2, da2, contacts);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(B1, B2, A1, A2, A3, N2, da2, contacts);
      // (0, 0, 0): Coplanar case
      else
      {
        assert(false);
        return 0;
      }
    }
  }
}

//==============================================================================
inline bool case4And(const Eigen::Vector3d& A1,
                    const Eigen::Vector3d& A2,
                    const Eigen::Vector3d& A3,
                    const Eigen::Vector3d& B1,
                    const Eigen::Vector3d& B2,
                    const Eigen::Vector3d& B3,
                    const Eigen::Vector3d& N1,
                    const Eigen::Vector3d& N2,
                    const double da1,
                    const double db1, const double db2, const double db3)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(A1, A2, A3, B3, B1, B2, N1, N2, da1, db3);
      // (+, +, 0): Case 1
      else
        return case1AndCase4(B3, A1, A2, A3, N2, da1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(A1, A2, A3, B2, B3, B1, N1, N2, da1, db2);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(A1, A2, A3, B1, B2, B3, N1, N2, da1, db1);
      // (+, -, 0): Case 3
      else
        return case3AndCase4(B3, B1, B2, A1, A2, A3, N2, N1, db1, da1);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(B2, A1, A2, A3, N2, da1);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(B2, B3, B1, A1, A2, A3, N2, N1, db3, da1);
      // (+, 0, 0): Case 2
      else
        return case2AndCase4(B2, B3, A1, A2, A3, N2, da1);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(A1, A2, A3, B1, B2, B3, N1, N2, da1, db1);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(A1, A2, A3, B2, B3, B1, N1, N2, da1, db2);
      // (-, +, 0): Case 3
      else
        return case3AndCase4(B3, B1, B2, A1, A2, A3, N2, N1, db1, da1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(A1, A2, A3, B3, B1, B2, N1, N2, da1, db3);
      // (-, -, 0): Case 1
      else
        return case1AndCase4(B3, A1, A2, A3, N2, da1);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(B2, B3, B1, A1, A2, A3, N2, N1, db3, da1);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(B2, A1, A2, A3, N2, da1);
      // (-, 0, 0): Case 2
      else
        return case2AndCase4(B2, B3, A1, A2, A3, N2, da1);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(B1, A1, A2, A3, N2, da1);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(B1, B2, B3, A1, A2, A3, N2, N1, db2, da1);
      // (0, +, 0): Case 2
      else
        return case2AndCase4(B3, B1, A1, A2, A3, N2, da1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(B1, B2, B3, A1, A2, A3, N2, N1, db2, da1);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(B1, A1, A2, A3, N2, da1);
      // (0, -, 0): Case 2
      else
        return case2AndCase4(B3, B1, A1, A2, A3, N2, da1);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(B1, B2, A1, A2, A3, N2, da1);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(B1, B2, A1, A2, A3, N2, da1);
      // (0, 0, 0): Coplanar case
      else
      {
        assert(false);
        return false;
      }
    }
  }
}

//==============================================================================
inline int case4And(const Eigen::Vector3d& A1,
                    const Eigen::Vector3d& A2,
                    const Eigen::Vector3d& A3,
                    const Eigen::Vector3d& B1,
                    const Eigen::Vector3d& B2,
                    const Eigen::Vector3d& B3,
                    Eigen::Vector3d* contacts,
                    const Eigen::Vector3d& N1,
                    const Eigen::Vector3d& N2,
                    const double da1,
                    const double db1, const double db2, const double db3)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(A1, A2, A3, B3, B1, B2, N1, N2, da1, db3, contacts);
      // (+, +, 0): Case 1
      else
        return case1AndCase4(B3, A1, A2, A3, N2, da1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(A1, A2, A3, B2, B3, B1, N1, N2, da1, db2, contacts);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(A1, A2, A3, B1, B2, B3, N1, N2, da1, db1, contacts);
      // (+, -, 0): Case 3
      else
        return case3AndCase4(B3, B1, B2, A1, A2, A3, N2, N1, db1, da1, contacts);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(B2, A1, A2, A3, N2, da1, contacts);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(B2, B3, B1, A1, A2, A3, N2, N1, db3, da1, contacts);
      // (+, 0, 0): Case 2
      else
        return case2AndCase4(B2, B3, A1, A2, A3, N2, da1, contacts);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(A1, A2, A3, B1, B2, B3, N1, N2, da1, db1, contacts);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(A1, A2, A3, B2, B3, B1, N1, N2, da1, db2, contacts);
      // (-, +, 0): Case 3
      else
        return case3AndCase4(B3, B1, B2, A1, A2, A3, N2, N1, db1, da1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(A1, A2, A3, B3, B1, B2, N1, N2, da1, db3, contacts);
      // (-, -, 0): Case 1
      else
        return case1AndCase4(B3, A1, A2, A3, N2, da1, contacts);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(B2, B3, B1, A1, A2, A3, N2, N1, db3, da1, contacts);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(B2, A1, A2, A3, N2, da1, contacts);
      // (-, 0, 0): Case 2
      else
        return case2AndCase4(B2, B3, A1, A2, A3, N2, da1, contacts);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(B1, A1, A2, A3, N2, da1, contacts);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(B1, B2, B3, A1, A2, A3, N2, N1, db2, da1, contacts);
      // (0, +, 0): Case 2
      else
        return case2AndCase4(B3, B1, A1, A2, A3, N2, da1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(B1, B2, B3, A1, A2, A3, N2, N1, db2, da1, contacts);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(B1, A1, A2, A3, N2, da1, contacts);
      // (0, -, 0): Case 2
      else
        return case2AndCase4(B3, B1, A1, A2, A3, N2, da1, contacts);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(B1, B2, A1, A2, A3, N2, da1, contacts);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(B1, B2, A1, A2, A3, N2, da1, contacts);
      // (0, 0, 0): Coplanar case
      else
      {
        assert(false);
        return 0;
      }
    }
  }
}

//==============================================================================
inline double cross2d(const Eigen::Vector2d& u, const Eigen::Vector2d& v)
{
  return ((u[0]) * (v[1]) - (u[1]) * (v[0]));
}

//==============================================================================
inline bool checkCrossingLines(
    const Eigen::Vector2d& a12,
    const Eigen::Vector2d& b12,
    const Eigen::Vector2d& a1b1)
{
  const auto crossLines = cross2d(a12, b12);

  // Parallel case is ignored because it's already handled in the vertex
  // checking stage.
  if (std::abs(crossLines) < DART_TRIANGLE_TRIANGLE_EPS)
    return false;

  const auto invCrossLines = 1.0 / crossLines;

  const auto u = cross2d(a1b1, a12) * invCrossLines;
  if (0.0 >= u || u >= 1.0)
    return false;

  const auto t = cross2d(a1b1, b12) * invCrossLines;
  if (0.0 >= t || t >= 1.0)
    return false;

  return true;
}

//==============================================================================
inline void checkCrossingLines(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A12,
    const Eigen::Vector2d& a12,
    const Eigen::Vector2d& b12,
    const Eigen::Vector2d& a1b1,
    Eigen::Vector3d* contacts,
    int& numContacts)
{
  const auto crossLines = cross2d(a12, b12);

  // Parallel case is ignored because it's already handled in the vertex
  // checking stage.
  if (std::abs(crossLines) < DART_TRIANGLE_TRIANGLE_EPS)
    return;

  const auto invCrossLines = 1.0 / crossLines;

  const auto u = cross2d(a1b1, a12) * invCrossLines;
  if (0.0 >= u || u >= 1.0)
    return;

  const auto t = cross2d(a1b1, b12) * invCrossLines;
  if (0.0 >= t || t >= 1.0)
    return;

  contacts[numContacts++] = A1 + t * A12;
}

//==============================================================================
inline bool checkPointInTriangle(
    const Eigen::Vector2d& A,
    const Eigen::Vector2d& B,
    const Eigen::Vector2d& C,
    const Eigen::Vector2d& P)
{
  // Compute vectors
  const Eigen::Vector2d v0 = C - A;
  const Eigen::Vector2d v1 = B - A;
  const Eigen::Vector2d v2 = P - A;

  // Compute dot products
  const auto dot00 = v0.dot(v0);
  const auto dot01 = v0.dot(v1);
  const auto dot02 = v0.dot(v2);
  const auto dot11 = v1.dot(v1);
  const auto dot12 = v1.dot(v2);

  // Compute barycentric coordinates
  const auto invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
  const auto u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  const auto v = (dot00 * dot12 - dot01 * dot02) * invDenom;

  // Check if point is in triangle
  return (u >= 0.0) && (v >= 0.0) && (u + v <= 1.0);
}

//==============================================================================
// (u, v)
// - (0, 0): p is on a
// - (1, 0): p is on b
// - (0, 1); p is on c
// - otherwise: p is strictly inside of the triangle (a, b, c)
inline int checkPointInTriangle(
    const Eigen::Vector2d& vb,
    const Eigen::Vector2d& vc,
    const Eigen::Vector2d& vp)
{
  // References:
  // - http://blackpawn.com/texts/pointinpoly/default.html
  // - Realtime Collision Detection, CRC

  // Compute dot products
  const auto dot00 = vb.dot(vb);
  const auto dot01 = vb.dot(vc);
  const auto dot02 = vb.dot(vp);
  const auto dot11 = vc.dot(vc);
  const auto dot12 = vc.dot(vp);

  // Compute barycentric coordinates
  const auto invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
  const auto u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  const auto v = (dot00 * dot12 - dot01 * dot02) * invDenom;

  // Check if point is in triangle
  return (u >= 0.0) && (v >= 0.0) && (u + v <= 1.0);

  // TODO(JS): consider margin
}

//==============================================================================
// (u, v)
// - (0, 0): p is on a
// - (1, 0): p is on b
// - (0, 1); p is on c
// - otherwise: p is strictly inside of the triangle (a, b, c)
inline int checkPointInTriangle(
    const Eigen::Vector2d& vb,
    const Eigen::Vector2d& vc,
    const Eigen::Vector2d& vp,
    double& u,
    double& v)
{
  // References:
  // - http://blackpawn.com/texts/pointinpoly/default.html
  // - Realtime Collision Detection, CRC

  // Compute dot products
  const auto dot00 = vb.dot(vb);
  const auto dot01 = vb.dot(vc);
  const auto dot02 = vb.dot(vp);
  const auto dot11 = vc.dot(vc);
  const auto dot12 = vc.dot(vp);

  // Compute barycentric coordinates
  const auto invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
  u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  v = (dot00 * dot12 - dot01 * dot02) * invDenom;

  // Check if point is in triangle
  return (u >= 0.0) && (v >= 0.0) && (u + v <= 1.0);

  // TODO(JS): consider margin
}

//==============================================================================
inline void updateVertexCheckNecessity(
    const double u, const double v,
    bool& inB1, bool& inB2, bool& inB3)
{
  if (u < DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (v < DART_TRIANGLE_TRIANGLE_EPS)
    {
      // P is at the vertex 1 of other triangle
      inB1 = true;
    }
    else if (1.0 - v < DART_TRIANGLE_TRIANGLE_EPS)
    {
      // P is at the vertex 3 of other triangle
      inB3 = true;
    }
  }
  else
  {
    if (v < DART_TRIANGLE_TRIANGLE_EPS)
    {
      // P is at the vertex 2 of other triangle
      inB2 = true;
    }
  }
}

//==============================================================================
inline void updateEdgeCheckNecessity(
    const double u, const double v,
    bool& needA12B12, bool& needA12B23, bool& needA12B31,
    bool& needA31B12, bool& needA31B23, bool& needA31B31)
{
  if (u < DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (v < DART_TRIANGLE_TRIANGLE_EPS)
    {
      // P is at V1
      needA12B12 = false;
      needA12B31 = false;

      needA31B12 = false;
      needA31B31 = false;
    }
    else if (1.0 - v < DART_TRIANGLE_TRIANGLE_EPS)
    {
      // P is at V3
      needA12B31 = false;
      needA12B23 = false;

      needA31B31 = false;
      needA31B23 = false;
    }
    else
    {
      // P is on Edge V3V1
      needA12B31 = false;

      needA31B31 = false;
    }
  }
  else if (1 - u < DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (v < DART_TRIANGLE_TRIANGLE_EPS)
    {
      // P is at V2
      needA12B23 = false;
      needA12B12 = false;

      needA31B23 = false;
      needA31B12 = false;
    }
  }
  else
  {
    if (v < DART_TRIANGLE_TRIANGLE_EPS)
    {
      // P is on Edge V1V2
      needA12B12 = false;

      needA31B12 = false;
    }
    else if (1.0 - (u + v) < DART_TRIANGLE_TRIANGLE_EPS)
    {
      // P is on Edge V2V3
      needA12B23 = false;

      needA31B23 = false;
    }
  }
}

//==============================================================================
inline int coplanar2d(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& A3,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector2d& a1,
    const Eigen::Vector2d& a2,
    const Eigen::Vector2d& a3,
    const Eigen::Vector2d& b1,
    const Eigen::Vector2d& b2,
    const Eigen::Vector2d& b3,
    Eigen::Vector3d* contacts,
    const bool /*returnAllContacts*/)
{
  int numContacts = 0;

  const Eigen::Vector2d b12 = b2 - b1;
  const Eigen::Vector2d b13 = b3 - b1;

  const Eigen::Vector2d b1a1 = a1 - b1;
  const Eigen::Vector2d b1a2 = a2 - b1;
  const Eigen::Vector2d b1a3 = a3 - b1;

  double u;
  double v;

  bool inB1 = false;
  bool inB2 = false;
  bool inB3 = false;

  bool needCheckA12B12 = true;
  bool needCheckA12B23 = true;
  bool needCheckA12B31 = true;
  bool needCheckA23B12 = true;
  bool needCheckA23B23 = true;
  bool needCheckA23B31 = true;
  bool needCheckA31B12 = true;
  bool needCheckA31B23 = true;
  bool needCheckA31B31 = true;

  const bool inA1 = checkPointInTriangle(b12, b13, b1a1, u, v);
  if (inA1)
  {
    contacts[numContacts++] = A1;
    updateVertexCheckNecessity(u, v, inB1, inB2, inB3);
    updateEdgeCheckNecessity(
          u, v,
          needCheckA12B12, needCheckA12B23, needCheckA12B31,
          needCheckA31B12, needCheckA31B23, needCheckA31B31);
  }

  const bool inA2 = checkPointInTriangle(b12, b13, b1a2, u, v);
  if (inA2)
  {
    contacts[numContacts++] = A2;
    updateVertexCheckNecessity(u, v, inB1, inB2, inB3);
    updateEdgeCheckNecessity(
          u, v,
          needCheckA23B12, needCheckA23B23, needCheckA23B31,
          needCheckA12B12, needCheckA12B23, needCheckA12B31);
  }

  const bool inA3 = checkPointInTriangle(b12, b13, b1a3, u, v);
  if (inA3)
  {
    contacts[numContacts++] = A3;
    updateVertexCheckNecessity(u, v, inB1, inB2, inB3);
    updateEdgeCheckNecessity(
          u, v,
          needCheckA31B12, needCheckA31B23, needCheckA31B31,
          needCheckA23B12, needCheckA23B23, needCheckA23B31);

    if (numContacts == 3)
      return numContacts;
  }

  const Eigen::Vector2d a12 = a2 - a1;
  const Eigen::Vector2d a13 = a3 - a1;

  const int numContactsOnA = numContacts;

  if (!inB1)
  {
    const Eigen::Vector2d b1a1 = b1 - a1;
    inB1 = checkPointInTriangle(a12, a13, b1a1, u, v);

    if (inB1)
    {
      contacts[numContacts++] = B1;
      updateEdgeCheckNecessity(
            u, v,
            needCheckA12B12, needCheckA23B12, needCheckA31B12,
            needCheckA12B31, needCheckA23B31, needCheckA31B31);
    }
  }

  if (!inB2)
  {
    const Eigen::Vector2d b2a1 = b2 - a1;
    inB2 = checkPointInTriangle(a12, a13, b2a1, u, v);

    if (inB2)
    {
      contacts[numContacts++] = B2;
      updateEdgeCheckNecessity(
            u, v,
            needCheckA12B23, needCheckA23B23, needCheckA31B23,
            needCheckA12B12, needCheckA23B12, needCheckA31B12);
    }
  }

  if (!inB3)
  {
    const Eigen::Vector2d b3a1 = b3 - a1;
    inB3 = checkPointInTriangle(a12, a13, b3a1, u, v);

    if (inB3)
    {
      contacts[numContacts++] = B3;
      updateEdgeCheckNecessity(
            u, v,
            needCheckA12B31, needCheckA23B31, needCheckA31B31,
            needCheckA12B23, needCheckA23B23, needCheckA31B23);

      const int numContactsOnB = numContacts - numContactsOnA;
      if (numContactsOnB == 3)
      {
        assert(numContactsOnA == 0);
        return numContacts;
      }
    }
  }

  if (inA1)
  {
    if (inA2)
    {
      needCheckA12B12 = false;
      needCheckA12B23 = false;
      needCheckA12B31 = false;
    }
    else if (inA3)
    {
      needCheckA31B12 = false;
      needCheckA31B23 = false;
      needCheckA31B31 = false;
    }
  }
  else if (inA2 && inA3)
  {
    needCheckA23B12 = false;
    needCheckA23B23 = false;
    needCheckA23B31 = false;
  }

  if (inB1)
  {
    if (inB2)
    {
      needCheckA12B12 = false;
      needCheckA23B12 = false;
      needCheckA31B12 = false;
    }
    else if (inB3)
    {
      needCheckA12B31 = false;
      needCheckA23B31 = false;
      needCheckA31B31 = false;
    }
  }
  else if (inB2 && inB3)
  {
    needCheckA12B23 = false;
    needCheckA23B23 = false;
    needCheckA31B23 = false;
  }

  if (needCheckA12B12)
  {
    checkCrossingLines(
          A1, A2 - A1, a2 - a1,
          b2 - b1,
          b1 - a1,
          contacts, numContacts);
  }

  if (needCheckA12B23)
  {
    checkCrossingLines(
          A1, A2 - A1, a2 - a1,
          b3 - b2,
          b2 - a1,
          contacts, numContacts);
  }

  if (needCheckA12B31)
  {
    checkCrossingLines(
          A1, A2 - A1, a2 - a1,
          b1 - b3,
          b3 - a1,
          contacts, numContacts);
  }

  if (needCheckA23B12)
  {
    checkCrossingLines(
          A2, A3 - A2, a3 - a2,
          b2 - b1,
          b1 - a2,
          contacts, numContacts);
  }

  if (needCheckA23B23)
  {
    checkCrossingLines(
          A2, A3 - A2, a3 - a2,
          b3 - b2,
          b2 - a2,
          contacts, numContacts);
  }

  if (needCheckA23B31)
  {
    checkCrossingLines(
          A2, A3 - A2, a3 - a2,
          b1 - b3,
          b3 - a2,
          contacts, numContacts);
  }

  if (needCheckA31B12)
  {
    checkCrossingLines(
          A3, A1 - A3, a1 - a3,
          b2 - b1,
          b1 - a3,
          contacts, numContacts);
  }

  if (needCheckA31B23)
  {
    checkCrossingLines(
          A3, A1 - A3, a1 - a3,
          b3 - b2,
          b2 - a3,
          contacts, numContacts);
  }

  if (needCheckA31B31)
  {
    checkCrossingLines(
          A3, A1 - A3, a1 - a3,
          b1 - b3,
          b3 - a3,
          contacts, numContacts);
  }

  return numContacts;
}

//==============================================================================
int coplanar3d(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& A3,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
    Eigen::Vector3d* contacts,
    Eigen::Vector3d& normal,
    double& penetrationDepth,
    const bool returnAllContacts)
{
  Eigen::Vector2d a1;
  Eigen::Vector2d a2;
  Eigen::Vector2d a3;
  Eigen::Vector2d b1;
  Eigen::Vector2d b2;
  Eigen::Vector2d b3;

  int index;
  N1.cwiseAbs().maxCoeff(&index);

  switch (index)
  {
    case 0:
    {
      a1 = A1.tail<2>();
      a2 = A2.tail<2>();
      a3 = A3.tail<2>();

      b1 = B1.tail<2>();
      b2 = B2.tail<2>();
      b3 = B3.tail<2>();

      break;
    }
    case 1:
    {
      a1[0] = A1[0];
      a1[1] = A1[1];
      a2[0] = A2[0];
      a2[1] = A2[1];
      a3[0] = A3[0];
      a3[1] = A3[1];

      b1[0] = B1[0];
      b1[1] = B1[1];
      b2[0] = B2[0];
      b2[1] = B2[1];
      b3[0] = B3[0];
      b3[1] = B3[1];

      break;
    }
    default:
    {
      a1 = A1.head<2>();
      a2 = A2.head<2>();
      a3 = A3.head<2>();

      b1 = B1.head<2>();
      b2 = B2.head<2>();
      b3 = B3.head<2>();

      break;
    }
  }

  normal = -N1;
  penetrationDepth = 0.0;

  return coplanar2d(
        A1, A2, A3, B1, B2, B3,
        a1, a2, a3, b1, b2, b3,
        contacts, returnAllContacts);
}

//==============================================================================
int collideTriangleTriangle(const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& A3,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    Eigen::Vector3d* contacts,
    Eigen::Vector3d& normal,
    double& penetrationDepth,
    const bool returnAllContacts)
{
  int numContacts;

  Eigen::Vector3d v1 = B2 - B1;
  Eigen::Vector3d v2 = B3 - B2;
  Eigen::Vector3d N2 = v1.cross(v2);

  v1 = A1 - B3;
  const double da1 = v1.dot(N2);
  v1 = A2 - B3;
  const double da2 = v1.dot(N2);
  v1 = A3 - B3;
  const double da3 = v1.dot(N2);

  if ((da1 > DART_TRIANGLE_TRIANGLE_EPS &&
       da2 > DART_TRIANGLE_TRIANGLE_EPS &&
       da3 > DART_TRIANGLE_TRIANGLE_EPS)
      || (da1 < -DART_TRIANGLE_TRIANGLE_EPS &&
          da2 < -DART_TRIANGLE_TRIANGLE_EPS &&
          da3 < -DART_TRIANGLE_TRIANGLE_EPS))
  {
    return 0;
  }

  // dot(3), cross(1)

  v1 = A2 - A1;
  v2 = A3 - A2;
  Eigen::Vector3d N1 = v1.cross(v2);

  v1 = B1 - A3;
  const double db1 = v1.dot(N1);
  v1 = B2 - A3;
  const double db2 = v1.dot(N1);
  v1 = B3 - A3;
  const double db3 = v1.dot(N1);

  if ((db1 > DART_TRIANGLE_TRIANGLE_EPS &&
       db2 > DART_TRIANGLE_TRIANGLE_EPS &&
       db3 > DART_TRIANGLE_TRIANGLE_EPS)
      || (db1 < -DART_TRIANGLE_TRIANGLE_EPS &&
          db2 < -DART_TRIANGLE_TRIANGLE_EPS &&
          db3 < -DART_TRIANGLE_TRIANGLE_EPS))
  {
    return 0;
  }

  // dot(6), cross(2)

  if (da1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (da2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case4And(A3, A1, A2, B1, B2, B3, contacts, N1, N2, da3, db1, db2, db3);
      // (+, +, 0): Case 1
      else
        numContacts = case1And(A3, B1, B2, B3, N1, db1, db2, db3, contacts);
    }
    else if (da2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case4And(A2, A3, A1, B1, B2, B3, contacts, N1, N2, da2, db1, db2, db3);
      // (+, -, -): Case 4
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case4And(A1, A2, A3, B1, B2, B3, contacts, N1, N2, da1, db1, db2, db3);
      // (+, -, 0): Case 3
      else
        numContacts = case3And(A3, A1, A2, B1, B2, B3, contacts, N1, N2, da1, db1, db2, db3);
    }
    else
    {
      // (+, 0, +): Case 1
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case1And(A2, B1, B2, B3, N1, db1, db2, db3, contacts);
      // (+, 0, -): Case 3
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case3And(A2, A3, A1, B1, B2, B3, contacts, N1, N2, da3, db1, db2, db3);
      // (+, 0, 0): Case 2
      else
        numContacts = case2And(A2, A3, B1, B2, B3, contacts, N1, db1, db2, db3);
    }
  }
  else if (da1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (da2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case4And(A1, A2, A3, B1, B2, B3, contacts, N1, N2, da1, db1, db2, db3);
      // (-, +, -): Case 4
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case4And(A2, A3, A1, B1, B2, B3, contacts, N1, N2, da2, db1, db2, db3);
      // (-, +, 0): Case 3
      else
        numContacts = case3And(A3, A1, A2, B1, B2, B3, contacts, N1, N2, da1, db1, db2, db3);
    }
    else if (da2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case4And(A3, A1, A2, B1, B2, B3, contacts, N1, N2, da3, db1, db2, db3);
      // (-, -, 0): Case 1
      else
        numContacts = case1And(A3, B1, B2, B3, N1, db1, db2, db3, contacts);
    }
    else
    {
      // (-, 0, +): Case 3
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case3And(A2, A3, A1, B1, B2, B3, contacts, N1, N2, da3, db1, db2, db3);
      // (-, 0, -): Case 1
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case1And(A2, B1, B2, B3, N1, db1, db2, db3, contacts);
      // (-, 0, 0): Case 2
      else
        numContacts = case2And(A2, A3, B1, B2, B3, contacts, N1, db1, db2, db3);
    }
  }
  else
  {
    if (da2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case1And(A1, B1, B2, B3, N1, db1, db2, db3, contacts);
      // (0, +, -): Case 3
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case3And(A1, A2, A3, B1, B2, B3, contacts, N1, N2, da2, db1, db2, db3);
      // (0, +, 0): Case 2
      else
        numContacts = case2And(A3, A1, B1, B2, B3, contacts, N1, db1, db2, db3);
    }
    else if (da2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case3And(A1, A2, A3, B1, B2, B3, contacts, N1, N2, da2, db1, db2, db3);
      // (0, -, -): Case 1
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case1And(A1, B1, B2, B3, N1, db1, db2, db3, contacts);
      // (0, -, 0): Case 2
      else
        numContacts = case2And(A3, A1, B1, B2, B3, contacts, N1, db1, db2, db3);
    }
    else
    {
      // (0, 0, +): Case 2
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case2And(A1, A2, B1, B2, B3, contacts, N1, db1, db2, db3);
      // (0, 0, -): Case 2
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        numContacts = case2And(A1, A2, B1, B2, B3, contacts, N1, db1, db2, db3);
      // (0, 0, 0): Coplanar case
      else
        return coplanar3d(
              A1, A2, A3, B1, B2, B3, N1,
              contacts, normal, penetrationDepth, returnAllContacts);
    }
  }

//  if (numContacts)
//  {
//    const auto invNorm1 = 1.0 / N1.norm();
//    const auto invNorm2 = 1.0 / N2.norm();

//    double maxDepthA = std::numeric_limits<double>::max();
//    if (da1 <= 0.0 && maxDepthA > da1)
//        maxDepthA = da1;
//    if (da2 <= 0.0 && maxDepthA > da2)
//        maxDepthA = da2;
//    if (da3 <= 0.0 && maxDepthA > da3)
//        maxDepthA = da3;
//    maxDepthA = maxDepthA * invNorm1;

//    double maxDepthB = std::numeric_limits<double>::max();
//    if (db1 <= 0.0 && maxDepthB > db1)
//        maxDepthA = da1;
//    if (db2 <= 0.0 && maxDepthB > db2)
//        maxDepthA = da2;
//    if (db3 <= 0.0 && maxDepthB > db3)
//        maxDepthA = da3;
//    maxDepthA = maxDepthA * invNorm1;

//    if (maxDepthA < maxDepthB)
//    {

//    }
//    else
//    {

//    }
//  }

  return numContacts;
}

//==============================================================================
bool collideTriangleTriangle(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& A3,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3)
{
  Eigen::Vector3d v1 = B2 - B1;
  Eigen::Vector3d v2 = B3 - B2;
  Eigen::Vector3d N2 = v1.cross(v2);

  v1 = A1 - B3;
  const double da1 = v1.dot(N2);
  v1 = A2 - B3;
  const double da2 = v1.dot(N2);
  v1 = A3 - B3;
  const double da3 = v1.dot(N2);

  if ((da1 > DART_TRIANGLE_TRIANGLE_EPS &&
       da2 > DART_TRIANGLE_TRIANGLE_EPS &&
       da3 > DART_TRIANGLE_TRIANGLE_EPS)
      || (da1 < -DART_TRIANGLE_TRIANGLE_EPS &&
          da2 < -DART_TRIANGLE_TRIANGLE_EPS &&
          da3 < -DART_TRIANGLE_TRIANGLE_EPS))
  {
    return false;
  }

  // dot(3), cross(1)

  v1 = A2 - A1;
  v2 = A3 - A2;
  Eigen::Vector3d N1 = v1.cross(v2);

  v1 = B1 - A3;
  const double db1 = v1.dot(N1);
  v1 = B2 - A3;
  const double db2 = v1.dot(N1);
  v1 = B3 - A3;
  const double db3 = v1.dot(N1);

  if ((db1 > DART_TRIANGLE_TRIANGLE_EPS &&
       db2 > DART_TRIANGLE_TRIANGLE_EPS &&
       db3 > DART_TRIANGLE_TRIANGLE_EPS)
      || (db1 < -DART_TRIANGLE_TRIANGLE_EPS &&
          db2 < -DART_TRIANGLE_TRIANGLE_EPS &&
          db3 < -DART_TRIANGLE_TRIANGLE_EPS))
  {
    return false;
  }

  if (da1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (da2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(A3, A1, A2, B1, B2, B3, N1, N2, da3, db1, db2, db3);
      // (+, +, 0): Case 1
      else
        return case1And(A3, B1, B2, B3, N1, db1, db2, db3);
    }
    else if (da2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(A2, A3, A1, B1, B2, B3, N1, N2, da2, db1, db2, db3);
      // (+, -, -): Case 4
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(A1, A2, A3, B1, B2, B3, N1, N2, da1, db1, db2, db3);
      // (+, -, 0): Case 3
      else
        return case3And(A3, A1, A2, B1, B2, B3, N1, N2, da1, db1, db2, db3);
    }
    else
    {
      // (+, 0, +): Case 1
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1And(A2, B1, B2, B3, N1, db1, db2, db3);
      // (+, 0, -): Case 3
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3And(A2, A3, A1, B1, B2, B3, N1, N2, da3, db1, db2, db3);
      // (+, 0, 0): Case 2
      else
        return case2And(A2, A3, B1, B2, B3, N1, db1, db2, db3);
    }
  }
  else if (da1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (da2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(A1, A2, A3, B1, B2, B3, N1, N2, da1, db1, db2, db3);
      // (-, +, -): Case 4
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(A2, A3, A1, B1, B2, B3, N1, N2, da2, db1, db2, db3);
      // (-, +, 0): Case 3
      else
        return case3And(A3, A1, A2, B1, B2, B3, N1, N2, da1, db1, db2, db3);
    }
    else if (da2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(A3, A1, A2, B1, B2, B3, N1, N2, da3, db1, db2, db3);
      // (-, -, 0): Case 1
      else
        return case1And(A3, B1, B2, B3, N1, db1, db2, db3);
    }
    else
    {
      // (-, 0, +): Case 3
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3And(A2, A3, A1, B1, B2, B3, N1, N2, da3, db1, db2, db3);
      // (-, 0, -): Case 1
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1And(A2, B1, B2, B3, N1, db1, db2, db3);
      // (-, 0, 0): Case 2
      else
        return case2And(A2, A3, B1, B2, B3, N1, db1, db2, db3);
    }
  }
  else
  {
    if (da2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1And(A1, B1, B2, B3, N1, db1, db2, db3);
      // (0, +, -): Case 3
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3And(A1, A2, A3, B1, B2, B3, N1, N2, da2, db1, db2, db3);
      // (0, +, 0): Case 2
      else
        return case2And(A3, A1, B1, B2, B3, N1, db1, db2, db3);
    }
    else if (da2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3And(A1, A2, A3, B1, B2, B3, N1, N2, da2, db1, db2, db3);
      // (0, -, -): Case 1
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1And(A1, B1, B2, B3, N1, db1, db2, db3);
      // (0, -, 0): Case 2
      else
        return case2And(A3, A1, B1, B2, B3, N1, db1, db2, db3);
    }
    else
    {
      // (0, 0, +): Case 2
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2And(A1, A2, B1, B2, B3, N1, db1, db2, db3);
      // (0, 0, -): Case 2
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2And(A1, A2, B1, B2, B3, N1, db1, db2, db3);
      // (0, 0, 0): Coplanar case
      else
        return coplanar3d(A1, A2, A3, B1, B2, B3, N1);
    }
  }

  return false;
}

//==============================================================================
inline bool coplanar2d(
    const Eigen::Vector2d& a1,
    const Eigen::Vector2d& a2,
    const Eigen::Vector2d& a3,
    const Eigen::Vector2d& b1,
    const Eigen::Vector2d& b2,
    const Eigen::Vector2d& b3)
{
  if (checkCrossingLines(a2 - a1, b2 - b1, b1 - a1))
    return true;

  if (checkCrossingLines(a2 - a1, b3 - b2, b2 - a1))
    return true;

  if (checkCrossingLines(a2 - a1, b1 - b3, b3 - a1))
    return true;

  if (checkCrossingLines(a3 - a2, b2 - b1, b1 - a2))
    return true;

  if (checkCrossingLines(a3 - a2, b3 - b2, b2 - a2))
    return true;

  if (checkCrossingLines(a3 - a2, b1 - b3, b3 - a2))
    return true;

  if (checkCrossingLines(a1 - a3, b2 - b1, b1 - a3))
    return true;

  if (checkCrossingLines(a1 - a3, b3 - b2, b2 - a3))
    return true;

  if (checkCrossingLines(a1 - a3, b1 - b3, b3 - a3))
    return true;

  const Eigen::Vector2d b12 = b2 - b1;
  const Eigen::Vector2d b13 = b3 - b1;

  const Eigen::Vector2d b1a1 = a1 - b1;
  if (checkPointInTriangle(b12, b13, b1a1))
  {
    const Eigen::Vector2d b1a2 = a2 - b1;
    if (checkPointInTriangle(b12, b13, b1a2))
    {
      const Eigen::Vector2d b1a3 = a3 - b1;
      if (checkPointInTriangle(b12, b13, b1a3))
        return true;
    }
  }

  const Eigen::Vector2d a12 = a2 - a1;
  const Eigen::Vector2d a13 = a3 - a1;

  const Eigen::Vector2d a1b1 = b1 - a1;
  if (checkPointInTriangle(a12, a13, a1b1))
  {
    const Eigen::Vector2d a1b2 = b2 - a1;
    if (checkPointInTriangle(a12, a13, a1b2))
    {
      const Eigen::Vector2d a1b3 = b3 - a1;
      if (checkPointInTriangle(a12, a13, a1b3))
        return true;
    }
  }

  return false;
}

//==============================================================================
bool coplanar3d(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& A3,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1)
{
  Eigen::Vector2d a1;
  Eigen::Vector2d a2;
  Eigen::Vector2d a3;
  Eigen::Vector2d b1;
  Eigen::Vector2d b2;
  Eigen::Vector2d b3;

  int index;
  N1.cwiseAbs().maxCoeff(&index);

  switch (index)
  {
    case 0:
    {
      a1 = A1.tail<2>();
      a2 = A2.tail<2>();
      a3 = A3.tail<2>();

      b1 = B1.tail<2>();
      b2 = B2.tail<2>();
      b3 = B3.tail<2>();

      break;
    }
    case 1:
    {
      a1[0] = A1[0];
      a1[1] = A1[1];
      a2[0] = A2[0];
      a2[1] = A2[1];
      a3[0] = A3[0];
      a3[1] = A3[1];

      b1[0] = B1[0];
      b1[1] = B1[1];
      b2[0] = B2[0];
      b2[1] = B2[1];
      b3[0] = B3[0];
      b3[1] = B3[1];

      break;
    }
    default:
    {
      a1 = A1.head<2>();
      a2 = A2.head<2>();
      a3 = A3.head<2>();

      b1 = B1.head<2>();
      b2 = B2.head<2>();
      b3 = B3.head<2>();

      break;
    }
  }

  return coplanar2d(a1, a2, a3, b1, b2, b3);
}

} // namespace v1

} // namespace collision
} // namespace dart
