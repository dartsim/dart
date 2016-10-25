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
inline int checkPointAndLine(const Eigen::Vector3d& a1,
                             const Eigen::Vector3d& b1,
                             const Eigen::Vector3d& b2,
                             Eigen::Vector3d* contacts)
{
  const Eigen::Vector3d b21 = b2 - b1;

#ifndef NDEBUG
  Eigen::Vector3d b1a1 = b1 - a1;
  Eigen::Vector3d b2a1 = b2 - a1;

  Eigen::Vector3d cross1 = b21.cross(b1a1);
  Eigen::Vector3d cross2 = b21.cross(b2a1);

  assert(cross1.norm() < DART_TRIANGLE_TRIANGLE_EPS);
  assert(cross2.norm() < DART_TRIANGLE_TRIANGLE_EPS);
#endif

  const auto dot = (a1 - b1).dot(b21);
  if (dot < 0.0)
    return 0;

  const auto squaredLength = b21.squaredNorm();
  if (dot > squaredLength)
    return 0;

  contacts[0] = a1;

  return 1;
}

//==============================================================================
inline int checkColinearLines(const Eigen::Vector3d& aMin,
                              const Eigen::Vector3d& aMax,
                              const Eigen::Vector3d& bMin,
                              const Eigen::Vector3d& bMax,
                              const double daMin,
                              const double daMax,
                              double dbMin,
                              double dbMax,
                              Eigen::Vector3d* contacts)
{
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
        contacts[0] = aMin;  // or bMax or 0.5*(aMin+bMax)
        return 1;
      }

      contacts[0] = aMin;
      contacts[1] = bMax;
      return 2;
    }
    else
    {
      // collision: (aMin, aMax)
      contacts[0] = aMin;
      contacts[1] = aMax;
      return 2;
    }
  }
  else if (dbMin < daMax)
  {
    if (dbMax < daMax)
    {
      // collision: (bMin, bMax)
      contacts[0] = bMin;
      contacts[1] = bMax;
      return 2;
    }
    else
    {
      // collision: (bMin, aMax)

      if (daMax - dbMin < DART_TRIANGLE_TRIANGLE_EPS)
      {
        contacts[0] = aMax; // or bMin or 0.5*(aMax+bMin)
        return 1;
      }

      contacts[0] = bMin;
      contacts[1] = aMax;
      return 2;
    }
  }
  else
  {
    return 0;
  }
}

//==============================================================================
inline int checkColinearLines(const Eigen::Vector3d& a1,
                            const Eigen::Vector3d& a2,
                            const Eigen::Vector3d& b1,
                            const Eigen::Vector3d& b2,
                            Eigen::Vector3d* contacts)
{
  const Eigen::Vector3d a21 = a2 - a1;

  const auto da1 = a21.dot(a1);
  const auto da2 = a21.dot(a2);
  const auto db1 = a21.dot(b1);
  const auto db2 = a21.dot(b2);

  if (da1 < da2)
  {
    if (db1 < db2)
      return checkColinearLines(a1, a2, b1, b2, da1, da2, db1, db2, contacts);
    else
      return checkColinearLines(a1, a2, b2, b1, da1, da2, db2, db1, contacts);
  }
  else
  {
    if (db1 < db2)
      return checkColinearLines(a2, a1, b1, b2, da2, da1, db1, db2, contacts);
    else
      return checkColinearLines(a2, a1, b2, b1, da2, da1, db2, db1, contacts);
  }
}

//==============================================================================
inline void computePointOnPlane(const Eigen::Vector3d& b2,
                                const Eigen::Vector3d& b3,
                                const Eigen::Vector3d& n1,
                                const double db2,
                                Eigen::Vector3d& b32OnPlaneA)
{
  const Eigen::Vector3d b32 = b3 - b2;
  b32OnPlaneA = b2;
  b32OnPlaneA.noalias() -= (db2 / n1.dot(b32)) * b32;
}

//==============================================================================
inline int case1AndCase1(const Eigen::Vector3d& a1,
                         const Eigen::Vector3d& b1,
                         Eigen::Vector3d* contacts)
{
  if (a1 == b1) // TODO(JS): ((a1 - b1).squaredNorm() < eps*eps) ?
  {
    contacts[0] = a1;
    return 1;
  }

  return 0;
}

//==============================================================================
inline int case1AndCase2(const Eigen::Vector3d& a1,
                         const Eigen::Vector3d& b1,
                         const Eigen::Vector3d& b2,
                         Eigen::Vector3d* contacts)
{
  return checkPointAndLine(a1, b1, b2, contacts);
}

//==============================================================================
inline int case1AndCase3(const Eigen::Vector3d& a1,
                  const Eigen::Vector3d& b1,
                  const Eigen::Vector3d& b2,
                  const Eigen::Vector3d& b3,
                  const Eigen::Vector3d& n1,
                  const double db2,
                  Eigen::Vector3d* contacts)
{
  Eigen::Vector3d b32OnPlaneA;
  computePointOnPlane(b2, b3, n1, db2, b32OnPlaneA);

  return checkPointAndLine(a1, b1, b32OnPlaneA, contacts);
}

//==============================================================================
inline int case1AndCase4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    const double db1,
    Eigen::Vector3d* contacts)
{
  Eigen::Vector3d b21OnPlaneA;
  Eigen::Vector3d b31OnPlaneA;
  computePointOnPlane(b1, b2, n1, db1, b21OnPlaneA);
  computePointOnPlane(b1, b3, n1, db1, b31OnPlaneA);

#ifndef NDEBUG
  auto tmp1 = n1.dot(b21OnPlaneA);
  auto tmp2 = n1.dot(b31OnPlaneA);
  auto tmp3 = n1.dot(a1);

  assert(std::abs(tmp1 - tmp2) < DART_TRIANGLE_TRIANGLE_EPS);
  assert(std::abs(tmp2 - tmp3) < DART_TRIANGLE_TRIANGLE_EPS);
#endif

  return checkPointAndLine(a1, b21OnPlaneA, b31OnPlaneA, contacts);
}

//==============================================================================
inline int case2AndCase2(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    Eigen::Vector3d* contacts)
{
  return checkColinearLines(a1, a2, b1, b2, contacts);
}

//==============================================================================
inline int case2AndCase3(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    const double db2,
    Eigen::Vector3d* contacts)
{
  Eigen::Vector3d b32OnPlaneA;
  computePointOnPlane(b2, b3, n1, db2, b32OnPlaneA);

  return checkColinearLines(a1, a2, b1, b32OnPlaneA, contacts);
}

//==============================================================================
inline int case2AndCase4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    const double db1,
    Eigen::Vector3d* contacts)
{
  Eigen::Vector3d b21OnPlaneA;
  Eigen::Vector3d b31OnPlaneA;
  computePointOnPlane(b1, b2, n1, db1, b21OnPlaneA);
  computePointOnPlane(b1, b3, n1, db1, b31OnPlaneA);

  return checkColinearLines(a1, a2, b21OnPlaneA, b31OnPlaneA, contacts);
}

//==============================================================================
inline int case3AndCase3(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    const Eigen::Vector3d& n2,
    const double da2, const double db2,
    Eigen::Vector3d* contacts)
{
  Eigen::Vector3d a32OnPlaneB;
  computePointOnPlane(a2, a3, n2, da2, a32OnPlaneB);

  Eigen::Vector3d b32OnPlaneA;
  computePointOnPlane(b2, b3, n1, db2, b32OnPlaneA);

  return checkColinearLines(a1, a32OnPlaneB, b1, b32OnPlaneA, contacts);
}

//==============================================================================
inline int case3AndCase4(
    const Eigen::Vector3d& a1, // 0
    const Eigen::Vector3d& a2, // +/-
    const Eigen::Vector3d& a3, // -/+
    const Eigen::Vector3d& b1, // +/-
    const Eigen::Vector3d& b2, // -/+
    const Eigen::Vector3d& b3, // -/+
    const Eigen::Vector3d& n1,
    const Eigen::Vector3d& n2,
    const double da2,
    const double db1,
    Eigen::Vector3d* contacts)
{
  Eigen::Vector3d a32OnPlaneB;
  computePointOnPlane(a2, a3, n2, da2, a32OnPlaneB);

  Eigen::Vector3d b21OnPlaneA;
  Eigen::Vector3d b31OnPlaneA;
  computePointOnPlane(b1, b2, n1, db1, b21OnPlaneA);
  computePointOnPlane(b1, b3, n1, db1, b31OnPlaneA);

  return checkColinearLines(a1, a32OnPlaneB, b21OnPlaneA, b31OnPlaneA, contacts);
}

//==============================================================================
inline int case4AndCase4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    const Eigen::Vector3d& n2,
    const double da1,
    const double db1,
    Eigen::Vector3d* contacts)
{
  Eigen::Vector3d a21OnPlaneB;
  Eigen::Vector3d a31OnPlaneB;
  computePointOnPlane(a1, a2, n2, da1, a21OnPlaneB);
  computePointOnPlane(a1, a3, n2, da1, a31OnPlaneB);

  Eigen::Vector3d b21OnPlaneA;
  Eigen::Vector3d b31OnPlaneA;
  computePointOnPlane(b1, b2, n1, db1, b21OnPlaneA);
  computePointOnPlane(b1, b3, n1, db1, b31OnPlaneA);

  return checkColinearLines(
        a21OnPlaneB, a31OnPlaneB, b21OnPlaneA, b31OnPlaneA, contacts);
}

//==============================================================================
inline int case1And(const Eigen::Vector3d& a1,
                    const Eigen::Vector3d& b1,
                    const Eigen::Vector3d& b2,
                    const Eigen::Vector3d& b3,
                    const Eigen::Vector3d& n1,
                    const double db1, const double db2, const double db3,
                    Eigen::Vector3d* contacts)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(a1, b3, b1, b2, n1, db3, contacts);
      // (+, +, 0): Case 1
      else
        return case1AndCase1(a1, b3, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(a1, b2, b3, b1, n1, db2, contacts);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(a1, b1, b2, b3, n1, db1, contacts);
      // (+, -, 0): Case 3
      else
        return case1AndCase3(a1, b3, b1, b2, n1, db1, contacts);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(a1, b2, contacts);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(a1, b2, b3, b1, n1, db3, contacts);
      // (+, 0, 0): Case 2
      else
        return case1AndCase2(a1, b2, b3, contacts);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(a1, b1, b2, b3, n1, db1, contacts);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(a1, b2, b3, b1, n1, db2, contacts);
      // (-, +, 0): Case 3
      else
        return case1AndCase3(a1, b3, b1, b2, n1, db1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(a1, b3, b1, b2, n1, db3, contacts);
      // (-, -, 0): Case 1
      else
        return case1AndCase1(a1, b3, contacts);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(a1, b2, b3, b1, n1, db3, contacts);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(a1, b2, contacts);
      // (-, 0, 0): Case 2
      else
        return case1AndCase2(a1, b2, b3, contacts);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(a1, b1, contacts);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(a1, b1, b2, b3, n1, db2, contacts);
      // (0, +, 0): Case 2
      else
        return case1AndCase2(a1, b3, b1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(a1, b1, b2, b3, n1, db2, contacts);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(a1, b1, contacts);
      // (0, -, 0): Case 2
      else
        return case1AndCase2(a1, b3, b1, contacts);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(a1, b1, b2, contacts);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(a1, b1, b2, contacts);
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
inline int case2And(const Eigen::Vector3d& a1,
             const Eigen::Vector3d& a2,
             const Eigen::Vector3d& b1,
             const Eigen::Vector3d& b2,
             const Eigen::Vector3d& b3,
             Eigen::Vector3d* contacts,
             const Eigen::Vector3d& n1,
             const double db1, const double db2, const double db3)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(a1, a2, b3, b1, b2, n1, db3, contacts);
      // (+, +, 0): Case 1
      else
        return case1AndCase2(b3, a1, a2, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(a1, a2, b2, b3, b1, n1, db2, contacts);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(a1, a2, b1, b2, b3, n1, db1, contacts);
      // (+, -, 0): Case 3
      else
        return case2AndCase3(a1, a2, b3, b1, b2, n1, db1, contacts);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(b2, a1, a2, contacts);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(a1, a2, b2, b3, b1, n1, db3, contacts);
      // (+, 0, 0): Case 2
      else
        return case2AndCase2(a1, a2, b2, b3, contacts);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(a1, a2, b1, b2, b3, n1, db1, contacts);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(a1, a2, b2, b3, b1, n1, db2, contacts);
      // (-, +, 0): Case 3
      else
        return case2AndCase3(a1, a2, b3, b1, b2, n1, db1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(a1, a2, b3, b1, b2, n1, db3, contacts);
      // (-, -, 0): Case 1
      else
        return case1AndCase2(b3, a1, a2, contacts);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(a1, a2, b2, b3, b1, n1, db3, contacts);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(b2, a1, a2, contacts);
      // (-, 0, 0): Case 2
      else
        return case2AndCase2(a1, a2, b2, b3, contacts);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(b1, a1, a2, contacts);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(a1, a2, b1, b2, b3, n1, db2, contacts);
      // (0, +, 0): Case 2
      else
        return case2AndCase2(a1, a2, b3, b1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(a1, a2, b1, b2, b3, n1, db2, contacts);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(b1, a1, a2, contacts);
      // (0, -, 0): Case 2
      else
        return case2AndCase2(a1, a2, b3, b1, contacts);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase2(a1, a2, b1, b2, contacts);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase2(a1, a2, b1, b2, contacts);
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
inline int case3And(const Eigen::Vector3d& a1,
             const Eigen::Vector3d& a2,
             const Eigen::Vector3d& a3,
             const Eigen::Vector3d& b1,
             const Eigen::Vector3d& b2,
             const Eigen::Vector3d& b3,
             Eigen::Vector3d* contacts,
             const Eigen::Vector3d& n1,
             const Eigen::Vector3d& n2,
             const double da2,
             const double db1, const double db2, const double db3)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(a1, a2, a3, b3, b1, b2, n1, n2, da2, db3, contacts);
      // (+, +, 0): Case 1
      else
        return case1AndCase3(b3, a1, a2, a3, n2, da2, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(a1, a2, a3, b2, b3, b1, n1, n2, da2, db2, contacts);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(a1, a2, a3, b1, b2, b3, n1, n2, da2, db1, contacts);
      // (+, -, 0): Case 3
      else
        return case3AndCase3(a1, a2, a3, b3, b1, b2, n2, n1, da2, db1, contacts);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(b2, a1, a2, a3, n2, da2, contacts);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(a1, a2, a3, b2, b3, b1, n2, n1, da2, db3, contacts);
      // (+, 0, 0): Case 2
      else
        return case2AndCase3(b2, b3, a1, a2, a3, n2, da2, contacts);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(a1, a2, a3, b1, b2, b3, n1, n2, da2, db1, contacts);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(a1, a2, a3, b2, b3, b1, n1, n2, da2, db2, contacts);
      // (-, +, 0): Case 3
      else
        return case3AndCase3(a1, a2, a3, b3, b1, b2, n2, n1, da2, db1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(a1, a2, a3, b3, b1, b2, n1, n2, da2, db3, contacts);
      // (-, -, 0): Case 1
      else
        return case1AndCase3(b3, a1, a2, a3, n2, da2, contacts);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(a1, a2, a3, b2, b3, b1, n2, n1, da2, db3, contacts);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(b2, a1, a2, a3, n2, da2, contacts);
      // (-, 0, 0): Case 2
      else
        return case2AndCase3(b2, b3, a1, a2, a3, n2, da2, contacts);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(b1, a1, a2, a3, n2, da2, contacts);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(a1, a2, a3, b1, b2, b3, n2, n1, da2, db2, contacts);
      // (0, +, 0): Case 2
      else
        return case2AndCase3(b3, b1, a1, a2, a3, n2, da2, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(a1, a2, a3, b1, b2, b3, n2, n1, da2, db2, contacts);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(b1, a1, a2, a3, n2, da2, contacts);
      // (0, -, 0): Case 2
      else
        return case2AndCase3(b3, b1, a1, a2, a3, n2, da2, contacts);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(b1, b2, a1, a2, a3, n2, da2, contacts);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(b1, b2, a1, a2, a3, n2, da2, contacts);
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
inline int case4And(const Eigen::Vector3d& a1,
                    const Eigen::Vector3d& a2,
                    const Eigen::Vector3d& a3,
                    const Eigen::Vector3d& b1,
                    const Eigen::Vector3d& b2,
                    const Eigen::Vector3d& b3,
                    Eigen::Vector3d* contacts,
                    const Eigen::Vector3d& n1,
                    const Eigen::Vector3d& n2,
                    const double da1,
                    const double db1, const double db2, const double db3)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(a1, a2, a3, b3, b1, b2, n1, n2, da1, db3, contacts);
      // (+, +, 0): Case 1
      else
        return case1AndCase4(b3, a1, a2, a3, n2, da1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(a1, a2, a3, b2, b3, b1, n1, n2, da1, db2, contacts);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(a1, a2, a3, b1, b2, b3, n1, n2, da1, db1, contacts);
      // (+, -, 0): Case 3
      else
        return case3AndCase4(b3, b1, b2, a1, a2, a3, n2, n1, db1, da1, contacts);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(b2, a1, a2, a3, n2, da1, contacts);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(b2, b3, b1, a1, a2, a3, n2, n1, db3, da1, contacts);
      // (+, 0, 0): Case 2
      else
        return case2AndCase4(b2, b3, a1, a2, a3, n2, da1, contacts);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(a1, a2, a3, b1, b2, b3, n1, n2, da1, db1, contacts);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(a1, a2, a3, b2, b3, b1, n1, n2, da1, db2, contacts);
      // (-, +, 0): Case 3
      else
        return case3AndCase4(b3, b1, b2, a1, a2, a3, n2, n1, db1, da1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(a1, a2, a3, b3, b1, b2, n1, n2, da1, db3, contacts);
      // (-, -, 0): Case 1
      else
        return case1AndCase4(b3, a1, a2, a3, n2, da1, contacts);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(b2, b3, b1, a1, a2, a3, n2, n1, db3, da1, contacts);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(b2, a1, a2, a3, n2, da1, contacts);
      // (-, 0, 0): Case 2
      else
        return case2AndCase4(b2, b3, a1, a2, a3, n2, da1, contacts);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(b1, a1, a2, a3, n2, da1, contacts);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(b1, b2, b3, a1, a2, a3, n2, n1, db2, da1, contacts);
      // (0, +, 0): Case 2
      else
        return case2AndCase4(b3, b1, a1, a2, a3, n2, da1, contacts);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(b1, b2, b3, a1, a2, a3, n2, n1, db2, da1, contacts);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(b1, a1, a2, a3, n2, da1, contacts);
      // (0, -, 0): Case 2
      else
        return case2AndCase4(b3, b1, a1, a2, a3, n2, da1, contacts);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(b1, b2, a1, a2, a3, n2, da1, contacts);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(b1, b2, a1, a2, a3, n2, da1, contacts);
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
inline double orient2d(const Eigen::Vector2d& a,
                       const Eigen::Vector2d& b,
                       const Eigen::Vector2d& c)
{
  // Returns 2d vector cross product of (a - c) and (b - c)
  return ((a[0] - c[0]) * (b[1] - c[1]) - (a[1] - c[1]) * (b[0] - c[0]));
}

//==============================================================================
inline double cross2d(const Eigen::Vector2d& u, const Eigen::Vector2d& v)
{
  return ((u[0]) * (v[1]) - (u[1]) * (v[0]));
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

  const auto t = cross2d(a1b1, b12) * invCrossLines;
  if (0.0 >= t || t >= 1.0)
    return;

  const auto u = cross2d(a1b1, a12) * invCrossLines;
  if (0.0 >= u || u >= 1.0)
    return;

  contacts[numContacts++] = A1 + t * A12;
}

//==============================================================================
inline int planeCase1(const Eigen::Vector3d& A1,
                      const Eigen::Vector3d& A2,
                      const Eigen::Vector3d& A3,
                      Eigen::Vector3d* contacts)
{
  contacts[0] = A1;
  contacts[1] = A2;
  contacts[2] = A3;
  return 3;
}

//==============================================================================
inline int planeCase2(const Eigen::Vector3d& A1,
                      const Eigen::Vector3d& A2,
                      Eigen::Vector3d* contacts)
{
  contacts[0] = A1;
  contacts[1] = A2;
  return 2;
}

//==============================================================================
inline int planeCase3(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& B1,
    const double ua1, const double va1,
    const double ua2, const double va2,
    const double ub1, const double vb1,
    Eigen::Vector3d* contacts,
    const bool returnAllContacts)
{
  assert(ua1 >= 0.0);
  assert(va1 >= 0.0);
  assert(ua2 >= 0.0);
  assert(va2 >= 0.0);
  assert(ua1 + va1 <= 1.0);
  assert(ua2 + va2 <= 1.0);

  if (ub1 < DART_TRIANGLE_TRIANGLE_EPS)
  {

  }
  else if (vb1 < DART_TRIANGLE_TRIANGLE_EPS)
  {

  }
  else if ((1.0 - (ub1 + vb1)) < DART_TRIANGLE_TRIANGLE_EPS)
  {

  }
  else
  {

  }

  // A1 is on (B2-B1)
  if (ua1 < DART_TRIANGLE_TRIANGLE_EPS)
  {
    // A2 is on (B2-B1)
    if (ua2 < DART_TRIANGLE_TRIANGLE_EPS)
    {
      contacts[0] = A1;
      contacts[1] = A2;
      return 2;
    }
    // A2 is on (B3-B1)
    else if (va2 < DART_TRIANGLE_TRIANGLE_EPS)
    {
      contacts[0] = A1;
      contacts[1] = A2;

      if (!returnAllContacts) // (returnAllContacts == 3)
      {
        contacts[2] = B1;
        return 3;
      }

      // TODO(JS): compute intersect point1 for contacts[2]
      contacts[3] = B1;
      return 4;

    }
    // A2 is on (B3-B2)
    else if ((1.0 - (ua2 + va2)) < DART_TRIANGLE_TRIANGLE_EPS)
    {
      contacts[0] = A2;
    }
    // A2 is strictly inside triangle B
    else
    {
      contacts[0] = A1;
      contacts[1] = A2;

      if (!returnAllContacts) // (returnAllContacts == 3)
      {
        contacts[2] = B1;
        return 3;
      }

      // TODO(JS): compute intersect point1 for contacts[2]
      contacts[3] = B1;
      return 4;
    }
  }
  // A1 is on (B3-B1)
  else if (va1 < DART_TRIANGLE_TRIANGLE_EPS)
  {
    // A2 is on (B2-B1)
    if (ua2 < DART_TRIANGLE_TRIANGLE_EPS)
    {

    }
    // A2 is on (B3-B1)
    else if (va2 < DART_TRIANGLE_TRIANGLE_EPS)
    {
      contacts[0] = A1;
      contacts[1] = A2;
      return 2;
    }
    // A2 is on (B3-B2)
    else if ((1.0 - (ua2 + va2)) < DART_TRIANGLE_TRIANGLE_EPS)
    {

    }
    // A2 is strictly inside triangle B
    else
    {

    }
  }
  // A1 is on (B3-B2)
  else if ((1.0 - (ua1 + va1)) < DART_TRIANGLE_TRIANGLE_EPS)
  {
    // A2 is on (B2-B1)
    if (ua2 < DART_TRIANGLE_TRIANGLE_EPS)
    {

    }
    // A2 is on (B3-B1)
    else if (va2 < DART_TRIANGLE_TRIANGLE_EPS)
    {

    }
    // A2 is on (B3-B2)
    else if ((1.0 - (ua2 + va2)) < DART_TRIANGLE_TRIANGLE_EPS)
    {
      contacts[0] = A1;
      contacts[1] = A2;
      return 2;
    }
    // A2 is strictly inside triangle B
    else
    {

    }
  }
  // A1 is strictly inside triangle B
  else
  {
    // A2 is on (B2-B1)
    if (ua2 < DART_TRIANGLE_TRIANGLE_EPS)
    {
      contacts[0] = A1;
      contacts[1] = A2;
      return 2;
    }
    // A2 is on (B3-B1)
    else if (va2 < DART_TRIANGLE_TRIANGLE_EPS)
    {

    }
    // A2 is on (B3-B2)
    else if ((1.0 - (ua2 + va2)) < DART_TRIANGLE_TRIANGLE_EPS)
    {

    }
    // A2 is strictly inside triangle B
    else
    {

    }
  }

//  contacts[0] = A1;
//  contacts[1] = A2;
//  contacts[2] = B1;

//  assert(returnAllContacts >= 3);
//  if (returnAllContacts < 4)
//    return 3;

//  // TODO(JS): implementation
//  // TODO(JS): consider absolute margin rather than proportional
//  if (std::abs(ua1 + va1 - 1.0) < DART_TRIANGLE_TRIANGLE_EPS)
//  {
//    contacts[1] = A1;
//  }

  return 0;
}

//==============================================================================
inline int planeCase4()
{
  return 0;
}

//==============================================================================
inline int planeCase5()
{
  return 0;
}

//==============================================================================
inline int planeCase6()
{
  return 0;
}

//==============================================================================
inline int planeCase7()
{
  return 0;
}

//==============================================================================
inline int planeCase8()
{
  return 0;
}

//==============================================================================
inline int planeCase9()
{
  return 0;
}

//==============================================================================
inline int coplanarAooo(const Eigen::Vector3d& A1,
                        const Eigen::Vector3d& A2,
                        const Eigen::Vector3d& A3,
                        Eigen::Vector3d* contacts)
{
  return planeCase1(A1, A2, A3, contacts);
}

//==============================================================================
inline int coplanarAoox(
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
    const bool inB1,
    const bool inB2,
    const bool inB3,
    const double ua1, const double va1,
    const double ua2, const double va2,
    const double ua3, const double va3,
    const double ub1, const double vb1,
    const double ub2, const double vb2,
    const double ub3, const double vb3,
    const int index,
    Eigen::Vector3d* contacts,
    const bool returnAllContacts)
{
  if (inB1)
  {
    if (inB2)
    {
      if (inB3)
      {
        // B: (O, O, O)

        // Case 1 (switch A and B)
        return planeCase1(B1, B2, B3, contacts);
      }
      else
      {
        // B: (O, O, X)

        assert((A1.isApprox(B1) && A2.isApprox(B2)) ||
               (A1.isApprox(B2) && A2.isApprox(B1)));

        // Case 2
        return planeCase2(A1, A2, contacts); // or return B1, B2
      }
    }
    else
    {
      if (inB3)
      {
        // B: (O, X, O)

        assert((A1.isApprox(B3) && A2.isApprox(B1)) ||
               (A1.isApprox(B1) && A2.isApprox(B3)));

        // Case 2
        return planeCase2(A1, A2, contacts); // or return B3, B1
      }
      else
      {
        // B: (O, X, X)

        // Case 3
        return planeCase3(A1, A2, B1, ua1, va1, ua2, va2, ub1, vb1, contacts, returnAllContacts);
      }
    }
  }
  else
  {
    if (inB2)
    {
      if (inB3)
      {
        // B: (X, 0, 0)

        assert((A1.isApprox(B2) && A2.isApprox(B3)) ||
               (A1.isApprox(B3) && A2.isApprox(B2)));

        // Case 2
        return planeCase2(A1, A2, contacts); // or return B2, B3
      }
      else
      {
        // B: (X, 0, X)

        // Case 3
        return 0;
      }
    }
    else
    {
      if (inB3)
      {
        // B: (X, X, 0)

        // Case 3
        return 0;
      }
      else
      {
        // B: (X, X, X)

        // Case 4
        return 0;
      }
    }
  }
}

//==============================================================================
inline int coplanarAoxx(
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
    const bool inB1,
    const bool inB2,
    const bool inB3,
    const double ua1, const double va1,
    const double ua2, const double va2,
    const double ua3, const double va3,
    const double ub1, const double vb1,
    const double ub2, const double vb2,
    const double ub3, const double vb3,
    const int index,
    Eigen::Vector3d* contacts,
    const bool returnAllContacts)
{
  if (inB1)
  {
    if (inB2)
    {
      if (inB3)
      {
        // B: (O, O, O)

        // Case 1 (switch A and B)
        return planeCase1(B1, B2, B3, contacts);
      }
      else
      {
        // B: (O, O, X)

        // Case 3 (switch A and B)
        return 0;
      }
    }
    else
    {
      if (inB3)
      {
        // B: (O, X, O)

        // Case 3 (switch A and B)
        return 0;
      }
      else
      {
        // B: (O, X, X)

        // Case 5
        return 0;
      }
    }
  }
  else
  {
    if (inB2)
    {
      if (inB3)
      {
        // B: (X, 0, 0)

        // Case 3 (switch A and B)
        return planeCase3(B2, B3, A1, ub2, vb2, ub3, vb3, ua1, va1, contacts, returnAllContacts);
      }
      else
      {
        // B: (X, 0, X)

        // Case 5
        return 0;
      }
    }
    else
    {
      if (inB3)
      {
        // B: (X, X, 0)

        // Case 5
        return 0;
      }
      else
      {
        // B: (X, X, X)

        // Case 6 or 7
        return 0;
      }
    }
  }
}

//==============================================================================
inline int coplanarAxxx(
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
    const bool inB1,
    const bool inB2,
    const bool inB3,
    const double ub1, const double vb1,
    const double ub2, const double vb2,
    const double ub3, const double vb3,
    const int index,
    Eigen::Vector3d* contacts,
    const bool returnAllContacts)
{
  if (inB1)
  {
    if (inB2)
    {
      if (inB3)
      {
        // B: (O, O, O)

        // Case 1 (switch A and B)
        return planeCase1(B1, B2, B3, contacts);
      }
      else
      {
        // B: (O, O, X)

        // Case 4 (switch A and B)
        return 0;
      }
    }
    else
    {
      if (inB3)
      {
        // B: (O, X, O)

        // Case 4 (switch A and B)
        return 0;
      }
      else
      {
        // B: (O, X, X)

        // Case 6 or 7 (switch A and B)
        return 0;
      }
    }
  }
  else
  {
    if (inB2)
    {
      if (inB3)
      {
        // B: (X, 0, 0)

        // Case 4 (switch A and B)
        return 0;
      }
      else
      {
        // B: (X, 0, X)

        // Case 6 or 7 (switch A and B)
        return 0;
      }
    }
    else
    {
      if (inB3)
      {
        // B: (X, X, 0)

        // Case 6 or 7 (switch A and B)
        return 0;
      }
      else
      {
        // B: (X, X, X)

        // Case 8 or 9 (switch A and B)
        return 0;
      }
    }
  }
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
    const bool returnAllContacts)
{
  int numContacts = 0;

  const Eigen::Vector2d b21 = b2 - b1;
  const Eigen::Vector2d b31 = b3 - b1;

  const Eigen::Vector2d a1b1 = a1 - b1;
  const Eigen::Vector2d a2b1 = a2 - b1;
  const Eigen::Vector2d a3b1 = a3 - b1;

  double ua1;
  double ua2;
  double ua3;

  double va1;
  double va2;
  double va3;

  const bool inA1 = checkPointInTriangle(b21, b31, a1b1, ua1, va1);
  const bool inA2 = checkPointInTriangle(b21, b31, a2b1, ua2, va2);
  const bool inA3 = checkPointInTriangle(b21, b31, a3b1, ua3, va3);

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

  if (inA1)
  {
    contacts[numContacts++] = A1;
    updateVertexCheckNecessity(ua1, va1, inB1, inB2, inB3);
    updateEdgeCheckNecessity(
          ua1, va1,
          needCheckA12B12, needCheckA12B23, needCheckA12B31,
          needCheckA31B12, needCheckA31B23, needCheckA31B31);
  }

  if (inA2)
  {
    contacts[numContacts++] = A2;
    updateVertexCheckNecessity(ua2, va2, inB1, inB2, inB3);
    updateEdgeCheckNecessity(
          ua2, va2,
          needCheckA23B12, needCheckA23B23, needCheckA23B31,
          needCheckA12B12, needCheckA12B23, needCheckA12B31);
  }

  if (inA3)
  {
    contacts[numContacts++] = A3;
    updateVertexCheckNecessity(ua3, va3, inB1, inB2, inB3);
    updateEdgeCheckNecessity(
          ua3, va3,
          needCheckA31B12, needCheckA31B23, needCheckA31B31,
          needCheckA23B12, needCheckA23B23, needCheckA23B31);

    if (numContacts == 3)
      return numContacts;
  }

  const Eigen::Vector2d a21 = a2 - a1;
  const Eigen::Vector2d a31 = a3 - a1;

  double ub1;
  double ub2;
  double ub3;

  double vb1;
  double vb2;
  double vb3;

  const int numContactsOnA = numContacts;

  if (!inB1)
  {
    const Eigen::Vector2d b1a1 = b1 - a1;
    inB1 = checkPointInTriangle(a21, a31, b1a1, ub1, vb1);

    if (inB1)
    {
      contacts[numContacts++] = B1;
      updateEdgeCheckNecessity(
            ub1, vb1,
            needCheckA12B12, needCheckA23B12, needCheckA31B12,
            needCheckA12B31, needCheckA23B31, needCheckA31B31);
    }
  }

  if (!inB2)
  {
    const Eigen::Vector2d b2a1 = b2 - a1;
    inB2 = checkPointInTriangle(a21, a31, b2a1, ub2, vb2);

    if (inB2)
    {
      contacts[numContacts++] = B2;
      updateEdgeCheckNecessity(
            ub2, vb2,
            needCheckA12B23, needCheckA23B23, needCheckA31B23,
            needCheckA12B12, needCheckA23B12, needCheckA31B12);
    }
  }

  if (!inB3)
  {
    const Eigen::Vector2d b3a1 = b3 - a1;
    inB3 = checkPointInTriangle(a21, a31, b3a1, ub3, vb3);

    if (inB3)
    {
      contacts[numContacts++] = B3;
      updateEdgeCheckNecessity(
            ub3, vb3,
            needCheckA12B31, needCheckA23B31, needCheckA31B31,
            needCheckA12B23, needCheckA23B23, needCheckA31B23);
    }

    if ((numContacts - numContactsOnA) == 3)
    {
      assert(numContactsOnA == 0);
      return numContacts;
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
//inline int coplanar2d(
//    const Eigen::Vector3d& A1,
//    const Eigen::Vector3d& A2,
//    const Eigen::Vector3d& A3,
//    const Eigen::Vector3d& B1,
//    const Eigen::Vector3d& B2,
//    const Eigen::Vector3d& B3,
//    const Eigen::Vector2d& a1,
//    const Eigen::Vector2d& a2,
//    const Eigen::Vector2d& a3,
//    const Eigen::Vector2d& b1,
//    const Eigen::Vector2d& b2,
//    const Eigen::Vector2d& b3,
//    const bool ccwA,
//    const bool ccwB,
//    const int index,
//    Eigen::Vector3d* contacts,
//    const bool returnAllContacts)
//{
//  if (ccwA)
//  {
//    if (ccwB)
//    {
//      return coplanar2dCcw(
//            A1, A2, A3, B1, B2, B3,
//            a1, a2, a3, b1, b2, b3, index, contacts, returnAllContacts);
//    }
//    else
//    {
//      return coplanar2dCcw(
//            A1, A2, A3, B3, B2, B1,
//            a1, a2, a3, b3, b2, b1, index, contacts, returnAllContacts);
//    }
//  }
//  else
//  {
//    if (ccwB)
//    {
//      return coplanar2dCcw(
//            A3, A2, A1, B1, B2, B3,
//            a3, a2, a1, b1, b2, b3, index, contacts, returnAllContacts);
//    }
//    else
//    {
//      return coplanar2dCcw(
//            A3, A2, A1, B3, B2, B1,
//            a3, a2, a1, b3, b2, b1, index, contacts, returnAllContacts);
//    }
//  }
//}

//==============================================================================
inline int coplanar3d(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& A3,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    const Eigen::Vector3d& N1,
//    const Eigen::Vector3d& N2,
    Eigen::Vector3d* contacts,
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

//  bool ccwA = true;
//  bool ccwB = true;

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

//      if (N1[0] < 0.0)
//        ccwA = false;

//      if (N2[0] < 0.0)
//        ccwB = false;

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

//      if (N1[1] < 0.0)
//        ccwA = false;

//      if (N2[1] < 0.0)
//        ccwB = false;

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

//      if (N1[2] < 0.0)
//        ccwA = false;

//      if (N2[2] < 0.0)
//        ccwB = false;

      break;
    }
  }

//  return coplanar2d(
//        A1, A2, A3, B1, B2, B3,
//        a1, a2, a3, b1, b2, b3,
//        ccwA, ccwB,
//        index,
//        contacts, returnAllContacts);

  return coplanar2d(
        A1, A2, A3, B1, B2, B3,
        a1, a2, a3, b1, b2, b3,
//        ccwA, ccwB,
//        index,
        contacts, returnAllContacts);
}

//==============================================================================
int collideTriangleTriangle(
    const Eigen::Vector3d& A1,
    const Eigen::Vector3d& A2,
    const Eigen::Vector3d& A3,
    const Eigen::Vector3d& B1,
    const Eigen::Vector3d& B2,
    const Eigen::Vector3d& B3,
    Eigen::Vector3d* contacts,
    const bool returnAllContacts)
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
        return case4And(A3, A1, A2, B1, B2, B3, contacts, N1, N2, da3, db1, db2, db3);
      // (+, +, 0): Case 1
      else
        return case1And(A3, B1, B2, B3, N1, db1, db2, db3, contacts);
    }
    else if (da2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(A2, A3, A1, B1, B2, B3, contacts, N1, N2, da2, db1, db2, db3);
      // (+, -, -): Case 4
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(A1, A2, A3, B1, B2, B3, contacts, N1, N2, da1, db1, db2, db3);
      // (+, -, 0): Case 3
      else
        return case3And(A3, A1, A2, B1, B2, B3, contacts, N1, N2, da1, db1, db2, db3);
    }
    else
    {
      // (+, 0, +): Case 1
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1And(A2, B1, B2, B3, N1, db1, db2, db3, contacts);
      // (+, 0, -): Case 3
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3And(A2, A3, A1, B1, B2, B3, contacts, N1, N2, da3, db1, db2, db3);
      // (+, 0, 0): Case 2
      else
        return case2And(A2, A3, B1, B2, B3, contacts, N1, db1, db2, db3);
    }
  }
  else if (da1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (da2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(A1, A2, A3, B1, B2, B3, contacts, N1, N2, da1, db1, db2, db3);
      // (-, +, -): Case 4
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(A2, A3, A1, B1, B2, B3, contacts, N1, N2, da2, db1, db2, db3);
      // (-, +, 0): Case 3
      else
        return case3And(A3, A1, A2, B1, B2, B3, contacts, N1, N2, da1, db1, db2, db3);
    }
    else if (da2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(A3, A1, A2, B1, B2, B3, contacts, N1, N2, da3, db1, db2, db3);
      // (-, -, 0): Case 1
      else
        return case1And(A3, B1, B2, B3, N1, db1, db2, db3, contacts);
    }
    else
    {
      // (-, 0, +): Case 3
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3And(A2, A3, A1, B1, B2, B3, contacts, N1, N2, da3, db1, db2, db3);
      // (-, 0, -): Case 1
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1And(A2, B1, B2, B3, N1, db1, db2, db3, contacts);
      // (-, 0, 0): Case 2
      else
        return case2And(A2, A3, B1, B2, B3, contacts, N1, db1, db2, db3);
    }
  }
  else
  {
    if (da2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1And(A1, B1, B2, B3, N1, db1, db2, db3, contacts);
      // (0, +, -): Case 3
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3And(A1, A2, A3, B1, B2, B3, contacts, N1, N2, da2, db1, db2, db3);
      // (0, +, 0): Case 2
      else
        return case2And(A3, A1, B1, B2, B3, contacts, N1, db1, db2, db3);
    }
    else if (da2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3And(A1, A2, A3, B1, B2, B3, contacts, N1, N2, da2, db1, db2, db3);
      // (0, -, -): Case 1
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1And(A1, B1, B2, B3, N1, db1, db2, db3, contacts);
      // (0, -, 0): Case 2
      else
        return case2And(A3, A1, B1, B2, B3, contacts, N1, db1, db2, db3);
    }
    else
    {
      // (0, 0, +): Case 2
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2And(A1, A2, B1, B2, B3, contacts, N1, db1, db2, db3);
      // (0, 0, -): Case 2
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2And(A1, A2, B1, B2, B3, contacts, N1, db1, db2, db3);
      // (0, 0, 0): Coplanar case
      else
        //return coplanar3d(A1, A2, A3, B1, B2, B3, N1, N2, contacts, returnAllContacts);
        return coplanar3d(A1, A2, A3, B1, B2, B3, N1, contacts, returnAllContacts);
    }
  }
}

} // namespace v1

} // namespace collision
} // namespace dart
