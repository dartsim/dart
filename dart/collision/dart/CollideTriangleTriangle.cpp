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

inline int case1And(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    double db1, double db2, double db3,
    Eigen::Vector3d& contact1);

inline int case2And(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    const Eigen::Vector3d& n1,
    double db1, double db2, double db3);

inline int case3And(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    const Eigen::Vector3d& n1,
    const Eigen::Vector3d& n2,
    double da2,
    double db1, double db2, double db3);

inline int case4And(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    const Eigen::Vector3d& n1,
    const Eigen::Vector3d& n2,
    double da1,
    double db1, double db2, double db3);

inline int case1AndCase1(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    Eigen::Vector3d& contact1);

inline int case1AndCase2(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    Eigen::Vector3d& contact1);

inline int case1AndCase3(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    double db2,
    Eigen::Vector3d& contact1);

inline int case1AndCase4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    double db1,
    Eigen::Vector3d& contact1);

inline int case2AndCase2(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

inline int case2AndCase3(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    double db2,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

inline int case2AndCase4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    double db1,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

inline int case3AndCase3(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    const Eigen::Vector3d& n2,
    double da2,
    double db2,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

inline int case3AndCase4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    const Eigen::Vector3d& n2,
    double da2,
    double db1,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

inline int case4AndCase4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    const Eigen::Vector3d& n2,
    double da2,
    double db2,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

int collideTriangleTriangle(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    Eigen::Vector3d& /*contact3*/)
{
  Eigen::Vector3d v1 = b2 - b1;
  Eigen::Vector3d v2 = b3 - b2;
  Eigen::Vector3d n2 = v1.cross(v2);

  v1 = a1 - b3;
  const double da1 = v1.dot(n2);
  v1 = a2 - b3;
  const double da2 = v1.dot(n2);
  v1 = a3 - b3;
  const double da3 = v1.dot(n2);

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

  v1 = a2 - a1;
  v2 = a3 - a2;
  Eigen::Vector3d n1 = v1.cross(v2);

  v1 = b1 - a3;
  const double db1 = v1.dot(n1);
  v1 = b2 - a3;
  const double db2 = v1.dot(n1);
  v1 = b3 - a3;
  const double db3 = v1.dot(n1);

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
        return case4And(a3, a1, a2, b1, b2, b3, contact1, contact2, n1, n2, da3, db1, db2, db3);
      // (+, +, 0): Case 1
      else
        return case1And(a3, b1, b2, b3, n1, db1, db2, db3, contact1);
    }
    else if (da2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(a2, a3, a1, b1, b2, b3, contact1, contact2, n1, n2, da2, db1, db2, db3);
      // (+, -, -): Case 4
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(a1, a2, a3, b1, b2, b3, contact1, contact2, n1, n2, da1, db1, db2, db3);
      // (+, -, 0): Case 3
      else
        return case3And(a3, a1, a2, b1, b2, b3, contact1, contact2, n1, n2, da1, db1, db2, db3);
    }
    else
    {
      // (+, 0, +): Case 1
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1And(a2, b1, b2, b3, n1, db1, db2, db3, contact1);
      // (+, 0, -): Case 3
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3And(a2, a3, a1, b1, b2, b3, contact1, contact2, n1, n2, da3, db1, db2, db3);
      // (+, 0, 0): Case 2
      else
        return case2And(a2, a3, b1, b2, b3, contact1, contact2, n1, db1, db2, db3);
    }
  }
  else if (da1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (da2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(a1, a2, a3, b1, b2, b3, contact1, contact2, n1, n2, da1, db1, db2, db3);
      // (-, +, -): Case 4
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(a2, a3, a1, b1, b2, b3, contact1, contact2, n1, n2, da2, db1, db2, db3);
      // (-, +, 0): Case 3
      else
        return case3And(a3, a1, a2, b1, b2, b3, contact1, contact2, n1, n2, da1, db1, db2, db3);
    }
    else if (da2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4And(a3, a1, a2, b1, b2, b3, contact1, contact2, n1, n2, da3, db1, db2, db3);
      // (-, -, 0): Case 1
      else
        return case1And(a3, b1, b2, b3, n1, db1, db2, db3, contact1);
    }
    else
    {
      // (-, 0, +): Case 3
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3And(a2, a3, a1, b1, b2, b3, contact1, contact2, n1, n2, da3, db1, db2, db3);
      // (-, 0, -): Case 1
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1And(a2, b1, b2, b3, n1, db1, db2, db3, contact1);
      // (-, 0, 0): Case 2
      else
        return case2And(a2, a3, b1, b2, b3, contact1, contact2, n1, db1, db2, db3);
    }
  }
  else
  {
    if (da2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1And(a1, b1, b2, b3, n1, db1, db2, db3, contact1);
      // (0, +, -): Case 3
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3And(a1, a2, a3, b1, b2, b3, contact1, contact2, n1, n2, da2, db1, db2, db3);
      // (0, +, 0): Case 2
      else
        return case2And(a3, a1, b1, b2, b3, contact1, contact2, n1, db1, db2, db3);
    }
    else if (da2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3And(a1, a2, a3, b1, b2, b3, contact1, contact2, n1, n2, da2, db1, db2, db3);
      // (0, -, -): Case 1
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1And(a1, b1, b2, b3, n1, db1, db2, db3, contact1);
      // (0, -, 0): Case 2
      else
        return case2And(a3, a1, b1, b2, b3, contact1, contact2, n1, db1, db2, db3);
    }
    else
    {
      // (0, 0, +): Case 2
      if (da3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2And(a1, a2, b1, b2, b3, contact1, contact2, n1, db1, db2, db3);
      // (0, 0, -): Case 2
      else if (da3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2And(a1, a2, b1, b2, b3, contact1, contact2, n1, db1, db2, db3);
      // (0, 0, 0): Coplanar case
      else
        return false;
    }
  }
}

//==============================================================================
inline int case1And(const Eigen::Vector3d& a1,
             const Eigen::Vector3d& b1,
             const Eigen::Vector3d& b2,
             const Eigen::Vector3d& b3,
             const Eigen::Vector3d& n1,
             double db1, double db2, double db3,
             Eigen::Vector3d& contact1)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(a1, b3, b1, b2, n1, db3, contact1);
      // (+, +, 0): Case 1
      else
        return case1AndCase1(a1, b3, contact1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(a1, b2, b3, b1, n1, db2, contact1);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(a1, b1, b2, b3, n1, db1, contact1);
      // (+, -, 0): Case 3
      else
        return case1AndCase3(a1, b3, b1, b2, n1, db1, contact1);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(a1, b2, contact1);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(a1, b2, b3, b1, n1, db3, contact1);
      // (+, 0, 0): Case 2
      else
        return case1AndCase2(a1, b2, b3, contact1);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(a1, b1, b2, b3, n1, db1, contact1);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(a1, b2, b3, b1, n1, db2, contact1);
      // (-, +, 0): Case 3
      else
        return case1AndCase3(a1, b3, b1, b2, n1, db1, contact1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(a1, b3, b1, b2, n1, db3, contact1);
      // (-, -, 0): Case 1
      else
        return case1AndCase1(a1, b3, contact1);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(a1, b2, b3, b1, n1, db3, contact1);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(a1, b2, contact1);
      // (-, 0, 0): Case 2
      else
        return case1AndCase2(a1, b2, b3, contact1);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(a1, b1, contact1);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(a1, b1, b2, b3, n1, db2, contact1);
      // (0, +, 0): Case 2
      else
        return case1AndCase2(a1, b3, b1, contact1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(a1, b1, b2, b3, n1, db2, contact1);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase1(a1, b1, contact1);
      // (0, -, 0): Case 2
      else
        return case1AndCase2(a1, b3, b1, contact1);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(a1, b1, b2, contact1);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(a1, b1, b2, contact1);
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
             Eigen::Vector3d& contact1,
             Eigen::Vector3d& contact2,
             const Eigen::Vector3d& n1,
             double db1, double db2, double db3)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(a1, a2, b3, b1, b2, n1, db3, contact1, contact2);
      // (+, +, 0): Case 1
      else
        return case1AndCase2(b3, a1, a2, contact1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(a1, a2, b2, b3, b1, n1, db2, contact1, contact2);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(a1, a2, b1, b2, b3, n1, db1, contact1, contact2);
      // (+, -, 0): Case 3
      else
        return case2AndCase3(a1, a2, b3, b1, b2, n1, db1, contact1, contact2);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(b2, a1, a2, contact1);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(a1, a2, b2, b3, b1, n1, db3, contact1, contact2);
      // (+, 0, 0): Case 2
      else
        return case2AndCase2(a1, a2, b2, b3, contact1, contact2);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(a1, a2, b1, b2, b3, n1, db1, contact1, contact2);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(a1, a2, b2, b3, b1, n1, db2, contact1, contact2);
      // (-, +, 0): Case 3
      else
        return case2AndCase3(a1, a2, b3, b1, b2, n1, db1, contact1, contact2);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(a1, a2, b3, b1, b2, n1, db3, contact1, contact2);
      // (-, -, 0): Case 1
      else
        return case1AndCase2(b3, a1, a2, contact1);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(a1, a2, b2, b3, b1, n1, db3, contact1, contact2);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(b2, a1, a2, contact1);
      // (-, 0, 0): Case 2
      else
        return case2AndCase2(a1, a2, b2, b3, contact1, contact2);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(b1, a1, a2, contact1);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(a1, a2, b1, b2, b3, n1, db2, contact1, contact2);
      // (0, +, 0): Case 2
      else
        return case2AndCase2(a1, a2, b3, b1, contact1, contact2);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(a1, a2, b1, b2, b3, n1, db2, contact1, contact2);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase2(b1, a1, a2, contact1);
      // (0, -, 0): Case 2
      else
        return case2AndCase2(a1, a2, b3, b1, contact1, contact2);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase2(a1, a2, b1, b2, contact1, contact2);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase2(a1, a2, b1, b2, contact1, contact2);
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
             Eigen::Vector3d& contact1,
             Eigen::Vector3d& contact2,
             const Eigen::Vector3d& n1,
             const Eigen::Vector3d& n2,
             double da2,
             double db1, double db2, double db3)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(a1, a2, a3, b3, b1, b2, n1, n2, da2, db3, contact1, contact2);
      // (+, +, 0): Case 1
      else
        return case1AndCase3(b3, a1, a2, a3, n2, da2, contact1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(a1, a2, a3, b2, b3, b1, n1, n2, da2, db2, contact1, contact2);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(a1, a2, a3, b1, b2, b3, n1, n2, da2, db1, contact1, contact2);
      // (+, -, 0): Case 3
      else
        return case3AndCase3(a1, a2, a3, b3, b1, b2, n2, n1, da2, db1, contact1, contact2);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(b2, a1, a2, a3, n2, da2, contact1);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(a1, a2, a3, b2, b3, b1, n2, n1, da2, db3, contact1, contact2);
      // (+, 0, 0): Case 2
      else
        return case2AndCase3(b2, b3, a1, a2, a3, n2, da2, contact1, contact2);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(a1, a2, a3, b1, b2, b3, n1, n2, da2, db1, contact1, contact2);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(a1, a2, a3, b2, b3, b1, n1, n2, da2, db2, contact1, contact2);
      // (-, +, 0): Case 3
      else
        return case3AndCase3(a1, a2, a3, b3, b1, b2, n2, n1, da2, db1, contact1, contact2);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(a1, a2, a3, b3, b1, b2, n1, n2, da2, db3, contact1, contact2);
      // (-, -, 0): Case 1
      else
        return case1AndCase3(b3, a1, a2, a3, n2, da2, contact1);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(a1, a2, a3, b2, b3, b1, n2, n1, da2, db3, contact1, contact2);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(b2, a1, a2, a3, n2, da2, contact1);
      // (-, 0, 0): Case 2
      else
        return case2AndCase3(b2, b3, a1, a2, a3, n2, da2, contact1, contact2);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(b1, a1, a2, a3, n2, da2, contact1);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(a1, a2, a3, b1, b2, b3, n2, n1, da2, db2, contact1, contact2);
      // (0, +, 0): Case 2
      else
        return case2AndCase3(b3, b1, a1, a2, a3, n2, da2, contact1, contact2);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase3(a1, a2, a3, b1, b2, b3, n2, n1, da2, db2, contact1, contact2);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase3(b1, a1, a2, a3, n2, da2, contact1);
      // (0, -, 0): Case 2
      else
        return case2AndCase3(b3, b1, a1, a2, a3, n2, da2, contact1, contact2);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(b1, b2, a1, a2, a3, n2, da2, contact1, contact2);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase3(b1, b2, a1, a2, a3, n2, da2, contact1, contact2);
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
             Eigen::Vector3d& contact1,
             Eigen::Vector3d& contact2,
             const Eigen::Vector3d& n1,
             const Eigen::Vector3d& n2,
             double da1,
             double db1, double db2, double db3)
{
  if (db1 > DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, +, -): Case 4
      if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(a1, a2, a3, b3, b1, b2, n1, n2, da1, db3, contact1, contact2);
      // (+, +, 0): Case 1
      else
        return case1AndCase4(b3, a1, a2, a3, n2, da1, contact1);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (+, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(a1, a2, a3, b2, b3, b1, n1, n2, da1, db2, contact1, contact2);
      // (+, -, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(a1, a2, a3, b1, b2, b3, n1, n2, da1, db1, contact1, contact2);
      // (+, -, 0): Case 3
      else
        return case3AndCase4(b3, b1, b2, a1, a2, a3, n2, n1, db1, da1, contact1, contact2);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(b2, a1, a2, a3, n2, da1, contact1);
      // (+, 0, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(b2, b3, b1, a1, a2, a3, n2, n1, db3, da1, contact1, contact2);
      // (+, 0, 0): Case 2
      else
        return case2AndCase4(b2, b3, a1, a2, a3, n2, da1, contact1, contact2);
    }
  }
  else if (db1 < -DART_TRIANGLE_TRIANGLE_EPS)
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, +, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(a1, a2, a3, b1, b2, b3, n1, n2, da1, db1, contact1, contact2);
      // (-, +, -): Case 4
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(a1, a2, a3, b2, b3, b1, n1, n2, da1, db2, contact1, contact2);
      // (-, +, 0): Case 3
      else
        return case3AndCase4(b3, b1, b2, a1, a2, a3, n2, n1, db1, da1, contact1, contact2);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (-, -, +): Case 4
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case4AndCase4(a1, a2, a3, b3, b1, b2, n1, n2, da1, db3, contact1, contact2);
      // (-, -, 0): Case 1
      else
        return case1AndCase4(b3, a1, a2, a3, n2, da1, contact1);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(b2, b3, b1, a1, a2, a3, n2, n1, db3, da1, contact1, contact2);
      // (-, 0, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(b2, a1, a2, a3, n2, da1, contact1);
      // (-, 0, 0): Case 2
      else
        return case2AndCase4(b2, b3, a1, a2, a3, n2, da1, contact1, contact2);
    }
  }
  else
  {
    if (db2 > DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, +, +): Case 1
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(b1, a1, a2, a3, n2, da1, contact1);
      // (0, +, -): Case 3
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(b1, b2, b3, a1, a2, a3, n2, n1, db2, da1, contact1, contact2);
      // (0, +, 0): Case 2
      else
        return case2AndCase4(b3, b1, a1, a2, a3, n2, da1, contact1, contact2);
    }
    else if (db2 < -DART_TRIANGLE_TRIANGLE_EPS)
    {
      // (0, -, +): Case 3
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case3AndCase4(b1, b2, b3, a1, a2, a3, n2, n1, db2, da1, contact1, contact2);
      // (0, -, -): Case 1
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case1AndCase4(b1, a1, a2, a3, n2, da1, contact1);
      // (0, -, 0): Case 2
      else
        return case2AndCase4(b3, b1, a1, a2, a3, n2, da1, contact1, contact2);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(b1, b2, a1, a2, a3, n2, da1, contact1, contact2);
      // (0, 0, -): Case 2
      else if (db3 < -DART_TRIANGLE_TRIANGLE_EPS)
        return case2AndCase4(b1, b2, a1, a2, a3, n2, da1, contact1, contact2);
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
inline int checkPointAndLine(const Eigen::Vector3d& a1,
                      const Eigen::Vector3d& b1,
                      const Eigen::Vector3d& b2,
                      Eigen::Vector3d& contact1)
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

  contact1 = a1;

  return 1;
}

//==============================================================================
inline int checkLineAndLine(const Eigen::Vector3d& aMin,
                            const Eigen::Vector3d& aMax,
                            const Eigen::Vector3d& bMin,
                            const Eigen::Vector3d& bMax,
                            const double daMin,
                            const double daMax,
                            const double dbMin,
                            const double dbMax,
                            Eigen::Vector3d& contact1,
                            Eigen::Vector3d& contact2)
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
        contact1 = aMin;  // or bMax or 0.5*(aMin+bMax)
        return 1;
      }

      contact1 = aMin;
      contact2 = bMax;
      return 2;
    }
    else
    {
      // collision: (aMin, aMax)
      contact1 = aMin;
      contact2 = aMax;
      return 2;
    }
  }
  else if (dbMin < daMax)
  {
    if (dbMax < daMax)
    {
      // collision: (bMin, bMax)
      contact1 = bMin;
      contact2 = bMax;
      return 2;
    }
    else
    {
      // collision: (bMin, aMax)

      if (daMax - dbMin < DART_TRIANGLE_TRIANGLE_EPS)
      {
        contact1 = aMax; // or bMin or 0.5*(aMax+bMin)
        return 1;
      }

      contact1 = bMin;
      contact2 = aMax;
      return 2;
    }
  }
  else
  {
    return 0;
  }
}

//==============================================================================
inline int checkLineAndLine(const Eigen::Vector3d& a1,
                            const Eigen::Vector3d& a2,
                            const Eigen::Vector3d& b1,
                            const Eigen::Vector3d& b2,
                            Eigen::Vector3d& contact1,
                            Eigen::Vector3d& contact2)
{
  const Eigen::Vector3d a21 = a2 - a1;

  const auto da1 = a21.dot(a1);
  const auto da2 = a21.dot(a2);
  const auto db1 = a21.dot(b1);
  const auto db2 = a21.dot(b2);

  if (da1 < da2)
  {
    if (db1 < db2)
      return checkLineAndLine(a1, a2, b1, b2, da1, da2, db1, db2, contact1, contact2);
    else
      return checkLineAndLine(a1, a2, b2, b1, da1, da2, db2, db1, contact1, contact2);
  }
  else
  {
    if (db1 < db2)
      return checkLineAndLine(a2, a1, b1, b2, da2, da1, db1, db2, contact1, contact2);
    else
      return checkLineAndLine(a2, a1, b2, b1, da2, da1, db2, db1, contact1, contact2);
  }
}

//==============================================================================
inline void computePointOnPlane(const Eigen::Vector3d& b2,
                                const Eigen::Vector3d& b3,
                                const Eigen::Vector3d& n1,
                                double db2,
                                Eigen::Vector3d& b32OnPlaneA)
{
  const Eigen::Vector3d b32 = b3 - b2;
  b32OnPlaneA.noalias() = b2;
  b32OnPlaneA.noalias() -= (db2 / n1.dot(b32)) * b32;
}

//==============================================================================
inline int case1AndCase1(const Eigen::Vector3d& a1,
                         const Eigen::Vector3d& b1,
                         Eigen::Vector3d& contact1)
{
  if (a1 == b1) // TODO(JS): ((a1 - b1).squaredNorm() < eps*eps) ?
  {
    contact1 = a1;
    return 1;
  }

  return 0;
}

//==============================================================================
inline int case1AndCase2(const Eigen::Vector3d& a1,
                         const Eigen::Vector3d& b1,
                         const Eigen::Vector3d& b2,
                         Eigen::Vector3d& contact1)
{
  return checkPointAndLine(a1, b1, b2, contact1);
}

//==============================================================================
inline int case1AndCase3(const Eigen::Vector3d& a1,
                  const Eigen::Vector3d& b1,
                  const Eigen::Vector3d& b2,
                  const Eigen::Vector3d& b3,
                  const Eigen::Vector3d& n1,
                  double db2,
                  Eigen::Vector3d& contact1)
{
  Eigen::Vector3d b32OnPlaneA;
  computePointOnPlane(b2, b3, n1, db2, b32OnPlaneA);

  return checkPointAndLine(a1, b1, b32OnPlaneA, contact1);
}

//==============================================================================
inline int case1AndCase4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    double db1,
    Eigen::Vector3d& contact1)
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

  return checkPointAndLine(a1, b21OnPlaneA, b31OnPlaneA, contact1);
}

//==============================================================================
inline int case2AndCase2(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2)
{
  return checkLineAndLine(a1, a2, b1, b2, contact1, contact2);
}

//==============================================================================
inline int case2AndCase3(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    double db2,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2)
{
  Eigen::Vector3d b32OnPlaneA;
  computePointOnPlane(b2, b3, n1, db2, b32OnPlaneA);

  return checkLineAndLine(a1, a2, b1, b32OnPlaneA, contact1, contact2);
}

//==============================================================================
inline int case2AndCase4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    double db1,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2)
{
  Eigen::Vector3d b21OnPlaneA;
  Eigen::Vector3d b31OnPlaneA;
  computePointOnPlane(b1, b2, n1, db1, b21OnPlaneA);
  computePointOnPlane(b1, b3, n1, db1, b31OnPlaneA);

  return checkLineAndLine(a1, a2, b21OnPlaneA, b31OnPlaneA, contact1, contact2);
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
    double da2, double db2,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2)
{
  Eigen::Vector3d a32OnPlaneB;
  computePointOnPlane(a2, a3, n2, da2, a32OnPlaneB);

  Eigen::Vector3d b32OnPlaneA;
  computePointOnPlane(b2, b3, n1, db2, b32OnPlaneA);

  return checkLineAndLine(a1, a32OnPlaneB, b1, b32OnPlaneA, contact1, contact2);
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
    double da2,
    double db1,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2)
{
  Eigen::Vector3d a32OnPlaneB;
  computePointOnPlane(a2, a3, n2, da2, a32OnPlaneB);

  Eigen::Vector3d b21OnPlaneA;
  Eigen::Vector3d b31OnPlaneA;
  computePointOnPlane(b1, b2, n1, db1, b21OnPlaneA);
  computePointOnPlane(b1, b3, n1, db1, b31OnPlaneA);

  return checkLineAndLine(a1, a32OnPlaneB, b21OnPlaneA, b31OnPlaneA, contact1, contact2);
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
    double da1,
    double db1,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2)
{
  Eigen::Vector3d a21OnPlaneB;
  Eigen::Vector3d a31OnPlaneB;
  computePointOnPlane(a1, a2, n2, da1, a21OnPlaneB);
  computePointOnPlane(a1, a3, n2, da1, a31OnPlaneB);

  Eigen::Vector3d b21OnPlaneA;
  Eigen::Vector3d b31OnPlaneA;
  computePointOnPlane(b1, b2, n1, db1, b21OnPlaneA);
  computePointOnPlane(b1, b3, n1, db1, b31OnPlaneA);

  return checkLineAndLine(
        a21OnPlaneB, a31OnPlaneB, b21OnPlaneA, b31OnPlaneA, contact1, contact2);
}

} // namespace v1

} // namespace collision
} // namespace dart
