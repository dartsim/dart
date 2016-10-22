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

namespace v1 {

//==============================================================================
int collideTriangleTriangle(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    Eigen::Vector3d& contact3)
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

  if ((da1 > 0.0 && da2 > 0.0 && da3 > 0.0)
      || (da1 < 0.0 && da2 < 0.0 && da3 < 0.0))
  {
    return false;
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

  if ((db1 > 0.0 && db2 > 0.0 && db3 > 0.0)
      || (db1 < 0.0 && db2 < 0.0 && db3 < 0.0))
  {
    return false;
  }

  // dot(6), cross(2)

  if (da1 > 0.0)
  {
    if (da2 > 0.0)
    {
      // (+, +, -): Case 4
      if (da3 < 0.0)
        return probablyCase4(a3, a1, a2, b1, b2, b3, contact1, contact2, db1, db2, db3);
      // (+, +, 0): Case 1
      else
        return case1(a3, b1, b2, b3, contact1);
    }
    else if (da2 < 0.0)
    {
      // (+, -, +): Case 4
      if (da3 > 0.0)
        return probablyCase4(a2, a3, a1, b1, b2, b3, contact1, contact2, db1, db2, db3);
      // (+, -, -): Case 4
      else if (da3 < 0.0)
        return probablyCase4(a1, a2, a3, b1, b2, b3, contact1, contact2, db1, db2, db3);
      // (+, -, 0): Case 3
      else
        return probablyCase3(a3, a1, a2, b1, b2, b3, contact1, contact2, db1, db2, db3);
    }
    else
    {
      // (+, 0, +): Case 1
      if (da3 > 0.0)
        return case1(a2, b1, b2, b3, contact1);
      // (+, 0, -): Case 3
      else if (da3 < 0.0)
        return probablyCase3(a2, a3, a1, b1, b2, b3, contact1, contact2, db1, db2, db3);
      // (+, 0, 0): Case 2
      else
        return probablyCase2(a2, a3, a1, b1, b2, b3, contact1, contact2, db1, db2, db3);
    }
  }
  else if (da1 < 0.0)
  {
    if (da2 > 0.0)
    {
      // (-, +, +): Case 4
      if (da3 > 0.0)
        return probablyCase4(a1, a2, a3, b1, b2, b3, contact1, contact2, db1, db2, db3);
      // (-, +, -): Case 4
      else if (da3 < 0.0)
        return probablyCase4(a2, a3, a1, b1, b2, b3, contact1, contact2, db1, db2, db3);
      // (-, +, 0): Case 3
      else
        return probablyCase3(a3, a1, a2, b1, b2, b3, contact1, contact2, db1, db2, db3);
    }
    else if (da2 < 0.0)
    {
      // (-, -, +): Case 4
      if (da3 > 0.0)
        return probablyCase4(a3, a1, a2, b1, b2, b3, contact1, contact2, db1, db2, db3);
      // (-, -, 0): Case 1
      else
        return case1(a3, b1, b2, b3, contact1);
    }
    else
    {
      // (-, 0, +): Case 3
      if (da3 > 0.0)
        return probablyCase3(a2, a3, a1, b1, b2, b3, contact1, contact2, db1, db2, db3);
      // (-, 0, -): Case 1
      else if (da3 < 0.0)
        return case1(a2, b1, b2, b3, contact1);
      // (-, 0, 0): Case 2
      else
        return probablyCase2(a2, a3, a1, b1, b2, b3, contact1, contact2, db1, db2, db3);
    }
  }
  else
  {
    if (da2 > 0.0)
    {
      // (0, +, +): Case 1
      if (da3 > 0.0)
        return case1(a1, b1, b2, b3, contact1);
      // (0, +, -): Case 3
      else if (da3 < 0.0)
        return probablyCase3(a1, a2, a3, b1, b2, b3, contact1, contact2, db1, db2, db3);
      // (0, +, 0): Case 2
      else
        return probablyCase2(a3, a1, a2, b1, b2, b3, contact1, contact2, db1, db2, db3);
    }
    else if (da2 < 0.0)
    {
      // (0, -, +): Case 3
      if (da3 > 0.0)
        return probablyCase3(a1, a2, a3, b1, b2, b3, contact1, contact2, db1, db2, db3);
      // (0, -, -): Case 1
      else if (da3 < 0.0)
        return case1(a1, b1, b2, b3, contact1);
      // (0, -, 0): Case 2
      else
        return probablyCase2(a3, a1, a2, b1, b2, b3, contact1, contact2, db1, db2, db3);
    }
    else
    {
      // (0, 0, +): Case 2
      if (da3 > 0.0)
        return probablyCase2(a1, a2, a3, b1, b2, b3, contact1, contact2, db1, db2, db3);
      // (0, 0, -): Case 2
      else if (da3 < 0.0)
        return probablyCase2(a1, a2, a3, b1, b2, b3, contact1, contact2, db1, db2, db3);
      // (0, 0, 0): Coplanar case
      else
        return false;
    }
  }

  return 0;
}

} // namespace v1

namespace v2 {

int collideTriangleTriangle(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    Eigen::Vector3d& contact3)
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

  if ((da1 > 0.0 && da2 > 0.0 && da3 > 0.0)
      || (da1 < 0.0 && da2 < 0.0 && da3 < 0.0))
  {
    return false;
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

  if ((db1 > 0.0 && db2 > 0.0 && db3 > 0.0)
      || (db1 < 0.0 && db2 < 0.0 && db3 < 0.0))
  {
    return false;
  }

  // dot(6), cross(2)

  if (da1 > 0.0)
  {
    if (da2 > 0.0)
    {
      // (+, +, -): Case 4
      if (da3 < 0.0)
        return false;
      // (+, +, 0): Case 1
      else
        return case1And(a3, b1, b2, b3, n1, db1, db2, db3, contact1);
    }
    else if (da2 < 0.0)
    {
      // (+, -, +): Case 4
      if (da3 > 0.0)
        return false;
      // (+, -, -): Case 4
      else if (da3 < 0.0)
        return false;
      // (+, -, 0): Case 3
      else
        return false;
    }
    else
    {
      // (+, 0, +): Case 1
      if (da3 > 0.0)
        return case1And(a2, b1, b2, b3, n1, db1, db2, db3, contact1);
      // (+, 0, -): Case 3
      else if (da3 < 0.0)
        return false;
      // (+, 0, 0): Case 2
      else
        return false;
    }
  }
  else if (da1 < 0.0)
  {
    if (da2 > 0.0)
    {
      // (-, +, +): Case 4
      if (da3 > 0.0)
        return false;
      // (-, +, -): Case 4
      else if (da3 < 0.0)
        return false;
      // (-, +, 0): Case 3
      else
        return false;
    }
    else if (da2 < 0.0)
    {
      // (-, -, +): Case 4
      if (da3 > 0.0)
        return false;
      // (-, -, 0): Case 1
      else
        return case1(a3, b1, b2, b3, contact1);
    }
    else
    {
      // (-, 0, +): Case 3
      if (da3 > 0.0)
        return false;
      // (-, 0, -): Case 1
      else if (da3 < 0.0)
        return case1(a2, b1, b2, b3, contact1);
      // (-, 0, 0): Case 2
      else
        return false;
    }
  }
  else
  {
    if (da2 > 0.0)
    {
      // (0, +, +): Case 1
      if (da3 > 0.0)
        return case1(a1, b1, b2, b3, contact1);
      // (0, +, -): Case 3
      else if (da3 < 0.0)
        return false;
      // (0, +, 0): Case 2
      else
        return false;
    }
    else if (da2 < 0.0)
    {
      // (0, -, +): Case 3
      if (da3 > 0.0)
        return false;
      // (0, -, -): Case 1
      else if (da3 < 0.0)
        return case1(a1, b1, b2, b3, contact1);
      // (0, -, 0): Case 2
      else
        return false;
    }
    else
    {
      // (0, 0, +): Case 2
      if (da3 > 0.0)
        return false;
      // (0, 0, -): Case 2
      else if (da3 < 0.0)
        return false;
      // (0, 0, 0): Coplanar case
      else
        return false;
    }
  }

  return 0;
}

} // namespace v2

//==============================================================================
inline int case1(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1)
{
  return test2dPointInTriangle(a1, b1, b2, b3, contact1);
}

//==============================================================================
inline int case2(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2)
{
  return test2dLineSegmentTriangle(a1, a2, b1, b2, b3, contact1, contact2);
}

//==============================================================================
inline int case3(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2)
{
  // TODO(JS): Compute point on the plane of B of the line segment of a2-a3



  return test2dLineSegmentTriangle(a1, a2, b1, b2, b3, contact1, contact2);
}

//==============================================================================
inline int case4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2)
{
  // TODO(JS): Compute points on the plane of B of the line segment of a1-a2 and
  // a1-a3
  return test2dLineSegmentTriangle(a1, a2, b1, b2, b3, contact1, contact2);
}

//==============================================================================
int probablyCase2(const Eigen::Vector3d& a1,
          const Eigen::Vector3d& a2,
          const Eigen::Vector3d& a3,
          const Eigen::Vector3d& b1,
          const Eigen::Vector3d& b2,
          const Eigen::Vector3d& b3,
          Eigen::Vector3d& contact1,
          Eigen::Vector3d& contact2,
          double db1, double db2, double db3)
{
  if (db1 > 0.0)
  {
    if (db2 > 0.0)
    {
      // (+, +, 0): Case 1
      if (db3 == 0.0)
        return case1(b3, a1, a2, a3, contact1);
    }
    else if (db2 == 0.0)
    {
      // (+, 0, +): Case 1
      if (db3 > 0.0)
        return case1(b2, a1, a2, a3, contact1);
    }
  }
  else if (db1 < 0.0)
  {
    if (db2 < 0.0)
    {
      // (-, -, 0): Case 1
      if (db3 == 0.0)
        return case1(b3, a1, a2, a3, contact1);
    }
    else if (db2 == 0.0)
    {
      // (-, 0, -): Case 1
      if (db3 < 0.0)
        return case1(b2, a1, a2, a3, contact1);
    }
  }
  else
  {
    if (db2 > 0.0)
    {
      // (0, +, +): Case 1
      if (db3 > 0.0)
        return case1(b1, a1, a2, a3, contact1);
    }
    else if (db2 < 0.0)
    {
      // (0, -, -): Case 1
      if (db3 < 0.0)
        return case1(b1, a1, a2, a3, contact1);
    }
  }

  return case2(a1, a2, b1, b2, b3, contact1, contact2);
}

//==============================================================================
int probablyCase3(const Eigen::Vector3d& a1,
          const Eigen::Vector3d& a2,
          const Eigen::Vector3d& a3,
          const Eigen::Vector3d& b1,
          const Eigen::Vector3d& b2,
          const Eigen::Vector3d& b3,
          Eigen::Vector3d& contact1,
          Eigen::Vector3d& contact2,
          double db1, double db2, double db3)
{
  if (db1 > 0.0)
  {
    if (db2 > 0.0)
    {
      // (+, +, 0): Case 1
      if (db3 == 0.0)
        return case1(b3, a1, a2, a3, contact1);
    }
    else if (db3 == 0.0)
    {
      // (+, 0, +): Case 1
      if (db3 > 0.0)
        return case1(b2, a1, a2, a3, contact1);
      // (+, 0, 0): Case 2
      else if (db3 == 0.0)
        return case2(b2, b3, a1, a2, a3, contact1, contact2);
    }
  }
  else if (db1 < 0.0)
  {
    if (db2 < 0.0)
    {
      // (-, -, 0): Case 1
      if (db3 == 0.0)
        return case1(b3, a1, a2, a3, contact1);
    }
    else if (db2 == 0.0)
    {
      // (-, 0, -): Case 1
      if (db3 < 0.0)
        return case1(b2, a1, a2, a3, contact1);
      // (-, 0, 0): Case 2
      else if (db3 == 0.0)
        return case2(b2, b3, a1, a2, a3, contact1, contact2);
    }
  }
  else
  {
    if (db2 > 0.0)
    {
      // (0, +, +): Case 1
      if (db3 > 0.0)
        return case1(b1, a1, a2, a3, contact1);
      // (0, +, 0): Case 2
      else if (db3 == 0.0)
        return case2(b3, b1, a1, a2, a3, contact1, contact2);
    }
    else if (db2 < 0.0)
    {
      // (0, -, -): Case 1
      if (db3 < 0.0)
        return case1(b1, a1, a2, a3, contact1);
      // (0, -, 0): Case 2
      else if (db3 == 0.0)
        return case2(b3, b1, a1, a2, a3, contact1, contact2);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > 0.0)
        return case2(b1, b2, a1, a2, a3, contact1, contact2);
      // (0, 0, -): Case 2
      else if (db3 < 0.0)
        return case2(b1, b2, a1, a2, a3, contact1, contact2);
    }
  }

  return case3(a1, a2, a3, b1, b2, b3, contact1, contact2);
}

//==============================================================================
int probablyCase4(const Eigen::Vector3d& a1,
          const Eigen::Vector3d& a2,
          const Eigen::Vector3d& a3,
          const Eigen::Vector3d& b1,
          const Eigen::Vector3d& b2,
          const Eigen::Vector3d& b3,
          Eigen::Vector3d& contact1,
          Eigen::Vector3d& contact2,
          double db1, double db2, double db3)
{
  if (db1 > 0.0)
  {
    if (db2 > 0.0)
    {
      // (+, +, 0): Case 1
      if (db3 == 0.0)
        return case1(b3, a1, a2, a3, contact1);
    }
    else if (db2 < 0.0)
    {
      // (+, -, 0): Case 3
      if (db3 == 0.0)
        return case3(b3, b1, b2, a1, a2, a3, contact1, contact2);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > 0.0)
        return case1(b2, a1, a2, a3, contact1);
      // (+, 0, -): Case 3
      else if (db3 < 0.0)
        return case3(b2, b3, b1, a1, a2, a3, contact1, contact2);
      // (+, 0, 0): Case 2
      else
        return case2(b2, b3, a1, a2, a3, contact1, contact2);
    }
  }
  else if (db1 < 0.0)
  {
    if (db2 > 0.0)
    {
      // (-, +, 0): Case 3
      if (db3 == 0.0)
        return case3(b3, b1, b2, a1, a2, a3, contact1, contact2);
    }
    else if (db2 < 0.0)
    {
      // (-, -, 0): Case 1
      if (db3 == 0.0)
        return case1(b3, a1, a2, a3, contact1);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > 0.0)
        return case3(b2, b3, b1, a1, a2, a3, contact1, contact2);
      // (-, 0, -): Case 1
      else if (db3 < 0.0)
        return case1(b2, a1, a2, a3, contact1);
      // (-, 0, 0): Case 2
      else
        return case2(b2, b3, a1, a2, a3, contact1, contact2);
    }
  }
  else
  {
    if (db2 > 0.0)
    {
      // (0, +, +): Case 1
      if (db3 > 0.0)
        return case1(b1, a1, a2, a3, contact1);
      // (0, +, -): Case 3
      else if (db3 < 0.0)
        return case3(b1, b2, b3, a1, a2, a3, contact1, contact2);
      // (0, +, 0): Case 2
      else
        return case2(b3, b1, a1, a2, a3, contact1, contact2);
    }
    else if (db2 < 0.0)
    {
      // (0, -, +): Case 3
      if (db3 > 0.0)
        return case3(b1, b2, b3, a1, a2, a3, contact1, contact2);
      // (0, -, -): Case 1
      else if (db3 < 0.0)
        return case1(b1, a1, a2, a3, contact1);
      // (0, -, 0): Case 2
      else
        return case2(b3, b1, a1, a2, a3, contact1, contact2);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > 0.0)
        return case2(b1, b2, a1, a2, a3, contact1, contact2);
      // (0, 0, -): Case 2
      else if (db3 < 0.0)
        return case2(b1, b2, a1, a2, a3, contact1, contact2);
    }
  }

  return case4(a1, a2, a3, b1, b2, b3, contact1, contact2);
}

//==============================================================================
int case1And(const Eigen::Vector3d& a1,
             const Eigen::Vector3d& b1,
             const Eigen::Vector3d& b2,
             const Eigen::Vector3d& b3,
             const Eigen::Vector3d& n1,
             double db1, double db2, double db3,
             Eigen::Vector3d& contact1)
{
  if (db1 > 0.0)
  {
    if (db2 > 0.0)
    {
      // (+, +, -): Case 4
      if (db3 < 0.0)
        return case1AndCase4(a1, b3, b1, b2, n1, db3, contact1);
      // (+, +, 0): Case 1
      else
        return case1AndCase1(a1, b3, contact1);
    }
    else if (db2 < 0.0)
    {
      // (+, -, +): Case 4
      if (db3 > 0.0)
        return case1AndCase4(a1, b2, b3, b1, n1, db2, contact1);
      // (+, -, -): Case 4
      else if (db3 < 0.0)
        return case1AndCase4(a1, b1, b2, b3, n1, db1, contact1);
      // (+, -, 0): Case 3
      else
        return case1AndCase3(a1, b3, b1, b2, n1, db1, contact1);
    }
    else
    {
      // (+, 0, +): Case 1
      if (db3 > 0.0)
        return case1AndCase1(a1, b2, contact1);
      // (+, 0, -): Case 3
      else if (db3 < 0.0)
        return case1AndCase3(a1, b2, b3, b1, n1, db3, contact1);
      // (+, 0, 0): Case 2
      else
        return case1AndCase2(a1, b2, b3, contact1);
    }
  }
  else if (db1 < 0.0)
  {
    if (db2 > 0.0)
    {
      // (-, +, +): Case 4
      if (db3 > 0.0)
        return case1AndCase4(a1, b1, b2, b3, n1, db1, contact1);
      // (-, +, -): Case 4
      else if (db3 < 0.0)
        return case1AndCase4(a1, b2, b3, b1, n1, db2, contact1);
      // (-, +, 0): Case 3
      else
        return case1AndCase3(a1, b3, b1, b2, n1, db1, contact1);
    }
    else if (db2 < 0.0)
    {
      // (-, -, +): Case 4
      if (db3 > 0.0)
        return case1AndCase4(a1, b3, b1, b2, n1, db3, contact1);
      // (-, -, 0): Case 1
      else
        return case1AndCase1(a1, b3, contact1);
    }
    else
    {
      // (-, 0, +): Case 3
      if (db3 > 0.0)
        return case1AndCase3(a1, b2, b3, b1, n1, db3, contact1);
      // (-, 0, -): Case 1
      else if (db3 < 0.0)
        return case1AndCase1(a1, b2, contact1);
      // (-, 0, 0): Case 2
      else
        return case1AndCase2(a1, b2, b3, contact1);
    }
  }
  else
  {
    if (db2 > 0.0)
    {
      // (0, +, +): Case 1
      if (db3 > 0.0)
        return case1AndCase1(a1, b1, contact1);
      // (0, +, -): Case 3
      else if (db3 < 0.0)
        return case1AndCase3(a1, b1, b2, b3, n1, db2, contact1);
      // (0, +, 0): Case 2
      else
        return case1AndCase2(a1, b3, b1, contact1);
    }
    else if (db2 < 0.0)
    {
      // (0, -, +): Case 3
      if (db3 > 0.0)
        return case1AndCase3(a1, b1, b2, b3, n1, db2, contact1);
      // (0, -, -): Case 1
      else if (db3 < 0.0)
        return case1AndCase1(a1, b1, contact1);
      // (0, -, 0): Case 2
      else
        return case1AndCase2(a1, b3, b1, contact1);
    }
    else
    {
      // (0, 0, +): Case 2
      if (db3 > 0.0)
        return case1AndCase2(a1, b1, b2, contact1);
      // (0, 0, -): Case 2
      else if (db3 < 0.0)
        return case1AndCase2(a1, b1, b2, contact1);
      // (0, 0, 0): Coplanar case
      else
        return false;
    }
  }

  return 0;
}

//==============================================================================
inline int checkPointAndLine(const Eigen::Vector3d& a1,
                             const Eigen::Vector3d& b1,
                             const Eigen::Vector3d& b2,
                             Eigen::Vector3d& contact1)
{
  const Eigen::Vector3d b21 = b2 - b1;

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
inline int checkLineAndLine(const Eigen::Vector3d& a1,
                            const Eigen::Vector3d& a2,
                            const Eigen::Vector3d& b1,
                            const Eigen::Vector3d& b2,
                            Eigen::Vector3d& contact1,
                            Eigen::Vector3d& contact2)
{
  const Eigen::Vector3d b21 = b2 - b1;

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
inline void computePointOnPlane(const Eigen::Vector3d& b2,
                                const Eigen::Vector3d& b3,
                                const Eigen::Vector3d& n1,
                                double db2,
                                Eigen::Vector3d& b32OnPlaneA)
{
  const Eigen::Vector3d b32 = b3 - b2;
  b32OnPlaneA = b2 - db2 / n1.dot(b32) * b32;
}

//==============================================================================
int case1AndCase1(const Eigen::Vector3d& a1,
                  const Eigen::Vector3d& b1,
                  Eigen::Vector3d& contact1)
{
  if (a1 == b1)
  {
    contact1 = a1;
    return 1;
  }

  return 0;
}

//==============================================================================
int case1AndCase2(const Eigen::Vector3d& a1,
                  const Eigen::Vector3d& b1,
                  const Eigen::Vector3d& b2,
                  Eigen::Vector3d& contact1)
{
  return checkPointAndLine(a1, b1, b2, contact1);
}

//==============================================================================
int case1AndCase3(const Eigen::Vector3d& a1,
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
int case1AndCase4(
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

  return checkPointAndLine(a1, b21OnPlaneA, b31OnPlaneA, contact1);
}

//==============================================================================
int case2AndCase2(
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
int case2AndCase3(
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
int case2AndCase4(
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
int case3AndCase3(
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
int case3AndCase4(
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
int case4AndCase4(
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

} // namespace collision
} // namespace dart
