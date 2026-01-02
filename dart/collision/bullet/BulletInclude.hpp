/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_COLLISION_BULLET_BULLETINCLUDE_HPP_
#define DART_COLLISION_BULLET_BULLETINCLUDE_HPP_

#include <dart/config.hpp>

class btVector3
{
public:
  btVector3() = default;
  btVector3(double x, double y, double z) : mX(x), mY(y), mZ(z) {}

  double x() const { return mX; }
  double y() const { return mY; }
  double z() const { return mZ; }

private:
  double mX{0.0};
  double mY{0.0};
  double mZ{0.0};
};

class btMatrix3x3
{
public:
  btMatrix3x3() { setIdentity(); }

  btMatrix3x3(
      double v00,
      double v01,
      double v02,
      double v10,
      double v11,
      double v12,
      double v20,
      double v21,
      double v22)
  {
    mData[0][0] = v00;
    mData[0][1] = v01;
    mData[0][2] = v02;
    mData[1][0] = v10;
    mData[1][1] = v11;
    mData[1][2] = v12;
    mData[2][0] = v20;
    mData[2][1] = v21;
    mData[2][2] = v22;
  }

  void setIdentity()
  {
    mData[0][0] = 1.0;
    mData[0][1] = 0.0;
    mData[0][2] = 0.0;
    mData[1][0] = 0.0;
    mData[1][1] = 1.0;
    mData[1][2] = 0.0;
    mData[2][0] = 0.0;
    mData[2][1] = 0.0;
    mData[2][2] = 1.0;
  }

private:
  double mData[3][3]{};
};

class btTransform
{
public:
  btTransform() = default;
  btTransform(const btMatrix3x3& basis, const btVector3& origin)
    : mBasis(basis), mOrigin(origin)
  {
  }

  void setOrigin(const btVector3& origin) { mOrigin = origin; }
  void setBasis(const btMatrix3x3& basis) { mBasis = basis; }

  const btVector3& getOrigin() const { return mOrigin; }
  const btMatrix3x3& getBasis() const { return mBasis; }

private:
  btMatrix3x3 mBasis;
  btVector3 mOrigin;
};

class btCollisionShape
{
public:
  virtual ~btCollisionShape() = default;
};

class btCollisionObject
{
public:
  virtual ~btCollisionObject() = default;

  void* getUserPointer() const { return mUserPointer; }
  void setUserPointer(void* userPointer) { mUserPointer = userPointer; }

private:
  void* mUserPointer{nullptr};
};

class btCollisionWorld
{
public:
  virtual ~btCollisionWorld() = default;
};

#endif // DART_COLLISION_BULLET_BULLETINCLUDE_HPP_
