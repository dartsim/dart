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

#include "dart/dynamics/Shape.hpp"

#include "dart/common/Console.hpp"

#define PRIMITIVE_MAGIC_NUMBER 1000

namespace dart {
namespace dynamics {

//==============================================================================
Shape::Shape(ShapeType type)
  : mBoundingBox(),
    mIsBoundingBoxDirty(true),
    mVolume(0.0),
    mIsVolumeDirty(true),
    mID(mCounter++),
    mVariance(STATIC),
    mType(type),
    onVersionChanged(mVersionChangedSignal)
{
  mVersion = 1;
}

//==============================================================================
Shape::Shape()
  : mBoundingBox(),
    mVolume(0.0),
    mID(mCounter++),
    mVariance(STATIC),
    mType(UNSUPPORTED),
    onVersionChanged(mVersionChangedSignal)
{
  mVersion = 1;
}

//==============================================================================
Shape::~Shape()
{
  // Do nothing
}

//==============================================================================
const math::BoundingBox& Shape::getBoundingBox() const
{
  if (mIsBoundingBoxDirty)
    updateBoundingBox();

  return mBoundingBox;
}

//==============================================================================
Eigen::Matrix3d Shape::computeInertiaFromDensity(double density) const
{
  return computeInertiaFromMass(density * getVolume());
}

//==============================================================================
Eigen::Matrix3d Shape::computeInertiaFromMass(double mass) const
{
  return computeInertia(mass);
}

//==============================================================================
double Shape::getVolume() const
{
  if (mIsVolumeDirty)
    updateVolume();

  return mVolume;
}

//==============================================================================
std::size_t Shape::getID() const
{
  return mID;
}

//==============================================================================
Shape::ShapeType Shape::getShapeType() const
{
  return mType;
}

//==============================================================================
void Shape::setDataVariance(unsigned int _variance)
{
  mVariance = _variance;
}

//==============================================================================
void Shape::addDataVariance(unsigned int _variance)
{
  mVariance |= _variance;
}

//==============================================================================
void Shape::removeDataVariance(unsigned int _variance)
{
  mVariance &= ~_variance;
}

//==============================================================================
unsigned int Shape::getDataVariance() const
{
  return mVariance;
}

//==============================================================================
bool Shape::checkDataVariance(DataVariance type) const
{
  if (STATIC == type)
    return STATIC == mVariance;

  return (type & mVariance) != 0x00;
}

//==============================================================================
void Shape::refreshData()
{
  // Do nothing
}

//==============================================================================
void Shape::notifyAlphaUpdate(double alpha)
{
  notifyAlphaUpdated(alpha);
}

//==============================================================================
void Shape::notifyAlphaUpdated(double /*alpha*/)
{
  // Do nothing
}

//==============================================================================
void Shape::notifyColorUpdate(const Eigen::Vector4d& color)
{
  notifyColorUpdated(color);
}

//==============================================================================
void Shape::notifyColorUpdated(const Eigen::Vector4d& /*color*/)
{
  // Do nothing
}

//==============================================================================
std::size_t Shape::incrementVersion()
{
  ++mVersion;
  mVersionChangedSignal.raise(this, mVersion);

  return mVersion;
}

//==============================================================================
std::atomic_int Shape::mCounter{PRIMITIVE_MAGIC_NUMBER};

} // namespace dynamics
} // namespace dart
