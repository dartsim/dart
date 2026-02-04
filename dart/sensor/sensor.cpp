/*
 * Copyright (c) 2011, The DART development contributors
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

#include "dart/sensor/sensor.hpp"

#include "dart/common/logging.hpp"
#include "dart/dynamics/frame.hpp"

#include <cmath>

namespace dart {
namespace sensor {

namespace {

constexpr double kUpdateEpsilon = 1e-12;

} // namespace

//==============================================================================
Sensor::Sensor() : Sensor(Properties{}) {}

//==============================================================================
Sensor::Sensor(const Properties& properties)
  : mProperties(properties),
    mLastUpdateContext(),
    mHasUpdate(false),
    mParentFrame(nullptr),
    mNameChangedSignal(),
    onNameChanged(mNameChangedSignal)
{
}

//==============================================================================
const std::string& Sensor::setName(std::string_view name)
{
  if (name == mProperties.name) {
    return mProperties.name;
  }

  const std::string oldName = mProperties.name;
  mProperties.name = name;
  mNameChangedSignal.raise(this, oldName, mProperties.name);
  return mProperties.name;
}

//==============================================================================
const std::string& Sensor::getName() const
{
  return mProperties.name;
}

//==============================================================================
void Sensor::setEnabled(bool enabled)
{
  mProperties.enabled = enabled;
}

//==============================================================================
bool Sensor::isEnabled() const
{
  return mProperties.enabled;
}

//==============================================================================
void Sensor::setUpdateRate(double rate)
{
  if (!std::isfinite(rate) || rate < 0.0) {
    DART_WARN(
        "Attempting to set an invalid sensor update rate ({}). "
        "Clamping to 0 (update every step).",
        rate);
    rate = 0.0;
  }

  mProperties.updateRate = rate;
}

//==============================================================================
double Sensor::getUpdateRate() const
{
  return mProperties.updateRate;
}

//==============================================================================
void Sensor::setRelativeTransform(const Eigen::Isometry3d& transform)
{
  mProperties.relativeTransform = transform;
}

//==============================================================================
const Eigen::Isometry3d& Sensor::getRelativeTransform() const
{
  return mProperties.relativeTransform;
}

//==============================================================================
Eigen::Isometry3d Sensor::getWorldTransform() const
{
  const auto* parent = getParentFrame();
  if (parent) {
    return parent->getWorldTransform() * mProperties.relativeTransform;
  }

  return mProperties.relativeTransform;
}

//==============================================================================
void Sensor::setParentFrame(dynamics::Frame* frame)
{
  mParentFrame = frame;
}

//==============================================================================
dynamics::Frame* Sensor::getParentFrame()
{
  return mParentFrame.get();
}

//==============================================================================
const dynamics::Frame* Sensor::getParentFrame() const
{
  return mParentFrame.get();
}

//==============================================================================
bool Sensor::hasParentFrame() const
{
  return mParentFrame.get() != nullptr;
}

//==============================================================================
const Sensor::Properties& Sensor::getProperties() const
{
  return mProperties;
}

//==============================================================================
void Sensor::reset()
{
  mHasUpdate = false;
  mLastUpdateContext = SensorUpdateContext{};
  resetImpl();
}

//==============================================================================
void Sensor::update(
    const simulation::World& world, const SensorUpdateContext& context)
{
  if (!shouldUpdate(context)) {
    return;
  }

  updateImpl(world, context);
  markUpdated(context);
}

//==============================================================================
bool Sensor::hasUpdate() const
{
  return mHasUpdate;
}

//==============================================================================
const SensorUpdateContext& Sensor::getLastUpdateContext() const
{
  return mLastUpdateContext;
}

//==============================================================================
void Sensor::resetImpl()
{
  // Default: no-op.
}

//==============================================================================
bool Sensor::shouldUpdate(const SensorUpdateContext& context) const
{
  if (!mProperties.enabled) {
    return false;
  }

  if (mProperties.updateRate <= 0.0) {
    return true;
  }

  if (!mHasUpdate) {
    return true;
  }

  const double period = 1.0 / mProperties.updateRate;
  const double elapsed = context.time - mLastUpdateContext.time;
  return elapsed + kUpdateEpsilon >= period;
}

//==============================================================================
void Sensor::markUpdated(const SensorUpdateContext& context)
{
  mLastUpdateContext = context;
  mHasUpdate = true;
}

} // namespace sensor
} // namespace dart
