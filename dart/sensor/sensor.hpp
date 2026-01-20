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

#ifndef DART_SENSOR_SENSOR_HPP_
#define DART_SENSOR_SENSOR_HPP_

#include <dart/simulation/fwd.hpp>

#include <dart/dynamics/fwd.hpp>

#include <dart/common/class_with_virtual_base.hpp>
#include <dart/common/signal.hpp>
#include <dart/common/sub_ptr.hpp>
#include <dart/common/subject.hpp>

#include <dart/Export.hpp>
#include <dart/sensor/fwd.hpp>

#include <Eigen/Geometry>

#include <string>
#include <string_view>

namespace dart {
namespace sensor {

struct SensorUpdateContext final
{
  double time{0.0};
  double timeStep{0.0};
  int frame{0};
};

DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_BEGIN
class DART_API Sensor : public virtual common::Subject
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using NameChangedSignal = common::Signal<void(
      const Sensor* _sensor,
      const std::string& _oldName,
      const std::string& _newName)>;

  struct Properties
  {
    std::string name{"sensor"};
    double updateRate{0.0};
    bool enabled{true};
    Eigen::Isometry3d relativeTransform{Eigen::Isometry3d::Identity()};
  };

  Sensor();
  explicit Sensor(const Properties& properties);
  virtual ~Sensor() override = default;

  /// Set the name for this sensor.
  const std::string& setName(std::string_view name);

  /// Get the current sensor name.
  const std::string& getName() const;

  /// Enable or disable this sensor.
  void setEnabled(bool enabled);

  /// Return whether this sensor is enabled.
  bool isEnabled() const;

  /// Set the update rate (Hz). A rate <= 0 updates every step.
  void setUpdateRate(double rate);

  /// Get the update rate (Hz).
  double getUpdateRate() const;

  /// Set the relative transform from the parent frame.
  void setRelativeTransform(const Eigen::Isometry3d& transform);

  /// Get the relative transform from the parent frame.
  const Eigen::Isometry3d& getRelativeTransform() const;

  /// Get the world transform of the sensor frame.
  Eigen::Isometry3d getWorldTransform() const;

  /// Set the parent frame of this sensor.
  void setParentFrame(dynamics::Frame* frame);

  /// Get the parent frame of this sensor.
  dynamics::Frame* getParentFrame();

  /// Get the parent frame of this sensor.
  const dynamics::Frame* getParentFrame() const;

  /// Return true if a parent frame is set.
  bool hasParentFrame() const;

  /// Get the current properties of the sensor.
  const Properties& getProperties() const;

  /// Reset sensor internal state.
  void reset();

  /// Update the sensor using the provided world context.
  void update(
      const simulation::World& world, const SensorUpdateContext& context);

  /// Return true if the sensor has produced an update.
  bool hasUpdate() const;

  /// Get the last update context.
  const SensorUpdateContext& getLastUpdateContext() const;

protected:
  virtual void updateImpl(
      const simulation::World& world, const SensorUpdateContext& context)
      = 0;

  virtual void resetImpl();

  bool shouldUpdate(const SensorUpdateContext& context) const;

  void markUpdated(const SensorUpdateContext& context);

private:
  Properties mProperties;
  SensorUpdateContext mLastUpdateContext;
  bool mHasUpdate{false};
  common::sub_ptr<dynamics::Frame> mParentFrame;
  NameChangedSignal mNameChangedSignal;

public:
  //--------------------------------------------------------------------------
  // Signals
  //--------------------------------------------------------------------------
  common::SlotRegister<NameChangedSignal> onNameChanged;
};
DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_END

} // namespace sensor
} // namespace dart

#endif // DART_SENSOR_SENSOR_HPP_
