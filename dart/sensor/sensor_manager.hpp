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

#ifndef DART_SENSOR_SENSORMANAGER_HPP_
#define DART_SENSOR_SENSORMANAGER_HPP_

#include <dart/common/name_manager.hpp>
#include <dart/common/signal.hpp>

#include <dart/Export.hpp>
#include <dart/sensor/sensor.hpp>

#include <map>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace dart {
namespace sensor {

class DART_API SensorManager
{
public:
  explicit SensorManager(
      std::string_view managerName = "SensorManager",
      std::string_view defaultName = "sensor");
  ~SensorManager();

  /// Set the NameManager label used in diagnostics.
  void setManagerName(std::string_view name);

  /// Get the NameManager label used in diagnostics.
  const std::string& getManagerName() const;

  /// Set the default name used for unnamed sensors.
  void setDefaultName(std::string_view defaultName);

  /// Get the default name used for unnamed sensors.
  const std::string& getDefaultName() const;

  /// Get the indexed sensor.
  SensorPtr getSensor(std::size_t index) const;

  /// Find a sensor by name.
  SensorPtr getSensor(std::string_view name) const;

  /// Get the number of sensors.
  std::size_t getNumSensors() const;

  /// Add a sensor to this manager.
  std::string addSensor(const SensorPtr& sensor);

  /// Remove a sensor from this manager.
  void removeSensor(const SensorPtr& sensor);

  /// Remove all sensors, returning them to the caller.
  std::set<SensorPtr> removeAllSensors();

  /// Returns whether this manager contains a sensor.
  bool hasSensor(const SensorPtr& sensor) const;

  /// Returns whether this manager contains a sensor by name.
  bool hasSensor(std::string_view name) const;

  /// Update all sensors using the current world state.
  void updateSensors(const simulation::World& world);

  /// Reset all sensors in this manager.
  void resetSensors();

private:
  void handleSensorNameChange(const Sensor* sensor);

  std::vector<SensorPtr> mSensors;
  std::vector<common::Connection> mNameConnections;
  std::map<const Sensor*, SensorPtr> mSensorToShared;
  common::NameManager<SensorPtr> mNameManager;
};

} // namespace sensor
} // namespace dart

#endif // DART_SENSOR_SENSORMANAGER_HPP_
