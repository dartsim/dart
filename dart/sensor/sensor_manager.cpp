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

#include "dart/sensor/sensor_manager.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/simulation/world.hpp"

#include <algorithm>

namespace dart {
namespace sensor {

//==============================================================================
SensorManager::SensorManager(
    std::string_view managerName, std::string_view defaultName)
  : mNameManager(std::string(managerName), std::string(defaultName))
{
}

//==============================================================================
SensorManager::~SensorManager()
{
  for (const auto& connection : mNameConnections)
    connection.disconnect();
}

//==============================================================================
void SensorManager::setManagerName(std::string_view name)
{
  mNameManager.setManagerName(std::string(name));
}

//==============================================================================
const std::string& SensorManager::getManagerName() const
{
  return mNameManager.getManagerName();
}

//==============================================================================
void SensorManager::setDefaultName(std::string_view defaultName)
{
  mNameManager.setDefaultName(std::string(defaultName));
}

//==============================================================================
const std::string& SensorManager::getDefaultName() const
{
  return mNameManager.getDefaultName();
}

//==============================================================================
SensorPtr SensorManager::getSensor(std::size_t index) const
{
  if (index < mSensors.size())
    return mSensors[index];

  return nullptr;
}

//==============================================================================
SensorPtr SensorManager::getSensor(std::string_view name) const
{
  return mNameManager.getObject(std::string(name));
}

//==============================================================================
std::size_t SensorManager::getNumSensors() const
{
  return mSensors.size();
}

//==============================================================================
std::string SensorManager::addSensor(const SensorPtr& sensor)
{
  if (!sensor) {
    DART_WARN("Attempting to add a nullptr Sensor to the manager!");
    return "";
  }

  if (std::ranges::find(mSensors, sensor) != mSensors.end()) {
    DART_WARN(
        "Sensor named [{}] is already in the manager.", sensor->getName());
    return sensor->getName();
  }

  mSensors.push_back(sensor);
  mSensorToShared[sensor.get()] = sensor;

  mNameConnections.push_back(sensor->onNameChanged.connect(
      [this](const Sensor* _sensor, const std::string&, const std::string&) {
        this->handleSensorNameChange(_sensor);
      }));

  sensor->setName(mNameManager.issueNewNameAndAdd(sensor->getName(), sensor));

  return sensor->getName();
}

//==============================================================================
void SensorManager::removeSensor(const SensorPtr& sensor)
{
  if (!sensor) {
    DART_WARN("Attempting to remove a nullptr Sensor from the manager!");
    return;
  }

  auto it = std::ranges::find(mSensors, sensor);
  if (it == mSensors.end()) {
    DART_WARN("Sensor named [{}] is not in the manager.", sensor->getName());
    return;
  }

  const std::size_t index = static_cast<std::size_t>(it - mSensors.begin());

  mSensors.erase(it);

  mNameConnections[index].disconnect();
  mNameConnections.erase(mNameConnections.begin() + index);

  mNameManager.removeName(sensor->getName());
  mSensorToShared.erase(sensor.get());
}

//==============================================================================
std::set<SensorPtr> SensorManager::removeAllSensors()
{
  std::set<SensorPtr> ptrs;
  for (const auto& sensor : mSensors)
    ptrs.insert(sensor);

  while (getNumSensors() > 0)
    removeSensor(getSensor(0));

  return ptrs;
}

//==============================================================================
bool SensorManager::hasSensor(const SensorPtr& sensor) const
{
  return std::ranges::find(mSensors, sensor) != mSensors.end();
}

//==============================================================================
bool SensorManager::hasSensor(std::string_view name) const
{
  return mNameManager.hasName(std::string(name));
}

//==============================================================================
void SensorManager::updateSensors(const simulation::World& world)
{
  SensorUpdateContext context;
  context.time = world.getTime();
  context.timeStep = world.getTimeStep();
  context.frame = world.getSimFrames();

  for (const auto& sensor : mSensors) {
    if (sensor)
      sensor->update(world, context);
  }
}

//==============================================================================
void SensorManager::resetSensors()
{
  for (const auto& sensor : mSensors) {
    if (sensor)
      sensor->reset();
  }
}

//==============================================================================
void SensorManager::handleSensorNameChange(const Sensor* sensor)
{
  if (!sensor) {
    DART_ERROR(
        "Received a name change callback for a nullptr Sensor. This is most "
        "likely a bug. Please report this!");
    DART_ASSERT(false);
    return;
  }

  auto it = mSensorToShared.find(sensor);
  if (it == mSensorToShared.end()) {
    DART_ERROR(
        "Could not find Sensor named [{}] in the shared_ptr map. This is most "
        "likely a bug. Please report this!",
        sensor->getName());
    DART_ASSERT(false);
    return;
  }

  const SensorPtr& sharedSensor = it->second;
  const std::string& newName = sensor->getName();
  const std::string issuedName
      = mNameManager.changeObjectName(sharedSensor, newName);

  if (!issuedName.empty() && issuedName != newName) {
    sharedSensor->setName(issuedName);
  } else if (issuedName.empty()) {
    DART_ERROR(
        "Sensor named [{}] ({}) does not exist in the NameManager. This is "
        "most likely a bug. Please report this!",
        sensor->getName(),
        sensor);
    DART_ASSERT(false);
    return;
  }
}

} // namespace sensor
} // namespace dart
