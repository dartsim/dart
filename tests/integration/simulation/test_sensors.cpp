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
#include "dart/simulation/world.hpp"

#include <gtest/gtest.h>

#include <memory>

namespace dart {
namespace {

class CountingSensor final : public sensor::Sensor
{
public:
  explicit CountingSensor(const Properties& properties = Properties())
    : Sensor(properties)
  {
  }

  int updateCount{0};
  sensor::SensorUpdateContext lastContext{};
  double lastWorldTime{0.0};

private:
  void updateImpl(
      const simulation::World& world,
      const sensor::SensorUpdateContext& context) override
  {
    ++updateCount;
    lastContext = context;
    lastWorldTime = world.getTime();
  }
};

TEST(SensorIntegration, WorldUpdateContext)
{
  auto world = simulation::World::create();
  auto sensor = std::make_shared<CountingSensor>();

  EXPECT_EQ(world->getNumSensors(), 0u);
  world->addSensor(sensor);
  EXPECT_EQ(world->getNumSensors(), 1u);
  EXPECT_EQ(world->getSensor(0), sensor);

  world->setTimeStep(0.01);
  world->step();

  EXPECT_EQ(sensor->updateCount, 1);
  EXPECT_NEAR(sensor->lastContext.time, world->getTime(), 1e-12);
  EXPECT_NEAR(sensor->lastContext.timeStep, world->getTimeStep(), 1e-12);
  EXPECT_EQ(sensor->lastContext.frame, world->getSimFrames());
  EXPECT_NEAR(sensor->lastWorldTime, world->getTime(), 1e-12);
}

TEST(SensorIntegration, NameManagementAndEnable)
{
  auto world = simulation::World::create();

  auto first = std::make_shared<CountingSensor>();
  first->setName("sensor");
  auto second = std::make_shared<CountingSensor>();
  second->setName("sensor");

  world->addSensor(first);
  world->addSensor(second);

  EXPECT_NE(first->getName(), second->getName());
  EXPECT_EQ(world->getSensor(first->getName()), first);
  EXPECT_EQ(world->getSensor(second->getName()), second);

  second->setEnabled(false);
  world->step();
  EXPECT_EQ(second->updateCount, 0);

  second->setEnabled(true);
  world->step();
  EXPECT_EQ(second->updateCount, 1);
}

TEST(SensorIntegration, ManagerDisconnectsOnDestruction)
{
  auto sensor = std::make_shared<CountingSensor>();
  {
    auto world = simulation::World::create();
    world->addSensor(sensor);
    EXPECT_TRUE(world->hasSensor(sensor));
  }

  sensor->setName("after-world");
  EXPECT_EQ(sensor->getName(), "after-world");
}

} // namespace
} // namespace dart
