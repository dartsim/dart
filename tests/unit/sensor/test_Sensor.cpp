// Copyright (c) 2011-2025, The DART development contributors

#include <dart/simulation/world.hpp>

#include <dart/dynamics/simple_frame.hpp>

#include <dart/sensor/sensor.hpp>

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using namespace dart;

class TestSensor : public sensor::Sensor
{
public:
  using Sensor::Sensor;

  int updateCount{0};

protected:
  void updateImpl(
      const simulation::World&, const sensor::SensorUpdateContext&) override
  {
    ++updateCount;
  }
};

TEST(SensorTest, DefaultConstruction)
{
  TestSensor sensor;
  EXPECT_EQ(sensor.getName(), "sensor");
  EXPECT_TRUE(sensor.isEnabled());
  EXPECT_DOUBLE_EQ(sensor.getUpdateRate(), 0.0);
  EXPECT_FALSE(sensor.hasParentFrame());
  EXPECT_FALSE(sensor.hasUpdate());
}

TEST(SensorTest, ConstructionWithProperties)
{
  sensor::Sensor::Properties props;
  props.name = "test_sensor";
  props.updateRate = 100.0;
  props.enabled = false;

  TestSensor sensor(props);
  EXPECT_EQ(sensor.getName(), "test_sensor");
  EXPECT_FALSE(sensor.isEnabled());
  EXPECT_DOUBLE_EQ(sensor.getUpdateRate(), 100.0);
}

TEST(SensorTest, SetName)
{
  TestSensor sensor;
  sensor.setName("new_name");
  EXPECT_EQ(sensor.getName(), "new_name");
}

TEST(SensorTest, SetEmptyName)
{
  TestSensor sensor;
  sensor.setName("valid_name");
  sensor.setName("");
  EXPECT_EQ(sensor.getName(), "");
}

TEST(SensorTest, SetEnabled)
{
  TestSensor sensor;
  EXPECT_TRUE(sensor.isEnabled());

  sensor.setEnabled(false);
  EXPECT_FALSE(sensor.isEnabled());

  sensor.setEnabled(true);
  EXPECT_TRUE(sensor.isEnabled());
}

TEST(SensorTest, SetValidUpdateRate)
{
  TestSensor sensor;
  sensor.setUpdateRate(60.0);
  EXPECT_DOUBLE_EQ(sensor.getUpdateRate(), 60.0);

  sensor.setUpdateRate(1000.0);
  EXPECT_DOUBLE_EQ(sensor.getUpdateRate(), 1000.0);

  sensor.setUpdateRate(0.0);
  EXPECT_DOUBLE_EQ(sensor.getUpdateRate(), 0.0);
}

TEST(SensorTest, SetNegativeUpdateRate)
{
  TestSensor sensor;
  sensor.setUpdateRate(100.0);
  sensor.setUpdateRate(-1.0);
  EXPECT_LE(sensor.getUpdateRate(), 0.0);
}

TEST(SensorTest, SetParentFrame)
{
  TestSensor sensor;
  EXPECT_FALSE(sensor.hasParentFrame());
  EXPECT_EQ(sensor.getParentFrame(), nullptr);

  auto frame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), "test_frame");
  sensor.setParentFrame(frame.get());

  EXPECT_TRUE(sensor.hasParentFrame());
  EXPECT_EQ(sensor.getParentFrame(), frame.get());
}

TEST(SensorTest, SetNullParentFrame)
{
  auto frame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), "test_frame");

  TestSensor sensor;
  sensor.setParentFrame(frame.get());
  EXPECT_TRUE(sensor.hasParentFrame());

  sensor.setParentFrame(nullptr);
  EXPECT_FALSE(sensor.hasParentFrame());
}

TEST(SensorTest, SetRelativeTransform)
{
  TestSensor sensor;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  sensor.setRelativeTransform(transform);
  EXPECT_TRUE(sensor.getRelativeTransform().isApprox(transform));
}

TEST(SensorTest, GetWorldTransformWithoutParent)
{
  TestSensor sensor;

  Eigen::Isometry3d relativeTransform = Eigen::Isometry3d::Identity();
  relativeTransform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  sensor.setRelativeTransform(relativeTransform);

  Eigen::Isometry3d worldTransform = sensor.getWorldTransform();
  EXPECT_TRUE(worldTransform.isApprox(relativeTransform));
}

TEST(SensorTest, UpdateWhenEnabled)
{
  TestSensor sensor;
  auto world = simulation::World::create();

  sensor::SensorUpdateContext context;
  context.time = 0.001;
  context.timeStep = 0.001;
  context.frame = 1;

  sensor.update(*world, context);
  EXPECT_EQ(sensor.updateCount, 1);
  EXPECT_TRUE(sensor.hasUpdate());
}

TEST(SensorTest, UpdateWhenDisabled)
{
  TestSensor sensor;
  sensor.setEnabled(false);
  auto world = simulation::World::create();

  sensor::SensorUpdateContext context;
  context.time = 0.001;
  context.timeStep = 0.001;
  context.frame = 1;

  sensor.update(*world, context);
  EXPECT_EQ(sensor.updateCount, 0);
}

TEST(SensorTest, Reset)
{
  TestSensor sensor;
  auto world = simulation::World::create();

  sensor::SensorUpdateContext context;
  context.time = 0.001;
  context.timeStep = 0.001;
  context.frame = 1;

  sensor.update(*world, context);
  EXPECT_TRUE(sensor.hasUpdate());

  sensor.reset();
  EXPECT_FALSE(sensor.hasUpdate());
}

TEST(SensorTest, GetLastUpdateContext)
{
  TestSensor sensor;
  auto world = simulation::World::create();

  sensor::SensorUpdateContext context;
  context.time = 0.123;
  context.timeStep = 0.001;
  context.frame = 42;

  sensor.update(*world, context);

  const auto& lastContext = sensor.getLastUpdateContext();
  EXPECT_DOUBLE_EQ(lastContext.time, 0.123);
  EXPECT_DOUBLE_EQ(lastContext.timeStep, 0.001);
  EXPECT_EQ(lastContext.frame, 42);
}

TEST(SensorTest, GetProperties)
{
  sensor::Sensor::Properties props;
  props.name = "prop_test";
  props.updateRate = 50.0;
  props.enabled = true;

  TestSensor sensor(props);
  const auto& retrievedProps = sensor.getProperties();

  EXPECT_EQ(retrievedProps.name, "prop_test");
  EXPECT_DOUBLE_EQ(retrievedProps.updateRate, 50.0);
  EXPECT_TRUE(retrievedProps.enabled);
}
