// Copyright (c) 2011-2025, The DART development contributors

#include <dart/simulation/world.hpp>

#include <dart/sensor/sensor_manager.hpp>

#include <gtest/gtest.h>

using namespace dart;

class TestSensor : public sensor::Sensor
{
public:
  using Sensor::Sensor;

  int updateCount{0};
  bool wasReset{false};

protected:
  void updateImpl(
      const simulation::World&, const sensor::SensorUpdateContext&) override
  {
    ++updateCount;
  }

  void resetImpl() override
  {
    wasReset = true;
    Sensor::resetImpl();
  }
};

TEST(SensorManagerTest, DefaultConstruction)
{
  sensor::SensorManager manager;
  EXPECT_EQ(manager.getManagerName(), "SensorManager");
  EXPECT_EQ(manager.getDefaultName(), "sensor");
  EXPECT_EQ(manager.getNumSensors(), 0u);
}

TEST(SensorManagerTest, ConstructionWithCustomNames)
{
  sensor::SensorManager manager("CustomManager", "custom_sensor");
  EXPECT_EQ(manager.getManagerName(), "CustomManager");
  EXPECT_EQ(manager.getDefaultName(), "custom_sensor");
}

TEST(SensorManagerTest, SetManagerName)
{
  sensor::SensorManager manager;
  manager.setManagerName("NewManagerName");
  EXPECT_EQ(manager.getManagerName(), "NewManagerName");
}

TEST(SensorManagerTest, SetDefaultName)
{
  sensor::SensorManager manager;
  manager.setDefaultName("new_default");
  EXPECT_EQ(manager.getDefaultName(), "new_default");
}

TEST(SensorManagerTest, AddSensor)
{
  sensor::SensorManager manager;

  auto sensor = std::make_shared<TestSensor>();
  sensor->setName("test_sensor");

  std::string assignedName = manager.addSensor(sensor);
  EXPECT_EQ(assignedName, "test_sensor");
  EXPECT_EQ(manager.getNumSensors(), 1u);
  EXPECT_TRUE(manager.hasSensor(sensor));
  EXPECT_TRUE(manager.hasSensor("test_sensor"));
}

TEST(SensorManagerTest, AddMultipleSensors)
{
  sensor::SensorManager manager;

  auto sensor1 = std::make_shared<TestSensor>();
  sensor1->setName("sensor1");
  auto sensor2 = std::make_shared<TestSensor>();
  sensor2->setName("sensor2");

  manager.addSensor(sensor1);
  manager.addSensor(sensor2);

  EXPECT_EQ(manager.getNumSensors(), 2u);
  EXPECT_TRUE(manager.hasSensor(sensor1));
  EXPECT_TRUE(manager.hasSensor(sensor2));
}

TEST(SensorManagerTest, AddSensorWithDuplicateName)
{
  sensor::SensorManager manager;

  auto sensor1 = std::make_shared<TestSensor>();
  sensor1->setName("duplicate");
  auto sensor2 = std::make_shared<TestSensor>();
  sensor2->setName("duplicate");

  manager.addSensor(sensor1);
  std::string assignedName = manager.addSensor(sensor2);

  EXPECT_NE(assignedName, "duplicate");
  EXPECT_EQ(manager.getNumSensors(), 2u);
}

TEST(SensorManagerTest, RemoveSensor)
{
  sensor::SensorManager manager;

  auto sensor = std::make_shared<TestSensor>();
  sensor->setName("to_remove");
  manager.addSensor(sensor);

  EXPECT_EQ(manager.getNumSensors(), 1u);
  EXPECT_TRUE(manager.hasSensor(sensor));

  manager.removeSensor(sensor);

  EXPECT_EQ(manager.getNumSensors(), 0u);
  EXPECT_FALSE(manager.hasSensor(sensor));
}

TEST(SensorManagerTest, RemoveAllSensors)
{
  sensor::SensorManager manager;

  auto sensor1 = std::make_shared<TestSensor>();
  auto sensor2 = std::make_shared<TestSensor>();
  manager.addSensor(sensor1);
  manager.addSensor(sensor2);

  EXPECT_EQ(manager.getNumSensors(), 2u);

  auto removed = manager.removeAllSensors();

  EXPECT_EQ(manager.getNumSensors(), 0u);
  EXPECT_EQ(removed.size(), 2u);
  EXPECT_TRUE(removed.count(sensor1) > 0);
  EXPECT_TRUE(removed.count(sensor2) > 0);
}

TEST(SensorManagerTest, GetSensorByIndex)
{
  sensor::SensorManager manager;

  auto sensor1 = std::make_shared<TestSensor>();
  sensor1->setName("first");
  auto sensor2 = std::make_shared<TestSensor>();
  sensor2->setName("second");

  manager.addSensor(sensor1);
  manager.addSensor(sensor2);

  EXPECT_EQ(manager.getSensor(0), sensor1);
  EXPECT_EQ(manager.getSensor(1), sensor2);
}

TEST(SensorManagerTest, GetSensorByIndexOutOfRange)
{
  sensor::SensorManager manager;

  auto sensor = std::make_shared<TestSensor>();
  manager.addSensor(sensor);

  EXPECT_EQ(manager.getSensor(999), nullptr);
}

TEST(SensorManagerTest, GetSensorByName)
{
  sensor::SensorManager manager;

  auto sensor = std::make_shared<TestSensor>();
  sensor->setName("named_sensor");
  manager.addSensor(sensor);

  EXPECT_EQ(manager.getSensor("named_sensor"), sensor);
}

TEST(SensorManagerTest, GetSensorByNameNotFound)
{
  sensor::SensorManager manager;

  auto sensor = std::make_shared<TestSensor>();
  sensor->setName("existing");
  manager.addSensor(sensor);

  EXPECT_EQ(manager.getSensor("nonexistent"), nullptr);
}

TEST(SensorManagerTest, UpdateSensors)
{
  sensor::SensorManager manager;

  auto sensor1 = std::make_shared<TestSensor>();
  auto sensor2 = std::make_shared<TestSensor>();
  manager.addSensor(sensor1);
  manager.addSensor(sensor2);

  auto world = simulation::World::create();
  world->setTimeStep(0.001);

  manager.updateSensors(*world);

  EXPECT_EQ(sensor1->updateCount, 1);
  EXPECT_EQ(sensor2->updateCount, 1);
}

TEST(SensorManagerTest, UpdateSensorsMultipleTimes)
{
  sensor::SensorManager manager;

  auto sensor = std::make_shared<TestSensor>();
  manager.addSensor(sensor);

  auto world = simulation::World::create();

  manager.updateSensors(*world);
  manager.updateSensors(*world);
  manager.updateSensors(*world);

  EXPECT_EQ(sensor->updateCount, 3);
}

TEST(SensorManagerTest, ResetSensors)
{
  sensor::SensorManager manager;

  auto sensor1 = std::make_shared<TestSensor>();
  auto sensor2 = std::make_shared<TestSensor>();
  manager.addSensor(sensor1);
  manager.addSensor(sensor2);

  auto world = simulation::World::create();
  manager.updateSensors(*world);

  EXPECT_TRUE(sensor1->hasUpdate());
  EXPECT_TRUE(sensor2->hasUpdate());

  manager.resetSensors();

  EXPECT_TRUE(sensor1->wasReset);
  EXPECT_TRUE(sensor2->wasReset);
}

TEST(SensorManagerTest, HasSensorByPointer)
{
  sensor::SensorManager manager;

  auto sensor = std::make_shared<TestSensor>();
  auto otherSensor = std::make_shared<TestSensor>();

  manager.addSensor(sensor);

  EXPECT_TRUE(manager.hasSensor(sensor));
  EXPECT_FALSE(manager.hasSensor(otherSensor));
}

TEST(SensorManagerTest, HasSensorByName)
{
  sensor::SensorManager manager;

  auto sensor = std::make_shared<TestSensor>();
  sensor->setName("my_sensor");
  manager.addSensor(sensor);

  EXPECT_TRUE(manager.hasSensor("my_sensor"));
  EXPECT_FALSE(manager.hasSensor("other_sensor"));
}
