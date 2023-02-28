/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/common/ecs/entity_manager.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace common;

// Test fixture for EntityManagerT tests
class EntityManagerTest : public testing::Test
{
protected:
  // Declare the entity manager as a member variable to be used in tests
  EntityManagerT<> em;
};

TEST_F(EntityManagerTest, Create)
{
  // Test creating an entity
  auto entity = em.create();
  EXPECT_TRUE(em.isValid(entity));
  EXPECT_EQ(entity.getId(), 0);
  EXPECT_EQ(entity.getVersion(), 0);
}

TEST_F(EntityManagerTest, Destroy)
{
  // Test destroying an entity
  auto entity = em.create();
  em.destroy(entity);
  EXPECT_FALSE(em.isValid(entity));
}

TEST_F(EntityManagerTest, IsValidAfterDestroy)
{
  // Test entity validity after destruction
  auto entity = em.create();
  em.destroy(entity);
  EXPECT_FALSE(em.isValid(entity));
}

TEST_F(EntityManagerTest, IsValidAfterMultipleOperations)
{
  // Test entity validity after multiple creations and destructions
  auto entity1 = em.create();
  auto entity2 = em.create();
  auto entity3 = em.create();

  em.destroy(entity1);
  EXPECT_FALSE(em.isValid(entity1));
  EXPECT_TRUE(em.isValid(entity2));
  EXPECT_TRUE(em.isValid(entity3));

  em.destroy(entity3);
  EXPECT_FALSE(em.isValid(entity1));
  EXPECT_TRUE(em.isValid(entity2));
  EXPECT_FALSE(em.isValid(entity3));

  auto entity4 = em.create();
  EXPECT_FALSE(em.isValid(entity1));
  EXPECT_TRUE(em.isValid(entity2));
  EXPECT_FALSE(em.isValid(entity3));
  EXPECT_TRUE(em.isValid(entity4));
}

TEST_F(EntityManagerTest, CreateAndDestroyManyEntities)
{
  // Create a large number of entities
#ifdef NDEBUG
  const int num_entities = 10000;
#else
  const int num_entities = 1000;
#endif
  std::vector<Entity> entities;
  for (int i = 0; i < num_entities; ++i) {
    entities.push_back(em.create());
  }

  // Destroy half of the entities
  std::vector<Entity> destroyed_entities;
  for (int i = 0; i < num_entities / 2; ++i) {
    em.destroy(entities[i]);
    destroyed_entities.push_back(entities[i]);
  }

  // Create more entities
  for (int i = 0; i < num_entities / 2; ++i) {
    entities.push_back(em.create());
  }

  // Verify that all the valid entities are valid
  for (const auto& entity : entities) {
    if (std::find(destroyed_entities.begin(), destroyed_entities.end(), entity)
        == destroyed_entities.end()) {
      EXPECT_TRUE(em.isValid(entity));
    }
  }
}

// Define a simple component type for testing purposes
struct TestComponent
{
  int value;
};

TEST_F(EntityManagerTest, CreateAndGetComponent)
{
  // Create an entity and add a TestComponent to it
  auto entity = em.create();
  auto comp = em.addComponent<TestComponent>(entity, 42);
  (void)comp;

  // Verify that the component can be retrieved
  EXPECT_TRUE(em.hasComponent<TestComponent>(entity));
  EXPECT_EQ(em.getComponent<TestComponent>(entity).value, 42);
}

TEST_F(EntityManagerTest, DestroyComponent)
{
  // Create an entity and add a TestComponent to it
  auto entity = em.create();
  auto comp = em.addComponent<TestComponent>(entity, 42);
  (void)comp;

  // Destroy the component and verify that it no longer exists
  em.removeComponent<TestComponent>(entity);
  EXPECT_FALSE(em.hasComponent<TestComponent>(entity));
  EXPECT_EQ(em.tryComponent<TestComponent>(entity), nullptr);
}

TEST_F(EntityManagerTest, TwoEntitiesSingleComponent)
{
  // Create an entity and add a TestComponent to it
  auto e_1 = em.create();
  auto e_2 = em.create();

  auto& e_1_comp = em.addComponent<TestComponent>(e_1, 12);
  EXPECT_EQ(e_1_comp.value, 12);
  auto& e_2_comp = em.addComponent<TestComponent>(e_2, 34);
  EXPECT_EQ(e_2_comp.value, 34);

  // Destroy the component and verify that it no longer exists
  em.removeComponent<TestComponent>(e_1);
  EXPECT_FALSE(em.hasComponent<TestComponent>(e_1));
  EXPECT_TRUE(em.hasComponent<TestComponent>(e_2));
  EXPECT_EQ(em.tryComponent<TestComponent>(e_1), nullptr);
  EXPECT_NE(em.tryComponent<TestComponent>(e_2), nullptr);
}

struct TestComponent1 {};
struct TestComponent2 {};

TEST_F(EntityManagerTest, HasComponents) {
  auto entity = em.create();

  // Test with no components
  EXPECT_FALSE(em.hasComponents<TestComponent1>(entity));

  // Add a component and test again
  em.addComponent<TestComponent1>(entity);
  EXPECT_TRUE(em.hasComponents<TestComponent1>(entity));
  EXPECT_FALSE(em.hasComponents<TestComponent2>(entity));

  // Add another component and test again
  em.addComponent<TestComponent2>(entity);
  auto has = em.hasComponents<TestComponent1, TestComponent2>(entity);
  EXPECT_TRUE(has);

  // Test with a non-existent entity
  EXPECT_FALSE(em.hasComponents<TestComponent1>(Entity()));
}

TEST_F(EntityManagerTest, StressTest)
{
  // Define the number of entities and components to create
  constexpr int num_entities = 10000;
  constexpr int num_comps = 10;

  // Create a vector to hold the entities
  std::vector<Entity> entities(num_entities);

  // Create the entities
  for (int i = 0; i < num_entities; ++i) {
    entities[i] = em.create();
    ASSERT_TRUE(em.isValid(entities[i])) << "Entity " << i << " is invalid";
  }

  // Create a vector to hold the component pointers
  std::vector<std::vector<int>> comps(num_entities);

  // Add the components
  for (int i = 0; i < num_comps; ++i) {
    for (int j = 0; j < num_entities; ++j) {
      int& ptr = em.addComponent<int>(entities[j], i);
      if (ptr) {
        ASSERT_TRUE(em.hasComponent<int>(entities[j]))
            << "Entity " << j << " does not have component " << i;
        ASSERT_EQ(ptr, i) << "Component " << i << " of entity " << j
                          << " is not equal to " << i;
        comps[j].push_back(ptr);
      }
    }
  }

  // Modify the components
  for (int i = 0; i < num_comps; ++i) {
    for (int j = 0; j < num_entities; ++j) {
      if (em.hasComponent<int>(entities[j])) {
        int& ptr = em.getComponent<int>(entities[j]);
        ptr = i;
        ASSERT_EQ(ptr, i) << "Component " << i << " of entity " << j
                          << " was not modified";
      }
    }
  }

  // Remove the components
  for (int i = 0; i < num_comps; ++i) {
    for (int j = 0; j < num_entities; ++j) {
      if (!comps[j].empty()) {
        em.removeComponent<int>(entities[j]);
        comps[j].pop_back();
        if (em.hasComponent<int>(entities[j])) {
          ASSERT_EQ(em.getComponent<int>(entities[j]), i - 1)
              << "Component of entity " << j << " was not removed";
        }
      }
    }
  }

  // Remove the entities
  for (int i = 0; i < num_entities; ++i) {
    em.destroy(entities[i]);
    ASSERT_FALSE(em.isValid(entities[i]))
        << "Entity " << i << " was not destroyed";
  }
}

TEST_F(EntityManagerTest, EntityAndComponentModification)
{
  // Test creating entities and adding/removing components in various orders

  // Create a few entities
  const Entity e1 = em.create();
  const Entity e2 = em.create();
  const Entity e3 = em.create();

  // Add some components to the entities
  int& e1_int = em.addComponent<int>(e1);
  float& e1_float = em.addComponent<float>(e1);
  double& e2_double = em.addComponent<double>(e2);

  // Set the component values
  e1_int = 42;
  e1_float = 3.14f;
  e2_double = 2.71828;

  // Add some more components to the entities
  char& e2_char = em.addComponent<char>(e2);
  double& e3_double = em.addComponent<double>(e3);
  int& e3_int = em.addComponent<int>(e3);

  // Set the component values
  e2_char = 'x';
  e3_double = 1.0;
  e3_int = 123;

  // Remove some components from the entities
  bool removed_int = em.removeComponent<int>(e2);
  bool removed_float = em.removeComponent<float>(e1);

  // Check that the components were removed correctly
  EXPECT_FALSE(removed_int);
  EXPECT_TRUE(removed_float);

  // Modify some components
  em.getComponent<int>(e1) = 100;
  em.getComponent<double>(e2) = 3.14159;
  em.getComponent<char>(e2) = 'y';
  em.getComponent<double>(e3) = 2.0;
  em.getComponent<int>(e3) = 456;

  // Remove some more components from the entities
  bool removed_char = em.removeComponent<char>(e2);
  bool removed_double = em.removeComponent<double>(e3);

  // Check that the components were removed correctly
  EXPECT_TRUE(removed_char);
  EXPECT_TRUE(removed_double);

  // Remove the entities
  EXPECT_TRUE(em.isValid(e1));
  EXPECT_TRUE(em.isValid(e2));
  EXPECT_TRUE(em.isValid(e3));
  em.destroy(e1);
  em.destroy(e2);
  em.destroy(e3);

  // Check that the entities were removed correctly
  EXPECT_FALSE(em.isValid(e1));
  EXPECT_FALSE(em.isValid(e2));
  EXPECT_FALSE(em.isValid(e3));
}

TEST_F(EntityManagerTest, ViewBasics)
{
  // Create an entity and add a TestComponent to it
  auto e_1 = em.create();
  auto e_2 = em.create();
  (void)e_2;

  auto& e_1_comp = em.addComponent<TestComponent>(e_1, 12);
  (void)e_1_comp;

  auto view = em.view<TestComponent>();
  (void)view;

  view.each([](const Entity& entity, const TestComponent& test_comp) {
    (void)entity;
    (void)test_comp;
  });
}
