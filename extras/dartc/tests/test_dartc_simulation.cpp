#include <gtest/gtest.h>
#include "dartc/skeleton.h"
#include "dartc/simulation.h"

//==============================================================================
GTEST_TEST(dartc_simulation, create_and_destroy)
{
  WorldId world = dart_world_create("my world");
  EXPECT_TRUE(world != nullptr);
  EXPECT_TRUE(strcmp(dart_world_get_name(world), "my world") == 0);
  EXPECT_TRUE(dart_world_get_num_skeletons(world) == 0);
  dart_world_destroy(world);
}

//==============================================================================
GTEST_TEST(dartc_simulation, add_an_empty_skeleton)
{
  WorldId world = dart_world_create();
  EXPECT_TRUE(dart_world_get_num_skeletons(world) == 0);
  SkeletonId skeleton = dart_skeleton_create();
  dart_world_add_skeleton(world, skeleton);
  EXPECT_TRUE(dart_world_get_num_skeletons(world) == 1);
  dart_world_remove_all_skeletons(world);
  EXPECT_TRUE(dart_world_get_num_skeletons(world) == 0);
  dart_world_destroy(world);
}

//==============================================================================
GTEST_TEST(dartc_simulation, set_get_gravity)
{
  WorldId world = dart_world_create();
  dart_world_set_gravity(world, 1, 2, 3);
  const double* gravity = dart_world_get_gravity(world);
  EXPECT_DOUBLE_EQ(gravity[0], 1);
  EXPECT_DOUBLE_EQ(gravity[1], 2);
  EXPECT_DOUBLE_EQ(gravity[2], 3);
  EXPECT_DOUBLE_EQ(dart_world_get_gravity_x(world), 1);
  EXPECT_DOUBLE_EQ(dart_world_get_gravity_y(world), 2);
  EXPECT_DOUBLE_EQ(dart_world_get_gravity_z(world), 3);
  dart_world_destroy(world);
}
