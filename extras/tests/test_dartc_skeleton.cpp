#include <gtest/gtest.h>
#include "dartc/skeleton.h"

GTEST_TEST(dartc_skeleton, dart_create_skeleton)
{
  SkeletonId skel = dart_create_skeleton("my skeleton");
  EXPECT_TRUE(skel != nullptr);
  EXPECT_TRUE(strcmp(dart_skeleton_get_name(skel), "my skeleton") == 0);
  EXPECT_TRUE(dart_skeleton_get_num_dofs(skel) == 0);
  dart_destroy_skeleton(skel);
}
