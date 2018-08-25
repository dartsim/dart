#include <gtest/gtest.h>
#include "dartc/skeleton.h"

GTEST_TEST(dartc_skeleton, dart_skeleton_create)
{
  SkeletonId skel = dart_skeleton_create("my skeleton");
  EXPECT_TRUE(skel != nullptr);
  EXPECT_TRUE(strcmp(dart_skeleton_get_name(skel), "my skeleton") == 0);
  EXPECT_TRUE(dart_skeleton_get_num_dofs(skel) == 0);
  dart_skeleton_destroy(skel);
}
