#ifndef SRC_UNITTESTS_TEST_MODEL3D_H
#define SRC_UNITTESTS_TEST_MODEL3D_H

#include "model3d/BodyNode.h"

TEST(MODEL3D, BODYNODE) {
  using model3d::BodyNode;
  BodyNode node;
  EXPECT_EQ(2 - 1, 1);
}

#endif // #ifndef SRC_UNITTESTS_TEST_MODEL3D_H

