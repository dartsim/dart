// Standard Libraries
#include <iostream>
using namespace std;
// External Libraries
#include <gtest/gtest.h>
// Local headers
#include "test_utils.h"
#include "test_model3d.h"

int main(int argc, char* argv[]) {
   ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
