// Standard Libraries
#include <iostream>
using namespace std;
// External Libraries
#include <gtest/gtest.h>

// Local headers
#include "test_utils.h"
#include "test_kinematics.h"
#include "test_dynamics.h"
// #include "test_optimizer.h"

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);

    // Init google libraries
//    ParseCommandLineFlags(&argc, &argv, true);
//    InitGoogleLogging(argv[0]);
//
//    // Define logging flag
//    FLAGS_alsologtostderr = true;
//    FLAGS_minloglevel = INFO;
//    FLAGS_v = 0; // Make this value 1 to see the messages from the library
//    //// FLAGS_log_dir = "./glog/";
//    todo replace with a _new_ log library

    return RUN_ALL_TESTS();
}
