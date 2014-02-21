/**
 * @file rrts01-forwardKinematics.cpp
 * @author Can Erdogan
 * @date Feb 03, 2013
 * @brief Checks forward kinematics for two DoF arm manipulators.
 * NOTE: The following is the reference frame description of the world frame. The x-axis is into the
 * page, z-axis is to the top of the page and the y-axis is to the left. At the zero angle, the links
 * are parallel to the z-axis and face the +x-axis.
 */

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"

std::vector <int> twoLinkIndices;

/* ********************************************************************************************* */
TEST(FORWARD_KINEMATICS, YAW_ROLL) {

    // Create the world
    const double l1 = 1.5, l2 = 1.0;
    Skeleton* robot = createTwoLinkRobot(Vector3d(0.3, 0.3, l1), DOF_YAW, Vector3d(0.3, 0.3, l2),
        DOF_ROLL);

    // Set the test cases with the joint values and the expected end-effector positions
    const size_t numTests = 2;
    double temp = sqrt(0.5*l2*l2);
    Vector2d joints [numTests] = {Vector2d(DART_PI/4.0, DART_PI/2.0), Vector2d(-DART_PI/4.0, -DART_PI/4.0)};
    Vector3d expectedPos [numTests] = {Vector3d(temp, -temp, l1), Vector3d(temp / sqrt(2.0), temp / sqrt(2.0), l1+temp)};

    // Check each case by setting the joint values and obtaining the end-effector position
    for (size_t i = 0; i < numTests; i++) {
        robot->setConfig(twoLinkIndices, joints[i]);
        Vector3d actual = robot->getBodyNode("ee")->getWorldTransform().translation();
        bool equality = equals(actual, expectedPos[i], 1e-3);
        EXPECT_TRUE(equality);
        if(!equality) {
            std::cout << "Joint values: " << joints[i].transpose() << std::endl;
            std::cout << "Actual pos: " << actual.transpose() << std::endl;
            std::cout << "Expected pos: " <<  expectedPos[i].transpose() << std::endl;
        }
    }
}


/* ********************************************************************************************* */
// TODO: Use link lengths in expectations explicitly
TEST(FORWARD_KINEMATICS, TWO_ROLLS) {

    // Create the world
    const double link1 = 1.5, link2 = 1.0;
    Skeleton* robot = createTwoLinkRobot(Vector3d(0.3, 0.3, link1), DOF_ROLL, Vector3d(0.3, 0.3, link2),
        DOF_ROLL);

    // Set the test cases with the joint values and the expected end-effector positions
    const size_t numTests = 2;
    Vector2d joints [numTests] = {Vector2d(0.0, DART_PI/2.0), Vector2d(3*DART_PI/4.0, -DART_PI/4.0)};
    Vector3d expectedPos [numTests] = {Vector3d(0.0, -1.0, 1.5), Vector3d(0.0, -2.06, -1.06)};

    // Check each case by setting the joint values and obtaining the end-effector position
    for (size_t i = 0; i < numTests; i++) {
        robot->setConfig(twoLinkIndices, joints[i]);
        Vector3d actual = robot->getBodyNode("ee")->getWorldTransform().translation();
        bool equality = equals(actual, expectedPos[i], 1e-3);
        EXPECT_TRUE(equality);
        if(!equality) {
            std::cout << "Joint values: " << joints[i].transpose() << std::endl;
            std::cout << "Actual pos: " << actual.transpose() << std::endl;
            std::cout << "Expected pos: " <<  expectedPos[i].transpose() << std::endl;
        }
    }
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {

	// Create the indices to set configuration for the 2D 
    twoLinkIndices.push_back(0);
    twoLinkIndices.push_back(1);

	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
