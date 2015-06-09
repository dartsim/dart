/**
 * @file rrts02-nearestNeighbors.cpp
 * @author Can Erdogan
 * @date Feb 04, 2013
 * @brief Checks if the nearest neighbor computation done by flann is correct.
 */

#include <iostream>
#include <gtest/gtest.h>
#include <flann/flann.hpp>
#include <Eigen/Core>
#include "TestHelpers.h"

/* ********************************************************************************************* */
TEST(NEAREST_NEIGHBOR, 2D) {

    // Build the index with the first node
    flann::Index<flann::L2<double> > index (flann::KDTreeSingleIndexParams(10, true));
    Eigen::VectorXd p1 (2);
    p1 << -3.04159, -3.04159;
    index.buildIndex(flann::Matrix<double>((double*)p1.data(), 1, p1.size()));

    // Add two more points
    Eigen::Vector2d p2 (-2.96751, -2.97443), p3 (-2.91946, -2.88672);
    index.addPoints(flann::Matrix<double>((double*)p2.data(), 1, p2.size()));
    index.addPoints(flann::Matrix<double>((double*)p3.data(), 1, p3.size()));

    // Check the size of the tree
    EXPECT_EQ(3, index.size());

    // Get the nearest neighbor index for a sample point
    Eigen::Vector2d sample (-2.26654, 2.2874);
    int nearest;
    double distance;
    const flann::Matrix<double> queryMatrix((double*)sample.data(), 1, sample.size());
    flann::Matrix<int> nearestMatrix(&nearest, 1, 1);
    flann::Matrix<double> distanceMatrix(flann::Matrix<double>(&distance, 1, 1));
    index.knnSearch(queryMatrix, nearestMatrix, distanceMatrix, 1,
        flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));
    EXPECT_EQ(2, nearest);

    // Get the nearest neighbor
    double* point = index.getPoint(nearest);
    bool equality = equals(Vector2d(point[0], point[1]), p3, 1e-3);
    EXPECT_TRUE(equality);
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
