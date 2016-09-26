/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/planning/PathShortener.hpp"

#include <ctime>
#include <cstdio>

#include "dart/simulation/World.hpp"
#include "dart/planning/RRT.hpp"
#include "dart/collision/CollisionDetector.hpp"
#include "dart/dynamics/Skeleton.hpp"

using namespace std;
using namespace Eigen;
using namespace dart;
using namespace simulation;

#define RAND12(N1,N2) N1 + ((N2-N1) * ((double)rand() / ((double)RAND_MAX + 1))) // random # between N&M

namespace dart {
namespace planning {

PathShortener::PathShortener() {}

PathShortener::PathShortener(WorldPtr world, dynamics::SkeletonPtr robot, const vector<std::size_t> &dofs, double stepSize) :
   world(world),
   robot(robot),
   dofs(dofs),
   stepSize(stepSize)
{}

PathShortener::~PathShortener()
{}

void PathShortener::shortenPath(list<VectorXd> &path)
{
	printf("--> Start Brute Force Shortener \n"); 
  srand(time(nullptr));

  VectorXd savedDofs = robot->getPositions(dofs);

	const int numShortcuts = path.size() * 5;
	
	// Number of checks
	for( int count = 0; count < numShortcuts; count++ ) {
		if( path.size() < 3 ) { //-- No way we can reduce something leaving out the extremes
			return;
		}
		
		int node1Index;
		int node2Index;
		do {
			node1Index = (int) RAND12(0, path.size());
			node2Index = (int) RAND12(0, path.size());
		} while(node2Index <= node1Index + 1);
		
		list<VectorXd>::iterator node1Iter = path.begin();
		advance(node1Iter, node1Index);
		list<VectorXd>::iterator node2Iter = node1Iter;
		advance(node2Iter, node2Index - node1Index);

		list<VectorXd> intermediatePoints;
		if(localPlanner(intermediatePoints, node1Iter, node2Iter)) {
			list<VectorXd>::iterator node1NextIter = node1Iter;
      ++node1NextIter;
			path.erase(node1NextIter, node2Iter);
			path.splice(node2Iter, intermediatePoints);
		}
	}
  // TODO(JS): What kinematic values should be updated here?
  robot->setPositions(dofs, savedDofs);

	printf("End Brute Force Shortener \n");
}

bool PathShortener::localPlanner(list<VectorXd> &intermediatePoints, list<VectorXd>::const_iterator it1, list<VectorXd>::const_iterator it2) {
	return segmentCollisionFree(intermediatePoints, *it1, *it2);
}

// true iff collision-free
// does not check endpoints
// interemdiatePoints are only touched if collision-free
bool PathShortener::segmentCollisionFree(list<VectorXd> &intermediatePoints, const VectorXd &config1, const VectorXd &config2) {
	const double length = (config1 - config2).norm();
	if(length <= stepSize) {
		return true;
	}

	const int n = (int)(length / stepSize) + 1; // number of intermediate segments
	int n1 = n / 2;
	int n2 = n / 2;
	if(n % 2 == 1) {
		n2 += 1;
	}

	VectorXd midpoint = (double)n2 / (double)n * config1 + (double)n1 / (double)n * config2;
	list<VectorXd> intermediatePoints1, intermediatePoints2;
  // TODO(JS): What kinematic values should be updated here?
  robot->setPositions(dofs, midpoint);
	if(!world->checkCollision() && segmentCollisionFree(intermediatePoints1, config1, midpoint)
			&& segmentCollisionFree(intermediatePoints2, midpoint, config2))
	{
		intermediatePoints.clear();
		intermediatePoints.splice(intermediatePoints.end(), intermediatePoints1);
		intermediatePoints.push_back(midpoint);
		intermediatePoints.splice(intermediatePoints.end(), intermediatePoints2);
		return true;
	}
	else {
		return false;
	}
}

} // namespace planning
} // namespace dart
