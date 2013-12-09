/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** 
 * @file RRT.cpp
 * @author Tobias Kunz, Can Erdogan
 * @date Jan 31, 2013
 * @brief The generic RRT implementation. It can be inherited for modifications to collision
 * checking, sampling and etc.
 */

#include "RRT.h"
#include "dart/simulation/World.h"
#include "dart/dynamics/GenCoord.h"
#include "dart/dynamics/Skeleton.h"
#include <flann/flann.hpp>

using namespace std;
using namespace Eigen;
using namespace dart;
using namespace simulation;
using namespace dynamics;

namespace dart {
namespace planning {

/* ********************************************************************************************* */
RRT::RRT(World* world, Skeleton* robot, const std::vector<int> &dofs, const VectorXd &root, double stepSize) :
	world(world),
	robot(robot),
	dofs(dofs),
	ndim(dofs.size()),
	stepSize(stepSize),
	index(new flann::Index<flann::L2<double> >(flann::KDTreeSingleIndexParams()))
{
	// Reset the random number generator and add the given start configuration to the flann structure
	srand(time(NULL));
	addNode(root, -1);
}

/* ********************************************************************************************* */
RRT::RRT(World* world, dynamics::Skeleton* robot, const std::vector<int> &dofs, const vector<VectorXd> &roots, double stepSize) :
	world(world),
	robot(robot),
	dofs(dofs),
	ndim(dofs.size()),
	stepSize(stepSize),
	index(new flann::Index<flann::L2<double> >(flann::KDTreeSingleIndexParams()))
{
	// Reset the random number generator and add the given start configurations to the flann structure
	srand(time(NULL));
	for(int i = 0; i < roots.size(); i++) {
		addNode(roots[i], -1);
	}
}

/* ********************************************************************************************* */
bool RRT::connect() {
	VectorXd qtry = getRandomConfig();
	return connect(qtry);
}

/* ********************************************************************************************* */
bool RRT::connect(const VectorXd &target) {

	// Get the index of the nearest neighbor in the tree to the given target
	int NNidx = getNearestNeighbor(target);

	// Keep taking steps towards the target until a collision happens
	StepResult result = STEP_PROGRESS;
	while(result == STEP_PROGRESS) {
		result = tryStepFromNode(target, NNidx);
		NNidx = configVector.size() - 1;
	}
	return (result == STEP_REACHED);
}

/* ********************************************************************************************* */
RRT::StepResult RRT::tryStep() {
	VectorXd qtry = getRandomConfig();
	return tryStep(qtry);
}

/* ********************************************************************************************* */
RRT::StepResult RRT::tryStep(const VectorXd &qtry) {
	int NNidx = getNearestNeighbor(qtry);
	return tryStepFromNode(qtry, NNidx);
}

/* ********************************************************************************************* */
RRT::StepResult RRT::tryStepFromNode(const VectorXd &qtry, int NNidx) {

	// Get the configuration of the nearest neighbor and check if already reached
	const VectorXd& qnear = *(configVector[NNidx]);
	if((qtry - qnear).norm() < stepSize) {
		return STEP_REACHED;
	}

	// Create the new node: scale the direction vector to stepSize and add to qnear
	VectorXd qnew = qnear + stepSize * (qtry - qnear).normalized();

	// Check for collision, make changes to the qNew and create intermediate points if necessary
	// NOTE: This is largely implementation dependent and in default, no points are created.
	list<VectorXd> intermediatePoints;
	bool collisionClear = newConfig(intermediatePoints, qnew, qnear, qtry);
	if(!collisionClear) return STEP_COLLISION;

	// Add the intermediate nodes and the final new node to the tree
	list <VectorXd>::iterator it = intermediatePoints.begin();
	for(; it != intermediatePoints.end(); it++) 
		NNidx = addNode(*it, NNidx);
	addNode(qnew, NNidx);
	return STEP_PROGRESS;
}

/* ********************************************************************************************* */
bool RRT::newConfig(list<VectorXd> &intermediatePoints, VectorXd &qnew, const VectorXd &qnear, const VectorXd &qtarget) {
	return !checkCollisions(qnew);
}

/* ********************************************************************************************* */
int RRT::addNode(const VectorXd &qnew, int parentId) {
	
	// Update the graph vector
	VectorXd* temp = new VectorXd(qnew);
	configVector.push_back(temp);
	parentVector.push_back(parentId);

	// Update the underlying flann structure (the kdtree)
	unsigned int id = configVector.size() - 1;
	if(id == 0) 
		index->buildIndex(flann::Matrix<double>((double*)temp->data(), 1, temp->size()));
	else 
		index->addPoints(flann::Matrix<double>((double*)temp->data(), 1, temp->size()));
	
	activeNode = id;
	return id;
}

/* ********************************************************************************************* */
inline int RRT::getNearestNeighbor(const VectorXd &qsamp) {
	int nearest;
	double distance;
	const flann::Matrix<double> queryMatrix((double*)qsamp.data(), 1, qsamp.size());
	flann::Matrix<int> nearestMatrix(&nearest, 1, 1);
	flann::Matrix<double> distanceMatrix(flann::Matrix<double>(&distance, 1, 1));
	index->knnSearch(queryMatrix, nearestMatrix, distanceMatrix, 1, 
		flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));
	activeNode = nearest;
	return nearest;
}

/* ********************************************************************************************* */
// random # between min & max
inline double RRT::randomInRange(double min, double max) {
	assert(max - min >= 0.0);
	assert(max - min < numeric_limits<double>::infinity());

	if(min == max) return min;
	return min + ((max-min) * ((double)rand() / ((double)RAND_MAX + 1)));
}

/* ********************************************************************************************* */
VectorXd RRT::getRandomConfig() {
	// Samples a random point for qtmp in the configuration space, bounded by the provided 
	// configuration vectors (and returns ref to it)
	VectorXd config(ndim);
	for (int i = 0; i < ndim; ++i) {
        config[i] = randomInRange(robot->getGenCoord(dofs[i])->get_qMin(), robot->getGenCoord(dofs[i])->get_qMax());
	}
	return config;
}

/* ********************************************************************************************* */
double RRT::getGap(const VectorXd &target) {
	return (target - *(configVector[activeNode])).norm();
}

/* ********************************************************************************************* */
void RRT::tracePath(int node, std::list<VectorXd> &path, bool reverse) {

	// Keep following the "linked list" in the given direction
	int x = node;
	while(x != -1) {
		if(!reverse) path.push_front(*(configVector[x]));	
		else path.push_back(*(configVector[x]));
		x = parentVector[x];
	}
}

/* ********************************************************************************************* */
bool RRT::checkCollisions(const VectorXd &c) {
	robot->setConfig(dofs, c);
	return world->checkCollision();
}

/* ********************************************************************************************* */
size_t RRT::getSize() {
	return configVector.size();
}

} // namespace planning
} // namespace dart
