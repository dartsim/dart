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

/** @file RRT.cpp
 *  @author Tobias Kunz
 */

#include "RRT.h"
#include "robotics/World.h"
#include "robotics/Robot.h"
#include "kinematics/Dof.h"

using namespace std;
using namespace Eigen;
using namespace robotics;

namespace planning {

RRT::RRT(World* world, int robot, const std::vector<int> &dofs, const VectorXd &root, double stepSize) :
	world(world),
	robot(robot),
	dofs(dofs),
	ndim(dofs.size()),
	stepSize(stepSize),
	numSamples(0),
	numCollisions(0),
	numNoProgress(0),
	numStepTooLarge(0),
	index(flann::KDTreeSingleIndexParams())
{
	srand(time(NULL));
	addNode(root, -1);
}


RRT::RRT(World* world, int robot, const std::vector<int> &dofs, const vector<VectorXd> &roots, double stepSize) :
	world(world),
	robot(robot),
	dofs(dofs),
	ndim(dofs.size()),
	stepSize(stepSize),
	numSamples(0),
	numCollisions(0),
	numNoProgress(0),
	numStepTooLarge(0),
	index(flann::KDTreeSingleIndexParams())
{
	srand(time(NULL));
	for(int i = 0; i < roots.size(); i++) {
		addNode(roots[i], -1);
	}
}

RRT::~RRT() {
}

bool RRT::connect() {
	VectorXd qtry = getRandomConfig();
	return connect(qtry);
}

bool RRT::connect(const VectorXd &target)
{
	int NNidx = getNearestNeighbor(target);
	StepResult result = STEP_PROGRESS;
	int i = 0;
	while(result == STEP_PROGRESS) {
		result = tryStepFromNode(target, NNidx);
		NNidx = configVector.size() - 1;
		i++;
	}
	return (result == STEP_REACHED);
}

RRT::StepResult RRT::tryStep() {
	VectorXd qtry = getRandomConfig();
	return tryStep(qtry);
}

RRT::StepResult RRT::tryStep(const VectorXd &qtry) {
	int NNidx = getNearestNeighbor(qtry);
	return tryStepFromNode(qtry, NNidx);
}

RRT::StepResult RRT::tryStepFromNode(const VectorXd &qtry, int NNidx)
{
	/*
	 * Calculates a new node to grow towards qtry, checks for collisions, and adds
	 */

	VectorXd qnear = configVector[NNidx];
	
	if((qtry - qnear).norm() < stepSize) {
		return STEP_REACHED;
	}

	// Scale this vector to step_size and add to end of qnear
	VectorXd qnew = qnear + stepSize * (qtry - qnear).normalized();
	list<VectorXd> intermediatePoints;

	if(!newConfig(intermediatePoints, qnew, qnear, qtry)) {
		return STEP_COLLISION;
	}
	else {
		for(list<VectorXd>::iterator it = intermediatePoints.begin(); it != intermediatePoints.end(); it++) {
			NNidx = addNode(*it, NNidx);
		}
		addNode(qnew, NNidx);
		return STEP_PROGRESS;
	}
}

// Provides a hook for child classes to alter a new config before it gets added to the tree
bool RRT::newConfig(list<VectorXd> &intermediatePoints, VectorXd &qnew, const VectorXd &qnear, const VectorXd &qtarget) {
	return !checkCollisions(qnew);
}

int RRT::addNode(const VectorXd &qnew, int parentId)
{
	// Update graph vectors
	configVector.push_back(qnew);
	parentVector.push_back(parentId);
	
	unsigned int id = configVector.size() - 1;
	if(id == 0) {
		index.buildIndex(flann::Matrix<double>((double*)qnew.data(), 1, qnew.size()));
	}
	else {
		index.addPoints(flann::Matrix<double>((double*)qnew.data(), 1, qnew.size()));
	}
	
	activeNode = id;
	return id;
}

inline int RRT::getNearestNeighbor(const VectorXd &qsamp)
{
	int nearest;
	double distance;
	const flann::Matrix<double> queryMatrix((double*)qsamp.data(), 1, qsamp.size());
	flann::Matrix<int> nearestMatrix(&nearest, 1, 1);
	flann::Matrix<double> distanceMatrix(flann::Matrix<double>(&distance, 1, 1));
	index.knnSearch(queryMatrix, nearestMatrix, distanceMatrix, 1, flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));
	activeNode = nearest;
	return nearest;
}

// random # between min & max
inline double RRT::randomInRange(double min, double max) {
	if(min == max) {
		return min;
	}
	return min + ((max-min) * ((double)rand() / ((double)RAND_MAX + 1)));
}

VectorXd RRT::getRandomConfig()
{
	/*
	 * Samples a random point for qtmp in the configuration space,
	 * bounded by the provided configuration vectors (and returns ref to it)
	 */
	VectorXd config(ndim);
	for (int i = 0; i < ndim; ++i) {
		config[i] = randomInRange(world->getRobot(robot)->getDof(dofs[i])->getMin(), world->getRobot(robot)->getDof(dofs[i])->getMax());
	}
	return config;
}

double RRT::getGap(const VectorXd &target) {
	return (target - configVector[activeNode]).norm();
}

void RRT::tracePath(int node, std::list<VectorXd> &path, bool reverse)
{
	int x = node;
	
	while(x != -1) {
		if(!reverse) {
			path.push_front(configVector[x]);
		}
		else {
			path.push_back(configVector[x]);
		}
		x = parentVector[x];
	}
}

bool RRT::checkCollisions(const VectorXd &c)
{
	world->getRobot(robot)->setDofs(c, dofs);
	return world->checkCollision();
}

unsigned int RRT::getSize() {
	return configVector.size();
}

}
