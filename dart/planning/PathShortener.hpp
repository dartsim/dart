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

#ifndef DART_PLANNING_PATHSHORTENER_HPP_
#define DART_PLANNING_PATHSHORTENER_HPP_

#include <list>
#include <vector>
#include <Eigen/Core>

#include "dart/simulation/World.hpp"

namespace dart {
namespace planning {

class PathShortener
{
public:
	PathShortener();
  PathShortener(simulation::WorldPtr world, dynamics::SkeletonPtr robot, const std::vector<std::size_t>& dofs, double stepSize = 0.1);
	~PathShortener();
	virtual void shortenPath(std::list<Eigen::VectorXd> &rawPath);
	bool segmentCollisionFree(std::list<Eigen::VectorXd> &waypoints, const Eigen::VectorXd &config1, const Eigen::VectorXd &config2);
protected:
  simulation::WorldPtr world;
  dynamics::SkeletonPtr robot;
    std::vector<std::size_t> dofs;
	double stepSize;
	virtual bool localPlanner(std::list<Eigen::VectorXd> &waypoints, std::list<Eigen::VectorXd>::const_iterator it1, std::list<Eigen::VectorXd>::const_iterator it2);
};

} // namespace planning
} // namespace dart

#endif // DART_PLANNING_PATHSHORTENER_HPP_
