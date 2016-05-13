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

// Algorithm details and publications: http://www.golems.org/node/1570

#ifndef DART_PLANNING_PATHFOLLOWINGTRAJECTORY_HPP_
#define DART_PLANNING_PATHFOLLOWINGTRAJECTORY_HPP_

#include <Eigen/Core>
#include "dart/planning/Path.hpp"
#include "dart/planning/Trajectory.hpp"

namespace dart {
namespace planning {

class PathFollowingTrajectory : public Trajectory
{
public:
	PathFollowingTrajectory(const Path &path, const Eigen::VectorXd &maxVelocity, const Eigen::VectorXd &maxAcceleration);
	~PathFollowingTrajectory(void);

	bool isValid() const;
	double getDuration() const;
	Eigen::VectorXd getPosition(double time) const;
	Eigen::VectorXd getVelocity(double time) const;
	double getMaxAccelerationError();

private:
	struct TrajectoryStep {
		TrajectoryStep() {}
		TrajectoryStep(double pathPos, double pathVel) :
			pathPos(pathPos),
      pathVel(pathVel),
      time(0.0)
		{}
		double pathPos;
		double pathVel;
		double time;
	};

	bool getNextSwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration);
	bool getNextAccelerationSwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration);
	bool getNextVelocitySwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration);
	bool integrateForward(std::list<TrajectoryStep> &trajectory, double acceleration);
	void integrateBackward(std::list<TrajectoryStep> &trajectory, std::list<TrajectoryStep> &startTrajectory, double acceleration);
	double getMinMaxPathAcceleration(double pathPosition, double pathVelocity, bool max);
	double getMinMaxPhaseSlope(double pathPosition, double pathVelocity, bool max);
	double getAccelerationMaxPathVelocity(double pathPos);
	double getVelocityMaxPathVelocity(double pathPos);
	double getAccelerationMaxPathVelocityDeriv(double pathPos);
	double getVelocityMaxPathVelocityDeriv(double pathPos);
	
	TrajectoryStep getIntersection(const std::list<TrajectoryStep> &trajectory, std::list<TrajectoryStep>::iterator &it, const TrajectoryStep &linePoint1, const TrajectoryStep &linePoint2);
	inline double getSlope(const TrajectoryStep &point1, const TrajectoryStep &point2);
	inline double getSlope(std::list<TrajectoryStep>::const_iterator lineEnd);
	
	std::list<TrajectoryStep>::const_iterator getTrajectorySegment(double time) const;
	
	Path path;
	Eigen::VectorXd maxVelocity;
	Eigen::VectorXd maxAcceleration;
	unsigned int n;
	bool valid;
	std::list<TrajectoryStep> trajectory;

	static const double eps;
	static const double timeStep;

	mutable double cachedTime;
	mutable std::list<TrajectoryStep>::const_iterator cachedTrajectorySegment;
};

} // namespace planning
} // namespace dart

#endif // DART_PLANNING_PATHFOLLOWINGTRAJECTORY_HPP_
