/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:a
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

#ifndef DART_PLANNING_PATH_HPP_
#define DART_PLANNING_PATH_HPP_

#include <list>
#include <Eigen/Core>

namespace dart {
namespace planning {

class PathSegment
{
public:
	PathSegment(double length = 0.0) :
    position(0.0), length(length)
	{
	}
	
	virtual ~PathSegment() {}

	double getLength() const {
		return length;
	}
	virtual Eigen::VectorXd getConfig(double s) const = 0;
	virtual Eigen::VectorXd getTangent(double s) const = 0;
	virtual Eigen::VectorXd getCurvature(double s) const = 0;
	virtual std::list<double> getSwitchingPoints() const = 0;
	virtual PathSegment* clone() const = 0;

	double position;
protected:
	double length;
};



class Path
{
public:
	Path(const std::list<Eigen::VectorXd> &path, double maxDeviation = 0.0);
	Path(const Path &path);
	~Path();
	double getLength() const;
	Eigen::VectorXd getConfig(double s) const;
	Eigen::VectorXd getTangent(double s) const;
	Eigen::VectorXd getCurvature(double s) const;
	double getNextSwitchingPoint(double s, bool &discontinuity) const;
	std::list<std::pair<double, bool> > getSwitchingPoints() const;
private:
	PathSegment* getPathSegment(double &s) const;
	double length;
	std::list<std::pair<double, bool> > switchingPoints;
	std::list<PathSegment*> pathSegments;
};

} // namespace planning
} // namespace dart

#endif // DART_PLANNING_PATH_HPP_
