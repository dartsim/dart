#pragma once

#include <list>
#include <vector>
#include <Eigen/Core>

#include "dart/simulation/World.hpp"

namespace dart {

namespace simulation { class World; }
namespace dynamics { class Skeleton; }

namespace planning {

class PathShortener
{
public:
	PathShortener();
  PathShortener(simulation::WorldPtr world, dynamics::SkeletonPtr robot, const std::vector<size_t>& dofs, double stepSize = 0.1);
	~PathShortener();
	virtual void shortenPath(std::list<Eigen::VectorXd> &rawPath);
	bool segmentCollisionFree(std::list<Eigen::VectorXd> &waypoints, const Eigen::VectorXd &config1, const Eigen::VectorXd &config2);
protected:
  simulation::WorldPtr world;
  dynamics::SkeletonPtr robot;
	std::vector<size_t> dofs;
	double stepSize;
	virtual bool localPlanner(std::list<Eigen::VectorXd> &waypoints, std::list<Eigen::VectorXd>::const_iterator it1, std::list<Eigen::VectorXd>::const_iterator it2);
};

} // namespace planning
} // namespace dart
