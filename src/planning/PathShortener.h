#pragma once

#include <list>
#include <vector>
#include <Eigen/Core>

namespace robotics { class World; }

namespace planning {
class PathShortener
{
public:
	PathShortener();
	PathShortener(robotics::World* world, int robotId, const std::vector<int> &dofs, double stepSize = 0.1);
	~PathShortener();
	virtual void shortenPath(std::list<Eigen::VectorXd> &rawPath);
	bool segmentCollisionFree(std::list<Eigen::VectorXd> &waypoints, const Eigen::VectorXd &config1, const Eigen::VectorXd &config2);
protected:
	robotics::World* world;
	int robotId;
	std::vector<int> dofs;
	double stepSize;
	virtual bool localPlanner(std::list<Eigen::VectorXd> &waypoints, std::list<Eigen::VectorXd>::const_iterator it1, std::list<Eigen::VectorXd>::const_iterator it2);
};
}