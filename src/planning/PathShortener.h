#pragma once

#include <list>
#include <Eigen/Core>

namespace robotics { class World; }

namespace planning {
class PathShortener
{
public:
	PathShortener();
	PathShortener(robotics::World* world, int robotId, const Eigen::VectorXi &dofs, double stepSize = 0.1);
	~PathShortener();
	virtual void shortenPath(std::list<Eigen::VectorXd> &rawPath);
	bool segmentCollisionFree(std::list<Eigen::VectorXd> &waypoints, const Eigen::VectorXd &config1, const Eigen::VectorXd &config2);
protected:
	robotics::World* world;
	int robotId;
	Eigen::VectorXi dofs;
	double stepSize;
	virtual bool localPlanner(std::list<Eigen::VectorXd> &waypoints, std::list<Eigen::VectorXd>::const_iterator it1, std::list<Eigen::VectorXd>::const_iterator it2);
};
}