/**
 * @file drawRRT.cpp
 * @author Can Erdogan
 * @date Feb 24, 2013
 * @brief Visualizes 2D and 3D rrts with gnuplot.
 */

#include "planning/PathPlanner.h"
#include "../unittests/TestHelpers.h" // TODO: Fix this hack
#include "simulation/World.h"

using namespace std;
using namespace planning;
using namespace simulation;



/// Visualizes a 3D RRT
void drawThreeLink () {

    // Create a robot and a world
    const double l1 = 1.5, l2 = 1.0, l3 = 0.5;
    SkeletonDynamics* r = createThreeLinkRobot(Vector3d(0.3, 0.3, l1), DOF_ROLL, Vector3d(0.3, 0.3, l2), DOF_ROLL, 
			Vector3d(0.3, 0.3, l3), DOF_ROLL);
    World w;
    w.addSkeleton(r);

    // Create a path planner and feed it an unfeasible goal to grow the RRT freely
    PathPlanner <> planner (w, false, false, 1e-1, 1e3, 0.0);
    vector <int> links;
    links.push_back(0);
    links.push_back(1);
    links.push_back(2);
    list <VectorXd> path;
    planner.planPath(0, links, Vector3d(-M_PI+0.1, -M_PI+0.1, -M_PI+0.1), Vector3d(M_PI-0.1, M_PI-0.1, M_PI+0.1),
 			path);

    // Print the nodes
    RRT* rrt = planner.start_rrt;
    RRT::draw(rrt, NULL);
}

/// Visualizes a 2D RRT
void drawTwoLink () {

    // Create a robot and a world
    const double l1 = 1.5, l2 = 1.0;
    SkeletonDynamics* r = createTwoLinkRobot(Vector3d(0.3, 0.3, l1), DOF_ROLL, Vector3d(0.3, 0.3, l2), DOF_ROLL);
    World w;
    w.addSkeleton(r);

    // Create a path planner and feed it an unfeasible goal to grow the RRT freely
    PathPlanner <> planner (w, false, false, 1e-1, 1e3, 0.0);
    vector <int> links;
    links.push_back(0);
    links.push_back(1);
    list <VectorXd> path;
    planner.planPath(0, links, Vector2d(-M_PI+0.1, -M_PI+0.1), Vector2d(M_PI-0.1, M_PI-0.1), path);

    // Print the nodes
    RRT* rrt = planner.start_rrt;
    RRT::draw(rrt, NULL);

}

/// The main thread
int main () {
	drawTwoLink();
	drawThreeLink();
}
