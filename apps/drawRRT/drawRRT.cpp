#include "planning/PathPlanner.h"
#include "../unittests/TestHelpers.h" // TODO: Fix this hack

using namespace std;
using namespace planning;

int main () {

    // Create a robot and a world
    const double l1 = 1.5, l2 = 1.0;
    Robot* r = createTwoLinkRobot(Vector3d(0.3, 0.3, l1), DOF_ROLL, Vector3d(0.3, 0.3, l2), DOF_ROLL);
    World w;
    w.addRobot(r);
    w.rebuildCollision();

    // Create a path planner and feed it an unfeasible goal to grow the RRT freely
    PathPlanner <> planner (w, false, false, 1e-1, 1e3, 0.0);
    vector <int> links;
    links.push_back(0);
    links.push_back(1);
    list <VectorXd> path;
    planner.planPath(0, links, Vector2d(-M_PI+0.1, -M_PI+0.1), Vector2d(M_PI-0.1, M_PI-0.1), path);

    // Print the nodes
    RRT* rrt = planner.start_rrt;
    rrt->draw();

}
