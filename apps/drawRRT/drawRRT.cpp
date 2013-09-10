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
using namespace dart;
using namespace planning;
using namespace simulation;

/* ********************************************************************************************* */
inline void saveLine (char* l1, char* l2, size_t off, const VectorXd& n1, const VectorXd& n2) {
	if(n1.size() == 2) {
		sprintf(l1 + off, "%+02.3lf %+02.3lf ", n1(0), n1(1));
		sprintf(l2 + off, "%+02.3lf %+02.3lf ", n2(0), n2(1));
	}
	else if(n1.size() == 3) {
		sprintf(l1 + off, "%+02.3lf %+02.3lf %+02.3lf ", n1(0), n1(1), n1(2));
		sprintf(l2 + off, "%+02.3lf %+02.3lf %+02.3lf ", n2(0), n2(1), n2(2));
	}
}

/* ********************************************************************************************* */
inline void drawLine (FILE* f, size_t numDofs, const char* color, size_t i, bool last = false) {
	if(numDofs == 2) {
		fprintf(f, "\".data\" u %lu:%lu with linespoints notitle ps 1 pt 6 lc rgb '#%s'%s ", 
			2*i+1, 2*i+2, color, (last ? "" : ","));
	}
	else if (numDofs == 3) {
		fprintf(f, "\".data\" u %lu:%lu:%lu with linespoints notitle ps 1 pt 6 lc rgb '#%s'%s ", 
			3*i+1, 3*i+2, 3*i+3, color, (last ? "" : ","));
	}
}

/* ********************************************************************************************* */
void draw (const RRT* t1, const RRT* t2) {

	// Check the size of a data point - we can only visualize 2 or 3
	size_t numDofs = t1->ndim;
	if((numDofs != 2) && (numDofs != 3)) return;

	// ====================================================================
	// Write the data to a file

	// File contains two lines, one for each endpoint of an edge
	FILE* data = fopen(".data", "w");
	size_t step = numDofs * 7;											// 7 characters per double
	size_t numEdges1 = (t1->configVector.size() - 1);
	size_t numEdges2 = ((t2 == NULL) ? 0 : t2->configVector.size() - 1);
	char* line1 = new char[(numEdges1 + numEdges2 + 5) * step];
	char* line2 = new char[(numEdges1 + numEdges2 + 5) * step];

	// Write each node and its parent in the respective lines
	size_t lineIndex = 0;
	const RRT* trees [2] = {t1, t2};
	for(size_t t = 0; t < 2; t++) {

		// Skip the tree if not there
		if(trees[t] == NULL) continue;

		// Draw the edges
		size_t numEdges = trees[t]->configVector.size() - 1;
		for(size_t i = 0; i < numEdges; i++, lineIndex += step) {
			const VectorXd& node = *trees[t]->configVector[i + 1];
			const VectorXd& parent = *trees[t]->configVector[trees[t]->parentVector[i + 1]];
			saveLine(line1, line2, lineIndex, node, parent);
		}
	}

	// Print the start to draw with a special color (we draw a 0 length edge)
	const VectorXd& goal = *t1->configVector[0];
	saveLine(line1, line2, lineIndex, goal, goal);
	lineIndex += step;

	// Write the lines to the file
	fprintf(data, "%s\n", line1);
	fprintf(data, "%s\n", line2);
	fclose(data);

	delete[] line1;
	delete[] line2;

	// ====================================================================
	// Run gnuplot with the pipe call

	// Open gnuplot in a shell terminal
	FILE* gnuplot;
	gnuplot = fopen(".gnuplot", "w");

	// Set options
	fprintf(gnuplot, "");
	fprintf(gnuplot, "set terminal wxt size 600,600;\n");
	fprintf(gnuplot, "set xrange [-3.14:3.14];\n");
	fprintf(gnuplot, "set yrange [-3.14:3.14];\n");
	fprintf(gnuplot, "set size ratio -1;\n");
	if(numDofs == 3) {
		fprintf(gnuplot, "set zrange [-3.14:3.14];\n");
		fprintf(gnuplot, "set xyplane at -3.14;\n");
	}

	// Draw the edges in the file but leave the last edge to draw a special color for the goal
	fprintf(gnuplot, "%s ", ((numDofs == 2) ? "plot" : "splot"));
	for(size_t i = 0; i < numEdges1; i++) 
		drawLine(gnuplot, numDofs, "0000ff", i);
	for(size_t i = 0; i < numEdges2; i++) 
		drawLine(gnuplot, numDofs, "00ffff", i + numEdges1);
	
	// Draw the goal point (fake edge)
	drawLine(gnuplot, numDofs, "00ff00", numEdges1 + numEdges2, true); 

	// Close the pipe
	fprintf(gnuplot, "\n");
	fprintf(gnuplot, "\n");
	fclose(gnuplot);

	// Make the system call
	int status = system("gnuplot -persist .gnuplot");
	assert((status != -1) && "Error in system call in RRT::draw()");
}

/// Visualizes a 3D RRT
void drawThreeLink () {

    // Create a robot and a world
    const double l1 = 1.5, l2 = 1.0, l3 = 0.5;
    Skeleton* r = createThreeLinkRobot(Vector3d(0.3, 0.3, l1), DOF_ROLL, Vector3d(0.3, 0.3, l2), DOF_ROLL,
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
    planner.planPath(r, links, Vector3d(-DART_PI+0.1, -DART_PI+0.1, -DART_PI+0.1), Vector3d(DART_PI-0.1, DART_PI-0.1, DART_PI+0.1),
 			path);

    // Print the nodes
    RRT* rrt = planner.start_rrt;
    draw(rrt, NULL);
}

/// Visualizes a 2D RRT
void drawTwoLink () {

    // Create a robot and a world
    const double l1 = 1.5, l2 = 1.0;
    Skeleton* r = createTwoLinkRobot(Vector3d(0.3, 0.3, l1), DOF_ROLL, Vector3d(0.3, 0.3, l2), DOF_ROLL);
    World w;
    w.addSkeleton(r);

    // Create a path planner and feed it an unfeasible goal to grow the RRT freely
    PathPlanner <> planner (w, false, false, 1e-1, 1e3, 0.0);
    vector <int> links;
    links.push_back(0);
    links.push_back(1);
    list <VectorXd> path;
    planner.planPath(r, links, Vector2d(-DART_PI+0.1, -DART_PI+0.1), Vector2d(DART_PI-0.1, DART_PI-0.1), path);

    // Print the nodes
    RRT* rrt = planner.start_rrt;
    draw(rrt, NULL);

}

/// The main thread
int main () {
	drawTwoLink();
	drawThreeLink();
}
