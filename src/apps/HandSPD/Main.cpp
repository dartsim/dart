/**
 * @file Main.cpp
 * @author Karen Liu
 * @author Can Erdogan - just comments
 * @date Aug 11, 2012
 * @brief The executable for the hand example that demonstrates the use of spd controller.
 */

#include "MyWindow.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

using namespace kinematics;
using namespace dynamics;

int main(int argc, char* argv[]) {

	// Find the model and DOF files
	const char* modelfile;
	const char* doffile;
	if(argc != 3) {
		modelfile = DART_DATA_PATH"skel/fixedHand.skel";
		doffile = DART_DATA_PATH"dof/fixedHand.dof";
	} else {
		modelfile = argv[1];
		doffile = argv[2];
	}

	// Load the model
	FileInfoSkel <SkeletonDynamics> model;
	model.loadFile(modelfile, SKEL);

	// Load the motion
	FileInfoDof *motion = new FileInfoDof(model.getSkel());
	motion->loadFile(doffile);

	// Create the user interface
	MyWindow window(motion, (SkeletonDynamics*) model.getSkel(), NULL);

	// Start with OpenGL
	glutInit(&argc, argv);
	window.initWindow(640, 480, "PD Control");
	glutMainLoop();

	return 0;
}
