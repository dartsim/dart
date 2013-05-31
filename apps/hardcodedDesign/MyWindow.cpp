/**
 * @file MyWindow.h
 * @author Can Erdogan
 * @date Feb 02, 2013
 * @brief Simple example of a skeleton created from scratch.
 */

#include "MyWindow.h"

using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

void MyWindow::draw() {
	glDisable (GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	skel->draw(mRI);
}

void MyWindow::keyboard(unsigned char key, int x, int y) {
  static bool inverse = false;
	static const double dDOF = 0.1;
	switch(key) {
		case '-': {
			inverse = !inverse;
		}	break;
		case '1':
		case '2': 
		case '3': {
			size_t dofIdx = key - 49;
			Eigen::VectorXd pose = skel->get_q();
			pose(dofIdx) = pose(dofIdx) + (inverse ? -dDOF : dDOF);
			skel->setPose(pose);
			std::cout << "Updated pose DOF " << dofIdx << ": " << pose.transpose() << std::endl;
			glutPostRedisplay();
		}	break;
		default:
			Win3D::keyboard(key, x, y);
	}
	glutPostRedisplay();
}

