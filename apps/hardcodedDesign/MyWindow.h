/**
 * @file MyWindow.h
 * @author Can Erdogan
 * @date Feb 02, 2013
 * @brief Simple example of a skeleton created from scratch.
 */

#pragma once

#include <cstdio>
#include <stdarg.h>

#include "collision/CollisionSkeleton.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Dof.h"
#include "kinematics/Joint.h"
#include "kinematics/ShapeMesh.h"
#include "kinematics/ShapeBox.h"
#include "kinematics/Transformation.h"
#include "kinematics/TrfmTranslate.h"
#include "kinematics/TrfmRotateEuler.h"
#include "robotics/Constants.h"
#include "utils/Paths.h"
#include "utils/Timer.h"
#include "utils/UtilsMath.h"
#include "yui/GLFuncs.h"
#include "yui/Win3D.h"

using namespace std;

namespace dynamics{
	class SkeletonDynamics;
	class ContactDynamics;
}

namespace integration{
	class IntegrableSystem;
}

class MyWindow : public yui::Win3D, public integration::IntegrableSystem {
public:

	/// The constructor - set the position of the skeleton
	MyWindow(dynamics::SkeletonDynamics* _skel): Win3D(), skel(_skel) {
		mTrans[1] = 200.f;
		mZoom = 0.3;
	}

	/// Draw the skeleton
	virtual void draw();

	/// Move the joints with the {1,2,3} keys and '-' to change direction
	virtual void keyboard(unsigned char key, int x, int y);

public:
	// Virtual functions normally set
	
	virtual void displayTimer(int _val) {}
	virtual Eigen::VectorXd getState() {}
	virtual Eigen::VectorXd evalDeriv() {}
	virtual void setState(const Eigen::VectorXd& state) {}

	dynamics::SkeletonDynamics* skel;
};
