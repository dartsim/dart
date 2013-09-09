/**
 * @file MyWindow.h
 * @author Can Erdogan
 * @date Feb 02, 2013
 * @brief Simple example of a skeleton created from scratch.
 */

#pragma once

#include <cstdio>
#include <stdarg.h>

#include "collision/CollisionDetector.h"
#include "dynamics/Skeleton.h"
#include "dynamics/BodyNode.h"
#include "constraint/ConstraintDynamics.h"
#include "dynamics/GenCoord.h"
#include "dynamics/Joint.h"
#include "dynamics/MeshShape.h"
#include "dynamics/BoxShape.h"
#include "dynamics/CylinderShape.h"
#include "dynamics/EllipsoidShape.h"
#include "utils/Paths.h"
#include "common/Timer.h"
#include "math/Helpers.h"
#include "yui/GLFuncs.h"
#include "yui/Win3D.h"

namespace dart {
namespace dynamics {
    class Skeleton;
    class ConstraintDynamics;
}
}

class MyWindow : public dart::yui::Win3D {
public:

	/// The constructor - set the position of the skeleton
    MyWindow(dart::dynamics::Skeleton* _skel): Win3D(), skel(_skel) {
		mTrans[1] = 200.f;
		mZoom = 0.3;
	}

	/// Draw the skeleton
	virtual void draw();

	/// Move the joints with the {1,2,3} keys and '-' to change direction
	virtual void keyboard(unsigned char key, int x, int y);

	/// Hardcoded skeleton
    dart::dynamics::Skeleton* skel;
};
