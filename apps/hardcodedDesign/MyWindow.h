/**
 * @file MyWindow.h
 * @author Can Erdogan
 * @date Feb 02, 2013
 * @brief Simple example of a skeleton created from scratch.
 */

#pragma once

#include <cstdio>
#include <stdarg.h>

#include "dart/collision/CollisionDetector.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/constraint/ConstraintDynamics.h"
#include "dart/dynamics/GenCoord.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/utils/Paths.h"
#include "dart/common/Timer.h"
#include "dart/math/Helpers.h"
#include "dart/yui/GLFuncs.h"
#include "dart/yui/Win3D.h"

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
