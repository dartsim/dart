/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/07/2013
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <gtest/gtest.h>

#include "fcl/collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"

class COLLISION : public testing::Test
{
public:
	void unrotatedTest(fcl::CollisionGeometry* _coll1,
					   fcl::CollisionGeometry* _coll2,
					   double expectedContactPoint, int _idxAxis);
	void printResult(const fcl::CollisionResult& _result);
};

void COLLISION::unrotatedTest(fcl::CollisionGeometry* _coll1,
							  fcl::CollisionGeometry* _coll2,
							  double expectedContactPoint,
							  int _idxAxis)
{
	fcl::CollisionResult result;
	fcl::CollisionRequest request;
	request.enable_contact = true;
	request.num_max_contacts = 100;

	fcl::Vec3f position(0, 0, 0);

	fcl::Transform3f coll1_transform;
	fcl::Transform3f coll2_transform;

	//==========================================================================
	// Approaching test
	//==========================================================================
	result.clear();
	double dpos = -0.001;
	double pos = 10.0;

	coll1_transform.setIdentity();
	coll1_transform.setTranslation(fcl::Vec3f(0, 0, 0));
	coll2_transform.setIdentity();

	// Let's drop box2 until it collide with box1
	do {
		position[_idxAxis] = pos;
		coll2_transform.setTranslation(position);

		fcl::collide(_coll1, coll1_transform,
					 _coll2, coll2_transform,
					 request, result);

		pos += dpos;
	}
	while (result.numContacts() == 0);

	//
	if (_idxAxis == 0)
		std::cout << "The object is collided when its x-axis position is: " << (pos - dpos) << std::endl;
	if (_idxAxis == 1)
		std::cout << "The object is collided when its y-axis position is: " << (pos - dpos) << std::endl;
	if (_idxAxis == 2)
		std::cout << "The object is collided when its z-axis position is: " << (pos - dpos) << std::endl;

	//printResult(result);

	for (int i = 0; i < result.numContacts(); ++i)
	{
		EXPECT_GE(result.getContact(i).penetration_depth, 0.0);
//		EXPECT_NEAR(result.getContact(i).normal[_idxAxis], -1.0);
		EXPECT_EQ(result.getContact(i).normal.length(), 1.0);
		EXPECT_NEAR(result.getContact(i).pos[_idxAxis], expectedContactPoint, -dpos*2.0);
	}
}

void COLLISION::printResult(const fcl::CollisionResult& _result)
{
	std::cout << "====== [ RESULT ] ======" << std::endl;
	std::cout << "The number of contacts: " << _result.numContacts() << std::endl;

	for (int i = 0; i < _result.numContacts(); ++i)
	{
		std::cout << "----- CONTACT " << i << " --------" << std::endl;
		std::cout << "contact_points: " << _result.getContact(i).pos << std::endl;
		std::cout << "penetration_depth: " << _result.getContact(i).penetration_depth << std::endl;
		std::cout << "normal: " << _result.getContact(i).normal << std::endl;
		//std::cout << std::endl;
	}
	std::cout << std::endl;
}

/* ********************************************************************************************* */

TEST_F(COLLISION, BOX_BOX_X) {
	fcl::Box box1(2, 2, 2);
	fcl::Box box2(1, 1, 1);
	unrotatedTest(&box1, &box2, 1.0, 0); // x-axis
}

TEST_F(COLLISION, BOX_BOX_Y) {
	fcl::Box box1(2, 2, 2);
	fcl::Box box2(1, 1, 1);
	unrotatedTest(&box1, &box2, 1.0, 1); // y-axis
}

TEST_F(COLLISION, BOX_BOX_Z) {
	fcl::Box box1(2, 2, 2);
	fcl::Box box2(1, 1, 1);
	unrotatedTest(&box1, &box2, 1.0, 2); // z-axis
}

TEST_F(COLLISION, BOX_SPHERE_X) {
	fcl::Box box1(2, 2, 2);
	fcl::Sphere sphere(0.5);
	unrotatedTest(&box1, &sphere, 1.0, 0); // x-axis
}

TEST_F(COLLISION, BOX_SPHERE_Y) {
	fcl::Box box1(2, 2, 2);
	fcl::Sphere sphere(0.5);
	unrotatedTest(&box1, &sphere, 1.0, 1); // y-axis
}

TEST_F(COLLISION, BOX_SPHERE_Z) {
	fcl::Box box1(2, 2, 2);
	fcl::Sphere sphere(0.5);
	unrotatedTest(&box1, &sphere, 1.0, 2); // z-axis
}

TEST_F(COLLISION, SPHERE_BOX_X) {
	fcl::Sphere obj1(0.5);
	fcl::Box obj2(2, 2, 2);
	unrotatedTest(&obj1, &obj2, 0.5, 0); // x-axis
}

TEST_F(COLLISION, SPHERE_BOX_Y) {
	fcl::Sphere obj1(0.5);
	fcl::Box obj2(2, 2, 2);
	unrotatedTest(&obj1, &obj2, 0.5, 1); // y-axis
}

TEST_F(COLLISION, SPHERE_BOX_Z) {
	fcl::Sphere obj1(0.5);
	fcl::Box obj2(2, 2, 2);
	unrotatedTest(&obj1, &obj2, 0.5, 2); // z-axis
}

TEST_F(COLLISION, SPHERE_SPHERE_X) {
	fcl::Sphere sphere1(1);
	fcl::Sphere sphere2(0.5);
	unrotatedTest(&sphere1, &sphere2, 1.0, 0); // x-axis
}

TEST_F(COLLISION, SPHERE_SPHERE_Y) {
	fcl::Sphere sphere1(1);
	fcl::Sphere sphere2(0.5);
	unrotatedTest(&sphere1, &sphere2, 1.0, 1); // y-axis
}

TEST_F(COLLISION, SPHERE_SPHERE_Z) {
	fcl::Sphere sphere1(1);
	fcl::Sphere sphere2(0.5);
	unrotatedTest(&sphere1, &sphere2, 1.0, 2); // z-axis
}

//TEST_F(COLLISION, PLANE_BOX_X) {
//	fcl::Plane obj1(1, 0, 0, 0);
//	fcl::Box obj2(1, 1, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 0); // x-axis
//}

//TEST_F(COLLISION, PLANE_BOX_Y) {
//	fcl::Plane obj1(0, 1, 0, 0);
//	fcl::Box obj2(1, 1, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 1); // x-axis
//}

//TEST_F(COLLISION, PLANE_BOX_Z) {
//	fcl::Plane obj1(0, 0, 1, 0);
//	fcl::Box obj2(1, 1, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 2); // x-axis
//}

//TEST_F(COLLISION, PLANE_SPHERE_X) {
//	fcl::Plane obj1(1, 0, 0, 0);
//	fcl::Sphere obj2(0.5);
//	unrotatedTest(&obj1, &obj2, 0.0, 0); // x-axis
//}

//TEST_F(COLLISION, PLANE_SPHERE_Y) {
//	fcl::Plane obj1(0, 1, 0, 0);
//	fcl::Sphere obj2(0.5);
//	unrotatedTest(&obj1, &obj2, 0.0, 1); // x-axis
//}

//TEST_F(COLLISION, PLANE_SPHERE_Z) {
//	fcl::Plane obj1(0, 0, 1, 0);
//	fcl::Sphere obj2(0.5);
//	unrotatedTest(&obj1, &obj2, 0.0, 2); // x-axis
//}

//TEST_F(COLLISION, PLANE_CYLINDER_X) {
//	fcl::Plane obj1(1, 0, 0, 0);
//	fcl::Cylinder obj2(0.5, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 0); // x-axis
//}

//TEST_F(COLLISION, PLANE_CYLINDER_Y) {
//	fcl::Plane obj1(0, 1, 0, 0);
//	fcl::Cylinder obj2(0.5, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 1); // x-axis
//}

//TEST_F(COLLISION, PLANE_CYLINDER_Z) {
//	fcl::Plane obj1(0, 0, 1, 0);
//	fcl::Cylinder obj2(0.5, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 2); // x-axis
//}

//TEST_F(COLLISION, PLANE_CAPSULE_X) {
//	fcl::Plane obj1(1, 0, 0, 0);
//	fcl::Capsule obj2(0.5, 2);
//	unrotatedTest(&obj1, &obj2, 0.0, 0); // x-axis
//}

//TEST_F(COLLISION, PLANE_CAPSULE_Y) {
//	fcl::Plane obj1(0, 1, 0, 0);
//	fcl::Capsule obj2(0.5, 2);
//	unrotatedTest(&obj1, &obj2, 0.0, 1); // x-axis
//}

//TEST_F(COLLISION, PLANE_CAPSULE_Z) {
//	fcl::Plane obj1(0, 0, 1, 0);
//	fcl::Capsule obj2(0.5, 2);
//	unrotatedTest(&obj1, &obj2, 0.0, 2); // x-axis
//}

//TEST_F(COLLISION, PLANE_CONE_X) {
//	fcl::Plane obj1(1, 0, 0, 0);
//	fcl::Cone obj2(0.5, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 0); // x-axis
//}

//TEST_F(COLLISION, PLANE_CONE_Y) {
//	fcl::Plane obj1(0, 1, 0, 0);
//	fcl::Cone obj2(0.5, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 1); // x-axis
//}

//TEST_F(COLLISION, PLANE_CONE_Z) {
//	fcl::Plane obj1(0, 0, 1, 0);
//	fcl::Cone obj2(0.5, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 2); // x-axis
//}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */

