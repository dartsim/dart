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
	void approaching(int idx);
	void printResult(const fcl::CollisionResult& _result);
};

void COLLISION::approaching(int idx)
{
	fcl::CollisionResult result;
	fcl::CollisionRequest request;
	request.enable_contact = true;
	request.num_max_contacts = 100;

	fcl::Vec3f position(0, 0, 0);

	fcl::Box box1(2, 2, 2);
	fcl::Transform3f box1_transform;

	fcl::Box box2(1, 1, 1);
	fcl::Transform3f box2_transform;

	double realContactPoint = 1.0;

	//==========================================================================
	// Key positions test
	//==========================================================================

	//--------------------------------------------------------------------------
	// CASE 1: No contact
	result.clear();
	position.setValue(0, 0, 0);
	box1_transform.setTranslation(position);
	position[idx] = 2;
	box2_transform.setTranslation(position);
	EXPECT_EQ(fcl::collide(&box1, box1_transform,
						   &box2, box2_transform,
						   request, result), 0);
	printResult(result);

	//--------------------------------------------------------------------------
	// CASE 2: Contact, no penetration
	result.clear();
	position[idx] = 1.5;
	box2_transform.setTranslation(position);
	EXPECT_GE(fcl::collide(&box1, box1_transform,
						   &box2, box2_transform,
						   request, result), 1);
	for (int i = 0; i < result.numContacts(); ++i)
	{
		EXPECT_EQ(result.getContact(i).pos[idx], realContactPoint);
	}
	printResult(result);

	//--------------------------------------------------------------------------
	// CASE 3: Contact, penetration
	result.clear();
	position[idx] = 1.0;
	box2_transform.setTranslation(position);
	EXPECT_GE(fcl::collide(&box1, box1_transform,
						   &box2, box2_transform,
						   request, result), 1);
	printResult(result);

	//--------------------------------------------------------------------------
	// CASE 4: Contact, in the same position
	result.clear();
	position[idx] = 0.0;
	box2_transform.setTranslation(position);
	EXPECT_GE(fcl::collide(&box1, box1_transform,
						   &box2, box2_transform,
						   request, result), 1);
	printResult(result);

	//==========================================================================
	// Approaching test
	//==========================================================================
	result.clear();
	double dz = -0.001;
	double z = 5.0;

	box1_transform.setIdentity();
	box1_transform.setTranslation(fcl::Vec3f(0, 0, 0));
	//fcl::Matrix3f rotation;
	//rotation.setEulerZYX(0.0, 0.5, 0.5);
	box2_transform.setIdentity();
	//box2_transform.setRotation(rotation);

	//std::cout << "box1" << std::endl;
	//std::cout << "rotation: " << box1_transform.getRotation() << std::endl;
	//std::cout << "position: " << box1_transform.getTranslation() << std::endl;

	//std::cout << "box2" << std::endl;
	//std::cout << "rotation: " << box2_transform.getRotation() << std::endl;

	// Let's drop box2 until it collide with box1
	do {
		position[idx] = z;
		box2_transform.setTranslation(position);

		//std::cout << "position: " << box2_transform.getTranslation() << std::endl;

		fcl::collide(&box1, box1_transform,
					 &box2, box2_transform,
					 request, result);

		//std::cout << "num_contact: " << result.numContacts() << std::endl;

		z += dz;
	}
	while (result.numContacts() == 0);

	//
	if (idx == 0)
		std::cout << "Box1 is collided when its x-axis position is: " << (z - dz) << std::endl;
	if (idx == 1)
		std::cout << "Box1 is collided when its y-axis position is: " << (z - dz) << std::endl;
	if (idx == 2)
		std::cout << "Box1 is collided when its z-axis position is: " << (z - dz) << std::endl;

	printResult(result);

	for (int i = 0; i < result.numContacts(); ++i)
	{
		EXPECT_NEAR(result.getContact(i).pos[idx], realContactPoint, -dz*2.0);
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
//TEST_F(COLLISION, BOX_BOX_X) {
//	approaching(0); // x-axis
//}

//TEST_F(COLLISION, BOX_BOX_Y) {
//	approaching(1); // y-axis
//}

TEST_F(COLLISION, BOX_BOX_Z) {
    approaching(2); // z-axis
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
