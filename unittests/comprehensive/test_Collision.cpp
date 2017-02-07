/*
 * Copyright (c) 2013-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2013-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/narrowphase/narrowphase.h>

#include "dart/config.hpp"
#include "dart/common/common.hpp"
#include "dart/math/math.hpp"
#include "dart/dynamics/dynamics.hpp"
#include "dart/collision/collision.hpp"
#include "dart/collision/bullet/bullet.hpp"
#include "dart/simulation/simulation.hpp"
#include "dart/utils/utils.hpp"
#include "TestHelpers.hpp"

using namespace dart;
using namespace common;
using namespace math;
using namespace collision;
using namespace dynamics;
using namespace simulation;
using namespace utils;

class COLLISION : public testing::Test
{
public:
    void unrotatedTest(fcl::CollisionGeometry* _coll1,
                       fcl::CollisionGeometry* _coll2,
                       double expectedContactPoint, int _idxAxis);
    void dropWithRotation(fcl::CollisionGeometry* _object,
                          double EulerZ, double EulerY, double EulerX);
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

    for (std::size_t i = 0; i < result.numContacts(); ++i)
    {
        EXPECT_GE(result.getContact(i).penetration_depth, 0.0);
//		EXPECT_NEAR(result.getContact(i).normal[_idxAxis], -1.0);
        EXPECT_EQ(result.getContact(i).normal.length(), 1.0);
        EXPECT_NEAR(result.getContact(i).pos[_idxAxis], expectedContactPoint, -dpos*2.0);
    }
}

void COLLISION::dropWithRotation(fcl::CollisionGeometry* _object,
                                 double EulerZ, double EulerY, double EulerX)
{
    // Collision test setting
    fcl::CollisionResult result;
    fcl::CollisionRequest request;
    request.enable_contact = true;
    request.num_max_contacts = 100;

    // Ground like box setting
    fcl::Box groundObject(100, 100, 0.1);
    fcl::Transform3f groundTransf;
    groundTransf.setIdentity();
    fcl::Vec3f ground_position(0, 0, 0);
    groundTransf.setTranslation(ground_position);

    // Dropping object setting
    fcl::Transform3f objectTransf;
    fcl::Matrix3f rot;
    rot.setEulerZYX(EulerZ,
                    EulerY,
                    EulerX);
    objectTransf.setRotation(rot);
    fcl::Vec3f dropping_position(0, 0, 0);
    objectTransf.setTranslation(dropping_position);

    //==========================================================================
    // Dropping test in x, y, z aixs each.
    //==========================================================================
    for (int _idxAxis = 0; _idxAxis < 3; ++_idxAxis)
    {
        result.clear();

        groundObject.side = fcl::Vec3f(100, 100, 100);
        groundObject.side[_idxAxis] = 0.1;
        ground_position.setValue(0, 0, 0);
        ground_position[_idxAxis] = -0.05;
        groundTransf.setTranslation(ground_position);

        // Let's drop the object until it collide with ground
        double posDelta = -0.0001;
        double initPos = 10.0;
        dropping_position.setValue(0, 0, 0);
        do {
            dropping_position[_idxAxis] = initPos;
            objectTransf.setTranslation(dropping_position);

            fcl::collide(_object, objectTransf,
                         &groundObject, groundTransf,
                         request, result);

            initPos += posDelta;
        }
        while (result.numContacts() == 0);

        std::cout << "Current position of the object: "
                  << objectTransf.getTranslation()
                  << std::endl
                  << "Number of contacts: "
                  << result.numContacts()
                  << std::endl;

        fcl::Transform3f objectTransfInv = objectTransf;
        objectTransfInv.inverse();
        for (std::size_t i = 0; i < result.numContacts(); ++i)
        {
            fcl::Vec3f posWorld = objectTransfInv.transform(result.getContact(i).pos);
            std::cout << "----- CONTACT " << i << " --------" << std::endl;
            std::cout << "contact_points: " << result.getContact(i).pos << std::endl;
            std::cout << "contact_points(w): " << posWorld << std::endl;
            std::cout << "norm: " << result.getContact(i).pos.length() << std::endl;
            std::cout << "penetration_depth: " << result.getContact(i).penetration_depth << std::endl;
            std::cout << "normal: " << result.getContact(i).normal << std::endl;
        }

        std::cout << std::endl;
    }
}

void COLLISION::printResult(const fcl::CollisionResult& _result)
{
	std::cout << "====== [ RESULT ] ======" << std::endl;
	std::cout << "The number of contacts: " << _result.numContacts() << std::endl;

  for (std::size_t i = 0; i < _result.numContacts(); ++i)
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
//	fcl::Box box1(2, 2, 2);
//	fcl::Box box2(1, 1, 1);
//	unrotatedTest(&box1, &box2, 1.0, 0); // x-axis
//}

//TEST_F(COLLISION, BOX_BOX_Y) {
//	fcl::Box box1(2, 2, 2);
//	fcl::Box box2(1, 1, 1);
//	unrotatedTest(&box1, &box2, 1.0, 1); // y-axis
//}

//TEST_F(COLLISION, BOX_BOX_Z) {
//	fcl::Box box1(2, 2, 2);
//	fcl::Box box2(1, 1, 1);
//	unrotatedTest(&box1, &box2, 1.0, 2); // z-axis
//}

//TEST_F(COLLISION, BOX_SPHERE_X) {
//	fcl::Box box1(2, 2, 2);
//	fcl::Sphere sphere(0.5);
//	unrotatedTest(&box1, &sphere, 1.0, 0); // x-axis
//}

//TEST_F(COLLISION, BOX_SPHERE_Y) {
//	fcl::Box box1(2, 2, 2);
//	fcl::Sphere sphere(0.5);
//	unrotatedTest(&box1, &sphere, 1.0, 1); // y-axis
//}

//TEST_F(COLLISION, BOX_SPHERE_Z) {
//	fcl::Box box1(2, 2, 2);
//	fcl::Sphere sphere(0.5);
//	unrotatedTest(&box1, &sphere, 1.0, 2); // z-axis
//}

//TEST_F(COLLISION, SPHERE_BOX_X) {
//	fcl::Sphere obj1(0.5);
//	fcl::Box obj2(2, 2, 2);
//	unrotatedTest(&obj1, &obj2, 0.5, 0); // x-axis
//}

//TEST_F(COLLISION, SPHERE_BOX_Y) {
//	fcl::Sphere obj1(0.5);
//	fcl::Box obj2(2, 2, 2);
//	unrotatedTest(&obj1, &obj2, 0.5, 1); // y-axis
//}

//TEST_F(COLLISION, SPHERE_BOX_Z) {
//	fcl::Sphere obj1(0.5);
//	fcl::Box obj2(2, 2, 2);
//	unrotatedTest(&obj1, &obj2, 0.5, 2); // z-axis
//}

//TEST_F(COLLISION, SPHERE_SPHERE_X) {
//	fcl::Sphere sphere1(1);
//	fcl::Sphere sphere2(0.5);
//	unrotatedTest(&sphere1, &sphere2, 1.0, 0); // x-axis
//}

//TEST_F(COLLISION, SPHERE_SPHERE_Y) {
//	fcl::Sphere sphere1(1);
//	fcl::Sphere sphere2(0.5);
//	unrotatedTest(&sphere1, &sphere2, 1.0, 1); // y-axis
//}

//TEST_F(COLLISION, SPHERE_SPHERE_Z) {
//	fcl::Sphere sphere1(1);
//	fcl::Sphere sphere2(0.5);
//	unrotatedTest(&sphere1, &sphere2, 1.0, 2); // z-axis
//}

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


TEST_F(COLLISION, DROP)
{
    dtdbg << "Unrotated box\n";
    fcl::Box box1(0.5, 0.5, 0.5);
    dropWithRotation(&box1, 0, 0, 0);

    dtdbg << "Rotated box\n";
    fcl::Box box2(0.5, 0.5, 0.5);
    dropWithRotation(&box2,
                     dart::math::random(-3.14, 3.14),
                     dart::math::random(-3.14, 3.14),
                     dart::math::random(-3.14, 3.14));

    dropWithRotation(&box2,
                     0.0,
                     0.1,
                     0.0);
}

TEST_F(COLLISION, FCL_BOX_BOX)
{
    double EulerZ = 1;
    double EulerY = 2;
    double EulerX = 3;

    // Collision test setting
    fcl::CollisionResult result;
    fcl::CollisionRequest request;
    request.enable_contact = true;
    request.num_max_contacts = 100;

    // Ground like box setting
    fcl::Box groundObject(100, 100, 0.1);
    fcl::Transform3f groundTransf;
    groundTransf.setIdentity();
    fcl::Vec3f ground_position(0.0, 0.0, -0.05);
    groundTransf.setTranslation(ground_position);

    // Dropping box object setting
    fcl::Box box(0.5, 0.5, 0.5);
    fcl::Transform3f objectTransf;
    fcl::Matrix3f rot;
    rot.setEulerZYX(EulerZ, EulerY, EulerX);
    objectTransf.setRotation(rot);
    fcl::Vec3f dropping_position(0.0, 0.0, 5.0);
    objectTransf.setTranslation(dropping_position);

    // Let's drop the object until it collide with ground
    do {
        objectTransf.setTranslation(dropping_position);

        fcl::collide(&box, objectTransf, &groundObject, groundTransf, request, result);

        dropping_position[2] -= 0.00001;
    }
    while (result.numContacts() == 0);

    std::cout << "Current position of the object: "
              << objectTransf.getTranslation()
              << std::endl
              << "Number of contacts: "
              << result.numContacts()
              << std::endl;

    for (std::size_t i = 0; i < result.numContacts(); ++i)
    {
        std::cout << "----- CONTACT " << i << " --------" << std::endl;
        std::cout << "contact_points: " << result.getContact(i).pos << std::endl;
        std::cout << "penetration_depth: " << result.getContact(i).penetration_depth << std::endl;
        std::cout << "normal: " << result.getContact(i).normal << std::endl;
    }
}

//TEST_F(COLLISION, OWN_COLLISION_CODE)
//{
//    double EulerZ = 1;
//    double EulerY = 2;
//    double EulerX = 3;

//    // Collision test setting
//    CollisionInfoArray result;

//    Vec3 size1(100.0, 100.0, 0.1);
//    SE3 T1(Vec3(0.0, 0.0, -0.05));

//    Vec3 size2(0.5, 0.5, 0.5);
//    Vec3 pos2(0.0, 0.0, 5.0);
//    SE3 T2;
//    T2 = EulerZYX(Vec3(EulerZ, EulerY, EulerX));
//    T2.setPosition(pos2);

//    // Let's drop the object until it collide with ground
//    do {
//        T2.setPosition(pos2);

//        _BoxBox_____________MARK8(size1, T1, size2, T2, result);

//        pos2[2] -= 0.00001;
//    }
//    while (result.size() == 0);

//    std::cout //<< "Current position of the object: "
//              //<< objectTransf.getTranslation()
//              //<< std::endl
//              << "Number of contacts: "
//              << result.size()
//              << std::endl;

//    for (int i = 0; i < result.size(); ++i)
//    {
//        std::cout << "----- CONTACT " << i << " --------" << std::endl;
//        std::cout << "contact_points: " << result.at(i).point;
//        std::cout << "penetration_depth: " << result.at(i).penetration << std::endl;
//        std::cout << "normal: " << result.at(i).normal << std::endl;
//    }


//}

//==============================================================================
void testSimpleFrames(const std::shared_ptr<CollisionDetector>& cd)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame3 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr shape2(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr shape3(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));

  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);
  simpleFrame3->setShape(shape3);

  auto group1 = cd->createCollisionGroup(simpleFrame1.get());
  auto group2 = cd->createCollisionGroup(simpleFrame2.get());
  auto group3 = cd->createCollisionGroup(simpleFrame3.get());

  auto groupAll = cd->createCollisionGroup(
        group1.get(), group2.get(), group3.get());

  EXPECT_EQ(group1->getNumShapeFrames(), 1u);
  EXPECT_EQ(group2->getNumShapeFrames(), 1u);
  EXPECT_EQ(group3->getNumShapeFrames(), 1u);
  EXPECT_EQ(groupAll->getNumShapeFrames(),
            group1->getNumShapeFrames()
            + group2->getNumShapeFrames()
            + group3->getNumShapeFrames());

  for(std::size_t i=0; i < group1->getNumShapeFrames(); ++i)
    EXPECT_EQ(groupAll->getShapeFrame(i), group1->getShapeFrame(i));

  std::size_t start = group1->getNumShapeFrames();
  std::size_t end = start + group2->getNumShapeFrames();
  for(std::size_t i=start; i < end; ++i)
    EXPECT_EQ(groupAll->getShapeFrame(i), group2->getShapeFrame(i-start));

  start = start + group2->getNumShapeFrames();
  end = start + group3->getNumShapeFrames();
  for(std::size_t i=start; i < end; ++i)
    EXPECT_EQ(groupAll->getShapeFrame(i), group3->getShapeFrame(i-start));

  collision::CollisionOption option;
  collision::CollisionResult result;

  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(1.1, 0.0, 0.0));
  simpleFrame3->setTranslation(Eigen::Vector3d(2.2, 0.0, 0.0));
  EXPECT_FALSE(group1->collide(option, &result));
  EXPECT_FALSE(group2->collide(option, &result));
  EXPECT_FALSE(group3->collide(option, &result));
  EXPECT_FALSE(groupAll->collide(option, &result));

  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(0.5, 0.0, 0.0));
  simpleFrame3->setTranslation(Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_TRUE(group1->collide(group2.get(), option, &result));
  EXPECT_TRUE(group1->collide(group2.get(), option, &result));
  EXPECT_TRUE(group2->collide(group3.get(), option, &result));
  EXPECT_TRUE(groupAll->collide(option, &result));

  auto group23 = cd->createCollisionGroup(
        simpleFrame2.get(), simpleFrame3.get());
  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(1.1, 0.0, 0.0));
  simpleFrame3->setTranslation(Eigen::Vector3d(1.6, 0.0, 0.0));
  EXPECT_FALSE(group1->collide(group2.get()));
  EXPECT_FALSE(group1->collide(group3.get()));
  EXPECT_TRUE(group2->collide(group3.get()));
  EXPECT_TRUE(group23->collide());
  if (cd->getType() == BulletCollisionDetector::getStaticType())
  {
    dtwarn << "Skipping group-group test for 'bullet' collision detector. "
           << "Please see Issue #717 for the detail.\n";
  }
  else
  {
    EXPECT_FALSE(group1->collide(group23.get()));
  }
}

//==============================================================================
TEST_F(COLLISION, SimpleFrames)
{
  auto fcl_mesh_dart = FCLCollisionDetector::create();
  fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
  testSimpleFrames(fcl_mesh_dart);

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testSimpleFrames(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testSimpleFrames(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testSimpleFrames(fcl_mesh_fcl);

#if HAVE_BULLET_COLLISION
  auto bullet = BulletCollisionDetector::create();
  testSimpleFrames(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testSimpleFrames(dart);
}

//==============================================================================
void testSphereSphere(const std::shared_ptr<CollisionDetector>& cd,
                      double tol = 1e-12)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new SphereShape(1.0));
  ShapePtr shape2(new SphereShape(0.5));
  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group1 = cd->createCollisionGroup(simpleFrame1.get());
  auto group2 = cd->createCollisionGroup(simpleFrame2.get());

  EXPECT_EQ(group1->getNumShapeFrames(), 1u);
  EXPECT_EQ(group2->getNumShapeFrames(), 1u);

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;

  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));
  result.clear();
  EXPECT_FALSE(group1->collide(group2.get(), option, &result));
  EXPECT_TRUE(result.getNumContacts() == 0u);

  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(1.5, 0.0, 0.0));
  result.clear();
  EXPECT_TRUE(group1->collide(group2.get(), option, &result));
  EXPECT_TRUE(result.getNumContacts() == 1u);
  EXPECT_TRUE(result.getContact(0).point.isApprox(
                Eigen::Vector3d(1.0, 0.0, 0.0), tol));
  if (cd->getType() == FCLCollisionDetector::getStaticType()
      && static_cast<FCLCollisionDetector*>(cd.get())->getPrimitiveShapeType()
         == FCLCollisionDetector::MESH)
  {
    EXPECT_TRUE(result.getContact(0).normal.isApprox(
                  -Eigen::Vector3d::UnitX(), tol * 1e+12));
    // FCL returns less accurate contact normals for sphere-sphere since we're
    // using sphere-like rough meshes instead of analytical sphere shapes.
  }
  else
  {
    EXPECT_TRUE(result.getContact(0).normal.isApprox(
                  -Eigen::Vector3d::UnitX(), tol));
  }

  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d::Zero());
  result.clear();
  if (cd->getType() == FCLCollisionDetector::getStaticType())
  {
    EXPECT_FALSE(group1->collide(group2.get(), option, &result));
    // FCL is not able to detect collisions when an object completely (strictly)
    // contanins the other object (no collisions between the hulls)
  }
  else
  {
    EXPECT_TRUE(group1->collide(group2.get(), option, &result));
    EXPECT_TRUE(result.getNumContacts() == 1u);
  }
  // The positions of contact point are different depending on the collision
  // detector. More comprehensive tests need to be added.
}

//==============================================================================
TEST_F(COLLISION, SphereSphere)
{
  auto fcl_mesh_dart = FCLCollisionDetector::create();
  fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
  testSphereSphere(fcl_mesh_dart);

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testSphereSphere(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testSphereSphere(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testSphereSphere(fcl_mesh_fcl);

#if HAVE_BULLET_COLLISION
  auto bullet = BulletCollisionDetector::create();
  testSphereSphere(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testSphereSphere(dart);
}

//==============================================================================
bool checkBoundingBox(const Eigen::Vector3d& min, const Eigen::Vector3d& max,
                      const Eigen::Vector3d& point, double tol = 1e-12)
{
  for (auto i = 0u; i < 3u; ++i)
  {
    if (min[i] - tol > point[i] || point[i] > max[i] + tol)
      return false;
  }

  return true;
}

//==============================================================================
void testBoxBox(const std::shared_ptr<CollisionDetector>& cd,
                double tol = 1e-12)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr shape2(new BoxShape(Eigen::Vector3d(0.5, 0.5, 0.5)));
  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  Eigen::Vector3d pos1 = Eigen::Vector3d(0.0, 0.0, -0.5);
  Eigen::Vector3d pos2 = Eigen::Vector3d(0.0, 0.5, 0.25);
  simpleFrame1->setTranslation(pos1);
  simpleFrame2->setTranslation(pos2);

  auto group1 = cd->createCollisionGroup(simpleFrame1.get());
  auto group2 = cd->createCollisionGroup(simpleFrame2.get());
  auto groupAll = cd->createCollisionGroup(group1.get(), group2.get());

  EXPECT_EQ(group1->getNumShapeFrames(), 1u);
  EXPECT_EQ(group2->getNumShapeFrames(), 1u);
  EXPECT_EQ(groupAll->getNumShapeFrames(),
            group1->getNumShapeFrames()
            + group2->getNumShapeFrames());

  collision::CollisionOption option;
  collision::CollisionResult result;

  EXPECT_TRUE(group1->collide(group2.get(), option, &result));

  Eigen::Vector3d min = Eigen::Vector3d(-0.25, 0.25, 0.0);
  Eigen::Vector3d max = Eigen::Vector3d(0.25, 0.5, 0.0);

  const auto numContacts = result.getNumContacts();

  const auto checkNumContacts = (numContacts <= 4u);
  EXPECT_TRUE(checkNumContacts);
  if (!checkNumContacts)
    std::cout << "# of contants: " << numContacts << "\n";

  for (const auto& contact : result.getContacts())
  {
    const auto& point = contact.point;

    const auto result = checkBoundingBox(min, max, point, tol);
    EXPECT_TRUE(result);

    if (!result)
      std::cout << "point: " << point.transpose() << "\n";
  }
}

//==============================================================================
TEST_F(COLLISION, BoxBox)
{
  auto fcl_mesh_dart = FCLCollisionDetector::create();
  fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
  testBoxBox(fcl_mesh_dart);

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testBoxBox(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testBoxBox(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testBoxBox(fcl_mesh_fcl);

#if HAVE_BULLET_COLLISION
  auto bullet = BulletCollisionDetector::create();
  testBoxBox(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testBoxBox(dart);
}

//==============================================================================
void testOptions(const std::shared_ptr<CollisionDetector>& cd)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame3 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr shape2(new BoxShape(Eigen::Vector3d(0.5, 0.5, 0.5)));
  ShapePtr shape3(new BoxShape(Eigen::Vector3d(0.5, 0.5, 0.5)));
  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);
  simpleFrame3->setShape(shape3);

  Eigen::Vector3d pos1 = Eigen::Vector3d(0.0, 0.0, -0.5);
  Eigen::Vector3d pos2 = Eigen::Vector3d(0.0, 0.5, 0.25);
  Eigen::Vector3d pos3 = Eigen::Vector3d(0.0, -0.5, 0.25);
  simpleFrame1->setTranslation(pos1);
  simpleFrame2->setTranslation(pos2);
  simpleFrame3->setTranslation(pos3);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());
  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  collision::CollisionOption option;
  collision::CollisionResult result;

  result.clear();
  option.maxNumContacts = 1000u;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 4u);

  result.clear();
  option.maxNumContacts = 2u;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 2u);

  group->addShapeFrame(simpleFrame3.get());
  result.clear();
  option.maxNumContacts = 1u;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 1u);

  // Binary check without passing result
  EXPECT_TRUE(group->collide(option));

  // Binary check without passing option and result
  EXPECT_TRUE(group->collide());

  // Zero maximum number of contacts
  option.maxNumContacts = 0u;
  option.enableContact = true;
  EXPECT_TRUE(group->collide());
  EXPECT_FALSE(group->collide(option));
  EXPECT_FALSE(group->collide(option, nullptr));
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);
  EXPECT_FALSE(result.isCollision());
}

//==============================================================================
TEST_F(COLLISION, Options)
{
  auto fcl_mesh_dart = FCLCollisionDetector::create();
  fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
  testOptions(fcl_mesh_dart);

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testOptions(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testOptions(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testOptions(fcl_mesh_fcl);

#if HAVE_BULLET_COLLISION
  auto bullet = BulletCollisionDetector::create();
  testOptions(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testOptions(dart);
}

//==============================================================================
void testFilter(const std::shared_ptr<CollisionDetector>& cd)
{
  // Create two bodies skeleton. The two bodies are placed at the same position
  // with the same size shape so that they collide by default.
  auto skel = Skeleton::create();
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(1, 1, 1));
  auto pair0 = skel->createJointAndBodyNodePair<RevoluteJoint>(nullptr);
  auto* body0 = pair0.second;
  body0->createShapeNodeWith<VisualAspect, CollisionAspect>(shape);
  auto pair1 = body0->createChildJointAndBodyNodePair<RevoluteJoint>();
  auto* body1 = pair1.second;
  body1->createShapeNodeWith<VisualAspect, CollisionAspect>(shape);

  // Create a collision group from the skeleton
  auto group = cd->createCollisionGroup(skel.get());
  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  // Default collision filter for Skeleton
  CollisionOption option;
  option.collisionFilter = std::make_shared<BodyNodeCollisionFilter>();

  skel->enableSelfCollisionCheck();
  skel->enableAdjacentBodyCheck();
  EXPECT_TRUE(skel->isEnabledSelfCollisionCheck());
  EXPECT_TRUE(skel->isEnabledAdjacentBodyCheck());
  EXPECT_TRUE(group->collide());  // without filter, always collision
  EXPECT_TRUE(group->collide(option));

  skel->enableSelfCollisionCheck();
  skel->disableAdjacentBodyCheck();
  EXPECT_TRUE(skel->isEnabledSelfCollisionCheck());
  EXPECT_FALSE(skel->isEnabledAdjacentBodyCheck());
  EXPECT_TRUE(group->collide());
  EXPECT_FALSE(group->collide(option));

  skel->disableSelfCollisionCheck();
  skel->enableAdjacentBodyCheck();
  EXPECT_FALSE(skel->isEnabledSelfCollisionCheck());
  EXPECT_TRUE(skel->isEnabledAdjacentBodyCheck());
  EXPECT_TRUE(group->collide());
  EXPECT_FALSE(group->collide(option));

  skel->disableSelfCollisionCheck();
  skel->disableAdjacentBodyCheck();
  EXPECT_FALSE(skel->isEnabledSelfCollisionCheck());
  EXPECT_FALSE(skel->isEnabledAdjacentBodyCheck());
  EXPECT_TRUE(group->collide());
  EXPECT_FALSE(group->collide(option));
}

//==============================================================================
TEST_F(COLLISION, Filter)
{
  auto fcl_mesh_dart = FCLCollisionDetector::create();
  fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
  testFilter(fcl_mesh_dart);

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testFilter(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testFilter(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testFilter(fcl_mesh_fcl);

#if HAVE_BULLET_COLLISION
  auto bullet = BulletCollisionDetector::create();
  testFilter(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testFilter(dart);
}

//==============================================================================
void testCreateCollisionGroups(const std::shared_ptr<CollisionDetector>& cd)
{
  Eigen::Vector3d size(1.0, 1.0, 1.0);
  Eigen::Vector3d pos1(0.0, 0.0, 0.0);
  Eigen::Vector3d pos2(0.5, 0.0, 0.0);

  auto boxSkeleton1 = createBox(size, pos1);
  auto boxSkeleton2 = createBox(size, pos2);

  auto boxBodyNode1 = boxSkeleton1->getBodyNode(0u);
  auto boxBodyNode2 = boxSkeleton2->getBodyNode(0u);

  auto boxShapeNode1 = boxBodyNode1->getShapeNodesWith<CollisionAspect>()[0];
  auto boxShapeNode2 = boxBodyNode2->getShapeNodesWith<CollisionAspect>()[0];

  collision::CollisionOption option;
  collision::CollisionResult result;

  auto skeletonGroup1 = cd->createCollisionGroup(boxSkeleton1.get());
  auto skeletonGroup2 = cd->createCollisionGroup(boxSkeleton2.get());

  auto bodyNodeGroup1 = cd->createCollisionGroup(boxBodyNode1);
  auto bodyNodeGroup2 = cd->createCollisionGroup(boxBodyNode2);

  auto shapeNodeGroup1 = cd->createCollisionGroup(boxShapeNode1);
  auto shapeNodeGroup2 = cd->createCollisionGroup(boxShapeNode2);

  EXPECT_TRUE(skeletonGroup1->collide(skeletonGroup2.get(), option, &result));
  EXPECT_TRUE(bodyNodeGroup1->collide(bodyNodeGroup2.get(), option, &result));
  EXPECT_TRUE(shapeNodeGroup1->collide(shapeNodeGroup2.get(), option, &result));

  // Binary check without passing option
  auto oldMaxNumContacts = option.maxNumContacts;
  option.maxNumContacts = 1u;
  EXPECT_TRUE(skeletonGroup1->collide(skeletonGroup2.get(), option));
  EXPECT_TRUE(bodyNodeGroup1->collide(bodyNodeGroup2.get(), option));
  EXPECT_TRUE(shapeNodeGroup1->collide(shapeNodeGroup2.get(), option));
  option.maxNumContacts = oldMaxNumContacts;

  // Binary check without passing option and result
  EXPECT_TRUE(skeletonGroup1->collide(skeletonGroup2.get()));
  EXPECT_TRUE(bodyNodeGroup1->collide(bodyNodeGroup2.get()));
  EXPECT_TRUE(shapeNodeGroup1->collide(shapeNodeGroup2.get()));

  // Regression test for #666
  auto world = common::make_unique<World>();
  world->getConstraintSolver()->setCollisionDetector(cd);
  world->addSkeleton(boxSkeleton1);
  world->addSkeleton(boxSkeleton2);
DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_FALSE(boxBodyNode1->isColliding());
  EXPECT_FALSE(boxBodyNode2->isColliding());
DART_SUPPRESS_DEPRECATED_END

  const collision::CollisionResult& result1 = world->getLastCollisionResult();
  EXPECT_FALSE(result1.inCollision(boxBodyNode1));
  EXPECT_FALSE(result1.inCollision(boxBodyNode2));

  world->step();
DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_TRUE(boxBodyNode1->isColliding());
  EXPECT_TRUE(boxBodyNode2->isColliding());
DART_SUPPRESS_DEPRECATED_END

  const collision::CollisionResult& result2 = world->getLastCollisionResult();
  EXPECT_TRUE(result2.inCollision(boxBodyNode1));
  EXPECT_TRUE(result2.inCollision(boxBodyNode2));
}

//==============================================================================
TEST_F(COLLISION, CreateCollisionGroupFromVariousObject)
{
  auto fcl_mesh_dart = FCLCollisionDetector::create();
  fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
  testCreateCollisionGroups(fcl_mesh_dart);

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testCreateCollisionGroups(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testCreateCollisionGroups(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testCreateCollisionGroups(fcl_mesh_fcl);

#if HAVE_BULLET_COLLISION
  auto bullet = BulletCollisionDetector::create();
  testCreateCollisionGroups(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testCreateCollisionGroups(dart);
}

//==============================================================================
TEST_F(COLLISION, CollisionOfPrescribedJoints)
{
  // There are one red plate (static skeleton) and 5 pendulums with different
  // actuator types. This test check if the motion prescribed joints are exactly
  // tracking the prescribed motion eventhough there are collision with other
  // objects.

  const double tol       = 1e-9;
  const double timeStep  = 1e-3;
  const std::size_t numFrames = 5e+0;  // 5 secs

  // Load world and skeleton
  WorldPtr world = SkelParser::readWorld(
        "dart://sample/skel/test/collision_of_prescribed_joints_test.skel");
  world->setTimeStep(timeStep);
  EXPECT_TRUE(world != nullptr);
  EXPECT_NEAR(world->getTimeStep(), timeStep, tol);

  SkeletonPtr skel1 = world->getSkeleton("skeleton 1");
  SkeletonPtr skel2 = world->getSkeleton("skeleton 2");
  SkeletonPtr skel3 = world->getSkeleton("skeleton 3");
  SkeletonPtr skel4 = world->getSkeleton("skeleton 4");
  SkeletonPtr skel5 = world->getSkeleton("skeleton 5");
  SkeletonPtr skel6 = world->getSkeleton("skeleton 6");
  EXPECT_TRUE(skel1 != nullptr);
  EXPECT_TRUE(skel2 != nullptr);
  EXPECT_TRUE(skel3 != nullptr);
  EXPECT_TRUE(skel4 != nullptr);
  EXPECT_TRUE(skel5 != nullptr);
  EXPECT_TRUE(skel6 != nullptr);

  Joint* joint1 = skel1->getJoint(0);
  Joint* joint2 = skel2->getJoint(0);
  Joint* joint3 = skel3->getJoint(0);
  Joint* joint4 = skel4->getJoint(0);
  Joint* joint5 = skel5->getJoint(0);
  Joint* joint6 = skel6->getJoint(0);
  EXPECT_TRUE(joint1 != nullptr);
  EXPECT_TRUE(joint2 != nullptr);
  EXPECT_TRUE(joint3 != nullptr);
  EXPECT_TRUE(joint4 != nullptr);
  EXPECT_TRUE(joint5 != nullptr);
  EXPECT_TRUE(joint6 != nullptr);
  EXPECT_EQ(joint1->getActuatorType(), Joint::FORCE);
  EXPECT_EQ(joint2->getActuatorType(), Joint::PASSIVE);
  EXPECT_EQ(joint3->getActuatorType(), Joint::SERVO);
  EXPECT_EQ(joint4->getActuatorType(), Joint::ACCELERATION);
  EXPECT_EQ(joint5->getActuatorType(), Joint::VELOCITY);
  EXPECT_EQ(joint6->getActuatorType(), Joint::LOCKED);

  for (std::size_t i = 0; i < numFrames; ++i)
  {
    const double time = world->getTime();

    joint1->setCommand(0, -0.5 * constantsd::pi() * std::cos(time));
    joint2->setCommand(0, -0.5 * constantsd::pi() * std::cos(time));
    joint3->setCommand(0, -0.5 * constantsd::pi() * std::cos(time));
    joint4->setCommand(0, -0.5 * constantsd::pi() * std::cos(time));
    joint5->setCommand(0, -0.5 * constantsd::pi() * std::sin(time));
    joint6->setCommand(0, -0.5 * constantsd::pi() * std::sin(time));  // ignored

    world->step(false);

    EXPECT_TRUE(joint1->isDynamic());
    EXPECT_TRUE(joint2->isDynamic());
    EXPECT_TRUE(joint3->isDynamic());

    // Check if the motion prescribed joints are following the prescribed motion
    // eventhough there is a collision with other objects
    EXPECT_TRUE(joint4->isKinematic());
    EXPECT_NEAR(joint4->getAcceleration(0), joint4->getCommand(0), tol);
    EXPECT_TRUE(joint5->isKinematic());
    EXPECT_NEAR(joint5->getVelocity(0), joint5->getCommand(0), tol);

    // The velocity and acceleration of locked joint always must be zero.
    EXPECT_TRUE(joint6->isKinematic());
    EXPECT_NEAR(joint6->getVelocity(0), 0.0, tol);
    EXPECT_NEAR(joint6->getAcceleration(0), 0.0, tol);
  }
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

