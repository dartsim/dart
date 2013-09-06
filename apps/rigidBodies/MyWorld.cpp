#include "MyWorld.h"
#include "RigidBody.h"
#include "CollisionInterface.h"
#include <iostream>
#include "kinematics/FileInfoSkel.hpp"
#include "robotics/parser/dart_parser/DartLoader.h"
#include "utils/Paths.h"
#include "dynamics/SkeletonDynamics.h"

using namespace Eigen;

MyWorld::MyWorld() {
    // Create a collision detector
    mCollisionDetector = new CollisionInterface();

    mBladeAngle = 0.0;
    // Load a blender and a blade
    DartLoader dl;
    string blenderFileName(DART_DATA_PATH"urdf/cylinder.urdf");
    mBlender = dl.parseSkeleton(blenderFileName);
    mCollisionDetector->addSkeleton(mBlender);

    string bladeFileName(DART_DATA_PATH"urdf/blade.urdf");
    mBlade = dl.parseSkeleton(bladeFileName);
    mCollisionDetector->addSkeleton(mBlade);

    // Add rigid bodies
    RigidBody *rb1 = new RigidBody(kinematics::Shape::P_BOX, Vector3d(0.1, 0.1, 0.1));
    mCollisionDetector->addRigidBody(rb1);
    mRigidBodies.push_back(rb1);
    
    RigidBody *rb2 = new RigidBody(kinematics::Shape::P_BOX, Vector3d(0.1, 0.2, 0.1));
    mCollisionDetector->addRigidBody(rb2);
    rb2->mPosition[0] = 0.1;
    rb2->mColor = Vector4d(0.2, 0.2, 0.8, 1.0); // Blue
    mRigidBodies.push_back(rb2);
    
}

MyWorld::~MyWorld() {
    for (int i = 0; i < mRigidBodies.size(); i++)
        delete mRigidBodies[i];
    mRigidBodies.clear();
    if (mCollisionDetector)
        delete mCollisionDetector;
    if (mBlender)
        delete mBlender;
    if (mBlade)
        delete mBlade;
}

void MyWorld::simulate() {
    // TODO: Replace the following code
    for (int i = 0; i < mRigidBodies.size(); i++)
        mRigidBodies[i]->mPosition[1] -= 0.005;

    // Compute colliding points
    mCollisionDetector->checkCollision();
    int nContacts = mCollisionDetector->getNumContacts();
    //    std::cout << nContacts << " contact points are found" << std::endl;
    for (int i = 0; i < nContacts; i++) 
        std::cout << mCollisionDetector->getContact(i).point << std::endl;

    // advance the blade angle
    mBladeAngle += 1.0;
    if (mBladeAngle > 360.0)
        mBladeAngle = 0.0;
}
