#include "MyWorld.h"
#include "RigidBody.h"
#include "CollisionInterface.h"
#include "utils/urdf/DartLoader.h"
#include "utils/Paths.h"
#include "dynamics/Skeleton.h"
#include "dynamics/EllipsoidShape.h"
#include "dynamics/BoxShape.h"
#include "utils/FileInfoDof.h"

using namespace Eigen;

MyWorld::MyWorld() {
    mFrame = 0;

    // Create a collision detector
    mCollisionDetector = new CollisionInterface();

    // Add rigid bodies (this will be replaced by your code) 
	RigidBody *rb1 = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.1, 0.1, 0.1));
    mCollisionDetector->addRigidBody(rb1); // Put rb1 in collision detector
    mRigidBodies.push_back(rb1);
    
	RigidBody *rb2 = new RigidBody(dart::dynamics::Shape::ELLIPSOID, Vector3d(0.1, 0.2, 0.1));
    mCollisionDetector->addRigidBody(rb2); // Put rb2 in collision detector
    rb2->mPosition[0] = 0.1;
    rb2->mColor = Vector4d(0.2, 0.8, 0.2, 1.0); // Blue
    mRigidBodies.push_back(rb2);

    // Load a blender and a blade
	dart::utils::DartLoader dl;
	std::string blenderFileName(DART_DATA_PATH"urdf/cylinder.urdf");
    mBlender = dl.parseSkeleton(blenderFileName);
    mCollisionDetector->addSkeleton(mBlender); // Put blender in collision detector

	dart::utils::FileInfoDof dofFile(mBlender);
    /* LOG(INFO) << "# frames = " << dofFile.getNumFrames(); */

	std::string bladeFileName(DART_DATA_PATH"urdf/blade.urdf");
    mBlade = dl.parseSkeleton(bladeFileName);
	VectorXd pose = mBlade->getState();
    pose[1] = -0.3;
	mBlade->setState(pose);
    mCollisionDetector->addSkeleton(mBlade); // Put blade in collision detector
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

    // Run collision detector
    mCollisionDetector->checkCollision();

    // Move the blade
    VectorXd pose = mBlade->getConfig();
    pose[4] += 0.01;
    if (pose[4] > 2 * 3.14)
        pose[4] = 0.0;
    mBlade->setConfig(pose);
	//true, false
}
