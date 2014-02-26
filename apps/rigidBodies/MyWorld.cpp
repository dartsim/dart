#include "MyWorld.h"
#include "RigidBody.h"
#include "CollisionInterface.h"
#include "utils/urdf/DartLoader.h"
#include "utils/Paths.h"
#include "dynamics/Skeleton.h"
#include "dynamics/EllipsoidShape.h"
#include "dynamics/BoxShape.h"
#include "dynamics/BodyNode.h"
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include "dynamics/MeshShape.h"

using namespace Eigen;

MyWorld::MyWorld() {
    mFrame = 0;

    // Create a collision detector
    mCollisionDetector = new CollisionInterface();

    // Add rigid bodies (this will be replaced by your code) 
    RigidBody *rb1 = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.9, 0.9, 0.9));
    mCollisionDetector->addRigidBody(rb1, "box"); // Put rb1 in collision detector
    rb1->mPosition[0] = -1.0;
    mRigidBodies.push_back(rb1);
    
    RigidBody *rb2 = new RigidBody(dart::dynamics::Shape::ELLIPSOID, Vector3d(0.9, 1.8, 0.9));
    mCollisionDetector->addRigidBody(rb2, "ellipse"); // Put rb2 in collision detector
    rb2->mPosition[0] = 1.0;
    rb2->mColor = Vector4d(0.2, 0.8, 0.2, 1.0); // Blue
    mRigidBodies.push_back(rb2);

    // Load a blender and a blade
    dart::utils::DartLoader dl;
    std::string blenderFileName(DART_DATA_PATH"urdf/cylinder.urdf");
    mBlender = dl.parseSkeleton(blenderFileName);
    if (mBlender) {
      mBlender->init();
    }
    mCollisionDetector->addSkeleton(mBlender); // Put blender in collision detector

    //Make the blender a WIREFRAME -- this is a hack as URDF doesn't have material properties signifying wireframes to
    //allow assimp to handle this autmatically
    dart::dynamics::MeshShape* shape = (dart::dynamics::MeshShape*)mBlender->getBodyNode(0)->getVisualizationShape(0);
    bool wireFrame = true;
    shape->getMesh()->mMaterials[0]->AddProperty(&wireFrame, 1, AI_MATKEY_ENABLE_WIREFRAME);


    std::string bladeFileName(DART_DATA_PATH"urdf/blade.urdf");
    mBlade = dl.parseSkeleton(bladeFileName);
    if (mBlade) {
      mBlade->init();
    }

    VectorXd pose = mBlade->getState();
    pose[4] = -4.0;
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
    mFrame++;
    // TODO: Replace the following code
    for (int i = 0; i < mRigidBodies.size(); i++)
        mRigidBodies[i]->mPosition += (mRigidBodies[i]->mMomentum / mRigidBodies[i]->mMass);

    // Run collision detector
    mCollisionDetector->checkCollision();

    // Move the blade
    VectorXd pose = mBlade->getState();
    pose[1] += 0.01;
    if (pose[1] > 2 * 3.14)
    {
        pose[1] = 0.0;
    }
    mBlade->setState(pose);
}
