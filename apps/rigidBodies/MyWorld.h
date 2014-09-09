#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include <Eigen/Dense>

namespace dart{
    namespace dynamics {
        class Skeleton;
    }
}

class RigidBody;
class CollisionInterface;

class MyWorld {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MyWorld();

    virtual ~MyWorld();

    int getNumRigidBodies() {
        return mRigidBodies.size();
    }

    RigidBody* getRigidBody(int _index) {
        return mRigidBodies[_index];
    }
   
    // TODO: your simulation and collision handling code goes here
    void simulate();
    void collisionHandling();

    dart::dynamics::Skeleton* getBlender() {
        return mBlender;
    }

    dart::dynamics::Skeleton* getBlade() {
        return mBlade;
    }
    
    
    CollisionInterface* getCollisionDetector() {
        return mCollisionDetector;
    }

    int getSimFrames() const { 
        return mFrame; 
    }

 protected:
    int mFrame;
    double mTimeStep;
    Eigen::Vector3d mGravity;
    std::vector<RigidBody*> mRigidBodies;
    CollisionInterface* mCollisionDetector; // Access to collision detection information
    dart::dynamics::Skeleton* mBlender;
    dart::dynamics::Skeleton* mBlade;
};

#endif
