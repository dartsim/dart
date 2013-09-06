#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>

namespace kinematics {
    class Skeleton;
}

class RigidBody;
class CollisionInterface;

class MyWorld {
 public:
    MyWorld();

    virtual ~MyWorld();

    int getNumRigidBodies() {
        return mRigidBodies.size();
    }

    RigidBody* getRigidBody(int _index) {
        return mRigidBodies[_index];
    }
   
    // TODO: your simulation code goes here
    void simulate();
   
    kinematics::Skeleton* getBlender() {
        return mBlender;
    }

    kinematics::Skeleton* getBlade() {
        return mBlade;
    }
    
    double getBladeAngle() {
        return mBladeAngle;
    }

 protected:
    std::vector<RigidBody*> mRigidBodies;
    CollisionInterface* mCollisionDetector;
    
    double mBladeAngle;
    kinematics::Skeleton* mBlender;
    kinematics::Skeleton* mBlade;
};

#endif
