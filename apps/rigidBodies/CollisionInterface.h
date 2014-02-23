#ifndef _COLLISIONINTERFACE_
#define _COLLISIONINTERFACE_

#include <vector>
#include <map>
#include <Eigen/Dense>

namespace kinematics {
    class BodyNode;
    class Skeleton;
}

namespace collision {
    class CollisionDetector;
}

class RigidBody;

struct RigidContact {
    Eigen::Vector3d point;
    Eigen::Vector3d normal;
    RigidBody* rb1;
    RigidBody* rb2;
};

class CollisionInterface {
 public:
    CollisionInterface();
    virtual ~CollisionInterface();

    void addSkeleton(kinematics::Skeleton* _skel);
    void addRigidBody(RigidBody *_rb);

    // Run the collision detector
    void checkCollision();

    int getNumContacts() {
        return mContacts.size();
    }

    // Retrieve the information from the collision detector:
    // For example, get the position and the normal of the fifth contact point
    // Vector3d  v = mWorld->getCollisionDetector()->getContact(5).point;
    // Vector3d n = mWorld->getCollisionDetector()->getContact(k).normal;
    RigidContact& getContact(int _index) {
        return mContacts[_index];
    }

 private:
    void updateBodyNodes();
    void postProcess();

    collision::CollisionDetector* mCollisionChecker;   
    std::vector<RigidContact> mContacts;
    std::map<kinematics::BodyNode*, RigidBody*> mNodeMap;
};

#endif
