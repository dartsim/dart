#ifndef _COLLISIONINTERFACE_
#define _COLLISIONINTERFACE_

#include <vector>
#include <map>
#include <Eigen/Dense>

namespace dart {
  namespace dynamics {
    class BodyNode;
    class Skeleton;
  }
}

namespace dart {
  namespace collision {
    class CollisionDetector;
  }
}

class RigidBody;

struct RigidContact {
  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  RigidBody* rb1;
  RigidBody* rb2;
  Eigen::Vector3d pinataVelocity;
};

class CollisionInterface {
 public:
  CollisionInterface();
  virtual ~CollisionInterface();

  void addSkeleton(dart::dynamics::Skeleton* _skel);
  void addRigidBody(RigidBody *_rb, const std::string& name);

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

  dart::collision::CollisionDetector* mCollisionChecker;
  std::vector<RigidContact> mContacts;
  std::vector<dart::dynamics::Skeleton*> mSkeletons;
  std::map<dart::dynamics::BodyNode*, RigidBody*> mNodeMap;
};

#endif
