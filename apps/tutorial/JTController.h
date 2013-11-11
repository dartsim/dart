#ifndef JTCONTROLLER_H
#define JTCONTROLLER_H

#include "Controller.h"

namespace kinematics {
    class BodyNode;
}

class JTController : public Controller {
 public:
 JTController(dynamics::SkeletonDynamics* _skel);
    virtual ~JTController() {};

    void setVirtualForce(Eigen::Vector3d _f) { mVirtualForce = _f; };
    void setLocalPointOfApp(Eigen::Vector3d _p) { mLocalPoint = _p; };
    virtual void computeTorques();

 private:
    Eigen::Vector3d mVirtualForce;
    Eigen::Vector3d mLocalPoint;
    kinematics::BodyNode* mBodyNode;
};

#endif // #JTCONTROLLER_H
