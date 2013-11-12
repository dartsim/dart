#ifndef JTCONTROLLER_H
#define JTCONTROLLER_H

#include "Controller.h"

namespace kinematics {
    class BodyNode;
}

class JTController : public Controller {
 public:
    JTController(dynamics::SkeletonDynamics* _skel, kinematics::BodyNode* _body);
    virtual ~JTController() {};

    void setVirtualForce(Eigen::Vector3d _f) { mVirtualForce = _f; };
    void setLocalPointOfApp(Eigen::Vector3d _p) { mLocalPoint = _p; };
    virtual void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);

 private:
    Eigen::Vector3d mVirtualForce;
    Eigen::Vector3d mLocalPoint;
    kinematics::BodyNode* mBodyNode;
    int mFrame;
};

#endif // #JTCONTROLLER_H
