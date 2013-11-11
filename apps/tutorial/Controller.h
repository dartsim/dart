#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "dynamics/SkeletonDynamics.h"
#include <Eigen/Dense>

class Controller {
 public:
    Controller(dynamics::SkeletonDynamics* _skel) {
        mSkel = _skel;
        int nDof = mSkel->getNumDofs();
        mTorques = Eigen::VectorXd::Zero(nDof);
    };
    virtual ~Controller() {};

    Eigen::VectorXd getTorques() { return mTorques; };
    double getTorque(int _index) { return mTorques[_index]; };

    virtual void computeTorques(){};
    virtual void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel){};

 protected:
    dynamics::SkeletonDynamics* mSkel;
    Eigen::VectorXd mTorques;
};

#endif // #CONTROLLER_H
