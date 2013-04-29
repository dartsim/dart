#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>

namespace dynamics{
    class SkeletonDynamics;
}

class Controller {
 public:
    Controller(dynamics::SkeletonDynamics *_skel);
    virtual ~Controller() {};

    Eigen::VectorXd getTorques() { return mTorques; };
    double getTorque(int _index) { return mTorques[_index]; };
    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; };
    dynamics::SkeletonDynamics* getSkel() { return mSkel; };
    Eigen::VectorXd getDesiredDofs() { return mDesiredDofs; };
    Eigen::VectorXd getKp() {return mKp; };
    Eigen::VectorXd getKd() {return mKd; };

    void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);

 protected:
    dynamics::SkeletonDynamics *mSkel;
    Eigen::VectorXd mTorques;
    Eigen::VectorXd mDesiredDofs;
    Eigen::VectorXd mKp;
    Eigen::VectorXd mKd;
};

#endif // #CONTROLLER_H
