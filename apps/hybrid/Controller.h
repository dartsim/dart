#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

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
    void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
    dynamics::SkeletonDynamics* getSkel() { return mSkel; };
    Eigen::VectorXd getDesiredDofs() { return mDesiredDofs; };
    Eigen::MatrixXd getKp() {return mKp; };

 protected: 
    dynamics::SkeletonDynamics *mSkel;
    Eigen::VectorXd mTorques;
    Eigen::VectorXd mDesiredDofs;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd;
    int mFrame;
};
    
    

#endif // #CONTROLLER_H
