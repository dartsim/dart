#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#define SPD

namespace dynamics{
    class SkeletonDynamics;
}

namespace kinematics{
    class FileInfoDof;
}

class Controller {
 public:
    Controller(kinematics::FileInfoDof *_motion, dynamics::SkeletonDynamics *_skel, double _t);
    virtual ~Controller() {};

    Eigen::VectorXd getTorques() { return mTorques; };
    void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
    dynamics::SkeletonDynamics* getSkel() { return mSkel; };
    Eigen::VectorXd getDesiredDofs() { return mDesiredDofs; };
    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; };

 protected: 
    kinematics::FileInfoDof *mMotion;
    dynamics::SkeletonDynamics *mSkel;
    Eigen::VectorXd mTorques;
    Eigen::VectorXd mDesiredDofs;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd;
    double mTimestep;
    int mSimFrame;
    int mInterval;
};
    
    

#endif // #CONTROLLER_H
