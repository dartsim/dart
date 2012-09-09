#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <vector>

namespace dynamics{
    class SkeletonDynamics;
}

namespace kinematics{
    class FileInfoDof;
}

class Controller {
 public:
    Controller(kinematics::FileInfoDof *_motion, dynamics::SkeletonDynamics *_skel, Eigen::Vector3d _grav);
    virtual ~Controller() {};

    Eigen::VectorXd& getTorques(int _frame);
    void computeTorques();
    dynamics::SkeletonDynamics* getSkel() { return mSkel; };

 protected: 
    dynamics::SkeletonDynamics *mSkel;
    kinematics::FileInfoDof *mMotion;
    std::vector<Eigen::VectorXd> mTorques;
    
    Eigen::Vector3d mGravity;
};
    
    

#endif // #CONTROLLER_H
