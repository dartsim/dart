#ifndef ANALYZER_H
#define ANALYZER_H

#include <Eigen/Dense>
#include <vector>

namespace dynamics{
    class SkeletonDynamics;
}

namespace kinematics{
    class FileInfoDof;
}

class Analyzer {
 public:
    Analyzer(kinematics::FileInfoDof *_motion, dynamics::SkeletonDynamics *_skel, Eigen::Vector3d _grav);
    virtual ~Analyzer() {};

    void computeTorques();
    void analyze();
    dynamics::SkeletonDynamics* getSkel() { return mSkel; };

 protected: 
    Eigen::Vector3d evalLinMomentum(const Eigen::VectorXd& _dofVel);
    Eigen::Vector3d evalAngMomentum(const Eigen::VectorXd& _dofVel);

    dynamics::SkeletonDynamics *mSkel;
    kinematics::FileInfoDof *mMotion;
    std::vector<Eigen::VectorXd> mTorques;
    std::vector<Eigen::Vector3d> mLinMomentum;
    std::vector<Eigen::Vector3d> mAngMomentum;
    
    Eigen::Vector3d mGravity;
};
    
    

#endif // #ANALYZER_H
