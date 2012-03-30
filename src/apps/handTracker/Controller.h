#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <vector>
namespace dynamics{
    class SkeletonDynamics;
}
namespace kinematics{
    class BodyNode;
}

class Controller {
 public:
    Controller(dynamics::SkeletonDynamics *_skel);
    virtual ~Controller() {};

    Eigen::VectorXd getTorques() { return mTorques; };
    double getTorque(int _index) { return mTorques[_index]; };
    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; };
    void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
 
 protected:
    double computeMassTree(kinematics::BodyNode *_bd);
    dynamics::SkeletonDynamics *mSkel;
    Eigen::VectorXd mTorques;
    Eigen::VectorXd mDesiredDofs;
    std::vector<double> mKs;
    std::vector<double> mKd;
    Eigen::VectorXd mMassTree;
};
    
    

#endif // #CONTROLLER_H
