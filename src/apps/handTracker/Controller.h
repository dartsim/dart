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
    Controller(dynamics::SkeletonDynamics *_skel, double _t);
    virtual ~Controller() {};

    //    Eigen::VectorXd getTorques() { return mTorques; };
    //    double getTorque(int _index) { return mTorques[_index]; };
    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; };
    void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
    Eigen::VectorXd getDesiredDofs() { return mDesiredDofs; };
    Eigen::MatrixXd getKp() {return mKp; };
    Eigen::MatrixXd getKd() {return mKd; };
 
 protected:
    //    double computeMassTree(kinematics::BodyNode *_bd);
    dynamics::SkeletonDynamics *mSkel;
    //    Eigen::VectorXd mTorques;
    Eigen::VectorXd mDesiredDofs;
    //    std::vector<double> mKs;
    //    std::vector<double> mKd;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd;
    double mTimestep;

    //    Eigen::VectorXd mMassTree;
};
    
    

#endif // #CONTROLLER_H
