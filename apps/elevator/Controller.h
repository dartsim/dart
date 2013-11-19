#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <vector>

namespace dynamics{
    class SkeletonDynamics;
    class ConstraintDynamics;
}

namespace kinematics{
    class BodyNode;
}

class Controller {
 public:
    Controller(dynamics::SkeletonDynamics* _skel, dynamics::ConstraintDynamics* _constr, double _t);
    virtual ~Controller() {};

    Eigen::VectorXd getTorques() { return mTorques; };
    double getTorque(int _index) { return mTorques[_index]; };
    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; };
    void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
    dynamics::SkeletonDynamics* getSkel() { return mSkel; };
    Eigen::VectorXd getDesiredDofs() { return mDesiredDofs; };
    Eigen::MatrixXd getKp() {return mKp; };
    Eigen::MatrixXd getKd() {return mKd; };


 private:
    // In case you want to use climbing strategy
    void leftHandGrab();
    void leftHandRelease();
    void rightHandGrab();
    void rightHandRelease();

    dynamics::SkeletonDynamics* mSkel;
    dynamics::ConstraintDynamics* mConstraintHandle; 

    Eigen::VectorXd mTorques;
    Eigen::VectorXd mDesiredDofs;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd;
    int mFrame;
    double mTimestep;
    bool mIsLHGrab;
    bool mIsRHGrab;
};

#endif // #CONTROLLER_H
