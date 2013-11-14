#ifndef SWINGCONTROLLER_H
#define SWINGCONTROLLER_H

#include "Controller.h"

namespace dynamics {
    class ConstraintDynamics;
}

class SwingController : public Controller {
 public:
    SwingController(dynamics::SkeletonDynamics* _skel, dynamics::ConstraintDynamics* _constraintHandle, double _timestep);
    virtual ~SwingController() {};

    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; };
    virtual void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);

 private:
    dynamics::ConstraintDynamics* mConstraintHandle;
    double mTimestep;
    Eigen::VectorXd mDesiredDofs;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd;
    int mFrame;
};

#endif // #JUMPCONTROLLER_H
