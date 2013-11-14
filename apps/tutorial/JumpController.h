#ifndef JUMPCONTROLLER_H
#define JUMPCONTROLLER_H

#include "Controller.h"

class JumpController : public Controller {
 public:
    JumpController(dynamics::SkeletonDynamics* _skel, double _timestep);
    virtual ~JumpController() {};

    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; };
    virtual void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);

 private:
    double mTimestep;
    Eigen::VectorXd mDesiredDofs;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd;
    int mFrame;
};

#endif // #JUMPCONTROLLER_H
