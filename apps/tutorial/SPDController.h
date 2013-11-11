#ifndef SPDCONTROLLER_H
#define SPDCONTROLLER_H

#include "Controller.h"

class SPDController : public Controller {
 public:
 SPDController(dynamics::SkeletonDynamics* _skel, double _timeStep);
    virtual ~SPDController() {};

    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; };
    virtual void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);

 private:
    double mTimestep;
    Eigen::VectorXd mDesiredDofs;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd;
};

#endif // #SPDCONTROLLER_H
