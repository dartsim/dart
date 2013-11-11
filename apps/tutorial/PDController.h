#ifndef PDCONTROLLER_H
#define PDCONTROLLER_H

#include "Controller.h"

class PDController : public Controller {
 public:
 PDController(dynamics::SkeletonDynamics* _skel);
    virtual ~PDController() {};

    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; };
    virtual void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);

 private:
    Eigen::VectorXd mDesiredDofs;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd;
};

#endif // #PDCONTROLLER_H
