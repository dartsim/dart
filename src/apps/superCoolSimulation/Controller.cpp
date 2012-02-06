#include "Controller.h"

void Controller::computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel) {
    for (unsigned int i = 0; i < mTorques.size(); i++) {
        mTorques[i] = -mKs[i] * (_dof[i] - mDesiredDofs[i])  -mKd[i] * _dofVel[i];
    }
}
