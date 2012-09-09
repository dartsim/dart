#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoDof.h"

using namespace Eigen;

Controller::Controller(kinematics::FileInfoDof *_motion, dynamics::SkeletonDynamics *_skel, Vector3d _grav) {
    mMotion = _motion;
    mSkel = _skel;
    mGravity = _grav;
    int nDof = mSkel->getNumDofs();
    int nFrame = mMotion->getNumFrames();
    mTorques.resize(nFrame);
    for (int i = 0; i < nFrame; i++) {
        mTorques[i].resize(nDof);
        mTorques[i].setZero();
    }
}

void Controller::computeTorques() {
    int nFrame = mMotion->getNumFrames();
    double timestep = 1.0 / mMotion->getFPS();
    for (int i = 0; i < nFrame - 2; i++) {
        VectorXd qdot = (mMotion->getPoseAtFrame(i + 1) - mMotion->getPoseAtFrame(i)) / timestep;

        VectorXd qddot = (mMotion->getPoseAtFrame(i + 2) - 2 * mMotion->getPoseAtFrame(i + 1)+ mMotion->getPoseAtFrame(i)) / (timestep * timestep);
        mSkel->setPose(mMotion->getPoseAtFrame(i), true, false);
        mTorques[i] = mSkel->computeInverseDynamicsLinear(mGravity, &qdot, &qddot, false, false);
    }
}

Eigen::VectorXd& Controller::getTorques(int _frame) {
    if (_frame < mTorques.size())
        return mTorques[_frame];
    else
        return mTorques[0];
}
