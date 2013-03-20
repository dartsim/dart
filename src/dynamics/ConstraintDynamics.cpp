
#include "ConstraintDynamics.h"

#include "kinematics/BodyNode.h"
#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"

using namespace Eigen;
using namespace utils;

namespace dynamics {
    ConstraintDynamics::ConstraintDynamics(SkeletonDynamics *_skel) {
        mSkel = _skel;
    }

    ConstraintDynamics::~ConstraintDynamics() {
    }

    void ConstraintDynamics::addConstraint(Constraint *_constr) {
        mConstraints.push_back(_constr);
        int nConstr = mConstraints.size();
        mJGlobal = MatrixXd::Zero(3 * nConstr, mSkel->getNumDofs());
        mJDotGlobal = MatrixXd::Zero(3 * nConstr, mSkel->getNumDofs());
        mCGlobal = VectorXd(3 * nConstr);
        mCDotGlobal = VectorXd(3 * nConstr);
    }

    void ConstraintDynamics::deleteConstraint(int _index) {
        delete mConstraints[_index];
        mConstraints.erase(mConstraints.begin() + _index);
        int nConstr = mConstraints.size();
        mJGlobal = MatrixXd::Zero(3 * nConstr, mSkel->getNumDofs());
        mJDotGlobal = MatrixXd::Zero(3 * nConstr, mSkel->getNumDofs());
        mCGlobal = VectorXd(3 * nConstr);
        mCDotGlobal = VectorXd(3 * nConstr);
    }            

    void ConstraintDynamics::applyConstraintForces(VectorXd& _qddot) {
        double ks = 100;
        double kd = 10;
        int nConstrs = mConstraints.size();
        int nDofs = mSkel->getNumDofs();
        VectorXd qDot = mSkel->getQDotVector();
        /*            MatrixXd J(MatrixXd::Zero(3 * nConstrs, nDofs));
                      MatrixXd JDot(MatrixXd::Zero(3 * nConstrs, nDofs));
                      VectorXd C(3 * nConstrs);
                      VectorXd CDot(3 * nConstrs);
        */
        for (int i = 0; i < nConstrs; i++) {
            mConstraints[i]->updateDynamics();
            mJGlobal.block(i * 3, 0, 3, nDofs) = mConstraints[i]->getJ();
            mJDotGlobal.block(i * 3, 0, 3, nDofs) = mConstraints[i]->getJDot();
            mCGlobal.segment<3>(i * 3) = mConstraints[i]->getC();
            mCDotGlobal.segment<3>(i * 3) = mConstraints[i]->getCDot();
        }
            
        MatrixXd A = mJGlobal * mSkel->getInvMassMatrix() * mJGlobal.transpose();
        VectorXd b = -mJDotGlobal * qDot - mJGlobal * _qddot - mCGlobal * ks - mCDotGlobal * kd;
        VectorXd lambda = A.inverse() * b;
        mConstrForce.noalias() = mJGlobal.transpose() * lambda;
    }
} // namespace dynamics

