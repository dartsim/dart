
#include "ClosedLoopConstraint.h"

#include "kinematics/BodyNode.h"
#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"
#include "utils/UtilsMath.h"

using namespace Eigen;
using namespace utils;

namespace dynamics {
    ClosedLoopConstraint::ClosedLoopConstraint(SkeletonDynamics *_skel, BodyNodeDynamics *_body1, BodyNodeDynamics *_body2, Vector3d _offset1, Vector3d _offset2, bool _approx, double _timestep) {
        mSkel = _skel;
        mBody1 = _body1;
        mBody2 = _body2;
        mOffset1 = _offset1;
        mOffset2 = _offset2;
        mApproxJDot = _approx;
        mTimestep = _timestep;
        mJ = MatrixXd::Zero(3, mSkel->getNumDofs());
        mPreJ = MatrixXd::Zero(3, mSkel->getNumDofs());
        mJDot = MatrixXd::Zero(3, mSkel->getNumDofs());
    }

    ClosedLoopConstraint::~ClosedLoopConstraint() {
    }
        
    void ClosedLoopConstraint::updateDynamics() {
        mPreJ = mJ;
        getJacobian();            
        getJacobianDot();
        Vector3d worldP1 = xformHom(mBody1->getWorldTransform(), mOffset1);
        Vector3d worldP2 = xformHom(mBody2->getWorldTransform(), mOffset2);
        VectorXd qDot = mSkel->getQDotVector();
        mC = worldP1 - worldP2;
        mCVel = mJ * qDot;
    }

    void ClosedLoopConstraint::getJacobian() {
        mJ.setZero();
        for(int i = 0; i < mBody1->getNumDependentDofs(); i++) {
            int dofIndex = mBody1->getDependentDof(i);
            VectorXd Jcol = xformHom(mBody1->getDerivWorldTransform(i), mOffset1);
            mJ.col(dofIndex) = Jcol;
        }
        for(int i = 0; i < mBody2->getNumDependentDofs(); i++) {
            int dofIndex = mBody2->getDependentDof(i);
            VectorXd Jcol = xformHom(mBody2->getDerivWorldTransform(i), mOffset2);
            mJ.col(dofIndex) -= Jcol;
        }
    }

    void ClosedLoopConstraint::getJacobianDot() {
        mJDot.setZero();
        if (mApproxJDot) {
            mJDot = (mJ - mPreJ) / mTimestep;
        } else {
            VectorXd qDot = mSkel->getQDotVector();
            int nLocalDof = mBody1->getNumDependentDofs();
            MatrixXd sum1(MatrixXd::Zero(3, nLocalDof));
            mBody1->updateSecondDerivatives(mOffset1);
            for (int i = 0; i < nLocalDof; i++) {
                int dofIndex = mBody1->getDependentDof(i);
                sum1 += mBody1->getJvDeriv(i) * qDot[dofIndex];
            }
            for (int i = 0; i < nLocalDof; i++) {
                int dofIndex = mBody1->getDependentDof(i);
                mJDot.col(dofIndex) = sum1.col(i);
            }

            nLocalDof = mBody2->getNumDependentDofs();
            MatrixXd sum2(MatrixXd::Zero(3, nLocalDof));
            mBody2->updateSecondDerivatives(mOffset2);
            for (int i = 0; i < nLocalDof; i++) {
                int dofIndex = mBody2->getDependentDof(i);
                sum2 += mBody2->getJvDeriv(i) * qDot[dofIndex];
            }

            for (int i = 0; i < nLocalDof; i++) {
                int dofIndex = mBody2->getDependentDof(i);
                mJDot.col(dofIndex) -= sum2.col(i);
            }
        }
    }
} // namespace dynamics

