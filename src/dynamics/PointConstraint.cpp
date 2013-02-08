
#include "PointConstraint.h"

#include "kinematics/BodyNode.h"
#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"
#include "utils/UtilsMath.h"

using namespace Eigen;
using namespace utils;

namespace dynamics {
    PointConstraint::PointConstraint(SkeletonDynamics *_skel, BodyNodeDynamics *_body, Vector3d _offset, Vector3d _target, bool _approx, double _timestep) {
        mSkel = _skel;
        mBody = _body;
        mOffset = _offset;
        mTarget = _target;
        mApproxJDot = _approx;
        mTimestep = _timestep;
        mJ = MatrixXd::Zero(3, mSkel->getNumDofs());
        mPreJ = MatrixXd::Zero(3, mSkel->getNumDofs());
        mJDot = MatrixXd::Zero(3, mSkel->getNumDofs());
    }

    PointConstraint::~PointConstraint() {
    }
        
    void PointConstraint::updateDynamics() {
        mPreJ = mJ;
        getJacobian();
        getJacobianDot();
        Vector3d worldP = xformHom(mBody->getWorldTransform(), mOffset);
        VectorXd qDot = mSkel->getQDotVector();
        mC = worldP - mTarget;
        mCVel = mJ * qDot;
    }

    void PointConstraint::getJacobian() {
        for(int i = 0; i < mBody->getNumDependentDofs(); i++) {
            int dofIndex = mBody->getDependentDof(i);
            VectorXd Jcol = xformHom(mBody->getDerivWorldTransform(i), mOffset);
            mJ.col(dofIndex) = Jcol;
        }
    }

    void PointConstraint::getJacobianDot() {
        if (mApproxJDot) {
            mJDot = (mJ - mPreJ) / mTimestep;
        } else {
            int nLocalDof = mBody->getNumDependentDofs();
            VectorXd qDot = mSkel->getQDotVector();
            MatrixXd sum(MatrixXd::Zero(3, nLocalDof));
            mBody->updateSecondDerivatives(mOffset);
            for (int i = 0; i < nLocalDof; i++) {
                int dofIndex = mBody->getDependentDof(i);
                sum += mBody->getJvDeriv(i) * qDot[dofIndex];
            }
            for (int i = 0; i < nLocalDof; i++) {
                int dofIndex = mBody->getDependentDof(i);
                mJDot.col(dofIndex) = sum.col(i);
            }
        }
    }
} // namespace dynamics

