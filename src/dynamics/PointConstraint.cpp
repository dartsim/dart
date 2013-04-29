
#include "PointConstraint.h"

#include "kinematics/BodyNode.h"
#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"
#include "utils/UtilsMath.h"

using namespace Eigen;
using namespace utils;

namespace dynamics {
    PointConstraint::PointConstraint(BodyNodeDynamics *_body, Vector3d _offset, Vector3d _target, int _skelIndex) {
        mBody = _body;
        mOffset = _offset;
        mTarget = _target;
        mSkelIndex = _skelIndex;
        mJ = MatrixXd::Zero(3, mBody->getSkel()->getNumDofs());
        mNumRows = 3;
    }

    PointConstraint::~PointConstraint() {
    }
        
    void PointConstraint::updateDynamics(std::vector<Eigen::MatrixXd> & _J, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
        getJacobian();
        SkeletonDynamics *skel = (SkeletonDynamics*)mBody->getSkel();
        _J[mSkelIndex].block(_rowIndex, 0, 3, skel->getNumDofs()) = mJ;
        Vector3d worldP = xformHom(mBody->getWorldTransform(), mOffset);
        VectorXd qDot = skel->getPoseVelocity();
        _C.segment(_rowIndex, 3) = worldP - mTarget;
        _CDot.segment(_rowIndex, 3) = mJ * qDot;
    }

    void PointConstraint::getJacobian() {
        for(int i = 0; i < mBody->getNumDependentDofs(); i++) {
            int dofIndex = mBody->getDependentDof(i);
            VectorXd Jcol = xformHom(mBody->getDerivWorldTransform(i), mOffset);
            mJ.col(dofIndex) = Jcol;
        }
    }
} // namespace dynamics

