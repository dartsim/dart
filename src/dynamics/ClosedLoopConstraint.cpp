
#include "ClosedLoopConstraint.h"

#include "kinematics/BodyNode.h"
#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"
#include "math/UtilsMath.h"

using namespace Eigen;
using namespace dart_math;

namespace dynamics {
    ClosedLoopConstraint::ClosedLoopConstraint(BodyNodeDynamics *_body1, BodyNodeDynamics *_body2, Vector3d _offset1, Vector3d _offset2, int _skelIndex1, int _skelIndex2) {
        mBody1 = _body1;
        mBody2 = _body2;
        mOffset1 = _offset1;
        mOffset2 = _offset2;
        mJ1 = MatrixXd::Zero(3, mBody1->getSkel()->getNumDofs());
        mJ2 = MatrixXd::Zero(3, mBody2->getSkel()->getNumDofs());
        mNumRows = 3;
        mSkelIndex1 = _skelIndex1;
        mSkelIndex2 = _skelIndex2;
    }

    ClosedLoopConstraint::~ClosedLoopConstraint() {
    }
        
    void ClosedLoopConstraint::updateDynamics(std::vector<Eigen::MatrixXd> & _J, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
        getJacobian();
        _J[mSkelIndex2].block(_rowIndex, 0, 3, mBody2->getSkel()->getNumDofs()).setZero();
        _J[mSkelIndex1].block(_rowIndex, 0, 3, mBody1->getSkel()->getNumDofs()) = mJ1;
        _J[mSkelIndex2].block(_rowIndex, 0, 3, mBody2->getSkel()->getNumDofs()) += mJ2;

        Vector3d worldP1 = xformHom(mBody1->getWorldTransform(), mOffset1);
        Vector3d worldP2 = xformHom(mBody2->getWorldTransform(), mOffset2);
        VectorXd qDot1 = ((SkeletonDynamics*)mBody1->getSkel())->get_dq();
        VectorXd qDot2 = ((SkeletonDynamics*)mBody2->getSkel())->get_dq();
        _C.segment(_rowIndex, 3) = worldP1 - worldP2;
        _CDot.segment(_rowIndex, 3) = mJ1 * qDot1 + mJ2 * qDot2;
    }

    void ClosedLoopConstraint::getJacobian() {
        for(int i = 0; i < mBody1->getNumDependentDofs(); i++) {
            int dofIndex = mBody1->getDependentDof(i);
            VectorXd Jcol = xformHom(mBody1->getDerivWorldTransform(i), mOffset1);
            mJ1.col(dofIndex) = Jcol;
        }
        for(int i = 0; i < mBody2->getNumDependentDofs(); i++) {
            int dofIndex = mBody2->getDependentDof(i);
            VectorXd Jcol = xformHom(mBody2->getDerivWorldTransform(i), mOffset2);
            mJ2.col(dofIndex) = -Jcol;
        }
    }
} // namespace dynamics

