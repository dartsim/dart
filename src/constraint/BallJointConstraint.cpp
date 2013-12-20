#include "BallJointConstraint.h"

#include "math/Helpers.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Skeleton.h"
#include "dynamics/BodyNode.h"

using namespace dart;
using namespace math;

namespace dart {
namespace constraint {

BallJointConstraint::BallJointConstraint(dynamics::BodyNode *_body1, dynamics::BodyNode *_body2, Eigen::Vector3d _offset1, Eigen::Vector3d _offset2, int _skelIndex1, int _skelIndex2) {
    mBody1 = _body1;
    mBody2 = _body2;
    mOffset1 = _offset1;
    mOffset2 = _offset2;
    mJ1 = Eigen::MatrixXd::Zero(3, mBody1->getSkeleton()->getNumGenCoords());
    mJ2 = Eigen::MatrixXd::Zero(3, mBody2->getSkeleton()->getNumGenCoords());
    mNumRows = 3;
    mSkelIndex1 = _skelIndex1;
    mSkelIndex2 = _skelIndex2;
}

    BallJointConstraint::BallJointConstraint(dynamics::BodyNode *_body1, Eigen::Vector3d _offset1, Eigen::Vector3d _offset2, int _skelIndex1) {
    mBody1 = _body1;
    mBody2 = NULL;
    mOffset1 = _offset1;
    mOffset2 = _offset2;
    mJ1 = Eigen::MatrixXd::Zero(3, mBody1->getSkeleton()->getNumGenCoords());
    mNumRows = 3;
    mSkelIndex1 = _skelIndex1;
    mSkelIndex2 = -1;
}

BallJointConstraint::~BallJointConstraint() {
}

void BallJointConstraint::updateDynamics(std::vector<Eigen::MatrixXd> & _J, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
    getJacobian();
    if (mBody2) {
        _J[mSkelIndex2].block(_rowIndex, 0, 3, mBody2->getSkeleton()->getNumGenCoords()).setZero();
        _J[mSkelIndex1].block(_rowIndex, 0, 3, mBody1->getSkeleton()->getNumGenCoords()) = mJ1;
        _J[mSkelIndex2].block(_rowIndex, 0, 3, mBody2->getSkeleton()->getNumGenCoords()) += mJ2;

        Eigen::Vector3d worldP1 = mBody1->getWorldTransform() * mOffset1;
        Eigen::VectorXd qDot1 = ((dynamics::Skeleton*)mBody1->getSkeleton())->get_dq();
        Eigen::VectorXd worldP2 = mBody2->getWorldTransform() * mOffset2;
        Eigen::VectorXd qDot2 = ((dynamics::Skeleton*)mBody2->getSkeleton())->get_dq();
    
        _C.segment(_rowIndex, 3) = worldP1 - worldP2;
        _CDot.segment(_rowIndex, 3) = mJ1 * qDot1 + mJ2 * qDot2;
    } else {
        _J[mSkelIndex1].block(_rowIndex, 0, 3, mBody1->getSkeleton()->getNumGenCoords()) = mJ1;
        Eigen::Vector3d worldP1 = mBody1->getWorldTransform() * mOffset1;
        Eigen::VectorXd qDot1 = ((dynamics::Skeleton*)mBody1->getSkeleton())->get_dq();
        _C.segment(_rowIndex, 3) = worldP1 - mOffset2;
        _CDot.segment(_rowIndex, 3) = mJ1 * qDot1;
    }
}
    /*
    void BallJointConstraint::getJacobian() {
        for(int i = 0; i < mBody1->getNumDependentDofs(); i++) {
            int dofIndex = mBody1->getDependentDof(i);
            Eigen::VectorXd Jcol = xformHom(mBody1->getDerivWorldTransform(i), mOffset1);
            mJ1.col(dofIndex) = Jcol;
        }
        for(int i = 0; i < mBody2->getNumDependentDofs(); i++) {
            int dofIndex = mBody2->getDependentDof(i);
            Eigen::VectorXd Jcol = xformHom(mBody2->getDerivWorldTransform(i), mOffset2);
            mJ2.col(dofIndex) = -Jcol;
        }
    }
    */
void BallJointConstraint::getJacobian() {
    Eigen::Vector3d offsetWorld = mBody1->getWorldTransform().rotation() * mOffset1;
    Eigen::MatrixXd JBody1 = mBody1->getWorldJacobian(offsetWorld).bottomRows<3>();
    for(int i = 0; i < mBody1->getNumDependentDofs(); i++) {
        int dofIndex = mBody1->getDependentDof(i);
        mJ1.col(dofIndex) = JBody1.col(dofIndex);
    }
    if (mBody2) {
        offsetWorld = mBody2->getWorldTransform().rotation() * mOffset2;
        Eigen::MatrixXd JBody2 = mBody2->getWorldJacobian(offsetWorld).bottomRows<3>();
        for(int i = 0; i < mBody2->getNumDependentDofs(); i++) {
            int dofIndex = mBody2->getDependentDof(i);
            mJ2.col(dofIndex) = -JBody2.col(dofIndex);
        }
    }
}
    
} // namespace constraint
} // namespace dart
