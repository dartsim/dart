#include "BallJointConstraint.h"

#include "math/Helpers.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Skeleton.h"

using namespace dart;
using namespace math;

namespace dart {
namespace constraint {

BallJointConstraint::BallJointConstraint(dynamics::BodyNode *_body1, dynamics::BodyNode *_body2, Eigen::Vector3d _offset1, Eigen::Vector3d _offset2) {
    mBodyNode1 = _body1;
    mBodyNode2 = _body2;
    mOffset1 = _offset1;
    mOffset2 = _offset2;
    mJ1 = Eigen::MatrixXd::Zero(3, mBodyNode1->getSkeleton()->getNumGenCoords());
    mJ2 = Eigen::MatrixXd::Zero(3, mBodyNode2->getSkeleton()->getNumGenCoords());
    mNumRows = 3;
}

    BallJointConstraint::BallJointConstraint(dynamics::BodyNode *_body1, Eigen::Vector3d _offset1, Eigen::Vector3d _target) {
    mBodyNode1 = _body1;
    mBodyNode2 = NULL;
    mOffset1 = _offset1;
    mOffset2 = _target;
    mJ1 = Eigen::MatrixXd::Zero(3, mBodyNode1->getSkeleton()->getNumGenCoords());
    mNumRows = 3;
}

BallJointConstraint::~BallJointConstraint() {
}

void BallJointConstraint::updateDynamics(Eigen::MatrixXd & _J1, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
    getJacobian();
    _J1.block(_rowIndex, 0, 3, mBodyNode1->getSkeleton()->getNumGenCoords()) = mJ1;
    Eigen::Vector3d worldP1 = mBodyNode1->getWorldTransform() * mOffset1;
    Eigen::VectorXd qDot1 = ((dynamics::Skeleton*)mBodyNode1->getSkeleton())->get_dq();
    _C.segment(_rowIndex, 3) = worldP1 - mOffset2;
    _CDot.segment(_rowIndex, 3) = mJ1 * qDot1;
}
 
void BallJointConstraint::updateDynamics(Eigen::MatrixXd & _J1, Eigen::MatrixXd & _J2, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
    getJacobian();
    _J2.block(_rowIndex, 0, 3, mBodyNode2->getSkeleton()->getNumGenCoords()).setZero();
    _J1.block(_rowIndex, 0, 3, mBodyNode1->getSkeleton()->getNumGenCoords()) = mJ1;
    _J2.block(_rowIndex, 0, 3, mBodyNode2->getSkeleton()->getNumGenCoords()) += mJ2;

    Eigen::Vector3d worldP1 = mBodyNode1->getWorldTransform() * mOffset1;
    Eigen::VectorXd qDot1 = ((dynamics::Skeleton*)mBodyNode1->getSkeleton())->get_dq();
    Eigen::VectorXd worldP2 = mBodyNode2->getWorldTransform() * mOffset2;
    Eigen::VectorXd qDot2 = ((dynamics::Skeleton*)mBodyNode2->getSkeleton())->get_dq();
    
    _C.segment(_rowIndex, 3) = worldP1 - worldP2;
    _CDot.segment(_rowIndex, 3) = mJ1 * qDot1 + mJ2 * qDot2;
}
 
void BallJointConstraint::getJacobian() {
    Eigen::Vector3d offsetWorld = mBodyNode1->getWorldTransform().rotation() * mOffset1;
    Eigen::MatrixXd JBody1 = mBodyNode1->getWorldJacobian(offsetWorld).bottomRows<3>();
    for(int i = 0; i < mBodyNode1->getNumDependentDofs(); i++) {
        int dofIndex = mBodyNode1->getDependentDof(i);
        mJ1.col(dofIndex) = JBody1.col(dofIndex);
    }
    if (mBodyNode2) {
        offsetWorld = mBodyNode2->getWorldTransform().rotation() * mOffset2;
        Eigen::MatrixXd JBody2 = mBodyNode2->getWorldJacobian(offsetWorld).bottomRows<3>();
        for(int i = 0; i < mBodyNode2->getNumDependentDofs(); i++) {
            int dofIndex = mBodyNode2->getDependentDof(i);
            mJ2.col(dofIndex) = -JBody2.col(dofIndex);
        }
    }
}
    
} // namespace constraint
} // namespace dart
