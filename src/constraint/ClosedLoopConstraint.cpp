#include "ClosedLoopConstraint.h"

#include "math/Helpers.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Skeleton.h"
#include "dynamics/BodyNode.h"

using namespace dart;
using namespace math;

namespace dart {
namespace constraint {

ClosedLoopConstraint::ClosedLoopConstraint(dynamics::BodyNode *_body1, dynamics::BodyNode *_body2, Eigen::Vector3d _offset1, Eigen::Vector3d _offset2, int _skelIndex1, int _skelIndex2) {
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

ClosedLoopConstraint::~ClosedLoopConstraint() {
}

void ClosedLoopConstraint::updateDynamics(std::vector<Eigen::MatrixXd> & _J, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
    getJacobian();
    _J[mSkelIndex2].block(_rowIndex, 0, 3, mBody2->getSkeleton()->getNumGenCoords()).setZero();
    _J[mSkelIndex1].block(_rowIndex, 0, 3, mBody1->getSkeleton()->getNumGenCoords()) = mJ1;
    _J[mSkelIndex2].block(_rowIndex, 0, 3, mBody2->getSkeleton()->getNumGenCoords()) += mJ2;

    Eigen::Vector3d worldP1 = mBody1->getWorldTransform() * mOffset1;
    Eigen::Vector3d worldP2 = mBody2->getWorldTransform() * mOffset2;
    Eigen::VectorXd qDot1 = ((dynamics::Skeleton*)mBody1->getSkeleton())->get_dq();
    Eigen::VectorXd qDot2 = ((dynamics::Skeleton*)mBody2->getSkeleton())->get_dq();
    _C.segment(_rowIndex, 3) = worldP1 - worldP2;
    _CDot.segment(_rowIndex, 3) = mJ1 * qDot1 + mJ2 * qDot2;
}

void ClosedLoopConstraint::getJacobian() {
    Eigen::MatrixXd JBody1 = mBody1->getWorldJacobian(mOffset1 - mBody1->getWorldTransform().translation()).bottomRows<3>();
    for(int i = 0; i < mBody1->getNumDependentGenCoords(); i++) {
        int dofIndex = mBody1->getDependentGenCoord(i);
        mJ1.col(dofIndex) = JBody1.col(dofIndex);
    }
    Eigen::MatrixXd JBody2 = mBody2->getWorldJacobian(mOffset2 - mBody2->getWorldTransform().translation()).bottomRows<3>();
    for(int i = 0; i < mBody2->getNumDependentGenCoords(); i++) {
        int dofIndex = mBody2->getDependentGenCoord(i);
        mJ2.col(dofIndex) = JBody2.col(dofIndex);
    }
}

} // namespace constraint
} // namespace dart
