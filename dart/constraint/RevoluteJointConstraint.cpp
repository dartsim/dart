#include "dart/constraint/RevoluteJointConstraint.h"

#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"

using namespace dart;
using namespace math;

namespace dart {
namespace constraint {

RevoluteJointConstraint::RevoluteJointConstraint(dynamics::BodyNode *_body1, dynamics::BodyNode *_body2, Eigen::Vector3d _axis1, Eigen::Vector3d _axis2) {
    mBodyNode1 = _body1;
    mBodyNode2 = _body2;    
    mAxis1 = _axis1.normalized();
    mAxis2 = _axis2.normalized();
    mNumRows = 3;
    mJ1 = Eigen::MatrixXd::Zero(mNumRows, mBodyNode1->getSkeleton()->getNumGenCoords());
    mJ2 = Eigen::MatrixXd::Zero(mNumRows, mBodyNode2->getSkeleton()->getNumGenCoords());
}

RevoluteJointConstraint::RevoluteJointConstraint(dynamics::BodyNode *_body1, Eigen::Vector3d _axis1, Eigen::Vector3d _target) {
    mBodyNode1 = _body1;
    mBodyNode2 = NULL;
    mAxis1 = _axis1.normalized();
    mAxis2 = _target.normalized();
 
    mNumRows = 3;
    mJ1 = Eigen::MatrixXd::Zero(mNumRows, mBodyNode1->getSkeleton()->getNumGenCoords());
}

RevoluteJointConstraint::~RevoluteJointConstraint() {
}

void RevoluteJointConstraint::updateDynamics(Eigen::MatrixXd & _J1, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
    getJacobian();
    _J1.block(_rowIndex, 0, mNumRows, mBodyNode1->getSkeleton()->getNumGenCoords()) = mJ1;
    Eigen::Vector3d v1 = mBodyNode1->getWorldTransform().rotation() * mAxis1;
    Eigen::VectorXd qDot1 = ((dynamics::Skeleton*)mBodyNode1->getSkeleton())->get_dq();
    _C.segment(_rowIndex, mNumRows) = v1 - mAxis2;
    _CDot.segment(_rowIndex, mNumRows) = mJ1 * qDot1;
}
 
void RevoluteJointConstraint::updateDynamics(Eigen::MatrixXd & _J1, Eigen::MatrixXd & _J2, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
    getJacobian();
    _J2.block(_rowIndex, 0, mNumRows, mBodyNode2->getSkeleton()->getNumGenCoords()).setZero();
    _J1.block(_rowIndex, 0, mNumRows, mBodyNode1->getSkeleton()->getNumGenCoords()) = mJ1;
    _J2.block(_rowIndex, 0, mNumRows, mBodyNode2->getSkeleton()->getNumGenCoords()) += mJ2;

    Eigen::Vector3d v1 = mBodyNode1->getWorldTransform().rotation() * mAxis1;
    Eigen::VectorXd qDot1 = ((dynamics::Skeleton*)mBodyNode1->getSkeleton())->get_dq();
    Eigen::VectorXd v2 = mBodyNode2->getWorldTransform().rotation() * mAxis2;
    Eigen::VectorXd qDot2 = ((dynamics::Skeleton*)mBodyNode2->getSkeleton())->get_dq();
    
    _C.segment(_rowIndex, mNumRows) = v1 - v2;
    _CDot.segment(_rowIndex, mNumRows) = mJ1 * qDot1 + mJ2 * qDot2;
}
 
void RevoluteJointConstraint::getJacobian() {
    Eigen::Vector3d offsetWorld = mBodyNode1->getWorldTransform().rotation() * mAxis1;
    Eigen::Vector3d origin(0.0, 0.0, 0.0);

    Eigen::MatrixXd JBody1 = mBodyNode1->getWorldJacobian(offsetWorld).bottomRows<3>() - mBodyNode1->getWorldJacobian(origin).bottomRows<3>();

    for(int i = 0; i < mBodyNode1->getNumDependentGenCoords(); i++) {
        int dofIndex = mBodyNode1->getDependentGenCoordIndex(i);
        mJ1.col(dofIndex) = JBody1.col(i);
    }

    if (mBodyNode2) {
        offsetWorld = mBodyNode2->getWorldTransform().rotation() * mAxis2;
        Eigen::MatrixXd JBody2 = mBodyNode2->getWorldJacobian(offsetWorld).bottomRows<3>() - mBodyNode2->getWorldJacobian(origin).bottomRows<3>();

        for(int i = 0; i < mBodyNode2->getNumDependentGenCoords(); i++) {
            int dofIndex = mBodyNode2->getDependentGenCoordIndex(i);
            mJ2.col(dofIndex) = -JBody2.col(i);
        }
    }
}
    
} // namespace constraint
} // namespace dart
