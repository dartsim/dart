#include "constraint/PointConstraint.h"

#include "math/Helpers.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Skeleton.h"
#include "dynamics/BodyNode.h"

using namespace dart;
using namespace math;

namespace dart {
namespace constraint {

PointConstraint::PointConstraint(dynamics::BodyNode *_body, Eigen::Vector3d _offset, Eigen::Vector3d _target, int _skelIndex) {
    mBody = _body;
    mOffset = _offset;
    mTarget = _target;
    mSkelIndex = _skelIndex;
    mJ = Eigen::MatrixXd::Zero(3, mBody->getSkeleton()->getNumGenCoords());
    mNumRows = 3;
}

PointConstraint::~PointConstraint() {
}

void PointConstraint::updateDynamics(std::vector<Eigen::MatrixXd> & _J, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
    getJacobian();
    dynamics::Skeleton *skel = mBody->getSkeleton();
    _J[mSkelIndex].block(_rowIndex, 0, 3, skel->getNumGenCoords()) = mJ;
    Eigen::Vector3d worldP = mBody->getWorldTransform() * mOffset;
    Eigen::VectorXd qDot = skel->get_dq();
    _C.segment(_rowIndex, 3) = worldP - mTarget;
    _CDot.segment(_rowIndex, 3) = mJ * qDot;
}

void PointConstraint::getJacobian() {
    Eigen::MatrixXd JBody = mBody->getWorldJacobian(mOffset - mBody->getWorldTransform().translation()).bottomRows<3>();
    for(int i = 0; i < mBody->getNumDependentGenCoords(); i++) {
        int dofIndex = mBody->getDependentGenCoord(i);
        mJ.col(dofIndex) = JBody.col(i);
    }
}

} // namespace constraint
} // namespace dart
