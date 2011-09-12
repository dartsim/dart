#include "PositionConstraint.h"
using namespace Eigen;

#include <glog/logging.h>

#include "kinematics/BodyNode.h"
#include "kinematics/Skeleton.h"
#include "optimizer/Var.h"
#include "utils/UtilsMath.h"

namespace optimizer {
    
    PositionConstraint::PositionConstraint(
        std::vector<Var *>& var, kinematics::Skeleton* skel, kinematics::BodyNode* node,
        const Eigen::Vector3d& offset, const Eigen::Vector3d& val)
        : Constraint(var), mSkel(skel), mNode(node), mTarget(val), mOffset(offset)
    {
        mNumRows = 3;

        mWeight = VectorXd::Ones(mNumRows);
        mConstTerm = VectorXd::Zero(mNumRows);
        mCompletion = VectorXd::Zero(mNumRows);
    }

    VectorXd PositionConstraint::evalCon() {
        Vector3d wp = mNode->evalWorldPos(mOffset);
        Vector3d C = wp - mTarget;
        VectorXd ret(C);
        // cout << "mNode = " << mNode->getModelIndex() << " : "
        //      << ret.transpose() << endl;
        return ret;
    }

    void PositionConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
    }

    void PositionConstraint::fillObjGrad(std::vector<double>& dG) {
        VectorXd dP = evalCon();

        for(int dofIndex = 0; dofIndex < mNode->getNumDependentDofs(); dofIndex++) {
            // VLOG(1) << "dofIndex = " << dofIndex;
            int i = mNode->getDependentDof(dofIndex);
            // VLOG(1) << "i = " << i;
            
            const Var* v = mVariables[i];
            double w = v->mWeight;
            // VLOG(1) << "w = " << w;

            VectorXd J = utils::xformHom(mNode->getDerivWorldTransform(dofIndex),mOffset);
            J /= w;
            // VLOG(1) << "J = " << J.transpose();
            dG.at(i) += dP.dot(J);
        }
    }

    void PositionConstraint::setTarget(const Eigen::Vector3d& target) {
        this->mTarget = target;
    }

    Vector3d PositionConstraint::getTarget() const {
        return mTarget;
    }
} // namespace optimizer
