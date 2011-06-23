#include "PositionConstraint.h"
using namespace Eigen;

#include "model3d/BodyNode.h"
#include "model3d/Skeleton.h"
#include "optimizer/Var.h"

namespace optimizer {
    
    PositionConstraint::PositionConstraint(
        std::vector<Var *>& var, model3d::Skeleton* skel, model3d::BodyNode* node,
        const Eigen::Vector3d& offset, const Eigen::Vector3d& val)
        : Constraint(var), mSkel(skel), mNode(node), mTarget(val), mOffset(offset)
    {
        mNumRows = 3;

        mWeight = VectorXd::Ones(mNumRows);
        mConstTerm = VectorXd::Zero(mNumRows);
        mCompletion = VectorXd::Zero(mNumRows);
    }

    VectorXd PositionConstraint::EvalC() {
        Vector3d wp = mNode->evalWorldPos(mOffset);
        Vector3d C = wp - mTarget;
        VectorXd ret(C);
        return ret;
    }

    void PositionConstraint::FillJ(VVD jEntry, VVB jMap, int index) {
    }

    void PositionConstraint::FilldG(std::vector<double>& dG) {
        VectorXd dP = EvalC();
        for(int i = 0; i < mNode->getNumDependantDofs(); i++) {
            int dofIndex = mNode->getDependantDof(i);
            
            const Var* v = mVariables[dofIndex];
            double w = v->mWeight;

            // VectorXd J = utils::transform(mNode->Wq.at(i),mOffset);
            // J /= w;
            // dG.at(i) += dP.dot(J);
        }
    }

    void PositionConstraint::setTarget(const Eigen::Vector3d& target) {
        this->mTarget = target;
    }

    Vector3d PositionConstraint::getTarget() const {
        return mTarget;
    }
} // namespace optimizer
