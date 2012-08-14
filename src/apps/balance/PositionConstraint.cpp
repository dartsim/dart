#include "PositionConstraint.h"
using namespace Eigen;
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
        for(int i = 0; i < mNode->getNumDependentDofs(); i++) {
            int dependDofIndex = mNode->getDependentDof(i);
            VectorXd J = utils::xformHom(mNode->getDerivWorldTransform(i), mOffset);
            for (int j = 0; j < 3; j++) {
                jEntry->at(index + j)->at(dependDofIndex) = J[j];
                jMap->at(index + j)->at(dependDofIndex) = 1;
            }
        }
    }

    void PositionConstraint::fillObjGrad(std::vector<double>& dG) {
        VectorXd dP = evalCon();
        for(int i = 0; i < mNode->getNumDependentDofs(); i++) {
            int dependDofIndex = mNode->getDependentDof(i);
            
            const Var* v = mVariables[dependDofIndex];
            double w = v->mWeight;

            VectorXd J = utils::xformHom(mNode->getDerivWorldTransform(i),mOffset);
            J /= w;
            dG.at(dependDofIndex) += dP.dot(J);
        }
    }

    void PositionConstraint::setTarget(const Eigen::Vector3d& target) {
        this->mTarget = target;
    }

    Vector3d PositionConstraint::getTarget() const {
        return mTarget;
    }
} // namespace optimizer
