#include "PositionConstraint.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Skeleton.h"
#include "optimizer/Var.h"
#include "math/UtilsMath.h"

using namespace kinematics;
using namespace dart_math;

namespace optimizer {
    
    PositionConstraint::PositionConstraint(vector<Var *>& var, Skeleton* skel, BodyNode* node, const Vector3d& offset, const Vector3d& val) : Constraint(var), mSkel(skel), mNode(node), mTarget(val), mOffset(offset) {
        mNumRows = 3;

        mWeight = VectorXd::Ones(mNumRows);
        mConstTerm = VectorXd::Zero(mNumRows);
        mCompletion = VectorXd::Zero(mNumRows);
    }

    VectorXd PositionConstraint::evalCon() {
        Vector3d wp = mNode->evalWorldPos(mOffset);
        Vector3d c = wp - mTarget;
        VectorXd ret(c);
        return ret;
    }

    void PositionConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
    }

    void PositionConstraint::fillObjGrad(std::vector<double>& dG) {
        VectorXd dP = evalCon();
        for(int dofIndex = 0; dofIndex < mNode->getNumDependentDofs(); dofIndex++) {
            int i = mNode->getDependentDof(dofIndex);            
            const Var* v = mVariables[i];
            double w = v->mWeight;
            VectorXd J = xformHom(mNode->getDerivWorldTransform(dofIndex), mOffset);
            J /= w;
            dG[i] += 2 * dP.dot(J);
        }
    }

    void PositionConstraint::setTarget(const Vector3d& target) {
        mTarget = target;
    }

    Vector3d PositionConstraint::getTarget() const {
        return mTarget;
    }
} // namespace optimizer
