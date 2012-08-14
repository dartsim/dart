#include "COMConstraint.h"
using namespace Eigen;

#include "kinematics/Skeleton.h"
#include "kinematics/BodyNode.h"

#include "optimizer/Var.h"
#include "utils/UtilsMath.h"

namespace optimizer {
    
    COMConstraint::COMConstraint(std::vector<Var *>& var, kinematics::Skeleton* skel, const Eigen::Vector3d& val) : Constraint(var), mSkel(skel), mTarget(val) {
        mNumRows = 3;

        mWeight = VectorXd::Ones(mNumRows);
        mConstTerm = VectorXd::Zero(mNumRows);
        mCompletion = VectorXd::Zero(mNumRows);
    }

    VectorXd COMConstraint::evalCon() {
        Vector3d wp = mSkel->getWorldCOM();
        Vector3d C = wp - mTarget;
        VectorXd ret(C);
        return ret;
    }

    void COMConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
        // need to verify that jEntry is cleared at entry
        //        cout << "index = " << index << endl;
        for (int i = 0; i < mSkel->getNumNodes(); i++) {
            kinematics::BodyNode *node = mSkel->getNode(i);
            MatrixXd localJ = node->getJacobianLinear() * node->getMass();
            localJ /= mSkel->getMass();
            for (int j = 0; j < node->getNumDependentDofs(); j++) {
                int dependDofIndex = node->getDependentDof(j);
                for (int k = 0; k < 3; k++) {
                    jEntry->at(index + k)->at(dependDofIndex) += localJ(k, j);
                    jMap->at(index + k)->at(dependDofIndex) = 1;
                }
                //                cout << "depedentDofIndex = " << dependDofIndex << endl;
            }
        }
    }

    void COMConstraint::fillObjGrad(std::vector<double>& dG) {
    }

    void COMConstraint::setTarget(const Eigen::Vector3d& target) {
        this->mTarget = target;
    }

    Vector3d COMConstraint::getTarget() const {
        return mTarget;
    }
} // namespace optimizer
