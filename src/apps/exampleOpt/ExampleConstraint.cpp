#include "ExampleConstraint.h"
#include <iostream>
using namespace std;
using namespace Eigen;
#include "optimizer/Var.h"
using namespace optimizer;

ExampleConstraint::ExampleConstraint(std::vector<optimizer::Var *>& var, int index, double target)
    : optimizer::Constraint(var), mIndex(index), mTarget(target) {
    mNumRows = 1;

    mWeight = VectorXd::Ones(1);
    mConstTerm = VectorXd::Zero(1);
    mCompletion = VectorXd::Zero(1);
}


VectorXd ExampleConstraint::evalCon() {
    VectorXd ret(1);
    VectorXd x(mVariables.size());
    for(unsigned int i = 0; i < mVariables.size(); i++)
        x[i] = mVariables[i]->mVal;
    ret(0) = x.dot(x) - mTarget;

    return ret;

}
 
void ExampleConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
    for (int i = 0; i < mNumRows; i++) {
        for(unsigned int j = 0; j < mVariables.size(); j++) {
            jEntry->at(index + i)->at(j) = 2.0 * mVariables[j]->mVal;
            jMap->at(index + i)->at(j) = true;
        }
    }
}

void ExampleConstraint::fillObjGrad(std::vector<double>& dG) {
    VectorXd dP = evalCon();
    VectorXd J(mVariables.size());
    for(unsigned int i = 0; i < mVariables.size(); i++){
        J[i] = 2.0 * mVariables[i]->mVal;
        J[i] /= mVariables[i]->mWeight;
        dG.at(i) += dP[0] * J[i];
    }    
}


