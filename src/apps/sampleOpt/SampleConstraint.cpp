#include "SampleConstraint.h"
#include <iostream>
using namespace std;
using namespace Eigen;
#include "optimizer/Var.h"
using namespace optimizer;

SampleConstraint::SampleConstraint(std::vector<optimizer::Var *>& var, int index, double target)
    : optimizer::Constraint(var), mIndex(index), mTarget(target) {
    mNumRows = 1;

    mWeight = VectorXd::Ones(1);
    mConstTerm = VectorXd::Zero(1);
    mCompletion = VectorXd::Zero(1);
}


Eigen::VectorXd SampleConstraint::EvalC() {
    std::vector<Var *>& vars = mVariables;
    VectorXd x(1);
    x(0) = vars[mIndex]->mVal - mTarget;

    return x;

}

void SampleConstraint::FilldG(std::vector<double>& dG) {
    VectorXd dP = EvalC();

    for(int i = 0; i < mVariables.size(); i++){
        const Var* var = mVariables[i];
        VectorXd J(1);
        if (i == mIndex) {
            J(0) = 1.0;
        } else {
            J(1) = 0.0;
        }

        J /= var->mWeight;
        dG.at(i) += dP.dot(J);
    }
}


