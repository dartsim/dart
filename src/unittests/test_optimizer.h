#ifndef UNITTESTS_TEST_OPTIMIZER_H
#define UNITTESTS_TEST_OPTIMIZER_H

// For problem
#include "optimizer/Var.h"
#include "optimizer/Constraint.h"
#include "optimizer/Problem.h"
#include "optimizer/ObjectiveBox.h"
#include "optimizer/snopt/SnoptSolver.h"

class SampleConstraint : public optimizer::Constraint {
public:
    SampleConstraint(std::vector<optimizer::Var *>& var, int index, double target)
        : optimizer::Constraint(var), mIndex(index), mTarget(target) {
        mNumRows = 1;

        mWeight = Eigen::VectorXd::Ones(1);
        mConstTerm = Eigen::VectorXd::Zero(1);
        mCompletion = Eigen::VectorXd::Zero(1);
    }
    
    virtual Eigen::VectorXd EvalC() {
        std::vector<optimizer::Var *>& vars = mVariables;
        Eigen::VectorXd x(1);
        x(0) = vars[mIndex]->mVal - mTarget;
        return x;
    }

    virtual void FillJ(optimizer::VVD, int index) {}
    virtual void FillJ(optimizer::VVD, optimizer::VVB, int index) {}
    virtual void FilldG(std::vector<double>& dG) {
        VectorXd dP = EvalC();

        for(int i = 0; i < mVariables.size(); i++){
            const optimizer::Var* var = mVariables[i];
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

private:
    int mIndex;
    double mTarget;
};


TEST(SIMPLE_SNOPT, OPTIMIZER) {
    using namespace optimizer;
    
    Problem prob;
    prob.addVariable(0.0, -10.0, 10.0);
    prob.createBoxes();

    SampleConstraint* c = new SampleConstraint(
        prob.vars(), 0, 3.0);
    prob.objBox()->Add(c);

    snopt::SnoptSolver solver(&prob);
    solver.solve();

    Eigen::VectorXd sol = solver.getState();
    const double TOLERANCE = 0.000001;
    EXPECT_EQ(sol.size(), 1);
    EXPECT_NEAR(sol(0), 3.0, TOLERANCE);
}


#endif
