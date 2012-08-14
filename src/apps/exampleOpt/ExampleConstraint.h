#ifndef EXAMPLE_CONSTRAINT_H
#define EXAMPLE_CONSTRAINT_H

#include "optimizer/Constraint.h"
namespace optimizer {
    class Var;
} // namespace optimizer

class ExampleConstraint : public optimizer::Constraint {
public:
    ExampleConstraint(std::vector<optimizer::Var *>& var, int index, double target);
    
    virtual Eigen::VectorXd evalCon();

    virtual void fillJac(optimizer::VVD, int index) {}
    virtual void fillJac(optimizer::VVD, optimizer::VVB, int index);
    virtual void fillObjGrad(std::vector<double>& dG);

private:
    int mIndex;
    double mTarget;
};

#endif
