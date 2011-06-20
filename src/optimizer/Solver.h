/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#ifndef OPTIMIZER_SOLVER_H
#define OPTIMIZER_SOLVER_H

#include <Eigen/Dense>

namespace optimizer {
    class Problem;
    
    class Solver {
    public:
        Solver();
        Solver(Problem *prob);
        virtual ~Solver();

        virtual bool solve() = 0;
        virtual Eigen::VectorXd getState();
        virtual void setState(const Eigen::VectorXd& x);


        virtual void setProblem(Problem * prob);
        Problem* getProblem() const;
    protected:
        Problem *mProb;
    };
} // namespace optimizer


#endif // #ifndef OPTIMIZER_SOLVER_H

