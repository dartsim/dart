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

        virtual Eigen::VectorXd solve() = 0;
        virtual Eigen::VectorXd getState() = 0;
        virtual void setState(const Eigen::VectorXd& x) = 0;


        void setProblem(Problem * prob);
        Problem* getProblem() const;
    protected:
        Problem *mProb;
    };
} // namespace optimizer


#endif // #ifndef OPTIMIZER_SOLVER_H

