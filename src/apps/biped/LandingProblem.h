#ifndef PROBLEM_H
#define PROBLEM_H

#include <vector>
#include "optimizer/Problem.h"

namespace dynamics {
    class SkeletonDynamics;
} // namespace kinematics

namespace optimizer {
    class Constraint;
    
    class LandingProblem : public optimizer::Problem {
    public:
        LandingProblem(dynamics::SkeletonDynamics *_skel);
        virtual ~LandingProblem();
    
        void initProblem();
        virtual void update(double* coefs);

        Constraint* getConstraint(int index) const;
        std::vector<Constraint*> & getConstraints();
    protected:
        dynamics::SkeletonDynamics *mSkel;
        std::vector<Constraint*> mConstraints;
    };
} // namespace optimizer

#endif
