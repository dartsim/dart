#ifndef PROBLEM_H
#define PROBLEM_H

#include <vector>
#include "optimizer/Problem.h"

namespace model3d {
    class Skeleton;
    template<class SkeletonType>
    class FileInfoSkel;
} // namespace model3d

namespace optimizer {
    class PositionConstraint;
    
    class IKProblem : public optimizer::Problem {
    public:
        IKProblem();
        virtual ~IKProblem();
    
        void initProblem();
        virtual void update(double* coefs);
        model3d::Skeleton* getSkel() const;

        PositionConstraint* getConstraint(int index) const;
    protected:
        model3d::FileInfoSkel<model3d::Skeleton>* mFileInfoSkel;
        std::vector<PositionConstraint*> mConstraints;
    };
} // namespace optimizer

#endif
