#ifndef PROBLEM_H
#define PROBLEM_H

#include <vector>
#include "optimizer/Problem.h"

namespace kinematics {
    class Skeleton;
    template<class SkeletonType>
    class FileInfoSkel;
} // namespace kinematics

namespace optimizer {
    class PositionConstraint;
    
    class IKProblem : public optimizer::Problem {
    public:
        IKProblem(const char* const filenameSkel);
        virtual ~IKProblem();
    
        void initProblem(const char* const filenameSkel);
        virtual void update(double* coefs);
        kinematics::Skeleton* getSkel() const;

        PositionConstraint* getConstraint(int index) const;
    protected:
        kinematics::FileInfoSkel<kinematics::Skeleton>* mFileInfoSkel;
        std::vector<PositionConstraint*> mConstraints;
    };
} // namespace optimizer

#endif
