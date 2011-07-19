#ifndef PROBLEM_H
#define PROBLEM_H

#include "optimizer/Problem.h"

namespace model3d {
    class FileInfoSkel;
    class Skeleton;
} // namespace model3d

namespace optimizer {
    class IKProblem : public optimizer::Problem {
    public:
        IKProblem();
        virtual ~IKProblem();
    
        void initProblem();
        virtual void update(double* coefs);
        model3d::Skeleton* getSkel() const;
    protected:
        model3d::FileInfoSkel* mFileInfoSkel;
    };
} // namespace optimizer

#endif
