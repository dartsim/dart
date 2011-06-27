#ifndef PROBLEM_H
#define PROBLEM_H

#include "optimizer/Problem.h"

namespace model3d {
    class FileInfoModel;
    class Skeleton;
} // namespace model3d

namespace optimizer {
    class IKProblem : public optimizer::Problem {
    public:
        IKProblem();
        virtual ~IKProblem();
    
        void initProblem();
        virtual void update(double* coefs);
        model3d::Skeleton* getModel() const;
    protected:
        model3d::FileInfoModel* mFileInfoModel;
    };
} // namespace optimizer

#endif
