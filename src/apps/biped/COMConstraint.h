#ifndef COM_CONSTRAINT_H
#define COM_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace kinematics {
    class Skeleton;
} // namespace kinematics

namespace optimizer {
    class Var;

    class COMConstraint : public Constraint {
    public:
        COMConstraint(std::vector<Var *>& var, kinematics::Skeleton* skel, const Eigen::Vector3d& val);
        virtual Eigen::VectorXd evalCon();
        virtual void fillJac(VVD, int){}
        virtual void fillJac(VVD, VVB, int);
        virtual void fillObjGrad(std::vector<double>&);

        void setTarget(const Eigen::Vector3d& target);
        Eigen::Vector3d getTarget() const;

    protected:
        Eigen::Vector3d mTarget;
        kinematics::Skeleton* mSkel;
    };
} // namespace optimizer
    
#endif // #ifndef COM_CONSTRAINT_H

