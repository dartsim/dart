#ifndef POSITION_CONSTRAINT_H
#define POSITION_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace model3d {
    class Skeleton;
    class BodyNode;
} // namespace model3d

namespace optimizer {
    class Var;

    class PositionConstraint : public Constraint {
    public:
        PositionConstraint(std::vector<Var *>& var, model3d::Skeleton* skel, model3d::BodyNode* node,
                           const Eigen::Vector3d& offset,
                           const Eigen::Vector3d& val);

        virtual Eigen::VectorXd evalCon();
        virtual void fillJac(VVD, int){}
        virtual void fillJac(VVD, VVB, int);
        virtual void fillObjGrad(std::vector<double>&);

        void setTarget(const Eigen::Vector3d& target);
        Eigen::Vector3d getTarget() const;

    protected:
        Eigen::Vector3d mTarget;
        Eigen::Vector3d mOffset;

        model3d::Skeleton* mSkel;
        model3d::BodyNode* mNode;
    };
} // namespace optimizer
    
#endif // #ifndef POSITION_CONSTRAINT_H

