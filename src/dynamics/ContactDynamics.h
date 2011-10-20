#ifndef DYNAMICS_CONTACT_DYNAMICS_H
#define DYNAMICS_CONTACT_DYNAMICS_H

#include <vector>
#include <Eigen/Dense>

namespace kinematics {
    class BodyNode;
} // namespace kinematics

namespace collision_checking {
    class SkeletonCollision;
} // namespace collision

namespace lcpsolver {
    class LCPSolver;
} //namespace lcpsolver


/*
  // Sample Usage
  dynamics::ContactDynamic contacts(skels, dt);
  contacts.applyContactForces();
 */
namespace dynamics {
    class SkeletonDynamics;
    
    class ContactDynamic {
    public:
        ContactDynamic(const std::vector<SkeletonDynamics*>& _skels, double _dt, double _mu = 1.0);
        virtual ~ContactDynamic();
        void applyContactForces();
    private:
        void initialize();
        void destroy();
        
        void fillMatrices();
        void solve();
        void applyExternalForces();

        inline int getNumSkels() const { return mSkels.size(); }
        inline int getNumTotalDofs() const { return mIndices[getNumSkels()]; }
        int getNumContacts() const;

        Eigen::MatrixXd getJacobian(kinematics::BodyNode* node,
                                    const Eigen::Vector3d& p) const;
        Eigen::MatrixXd getNormalMatirx() const;
        
        std::vector<SkeletonDynamics*> mSkels;
        std::vector<int> mBodyIndexToSkelIndex;
        std::vector<int> mIndices;
        collision_checking::SkeletonCollision* mCollision;
        double mDt;
        double mMu;
    }; 
} // namespace dynamics

#endif // #ifndef DYNAMICS_CONTACT_DYNAMICS_H

