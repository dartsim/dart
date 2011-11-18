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
        ContactDynamic(const std::vector<SkeletonDynamics*>& _skels, double _dt, double _mu = 0.8, int _d = 4);
        virtual ~ContactDynamic();
        void applyContactForces();
    private:
        void initialize();
        void destroy();
        
        void fillMatrices();
        bool solve();
        void applyExternalForces();

        inline int getNumSkels() const { return mSkels.size(); }
        inline int getNumTotalDofs() const { return mIndices[getNumSkels()]; }
        inline int getNumContactDirections() const { return mNumDir; }
        int getNumContacts() const;

        Eigen::MatrixXd getJacobian(kinematics::BodyNode* node,
                                    const Eigen::Vector3d& p) const;

        // Helper functions to compute all of the matrices
        // Notation is similar to that used in derivation:
        // Mqddot + Cqdot + kq = tau + (J^T)(f_n)N + (J^T)D(f_d) -> Mqdot = tau* + N(f_n) + B(f_d)
        inline Eigen::MatrixXd getMassMatrix() const { return mM; } // M matrix
        inline Eigen::VectorXd getTauStarVector() const { return mTauStar; } // T* vector (not T)
        Eigen::MatrixXd getNormalMatrix() const; // N matrix
        Eigen::MatrixXd getBasisMatrix() const; // B matrix 
        Eigen::MatrixXd getTangentBasisMatrix(const Eigen::Vector3d& p, const Eigen::Vector3d& n) const; // gets a matrix of tangent dirs.
        Eigen::MatrixXd getContactMatrix() const; // E matrix
        Eigen::MatrixXd getMuMatrix() const; // mu matrix
        
        std::vector<SkeletonDynamics*> mSkels;
        std::vector<int> mBodyIndexToSkelIndex;
        std::vector<int> mIndices;
        collision_checking::SkeletonCollision* mCollision;
        double mDt; // timestep
        double mMu; // friction coeff.
        int mNumDir; // number of basis directions

        // Cached (aggregated) mass/tau matrices
        Eigen::MatrixXd mM;
        Eigen::VectorXd mTauStar;

        // Matrices to pass to solver
        Eigen::MatrixXd mA;
        Eigen::VectorXd mQBar;
        Eigen::VectorXd mX;
    }; 
} // namespace dynamics

#endif // #ifndef DYNAMICS_CONTACT_DYNAMICS_H

