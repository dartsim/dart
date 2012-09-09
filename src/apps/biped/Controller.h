#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <vector>
<<<<<<< HEAD

namespace dynamics{
    class SkeletonDynamics;
=======
#include <fstream>

namespace dynamics{
    class SkeletonDynamics;
    class ContactDynamics;
>>>>>>> karen
}

namespace kinematics{
    class BodyNode;
}

<<<<<<< HEAD
class Controller {
 public:
    Controller(dynamics::SkeletonDynamics *_skel, double _t);
=======
namespace optimizer{
    class LandingProblem;
    namespace snopt{
        class SnoptSolver;
    }
}

class Controller {
 public:
    enum State {
        TAKEOFF,
        AIRBORNE,
        LANDING,
        GROUND
    };

    Controller(dynamics::SkeletonDynamics *_skel, dynamics::ContactDynamics *_collisionHandle, double _t);
>>>>>>> karen
    virtual ~Controller() {};

    Eigen::VectorXd getTorques() { return mTorques; };
    double getTorque(int _index) { return mTorques[_index]; };
    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; };
<<<<<<< HEAD
    void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
=======
    Eigen::VectorXd computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
>>>>>>> karen
    dynamics::SkeletonDynamics* getSkel() { return mSkel; };
    Eigen::VectorXd getDesiredDofs() { return mDesiredDofs; };
    Eigen::MatrixXd getKp() {return mKp; };
    Eigen::MatrixXd getKd() {return mKd; };
<<<<<<< HEAD

 protected:
    double computeMassTree(kinematics::BodyNode *_bd);
    dynamics::SkeletonDynamics *mSkel;
=======
    std::vector<Eigen::Vector3d> mLinHist;
    std::vector<Eigen::Vector3d> mAngHist;
    std::vector<Eigen::Vector3d> mLaunchLin;
    std::vector<Eigen::Vector3d> mLaunchAng;
    std::vector<Eigen::Vector3d> mLandingLin;
    std::vector<Eigen::Vector3d> mLandingAng;

    void setConstrForces(const Eigen::VectorXd& _constrForce) { mConstrForces = _constrForce; }

 public: // being sloppy here. should change back to "protected"
    double computeMassTree(kinematics::BodyNode *_bd);
    void prepareJump(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
    void computeLandingPose();
    void trackLandingPose(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
    void maintainBalance(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
    void detectJumpState();
    Eigen::Vector3d evalLinMomentum(const Eigen::VectorXd& _dofVel);
    Eigen::Vector3d evalAngMomentum(const Eigen::VectorXd& _dofVel);
    void correctTorque(Eigen::VectorXd _deltaMomentum, Eigen::VectorXd _controlledAxis, bool _upperBodyOnly);
    Eigen::VectorXd verifyTorque(Eigen::VectorXd _torque);
    Eigen::Vector3d checkContactForceValidity(Eigen::Vector3d _force, Eigen::Vector3d _normal, double _angle);
    Eigen::VectorXd computeVirtualTorque(Eigen::Vector3d _vForce);


    dynamics::SkeletonDynamics *mSkel;
    dynamics::ContactDynamics *mCollisionHandle;
>>>>>>> karen
    Eigen::VectorXd mTorques;
    Eigen::VectorXd mDesiredDofs;
    Eigen::VectorXd mMassTree;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd; // For SPD, we use mKd defined in skeleton
    int mFrame;
<<<<<<< HEAD
    double mTimestep;
=======
    int mPrepareFrame;
    int mLandingFrame;
    double mTimestep;
    optimizer::LandingProblem *mProblem;
    optimizer::snopt::SnoptSolver *mSolver;
    State mJumpState;
    Eigen::VectorXd mLandingPose;
    std::ofstream mOutFile;
    Eigen::VectorXd mControlBias;
    Eigen::VectorXd mConstrForces;

>>>>>>> karen
};
    
    

#endif // #CONTROLLER_H
