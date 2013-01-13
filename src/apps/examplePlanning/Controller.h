#pragma once

#include <vector>
#include <Eigen/Core>

namespace dynamics { class SkeletonDynamics; }

namespace planning {

class Trajectory;

class Controller {
public:
    Controller(dynamics::SkeletonDynamics* _skel, const std::vector<int> &_actuatedDofs,
               const Eigen::VectorXd &_kP, const Eigen::VectorXd &_kD, const std::vector<int> &_ankleDofs, const Eigen::VectorXd &_anklePGains, const Eigen::VectorXd &_ankleDGains);
    virtual ~Controller() {};

    void setTrajectory(const Trajectory* _trajectory, double _startTime, const std::vector<int> &_dofs);

    // Returns zero torque for nonactuated DOFs
    Eigen::VectorXd getTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel, double _time);

protected: 
    Eigen::Vector3d evalAngMomentum(const Eigen::VectorXd& _dofVel);
    Eigen::VectorXd adjustAngMomentum(Eigen::VectorXd _deltaMomentum, Eigen::VectorXd _controlledAxis);

    dynamics::SkeletonDynamics* mSkel;
    std::vector<int> mTrajectoryDofs;
    Eigen::VectorXd mDesiredDofs;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd;
    Eigen::MatrixXd mSelectionMatrix;
    const Trajectory* mTrajectory;
    double mStartTime;
    double mPreOffset;
    std::vector<int> mAnkleDofs;
    Eigen::VectorXd mAnklePGains;
    Eigen::VectorXd mAnkleDGains;
};

}