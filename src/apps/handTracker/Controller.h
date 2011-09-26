#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <vector>

class Controller {
 public:
    Controller(int _nDof) { 
        mTorques.resize(_nDof);
        mDesiredDofs.resize(_nDof);
        mKd.resize(_nDof);
        mKs.resize(_nDof);
        for (int i = 0; i < _nDof; i++) {
            mKs[i] = 250.0;
            mKd[i] = 2 * sqrt(250.0);
        }
    };
    virtual ~Controller() {};

    Eigen::VectorXd getTorques() { return mTorques; };
    double getTorque(int _index) { return mTorques[_index]; };
    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; };
    void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
 
 protected:
    Eigen::VectorXd mTorques;
    Eigen::VectorXd mDesiredDofs;
    std::vector<double> mKs;
    std::vector<double> mKd;
};
    
    

#endif // #CONTROLLER_H
