#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include "dart/dynamics/Skeleton.h"


class MyWorld {
 public:
    MyWorld();
    virtual ~MyWorld();
    dart::dynamics::Skeleton* getSkel() {
        return mSkel;
    }

    void solve();
    void createConstraint(int _index);
    void modifyConstraint(Eigen::Vector3d _deltaP);
    void removeConstraint(int _index);

 protected:
    Eigen::VectorXd updateGradients();

    dart::dynamics::Skeleton *mSkel;
    Eigen::Vector3d mC;
    Eigen::MatrixXd mJ;
    Eigen::Vector3d mTarget; // The target location of the constriant
    int mConstrainedMarker; // The index of the constrained marker
};

#endif
