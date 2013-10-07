#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/Skeleton.h"
#include "kinematics/BodyNode.h"

namespace kinematics {
    class Skeleton;
}


class MyWorld {
 public:
    MyWorld();
    virtual ~MyWorld();
    kinematics::Skeleton* getSkel() {
        return mSkel;
    }

    // TODO: solve IK problem. You can change the function prototype as you see fit.
    void solve();

 protected:
    kinematics::FileInfoSkel<kinematics::Skeleton> mSkelInfo;
    kinematics::Skeleton *mSkel;
};

#endif
