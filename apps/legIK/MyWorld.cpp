#include "MyWorld.h"
#include "utils/Paths.h"

using namespace Eigen;
using namespace kinematics;

MyWorld::MyWorld() {

    // Load a skeleton from file
    const char* modelfile;
    modelfile = DART_DATA_PATH"skel/leg.skel";
    mSkelInfo.loadFile(modelfile, SKEL);

    mSkel = (Skeleton*)mSkelInfo.getSkel();
}

MyWorld::~MyWorld() {
    delete mSkel;
}

// TODO: implement the IK solver.
void MyWorld::solve() {
    
}
