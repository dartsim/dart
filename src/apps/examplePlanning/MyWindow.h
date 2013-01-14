#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "collision/CollisionShapes.h"
#include "collision/CollisionSkeleton.h"
#include "utils/Parser/dart_parser/DartLoader.h"
#include "robotics/World.h"
#include "utils/Paths.h"
#include "robotics/World.h"
#include "dynamics/SkeletonDynamics.h"
#include "robotics/Object.h"
#include "kinematics/ShapeCube.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/Joint.h"
#include "kinematics/ShapeMesh.h"

using namespace collision_checking;
using namespace std;

namespace dynamics{
    class SkeletonDynamics;
    class ContactDynamics;
}

namespace planning { class Controller; }

class MyWindow : public yui::Win3D {
public:
    MyWindow();

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);

protected:
    enum playstate_enum {
        SIMULATE,
        RECORD,
        PLAYBACK,
        PAUSED
    };
    playstate_enum mPlayState;
    playstate_enum mPlayStateLast;
    int mSimFrame;
    int mPlayFrame;
    int mMovieFrame;
    bool mScreenshotScheduled;

    bool mShowMarker;
    std::vector<Eigen::VectorXd> mBakedStates;

    robotics::World* mWorld;
    planning::Controller* mController;

    void bake();
    void retrieveBakedState(int frame);
};

#endif
