#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
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

namespace integration { class IntegrableSystem; }
namespace planning { class Controller; }

class MyWindow : public yui::Win3D, public integration::IntegrableSystem {
public:
    MyWindow();

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);

    // Needed for integration
    virtual Eigen::VectorXd getState();
    virtual Eigen::VectorXd evalDeriv();
    virtual void setState(Eigen::VectorXd state);	
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
    double mTime;

    bool mShowMarker;
    std::vector<Eigen::VectorXd> mBakedStates;

    integration::RK4Integrator mIntegrator;

    robotics::World* mWorld;
    std::vector<Eigen::VectorXd> mDofVels;
    std::vector<Eigen::VectorXd> mDofs;
    planning::Controller* mController;
    double mTimeStep;
    Eigen::Vector3d mGravity;
    std::vector<int> mIndices;

    void initDyn();
    void bake();
    void retrieveBakedState(int frame);
};

#endif
