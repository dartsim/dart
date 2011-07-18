#include <iostream>
using namespace std;
#include <glog/logging.h>
#include <gflags/gflags.h>
using namespace google;

#include "model3d/Skeleton.h"
#include "model3d/FileInfoC3D.h"
#include "model3d/FileInfoDof.h"
using namespace model3d;
#include "IKProblem.h"
#include "PositionConstraint.h"
#include "optimizer/ObjectiveBox.h"
using namespace optimizer;

#include "optimizer/snopt/SnoptSolver.h"

using namespace Eigen;

int main(int argc, char* argv[]) {
    // Init google libraries
    ParseCommandLineFlags(&argc, &argv, true);
    InitGoogleLogging(argv[0]);

    // Define logging flag
    FLAGS_alsologtostderr = true;
    FLAGS_minloglevel = INFO;
    FLAGS_v = 1;

    LOG(INFO) << "simpleik begins";

    CHECK(1 + 2 == 3) << "more explanation";


    IKProblem prob;

    FileInfoC3D c3dFile;
    bool result = c3dFile.loadFile("./squat.c3d");
    CHECK(result);

    FileInfoDof resultDof(prob.getModel());

    snopt::SnoptSolver solver(&prob);
    for (int i = 0; i < c3dFile.getNumFrames(); i++) {
        LOG(INFO) << "Frame Index = " << i;
        for (int j = 0; j < c3dFile.getNumMarkers(); j++) {
            PositionConstraint* p = dynamic_cast<PositionConstraint*>(prob.objBox()->getConstraint(j));
            Vector3d target = c3dFile.getDataAt(i, j);
            p->setTarget(target);
        }
        LOG(INFO) << "Update OK";

        solver.solve();

        vector<double> pose;
        prob.getModel()->getPose(pose);
        resultDof.addDof(pose);
    }
    resultDof.saveFile("./result.dof", 0, resultDof.getNumFrames());
    LOG(INFO) << "Save OK";

    LOG(INFO) << "simpleik OK";
}
