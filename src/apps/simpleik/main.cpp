#include <iostream>
#include <string>
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

#include "utils/Paths.h"

DEFINE_string(skel, GROUNDZERO_DATA_PATH"skel/Nick01.vsk",
              "The input Skel file");
DEFINE_string(c3d, GROUNDZERO_DATA_PATH"c3d/nick_freeform_001.c3d",
              "The input C3D file");
DEFINE_string(dof, GROUNDZERO_DATA_PATH"dof/result.dof",
              "The output dof file");


int main(int argc, char* argv[]) {
    // Init google libraries
    ParseCommandLineFlags(&argc, &argv, true);
    InitGoogleLogging(argv[0]);

    // Define logging flag
    FLAGS_alsologtostderr = true;
    FLAGS_minloglevel = INFO;
    FLAGS_v = 1;

    string filename_skel = FLAGS_skel;
    string filename_c3d = FLAGS_c3d;
    string filename_dof = FLAGS_dof;

    LOG(INFO) << "filename_skel = " << filename_skel;
    LOG(INFO) << "filename_c3d  = " << filename_c3d;
    LOG(INFO) << "filename_dof  = " << filename_dof;

    LOG(INFO) << "simpleik begins";

    IKProblem prob(filename_skel.c_str());

    FileInfoC3D c3dFile;
    bool result = c3dFile.loadFile(filename_c3d.c_str());
    CHECK(result);

    FileInfoDof resultDof(prob.getSkel());

    snopt::SnoptSolver solver(&prob);
    for (int i = 0; i < c3dFile.getNumFrames(); i++) {
        LOG(INFO) << "Frame Index = " << i;
        for (int j = 0; j < c3dFile.getNumMarkers(); j++) {
            // PositionConstraint* p = dynamic_cast<PositionConstraint*>(prob.objBox()->getConstraint(j));

            PositionConstraint* p = prob.getConstraint(j);

            Vector3d target = c3dFile.getDataAt(i, j);
            p->setTarget(target);

            bool isInBox = (prob.objBox()->isInBox(p) != -1);
            bool isMissing = (target.norm() < 0.001);

            if (isMissing && isInBox) {
                int result = prob.objBox()->remove(p); 
                LOG(INFO) << "\tDetect missing marker " << j << " ";
                LOG(INFO) << "TakeOut() = " << result << " "
                          << "size = " << prob.objBox()->getNumConstraints();

            }
            else if (!isMissing && !isInBox) {
                prob.objBox()->add(p);
                LOG(INFO) << "\tWe got the marker " << j << " back!!! ";
                LOG(INFO) << "size = " << prob.objBox()->getNumConstraints();
            }

        }
        LOG(INFO) << "Update OK";

        solver.solve();

        vector<double> pose;
        prob.getSkel()->getPose(pose);
        resultDof.addDof(pose);
    }
    
    resultDof.saveFile(filename_dof.c_str(), 0, resultDof.getNumFrames());
    LOG(INFO) << "Save the result to [" << filename_dof.c_str() << "]";
    LOG(INFO) << "Save OK";

    LOG(INFO) << "simpleik OK";
}
