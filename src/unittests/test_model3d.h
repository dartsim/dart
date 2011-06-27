#ifndef SRC_UNITTESTS_TEST_MODEL3D_H
#define SRC_UNITTESTS_TEST_MODEL3D_H

#include "model3d/FileInfoModel.h"
#include "model3d/Skeleton.h"
#include "model3d/BodyNode.h"
#include "model3d/FileInfoDof.h"
#include "model3d/FileInfoC3D.h"

TEST(MODEL3D, VSK_LOADER) {
    using namespace Eigen;
    using namespace model3d;
  
    FileInfoModel modelFile;
    modelFile.loadFile("Yuting.vsk", FileInfoModel::VSK);
    Skeleton* skel = modelFile.getModel();

    EXPECT_TRUE(skel != NULL);

    EXPECT_EQ(skel->getNumDofs(), 66);
    EXPECT_EQ(skel->getNumNodes(), 30);
    EXPECT_EQ(skel->getNumHandles(), 53);
}

TEST(MODEL3D, C3D_LOADER) {
    using namespace Eigen;
    using namespace model3d;
  
    FileInfoC3D c3dFile;
    bool result = c3dFile.loadFile("./squat.c3d");
    EXPECT_TRUE(result);

    EXPECT_EQ(c3dFile.getNumMarkers(), 53);
    EXPECT_EQ(c3dFile.getNumFrames(), 2539);
}


TEST(MODEL3D, TRANS_AND_DERIV) {
    using namespace Eigen;
    using namespace model3d;
  
    FileInfoModel modelFile;
    bool result = false;
    result = modelFile.loadFile("./SehoonVSK3.vsk", FileInfoModel::VSK);
    EXPECT_TRUE(result);
    
    Skeleton* skel = modelFile.getModel();
    EXPECT_TRUE(skel != NULL);
    /* LOG(INFO) << "# Dofs = " << skel->getNumDofs(); */
    /* LOG(INFO) << "# Nodes  = " << skel->getNumNodes(); */


    FileInfoDof dofFile(skel);
    result = dofFile.loadFile("./init_Tpose.dof");
    EXPECT_TRUE(result);
    /* LOG(INFO) << "# frames = " << dofFile.getNumFrames(); */

    vector<double>& pose = dofFile.getPoseAtFrame(0);
    skel->setState(pose);

    const int nodeIndex = 1;
    BodyNode* node = skel->getNode(nodeIndex);
    EXPECT_TRUE(node != NULL);

    const double TOLERANCE = 0.000001;

    // Check the one global transform matrix
    const Matrix4d& W = skel->getNode(20)->getWorldTransform();
    Matrix4d W_truth;
    W_truth << -0.110662, 0.991044, 0.074734, 0.642841
        , -0.987564, -0.118099, 0.103775, 0.327496
        , 0.111672, -0.0623207, 0.991789, -0.12855
        , 0, 0, 0, 1;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            EXPECT_NEAR(W(i, j), W_truth(i, j), TOLERANCE);
        }
    }


    // Check the derivative of one global transform matrix
    const Matrix4d& Wq = skel->getNode(10)->Wq.at(4);
    Matrix4d Wq_truth;
    Wq_truth << 0.121382, 0.413015, 0.902378, 0.161838
        ,-0.0175714, 0.00698451, 0.0149899, 0.00571836
        ,-0.992336, 0.0556535, 0.107702, 0.175059
        ,0, 0, 0, 0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            EXPECT_NEAR(Wq(i, j), Wq_truth(i, j), TOLERANCE);
        }
    }

    
}


#endif // #ifndef SRC_UNITTESTS_TEST_MODEL3D_H

