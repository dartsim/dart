#ifndef SRC_UNITTESTS_TEST_MODEL3D_H
#define SRC_UNITTESTS_TEST_MODEL3D_H

#include "model3d/FileInfoSkel.hpp"
#include "model3d/Skeleton.h"
#include "model3d/BodyNode.h"
#include "model3d/FileInfoDof.h"
#include "model3d/FileInfoC3D.h"
#include "utils/Paths.h"

TEST(MODEL3D, VSK_LOADER) {
    using namespace Eigen;
    using namespace model3d;
  
    FileInfoSkel<Skeleton> modelFile;
    modelFile.loadFile(GROUNDZERO_DATA_PATH"skel/Yuting.vsk", model3d::VSK);
    // Skeleton* skel = modelFile.getSkel();

    // EXPECT_TRUE(skel != NULL);

    // EXPECT_EQ(skel->getNumDofs(), 66);
    // EXPECT_EQ(skel->getNumNodes(), 30);
    // EXPECT_EQ(skel->getNumHandles(), 53);
}

TEST(MODEL3D, C3D_LOADER) {
    using namespace Eigen;
    using namespace model3d;
  
    FileInfoC3D c3dFile;
    bool result = c3dFile.loadFile(GROUNDZERO_DATA_PATH"c3d/squat.c3d");
    ASSERT_TRUE(result);

    EXPECT_EQ(c3dFile.getNumMarkers(), 53);
    EXPECT_EQ(c3dFile.getNumFrames(), 2539);
}


TEST(MODEL3D, TRANS_AND_DERIV) {
    using namespace Eigen;
    using namespace model3d;
  
    FileInfoSkel<Skeleton> modelFile;
    bool loadModelResult = modelFile.loadFile(GROUNDZERO_DATA_PATH"skel/SehoonVSK3.vsk", model3d::VSK);
    ASSERT_TRUE(loadModelResult);
    
    Skeleton* skel = modelFile.getSkel();
    EXPECT_TRUE(skel != NULL);
    /* LOG(INFO) << "# Dofs = " << skel->getNumDofs(); */
    /* LOG(INFO) << "# Nodes  = " << skel->getNumNodes(); */


    FileInfoDof dofFile(skel);
    bool loadDofResult = dofFile.loadFile(GROUNDZERO_DATA_PATH"dof/init_Tpose.dof");
    ASSERT_TRUE(loadDofResult);
    /* LOG(INFO) << "# frames = " << dofFile.getNumFrames(); */

    vector<double>& pose = dofFile.getPoseAtFrame(0);
    skel->setPose(pose, true, true);

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
    const Matrix4d& Wq = skel->getNode(10)->mWq.at(4);
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


    // Check Jacobians
    BodyNode *nodecheck = skel->getNode(23);

    // Linear Jacobian
    const MatrixXd Jv = nodecheck->mJv;
    MatrixXd Jv_truth = MatrixXd::Zero(Jv.rows(), Jv.cols());
    Jv_truth<<1, 0, 0, 0.013694, -0.0194371, -0.422612, 0.0157281, -0.00961473, -0.421925, 0.0026645, -0.0048767, -0.179914, -0.0013539, -0.00132969, -0.00885535, 
        0, 1, 0, 0.0321939, 0.00226492, -0.135736, 0.0330525, 0.00462613, -0.135448, 0.0181491, 0.00758086, -0.122187, 0.00532694, 0.0049221, -0.128917,
        0, 0, 1, 0.423948, 0.130694, 0.0166546, 0.42642, 0.118176, 0.0221309, 0.180296, 0.120584, 0.0105059, 0.0838259, 0.081304, 0.00719314;
    for (int i = 0; i < Jv.rows(); i++) {
        for (int j = 0; j < Jv.cols(); j++) {
            EXPECT_NEAR(Jv(i, j), Jv_truth(i, j), TOLERANCE);
        }
    }

    // Angular Jacobian
    const MatrixXd Jwbody = nodecheck->mW.topLeftCorner(3,3).transpose()*nodecheck->mJw;
    MatrixXd Jwbody_truth = MatrixXd::Zero(Jwbody.rows(), Jwbody.cols());
    Jwbody_truth << 0, 0, 0, 0.0818662, 0.996527, 0.000504051, 0.110263, 0.993552, 0.0249018, 0.0772274, 0.995683, 0.0423836, 0.648386, 0.628838, 0.0338389,
        0, 0, 0, -0.995079, 0.082001, -0.0518665, -0.992054, 0.111479, -0.0554717, -0.996033, 0.078601, -0.0313861, -0.629201, 0.649145, -0.00994729,
        0, 0, 0, -0.0518265, 0.00391001, 0.998406, -0.0579139, -0.0187244, 0.998021, -0.0343359, -0.0398718, 0.998564, -0.0262454, -0.0235626, 0.999159;

    cout<<Jwbody<<endl;
    cout<<Jwbody_truth<<endl;

    for (int i = 0; i < Jwbody.rows(); i++) {
        for (int j = 0; j < Jwbody.cols(); j++) {
            EXPECT_NEAR(Jwbody(i, j), Jwbody_truth(i, j), TOLERANCE);
        }
    }
}


#endif // #ifndef SRC_UNITTESTS_TEST_MODEL3D_H

