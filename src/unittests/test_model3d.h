#ifndef SRC_UNITTESTS_TEST_MODEL3D_H
#define SRC_UNITTESTS_TEST_MODEL3D_H

#include "model3d/FileInfoModel.h"
#include "model3d/Skeleton.h"
#include "model3d/BodyNode.h"

TEST(MODEL3D, VSK) {
    using namespace Eigen;
    using namespace model3d;
  
    FileInfoModel modelFile;
    modelFile.loadFile("Yuting.vsk", FileInfoModel::VSK);
    Skeleton* skel = modelFile.getModel();

    EXPECT_TRUE(skel != NULL);

    EXPECT_EQ(skel->getNumDofs(), 66);
    EXPECT_EQ(skel->getNumNodes(), 30);
    EXPECT_EQ(skel->getNumHandles(), 53);

    // Pick any node
    BodyNode* node = skel->getNode(18);
    Vector3d nodePos = node->evalWorldPos(Vector3d::Zero());

    const double TOLERANCE = 0.000001;
    EXPECT_NEAR(nodePos(0), 0.396699, TOLERANCE);
    EXPECT_NEAR(nodePos(1), 0.000000, TOLERANCE);
    EXPECT_NEAR(nodePos(2), 0.356131, TOLERANCE);
}

#endif // #ifndef SRC_UNITTESTS_TEST_MODEL3D_H

