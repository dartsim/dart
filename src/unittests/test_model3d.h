#ifndef SRC_UNITTESTS_TEST_MODEL3D_H
#define SRC_UNITTESTS_TEST_MODEL3D_H

#include "model3d/FileInfoModel.h"
#include "model3d/Skeleton.h"

TEST(MODEL3D, VSK) {
  using namespace model3d;
  
  FileInfoModel modelFile;
  modelFile.loadFile("Yuting.vsk", FileInfoModel::VSK);
  Skeleton* skel = modelFile.getSkel();

  EXPECT_TRUE(skel != NULL);

  EXPECT_EQ(skel->getNumDofs(), 66);
  EXPECT_EQ(skel->getNumNodes(), 30);
  EXPECT_EQ(skel->getNumHandles(), 53);
  
  

}

#endif // #ifndef SRC_UNITTESTS_TEST_MODEL3D_H

