#include "MyWindow.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

using namespace kinematics;

int main(int argc, char* argv[]) {
    FileInfoSkel<Skeleton> modelFile;
    modelFile.loadFile(GROUNDZERO_DATA_PATH"skel/Yuting.vsk", kinematics::VSK);
    // modelFile.loadFile("Yuting.skel",FileInfoModel::SKEL);

    MyWindow window(modelFile.getSkel());

    glutInit(&argc, argv);
    window.initWindow(640,480,"Load Model");
    glutMainLoop();
    return 0;
}
