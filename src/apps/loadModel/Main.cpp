#include "MyWindow.h"
#include "model3d/FileInfoModel.h"

using namespace model3d;

int main(int argc, char* argv[]) {
    FileInfoModel modelFile;
    modelFile.loadFile("Yuting.vsk", FileInfoModel::VSK);
    // modelFile.loadFile("Yuting.skel",FileInfoModel::SKEL);

    MyWindow window(modelFile.getModel());

    glutInit(&argc, argv);
    window.initWindow(640,480,"Load Model");
    glutMainLoop();
    return 0;
}
