#include "MyWindow.h"
#include "model3d/FileInfoSkel.h"

using namespace model3d;

int main(int argc, char* argv[]) {
    FileInfoSkel modelFile;
    modelFile.loadFile("Yuting.vsk", FileInfoSkel::VSK);
    // modelFile.loadFile("Yuting.skel",FileInfoModel::SKEL);

    MyWindow window(modelFile.getSkel());

    glutInit(&argc, argv);
    window.initWindow(640,480,"Load Model");
    glutMainLoop();
    return 0;
}
