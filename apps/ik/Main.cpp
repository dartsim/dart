#include "MyWindow.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

using namespace kinematics;

int main(int argc, char* argv[]) 
{
    const char* modelfile;
    modelfile = DART_DATA_PATH"skel/fullbody1.skel";

    FileInfoSkel<Skeleton> model;
    model.loadFile(modelfile, SKEL);

    MyWindow window((Skeleton*)model.getSkel());
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Inverse Kinematics");
    glutMainLoop();
    return 0;
}
