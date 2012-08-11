#include "MyWindow.h"
#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/BodyNode.h"
#include "kinematics/Shape.h"
#include "utils/Paths.h"
#include <fstream>
#include <iostream>

using namespace kinematics;
using namespace dynamics;
using namespace Eigen;
using namespace std;

int main(int argc, char* argv[])
{
    /*
    ifstream infile;
    infile.open("sphere1.obj");
    infile.precision(10);
    char buffer[80];
    vector<Vector3d> vertices; 
    Vector3d v;
    vector<Vector3d> faces; 
    Vector3d f;
    infile >> buffer;
    while (buffer[0] == 'v') {
        for (int i = 0; i < 3; i++) {
            infile >> buffer;
            v[i] = atof(buffer);
        }
        vertices.push_back(v);
        infile >> buffer;
    }
    while (buffer[0] != 'f')
        infile >> buffer;
    while (!infile.eof()) {
        infile >> buffer;
        char *position = strchr(buffer, '/');
        position = NULL;
        f[0] = atof(buffer);
        infile >> buffer;
        position = strchr(buffer, '/');
        position = NULL;
        f[1] = atof(buffer);
        infile >> buffer;
        position = strchr(buffer, '/');
        position = NULL;
        f[2] = atof(buffer);
        faces.push_back(f);
        infile >> buffer;
    }
    for (int i = 0; i < vertices.size(); i++) 
        cout << "{" << vertices[i][0] << ", " << vertices[i][1] << ", " << vertices[i][2] << "}," << endl;
    
    for (int i = 0; i < faces.size(); i++) 
        cout << "{" << faces[i][0] << ", " << faces[i][1] << ", " << faces[i][2] << "}," << endl;
    */
  
    FileInfoSkel<SkeletonDynamics> model, model2, model3;
    model.loadFile(DART_DATA_PATH"/skel/ground1.skel", SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/manipulator.skel", SKEL);
    model3.loadFile(DART_DATA_PATH"/skel/sphere.skel", SKEL);

    Vector3d red(1.0, 0.0, 0.0);
    Vector3d gray(0.9, 0.9, 0.9);
    model.getSkel()->getNode(0)->getShape()->setColor(gray);
    model3.getSkel()->getNode(0)->getShape()->setColor(red);

    MyWindow window((SkeletonDynamics*)model.getSkel(), (SkeletonDynamics*)model2.getSkel(), (SkeletonDynamics*)model3.getSkel(), NULL);
   
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Manipulator");
    glutMainLoop();

    return 0;
}
