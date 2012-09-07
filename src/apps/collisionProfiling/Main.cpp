#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

// To load Mesh and Skel
#include  <kinematics/Joint.h>
#include <kinematics/ShapeMesh.h>
#include <geometry/Mesh3DTriangle.h>
#include <kinematics/Transformation.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/TrfmRotateEuler.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Dof.h>

    


using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

void makePolarBear(const char* name, dynamics::SkeletonDynamics* MeshSkel)
{
    char nameBuf[512];

    // Always set the root node ( 6DOF for rotation and translation )
    kinematics::Joint* joint;
    dynamics::BodyNodeDynamics* node;
    kinematics::Transformation* trans;

    // Set the initial Rootnode that controls the position and orientation of the whole robot
    strcpy(nameBuf, name); strcat(nameBuf, "Node");
    node = (dynamics::BodyNodeDynamics*) MeshSkel->createBodyNode(nameBuf);
    strcpy(nameBuf, name); strcat(nameBuf, "Joint");
    joint = new kinematics::Joint( NULL, node, nameBuf);
    
    // Add RPY and XYZ of the whole robot
    strcpy(nameBuf, name); strcat(nameBuf, "X");
    trans = new kinematics::TrfmTranslateX( new kinematics::Dof( 0, nameBuf ), "Tx" );
    joint->addTransform( trans, true );
    MeshSkel->addTransform( trans );

    strcpy(nameBuf, name); strcat(nameBuf, "Y");
    trans = new kinematics::TrfmTranslateY( new kinematics::Dof( 0, nameBuf ), "Ty" );
    joint->addTransform( trans, true );
    MeshSkel->addTransform( trans );
    
    strcpy(nameBuf, name); strcat(nameBuf, "Z");
    trans = new kinematics::TrfmTranslateZ( new kinematics::Dof( 0, nameBuf ), "Tz" );
    joint->addTransform( trans, true );
    MeshSkel->addTransform( trans );

    strcpy(nameBuf, name); strcat(nameBuf, "Yaw");
    trans = new kinematics::TrfmRotateEulerZ( new kinematics::Dof( 0, nameBuf ), "Try" );
    joint->addTransform( trans, true );
    MeshSkel->addTransform( trans );

    strcpy(nameBuf, name); strcat(nameBuf, "Pitch");
    trans = new kinematics::TrfmRotateEulerY( new kinematics::Dof( 0, nameBuf ), "Trp" );
    joint->addTransform( trans, true );
    MeshSkel->addTransform( trans );

    strcpy(nameBuf, name); strcat(nameBuf, "Roll");
    trans = new kinematics::TrfmRotateEulerX( new kinematics::Dof( 0, nameBuf ), "Trr" );
    joint->addTransform( trans, true );
    MeshSkel->addTransform( trans );

    //  Create Shape and assign it to node
    kinematics::ShapeMesh *BearShape = new kinematics::ShapeMesh( Eigen::Vector3d(0, 0, 0), 0 );
    node->setShape( BearShape );

    // Load a Mesh3DTriangle to save in Shape
    geometry::Mesh3DTriangle* m3d = new geometry::Mesh3DTriangle(DART_DATA_PATH"/obj/polarBear.obj", geometry::Mesh3D::OBJ);
    printf("Read mesh data. Resulting mesh has: %d vertices, %d faces\n", m3d->mNumVertices, m3d->mNumFaces);

    // Save Mesh3D in Shape (vizMesh)
    BearShape->setVizMesh( m3d );
    BearShape->setCollisionMesh( m3d );
    BearShape->setMass(1);
    Matrix3d M; M << 0.000416667, 0.0, 0.0, 0.0, 0.000416667, 0.0, 0.0, 0.0, 0.000416667;
    BearShape->setInertia(M);

    // Add node to Skel
    MeshSkel->addNode( node );
}


int main(int argc, char* argv[])
{
    // **********
    //-- Create a skeleton
    dynamics::SkeletonDynamics MeshSkel1;
    dynamics::SkeletonDynamics MeshSkel2;
    dynamics::SkeletonDynamics MeshSkel3;
    dynamics::SkeletonDynamics MeshSkel4;
    dynamics::SkeletonDynamics MeshSkel5;

    // fill it up with polar bears
    makePolarBear("bear1", &MeshSkel1);
    makePolarBear("bear2", &MeshSkel2);
    makePolarBear("bear3", &MeshSkel3);
    makePolarBear("bear4", &MeshSkel4);
    makePolarBear("bear5", &MeshSkel5);

    //-- Initialize mySkeleton
    MeshSkel1.initSkel();
    MeshSkel2.initSkel();
    MeshSkel3.initSkel();
    MeshSkel4.initSkel();
    MeshSkel5.initSkel();

    // // Verify that our skeleton has something inside :)
    // printf( "Our skeletons have %d nodes \n", MeshSkel.getNumNodes() );
    // printf( "Our skeletons have %d joints \n", MeshSkel.getNumJoints() );
    // printf( "Our skeletons have %d DOFs \n", MeshSkel.getNumDofs() );

    MyWindow window(&MeshSkel1, &MeshSkel2, &MeshSkel3, &MeshSkel4, &MeshSkel5, NULL);

    cout << "space bar: simulation on/off" << endl;
    cout << "'s': simulate one step" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << "'1' and '2': programmed interaction" << endl;
    
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Collision Profiler");
    glutMainLoop();

    return 0;
}
