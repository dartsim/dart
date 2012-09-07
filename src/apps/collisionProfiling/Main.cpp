#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "utils/Paths.h"
#include <time.h>
#include "collision/collision_skeleton.h"
#include "kinematics/FileInfoSkel.hpp"

// To load Mesh and Skel
#include <kinematics/Joint.h>
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
    geometry::Mesh3DTriangle* m3d = new geometry::Mesh3DTriangle(DART_DATA_PATH"obj/polarBear.obj", geometry::Mesh3D::OBJ);
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
    const int n = 2;
    const int trials = 10;
    const double stepsize = 0.01;
	const int numFrames = 2001; //1 + (int)(trials * (2.0 / stepsize))
    
    Eigen::VectorXd pose;

    collision_checking::SkeletonCollision* mSkeletonCollision;

    clock_t frameTimes[numFrames];
    int collisionsDetected[numFrames];
    int frame = 0;

    // Create some skeletons, fill them up with polar bears, and
    // initialize them
    dynamics::SkeletonDynamics* MeshSkels[(n * 2) + 1];
    for (int i = 0; i < (2 * n) + 1; i++)
    {
        MeshSkels[i] = new dynamics::SkeletonDynamics();
        makePolarBear("bear", MeshSkels[i]);
        MeshSkels[i]->initSkel();
        // make sure each skeleton has something inside
        printf( "Skeleton %d has nodes, %d joints, and %d DOFs \n",
                i,
                MeshSkels[i]->getNumNodes(),
                MeshSkels[i]->getNumJoints(),
                MeshSkels[i]->getNumDofs());
        MeshSkels[i]->setImmobileState(true);
    }
    mSkeletonCollision = new collision_checking::SkeletonCollision();
    for (unsigned int i = 0; i < (n * 2) + 1; i++) {
        for (unsigned int j = 0; j < MeshSkels[i]->getNumNodes(); j++) {
            mSkeletonCollision->addCollisionSkeletonNode(MeshSkels[i]->getNode(j), false);
        }
    }

    clock_t startTime = clock();
    for (int trial = 0; trial < trials; trial++)
        for (double position = 1.0; position > -1.0; position -= stepsize)
        {
            clock_t frameStartTime = clock();
            for (int i = -n; i < n; i++)
            {
                pose.resize(MeshSkels[n+i]->getNumDofs());
                pose[2] = position * (double)i;
                MeshSkels[n+i]->setPose(pose, true, false);
            }
            mSkeletonCollision->checkCollision();
            clock_t frameEndTime = clock();
            frameTimes[frame] = frameEndTime - frameStartTime;
            collisionsDetected[frame++] = mSkeletonCollision->getNumContact();
        }
    clock_t endTime = clock();

    std::cout << "Time: " << (double)(endTime - startTime) / (double) CLOCKS_PER_SEC / (double)trials << std::endl;
    std::cout << "frame,seconds,collisions" << std::endl;
    for (int i = 0; i < 1 + (int)(trials * (2.0 / stepsize)); i++)
        cout <<
            i << "," <<
            (double)frameTimes[i] / (double)CLOCKS_PER_SEC << "," <<
            collisionsDetected[i] << std::endl;
    
    return 0;
}
