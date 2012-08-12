/**
  * @file Main.cpp
  * @author A. Huaman and C. Erdogan
  * @date 2012-08-12
  */

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
#include <robotics/Constants.h>

using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

// Function headers
enum TypeOfDOF { GOLEM_X, GOLEM_Y, GOLEM_Z, GOLEM_ROLL, GOLEM_PITCH, GOLEM_YAW };
void  add_XyzRpy( kinematics::Joint* _joint, double _x, double _y, double _z, double _rr, double _rp, double _ry );
void add_DOF( dynamics::SkeletonDynamics* _skel, kinematics::Joint* _joint,  double _val, int _DOF_TYPE );
void   add_Shape( dynamics::BodyNodeDynamics* _node, const char *_meshObjPath, double _mass, Eigen::Matrix3d _inertiaMatrix );

/**
  * @function main
  */
int main(int argc, char* argv[])
{
    // **********
   //-- Create Left Leg skeleton
   dynamics::SkeletonDynamics LeftLegSkel;

    // Pointers to be used during the Skeleton building
    kinematics::Joint* joint;
    dynamics::BodyNodeDynamics* node;
    dynamics::BodyNodeDynamics* parent_node;

    double x, y, z, roll, pitch, yaw; // For Local rigid transformations
    double val; // For DOF
    Eigen::Matrix3d inertiaMatrix;
    double mass;

    //-- *****  BodyNode 1: Left Hip Yaw (LHY) *****
    node = (dynamics::BodyNodeDynamics*) LeftLegSkel.createBodyNode("LHY");
    joint = new kinematics::Joint( NULL, node, "LHY" );

   // Add rigids displacement (local)
    x = 0; y = 0; z = 0; roll = 0; pitch = 0; yaw = 0;
	 add_XyzRpy( joint, x, y, z, roll, pitch, yaw );

   // Add DOF
   val = 0;
   add_DOF( &LeftLegSkel, joint, val, GOLEM_YAW );    

   // Add Shape
   inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
   mass = 1.0;
  add_Shape( node, DART_DATA_PATH"obj/leftLeg/Body_LHY.obj", mass, inertiaMatrix ); 

   // Add node to Skel
   LeftLegSkel.addNode( node );

    //-- ***** BodyNode 2: Left Hip Roll (LHR)  whose parent is: LHY*****
    parent_node = (dynamics::BodyNodeDynamics*) LeftLegSkel.getNode( "LHY" );
    node = (dynamics::BodyNodeDynamics*) LeftLegSkel.createBodyNode("LHR");
    joint = new kinematics::Joint( parent_node, node, "LHR" );

   // Add rigids displacement (local)
    x = 0; y = 0; z = 0; roll = 0; pitch = 0; yaw = 0;
	 add_XyzRpy( joint, x, y, z, roll, pitch, yaw );

   // Add DOF
   val = 0;
   add_DOF( &LeftLegSkel, joint, val, GOLEM_YAW );    

   // Add Shape
   inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
   mass = 1.0;
  add_Shape( node, DART_DATA_PATH"obj/leftLeg/Body_LHR.obj", mass, inertiaMatrix ); 

   // Add node to Skel
   LeftLegSkel.addNode( node );

    //-- ***** BodyNode 3: Left Hip Pitch (LHP) whose parent is: LHR*****
    parent_node = (dynamics::BodyNodeDynamics*) LeftLegSkel.getNode( "LHR" );
    node = (dynamics::BodyNodeDynamics*) LeftLegSkel.createBodyNode("LHP");
    joint = new kinematics::Joint( parent_node, node, "LHP" );

   // Add rigids displacement (local)
    x = 0; y = 0; z = 0; roll = 0; pitch = 0; yaw = 0;
	 add_XyzRpy( joint, x, y, z, roll, pitch, yaw );

   // Add DOF
   val = 0;
   add_DOF( &LeftLegSkel, joint, val, GOLEM_YAW );    

   // Add Shape
   inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
   mass = 1.0;
  add_Shape( node, DART_DATA_PATH"obj/leftLeg/Body_LHP.obj", mass, inertiaMatrix ); 

   // Add node to Skel
   LeftLegSkel.addNode( node );

    //-- ***** BodyNode 4: Left Knee Pitch (LKP) whose parent is: LHP*****
    parent_node = (dynamics::BodyNodeDynamics*) LeftLegSkel.getNode( "LHP" );
    node = (dynamics::BodyNodeDynamics*) LeftLegSkel.createBodyNode("LKP");
    joint = new kinematics::Joint( parent_node, node, "LKP" );

   // Add rigids displacement (local)
    x = 0; y = 0; z = 0; roll = 0; pitch = 0; yaw = 0;
	 add_XyzRpy( joint, x, y, z, roll, pitch, yaw );

   // Add DOF
   val = 0;
   add_DOF( &LeftLegSkel, joint, val, GOLEM_YAW );    

   // Add Shape
   inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
   mass = 1.0;
  add_Shape( node, DART_DATA_PATH"obj/leftLeg/Body_LKP.obj", mass, inertiaMatrix ); 

   // Add node to Skel
   LeftLegSkel.addNode( node );

    //-- ***** BodyNode 5: Left Ankle Pitch (LAP) whose parent is: LKP*****
    parent_node = (dynamics::BodyNodeDynamics*) LeftLegSkel.getNode( "LKP" );
    node = (dynamics::BodyNodeDynamics*) LeftLegSkel.createBodyNode("LAP");
    joint = new kinematics::Joint( parent_node, node, "LAP" );

   // Add rigids displacement (local)
    x = 0; y = 0; z = 0; roll = 0; pitch = 0; yaw = 0;
	 add_XyzRpy( joint, x, y, z, roll, pitch, yaw );

   // Add DOF
   val = 0;
   add_DOF( &LeftLegSkel, joint, val, GOLEM_YAW );    

   // Add Shape
   inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
   mass = 1.0;
  add_Shape( node, DART_DATA_PATH"obj/leftLeg/Body_LAP.obj", mass, inertiaMatrix ); 


   // Add node to Skel
   LeftLegSkel.addNode( node );

    //-- ***** BodyNode 6: Left Ankle Roll (LAR) whose parent is: LAP*****
    parent_node = (dynamics::BodyNodeDynamics*)  LeftLegSkel.getNode( "LAP" );
    node = (dynamics::BodyNodeDynamics*) LeftLegSkel.createBodyNode("LAR");
    joint = new kinematics::Joint( parent_node, node, "LAR" );

   // Add rigids displacement (local)
    x = 0; y = 0; z = 0; roll = 0; pitch = 0; yaw = 0;
	 add_XyzRpy( joint, x, y, z, roll, pitch, yaw );

   // Add DOF
   val = 0;
   add_DOF( &LeftLegSkel, joint, val, GOLEM_YAW );    

   // Add Shape
   inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
   mass = 1.0;
  add_Shape( node, DART_DATA_PATH"obj/leftLeg/Body_LAR.obj", mass, inertiaMatrix ); 

   // Add node to Skel
   LeftLegSkel.addNode( node );


   // ********** END, NOW INITIALIZE THIS GUY *********
   //-- Initialize mySkeleton
   LeftLegSkel.initSkel();

   // Verify that our skeleton has something inside :)
   printf( "Our LeftLeg has %d nodes \n", LeftLegSkel.getNumNodes() );
   printf( "Our LeftLeg has %d joints \n", LeftLegSkel.getNumJoints() );
   printf( "Our LeftLeg has %d DOFs \n", LeftLegSkel.getNumDofs() );

   MyWindow window( &LeftLegSkel, NULL); 

    cout << "space bar: simulation on/off" << endl;
    cout << "'s': simulate one step" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << "'1' and '2': programmed interaction" << endl;
    
   
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Cubes");
    glutMainLoop();

    return 0;
}

/**
  * @function add_XyzRpy
  */
void  add_XyzRpy( kinematics::Joint* _joint, double _x, double _y, double _z, double _rr, double _rp, double _ry ){

  kinematics::Transformation* trans;

	trans = new kinematics::TrfmTranslate( new kinematics::Dof(_x), new kinematics::Dof(_y), new kinematics::Dof(_z), "Translate" );
	_joint->addTransform( trans, false ); 

	trans = new kinematics::TrfmRotateEulerZ( new::kinematics::Dof( DEG2RAD(_ry) ) );
	_joint->addTransform( trans, false );  
	
	trans = new kinematics::TrfmRotateEulerY( new::kinematics::Dof( DEG2RAD(_rp) ) );
	_joint->addTransform( trans, false );  
	
	trans = new kinematics::TrfmRotateEulerX( new::kinematics::Dof( DEG2RAD(_rr)) );
	_joint->addTransform( trans, false );  
 }

/**
  * @function add_DOF
  */
void add_DOF( dynamics::SkeletonDynamics* _skel, kinematics::Joint* _joint,  double _val, int _DOF_TYPE ) {

  kinematics::Transformation* trans;

    if( _DOF_TYPE == GOLEM_X ) {
    	trans = new kinematics::TrfmTranslateX( new kinematics::Dof( 0, "rootX" ), "Tx" );
    	_joint->addTransform( trans, true );
     _skel->addTransform( trans );
   }
   else if ( _DOF_TYPE == GOLEM_Y ) {
    	trans = new kinematics::TrfmTranslateY( new kinematics::Dof( 0, "rootY" ), "Ty" );
    	_joint->addTransform( trans, true );
     _skel->addTransform( trans );
   }
   else if ( _DOF_TYPE == GOLEM_Z ) {
    	trans = new kinematics::TrfmTranslateZ( new kinematics::Dof( 0, "rootZ" ), "Tz" );
    	_joint->addTransform( trans, true );
     _skel->addTransform( trans );
   }
   else if ( _DOF_TYPE == GOLEM_YAW ) {
    trans = new kinematics::TrfmRotateEulerZ( new kinematics::Dof( 0, "rootYaw" ), "Try" );
    _joint->addTransform( trans, true );
    _skel->addTransform( trans );
   }
   else if ( _DOF_TYPE == GOLEM_PITCH ) {
    trans = new kinematics::TrfmRotateEulerY( new kinematics::Dof( 0, "rootPitch" ), "Trp" );
    _joint->addTransform( trans, true );
    _skel->addTransform( trans );
   }
   else if ( _DOF_TYPE == GOLEM_ROLL ) {
    trans = new kinematics::TrfmRotateEulerX( new kinematics::Dof( 0, "rootRoll" ), "Trr" );
    _joint->addTransform( trans, true );
    _skel->addTransform( trans );
   }
   else {
      printf(" WATCH OUT! THIS SHOULD NOT HAPPEN, NO DOF SET \n");
   }

}

/**
  * @function add_Shape
  */
void  add_Shape( dynamics::BodyNodeDynamics* _node, const char *_meshObjPath, double _mass, Eigen::Matrix3d _inertiaMatrix ) {

	   kinematics::ShapeMesh *shape = new kinematics::ShapeMesh( Eigen::Vector3d(0, 0, 0), 0 );
	   _node->setShape( shape );

    // Load a Mesh3DTriangle to save in Shape
    geometry::Mesh3DTriangle* m3d = new geometry::Mesh3DTriangle();
    bool b = m3d->readMesh( _meshObjPath, geometry::Mesh3D::OBJ );
    printf("[AddShape] -- Status of  reading MESH: Reading mesh result was: %d \n", b );

  printf("Num of Vertices is: %d \n", m3d->mNumVertices);

 printf("Num of  Faces is: %d \n", m3d->mNumFaces);

 printf("Num of  vertex normals is: %d \n", m3d->mVertexNormals.size());
 printf("Num of  Faces vector is: %d \n", m3d->mFaces.size() );
  printf("---------------------------------------------------:D \n");

    // Save Mesh3D in Shape (vizMesh)
    shape->setVizMesh( m3d );
    shape->setCollisionMesh( m3d );
    shape->setMass( _mass ); 
    shape->setInertia( _inertiaMatrix ); 
   }

