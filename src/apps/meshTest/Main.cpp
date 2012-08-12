/**
 * @file Main.cpp
 * @author A. Huaman
 * @date 2011-10-04
 * @brief Main
 */
#include "MyWindow.h"

#include <dynamics/SkeletonDynamics.h>
#include <dynamics/BodyNodeDynamics.h>
#include  <kinematics/Joint.h>
#include <kinematics/ShapeMesh.h>
#include <geometry/Mesh3DTriangle.h>
// Define DART_DATA_PATH
#include "utils/Paths.h"

#include <iostream>
#include <stdio.h>
#include <string>

/**
 * @function main
 */
int main( int argc, char* argv[] )
{
   //-- Create a skeleton (mySkeleton)
   dynamics::SkeletonDynamics skel;

   //-- Add a BodyNode
   dynamics::BodyNodeDynamics *L0 = (dynamics::BodyNodeDynamics *)skel.createBodyNode( "L0" );
   kinematics::Joint *J0 = new kinematics::Joint( NULL, L0, "J0" );

	  kinematics::ShapeMesh *Shape0 = new kinematics::ShapeMesh( Eigen::Vector3d(0, 0, 0), 0 );
	  L0->setShape( Shape0 );

    // Load a Mesh3DTriangle to save in Shape
    geometry::Mesh3DTriangle m3d;
    bool b = m3d.readMesh( DART_DATA_PATH"/obj/foot.obj", geometry::Mesh3D::OBJ );
    printf("Status of  reading MESH: Reading mesh result was: %d \n", b );

    // Save Mesh3D in Shape (vizMesh)
    Shape0->setVizMesh( &m3d );
    Shape0->setCollisionMesh( &m3d );

   // Add node to Skel
   skel.addNode( L0 );

   //-- Initialize mySkeleton
   skel.initSkel();

   // Initialize Window
   MyWindow window( &skel );
  
   // Verify that our skeleton has something inside :)
   printf( "Our skeleton has %d nodes \n", skel.getNumNodes() );
   printf( "Our skeleton has %d joints \n", skel.getNumJoints() );
   printf( "Our skeleton has %d DOFs \n", skel.getNumDofs() );

   // OpenGL general menu
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

