#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

// To load Mesh and Skel
#include  <kinematics/Joint.h>
#include <kinematics/ShapeMesh.h>
#include <kinematics/Transformation.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/TrfmRotateEuler.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Dof.h>
#include <assimp/cimport.h>


using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

int main(int argc, char* argv[])
{
	FileInfoSkel<SkeletonDynamics> model, model2, model3, model4, model5;
	model.loadFile(DART_DATA_PATH"/skel/ground1.skel", kinematics::SKEL);
	model2.loadFile(DART_DATA_PATH"/skel/cube2.skel", kinematics::SKEL);
	model3.loadFile(DART_DATA_PATH"/skel/cube1.skel", kinematics::SKEL);
	// Replace with real mesh
	model4.loadFile(DART_DATA_PATH"/skel/cube1.skel", kinematics::SKEL);
	model5.loadFile(DART_DATA_PATH"/skel/cube1.skel", kinematics::SKEL);

	// **********
	//-- Create a skeleton
	dynamics::SkeletonDynamics MeshSkel;

	// Always set the root node ( 6DOF for rotation and translation )
	kinematics::Joint* joint;
	dynamics::BodyNodeDynamics* node;
	kinematics::Transformation* trans;

	// Set the initial Rootnode that controls the position and orientation of the whole robot
	node = (dynamics::BodyNodeDynamics*) MeshSkel.createBodyNode("rootBodyNode");
	joint = new kinematics::Joint( NULL, node, "rootJoint" );

	// Add RPY and XYZ of the whole robot
	trans = new kinematics::TrfmTranslateX( new kinematics::Dof( 0, "rootX" ), "Tx" );
	joint->addTransform( trans, true );
	MeshSkel.addTransform( trans );

	trans = new kinematics::TrfmTranslateY( new kinematics::Dof( 0, "rootY" ), "Ty" );
	joint->addTransform( trans, true );
	MeshSkel.addTransform( trans );

	trans = new kinematics::TrfmTranslateZ( new kinematics::Dof( 0, "rootZ" ), "Tz" );
	joint->addTransform( trans, true );
	MeshSkel.addTransform( trans );

	trans = new kinematics::TrfmRotateEulerZ( new kinematics::Dof( 0, "rootYaw" ), "Try" );
	joint->addTransform( trans, true );
	MeshSkel.addTransform( trans );

	trans = new kinematics::TrfmRotateEulerY( new kinematics::Dof( 0, "rootPitch" ), "Trp" );
	joint->addTransform( trans, true );
	MeshSkel.addTransform( trans );

	trans = new kinematics::TrfmRotateEulerX( new kinematics::Dof( 0, "rootRoll" ), "Trr" );
	joint->addTransform( trans, true );
	MeshSkel.addTransform( trans );

	// Load a Mesh3DTriangle to save in Shape
	const aiScene* m3d = ShapeMesh::loadMesh( DART_DATA_PATH"/obj/foot.obj");

	//  Create Shape and assign it to node
	kinematics::ShapeMesh *Shape0 = new kinematics::ShapeMesh(Eigen::Vector3d(1.0, 1.0, 1.0), m3d);

	node->addVisualizationShape(Shape0);
	node->addCollisionShape(Shape0);
	Matrix3d M;
	M << 0.000416667, 0.0, 0.0, 0.0, 0.000416667, 0.0, 0.0, 0.0, 0.000416667;
	node->setLocalInertia(M);
	node->setMass(1); // 1 Kg according to cube1.skel

	// Add node to Skel
	MeshSkel.addNode( node );

	//-- Initialize mySkeleton
	MeshSkel.initSkel();

	// Verify that our skeleton has something inside :)
	printf( "Our skeleton has %d nodes \n", MeshSkel.getNumNodes() );
	printf( "Our skeleton has %d joints \n", MeshSkel.getNumJoints() );
	printf( "Our skeleton has %d DOFs \n", MeshSkel.getNumDofs() );

	//   exit(0);

	MyWindow window((SkeletonDynamics*)model.getSkel(), &MeshSkel, NULL); //
	//    MyWindow window((SkeletonDynamics*)model.getSkel(), (SkeletonDynamics*)model2.getSkel(), (SkeletonDynamics*)model3.getSkel(), (SkeletonDynamics*)model4.getSkel(), &MeshSkel, NULL); //

	cout << "space bar: simulation on/off" << endl;
	cout << "'s': simulate one step" << endl;
	cout << "'p': playback/stop" << endl;
	cout << "'[' and ']': play one frame backward and forward" << endl;
	cout << "'v': visualization on/off" << endl;
	cout << "'1' and '2': programmed interaction" << endl;


	glutInit(&argc, argv);
	window.initWindow(640, 480, "Cubes");
	glutMainLoop();

	aiReleaseImport(m3d);

	return 0;
}
