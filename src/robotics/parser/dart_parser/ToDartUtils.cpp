/**
 * @file ToDartUtils.cpp
 * @brief Functions that convert ModelInterface objects to their Dart counterparts
 * @author A. Huaman Quispe
 * @date 2012 / 10 / 12
 */
#include "DartLoader.h"
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/Joint.h>
#include <kinematics/BodyNode.h>
#include <dynamics/BodyNodeDynamics.h>
#include "../urdf_parser/urdf_parser.h"

kinematics::Joint* DartLoader::createDartRootJoint( boost::shared_ptr<const urdf::Link> _rootLink,
						    dynamics::SkeletonDynamics* _skel,
						    bool _createdDummyRoot ) {
  
  // Parent will be NULL. Get root node as child  
  dynamics::BodyNodeDynamics* rootLoadedNode = getNode( _rootLink->name );
  kinematics::Joint* rootLoadedJoint;
  if(debug) printf ("Root loaded name: %s \n", _rootLink->name.c_str());

  if( _createdDummyRoot == false ) {

    // This joint connects with the world 
    rootLoadedJoint = new kinematics::Joint( NULL,
					     rootLoadedNode,
					     "RootLoadedJoint" );
    
    // Add Rigid transform 
    add_XyzRpy( rootLoadedJoint, 0, 0, 0, 0, 0, 0 );  
  }

  else {
    // This body node should connect with dummy Root
    if( _skel->getRoot() == NULL ) { if(debug) printf ("[X] You did not create the dummy Root! \n"); }
    if(debug) printf ("Created dummy node name: %s \n", _skel->getRoot()->getName() );
    rootLoadedJoint = new kinematics::Joint( _skel->getRoot(),
					     rootLoadedNode,
					     "RootLoadedJoint" );
    
    // Add Rigid transform 
    add_XyzRpy( rootLoadedJoint, 0, 0, 0, 0, 0, 0 );  

  }

  return rootLoadedJoint;
}


/**
 * @function createDartJoint
 */
kinematics::Joint* DartLoader::createDartJoint( boost::shared_ptr<urdf::Joint> _jt,
						dynamics::SkeletonDynamics* _skel ) {


  double x, y, z; 
  double roll, pitch, yaw;
  double val;
  double vmin, vmax;
  int axis;
 
  // Get parent and child body Nodes
  const char* jointName = ( _jt->name).c_str();
  dynamics::BodyNodeDynamics* parent = getNode( _jt->parent_link_name );
  dynamics::BodyNodeDynamics* child = getNode( _jt->child_link_name );
  if(debug) printf ("Joint: %s Parent: %s child : %s \n",   jointName, parent->getName(), 
		    child->getName() );
  // Create joint
  kinematics::Joint* joint = new kinematics::Joint( parent,
						    child,
						    jointName );
  // Add Rigid transform
  urdf::Pose p = _jt->parent_to_joint_origin_transform;
  x = p.position.x;
  y = p.position.y;
  z = p.position.z;
  p.rotation.getRPY( roll, pitch, yaw );
  add_XyzRpy( joint, x, y, z, roll, pitch, yaw );
  
  // Add DOF if prismatic or revolute joint
  val = 0;
  if( _jt->type == urdf::Joint::REVOLUTE || _jt->type == urdf::Joint::PRISMATIC ) {
    
    // Revolute
    if(   _jt->type == urdf::Joint::REVOLUTE ) {  
      if( _jt->axis.x == 1 /*&& _jt->axis.y == 0 && _jt->axis.z == 0 */) {
	axis = GOLEM_ROLL; 
      } else if( _jt->axis.y == 1 /* && _jt->axis.x == 0 && _jt->axis.z == 0 */ ) {
	axis = GOLEM_PITCH; 
      } else if( _jt->axis.z == 1  /* && _jt->axis.x == 0 && _jt->axis.y == 0 */) {
	axis = GOLEM_YAW; 
      } else if( _jt->axis.x == -1  /* && _jt->axis.y == 0 && _jt->axis.z == 0 */) {
	axis = GOLEM_ROLL; printf("Joint %s has negative x \n", jointName );
      } else if( _jt->axis.y == -1 /* && _jt->axis.x == 0 && _jt->axis.z == 0 */ ) { 
	axis = GOLEM_PITCH;  printf("Joint %s has negative y \n", jointName );
      }
      else if(  _jt->axis.z == -1 /* && _jt->axis.x == 0 && _jt->axis.y == 0 */ ) { 
	axis = GOLEM_YAW;  printf("Joint %s has negative z \n", jointName );
      }
      else { 
	//if(debug) 
	printf ("[ERROR] [%s] No axis defined for Revolute DOF (%f %f %f) !  \n", jointName, _jt->axis.x, _jt->axis.y, _jt->axis.z ); 
      }
    }
    // Prismatic
    else if( _jt->type == urdf::Joint::PRISMATIC ) {
      
      if( _jt->axis.x == 1 || _jt->axis.x == -1  ) { axis = GOLEM_X; }
      else if( _jt->axis.y == 1 || _jt->axis.y == -1  ) { axis = GOLEM_Y; }
      else if( _jt->axis.z == 1 || _jt->axis.z == -1 ) { axis = GOLEM_Z; }
      else { printf (" [ERROR] No axis defined for Prismatic DOF! \n"); }
    }
    
    vmin = _jt->limits->lower;
    vmax = _jt->limits->upper;
    
    add_DOF( _skel, joint, val, vmin, vmax, axis );
  }
  
  // Fixed, do not add DOF
  else if( _jt->type == urdf::Joint::FIXED ) {
    //if(debug) 
      printf ("Fixed joint: %s \n", jointName );
  }
  
  // None of the above
  else {
    printf ("[createDartJoint] ERROR: Parsing %s joint: No PRISMATIC or REVOLUTE or FIXED \n", jointName );
  }
  
  return joint;
}

/**
 * @function createDartNode
 */
dynamics::BodyNodeDynamics* DartLoader::createDartNode( boost::shared_ptr<urdf::Link> _lk,
							dynamics::SkeletonDynamics* _skel ) {
  
  std::string fullVisualPath;
  std::string fullCollisionPath;

  const char* lk_name = _lk->name.c_str();
  if(debug) printf ("** Creating dart node: %s \n", lk_name );
  
  dynamics::BodyNodeDynamics* node =  (dynamics::BodyNodeDynamics*) _skel->createBodyNode( lk_name );
  
  // Mesh Loading
  double mass = 0.1;
  Eigen::Matrix3d inertia = Eigen::MatrixXd::Identity(3,3);
  inertia *= 0.1;
  
  // Load Inertial information
  if( _lk->inertial ) {
    boost::shared_ptr<urdf::Inertial>inert= (_lk->inertial);
    
    // Load mass
    mass = inert->mass;

    // Load Inertia matrix
    inertia(0,0) = inert->ixx;
    inertia(0,1) = -1*(inert->ixy);
    inertia(0,2) = -1*(inert->ixz);
    
    inertia(1,0) = -1*(inert->ixy);
    inertia(1,1) = (inert->iyy);
    inertia(1,2) = -1*(inert->iyz);
    
    inertia(2,0) = -1*(inert->ixz);
    inertia(2,1) = -1*(inert->iyz);
    inertia(2,2) = inert->izz;
    
    // Load local CoM
    Eigen::Vector3d localCOM;
    localCOM << inert->origin.position.x, 
      inert->origin.position.y, 
      inert->origin.position.z;
    // Set it to Node
    //node->setLocalCOM( localCOM );  // Temporary Hack: Do not set COM since DART uses that as the location of the mesh
    
    if( debug ) { printf ("* Mass is: %f \n", mass); }
    if( debug ) { std::cout<< "* Inertia is: \n"<< inertia << std::endl; }
  }
  
  if( !_lk->visual ) {
    add_Shape( node,  mass, inertia ); 
  }
  else {
    
    // Get the visTransform from visual
    // origin
    urdf::Pose visPose = _lk->visual->origin;

    if( _lk->visual->geometry->type == urdf::Geometry::MESH ) {
      
      boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>( _lk->visual->geometry );
      fullVisualPath = mPath;
      fullVisualPath.append( mesh->filename );

      // Check collision
      if( _lk->collision ) {
	if( _lk->collision->geometry->type == urdf::Geometry::MESH ) {
	  boost::shared_ptr<urdf::Mesh> collisionMesh = boost::static_pointer_cast<urdf::Mesh>( _lk->collision->geometry );
	  fullCollisionPath = mPath;
	  fullCollisionPath.append( collisionMesh->filename );
	}
      }

      add_ShapeMesh( node, 
		     (fullVisualPath).c_str(), 
		     mass,
		     inertia,
		     (fullCollisionPath).c_str(),
		     visPose ); 
      
    }
    
    // Empty
    else {
      add_Shape( node, mass, inertia ); 
    }
  } // end else : We have visual
  
  return node;
}
