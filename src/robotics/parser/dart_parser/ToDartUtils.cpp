/**
 * @file ToDartUtils.cpp
 * @brief Functions that convert ModelInterface objects to their Dart counterparts
 * @author A. Huaman Quispe
 * @date 2012 / 10 / 12
 */
#include "DartLoader.h"
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/Joint.h>
#include <kinematics/Transformation.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/TrfmRotateEuler.h>
#include <kinematics/Dof.h>
#include <kinematics/BodyNode.h>
#include <dynamics/BodyNodeDynamics.h>
#include "../urdf_parser/urdf_parser.h"

// For continuous joint limit
#include <limits>

/**
 * @function createDartRootJoint
 * @brief Create a joint if floating root node, nothing if world is root
 */
kinematics::Joint* DartLoader::createDartRootJoint( dynamics::BodyNodeDynamics* _node,
						    dynamics::SkeletonDynamics* _skel ) {

  
  // Parent will be NULL.   
  kinematics::Joint* rootJoint;
  if(debug) std::cout<<"[debug] Creating joint for root node: "<< _node->getName() <<std::endl;

  // This joint connects with the world 
  rootJoint = new kinematics::Joint( NULL, _node, "worldJoint" );
   
  // This will be a FREEEULER joint type. We have 3 DOF for rotation and 3 DOF for translation
  kinematics::Transformation* trans;

    
    // Add DOFs for RPY and XYZ of the whole robot
    trans = new kinematics::TrfmTranslateX( new kinematics::Dof( 0, "rootX" ), "Tx" );
    rootJoint->addTransform( trans, true );
    _skel->addTransform( trans );
    
    trans = new kinematics::TrfmTranslateY( new kinematics::Dof( 0, "rootY" ), "Ty" );
    rootJoint->addTransform( trans, true );
    _skel->addTransform( trans );

    trans = new kinematics::TrfmTranslateZ( new kinematics::Dof( 0, "rootZ" ), "Tz" );
    rootJoint->addTransform( trans, true );
    _skel->addTransform( trans );
   
    trans = new kinematics::TrfmRotateEulerZ( new kinematics::Dof( 0, "rootYaw" ), "Try" );
    rootJoint->addTransform( trans, true );
    _skel->addTransform( trans );
 
    trans = new kinematics::TrfmRotateEulerY( new kinematics::Dof( 0, "rootPitch" ), "Trp" );
    rootJoint->addTransform( trans, true );
    _skel->addTransform( trans );

    trans = new kinematics::TrfmRotateEulerX( new kinematics::Dof( 0, "rootRoll" ), "Trr" );
    rootJoint->addTransform( trans, true );
    _skel->addTransform( trans );

    // Set this first node as root node
    //_skel->addNode( _node );
    //_skel->initSkel();    
 

  return rootJoint;
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
  std::string jointName = ( _jt->name);
  std::string parentName = (_jt->parent_link_name);
  std::string childName =  (_jt->child_link_name);
  dynamics::BodyNodeDynamics* parent = getNode( parentName );
  dynamics::BodyNodeDynamics* child = getNode( childName );

  // Create joint
  kinematics::Joint* joint;
  if( parent == NULL && parentName == "world" ) {    
    joint = new kinematics::Joint( NULL, child, jointName.c_str() );
    std::cout<<"Creating joint between world and "<<childName<<std::endl;
  } else {
    joint = new kinematics::Joint( parent, child, jointName.c_str() );
  }


  // Add Rigid transform
  urdf::Pose p = _jt->parent_to_joint_origin_transform;
  x = p.position.x;
  y = p.position.y;
  z = p.position.z;
  p.rotation.getRPY( roll, pitch, yaw );
  add_XyzRpy( joint, x, y, z, roll, pitch, yaw );
  
  // Add DOF if prismatic or revolute joint
  val = 0;
  if( _jt->type == urdf::Joint::REVOLUTE || 
      _jt->type == urdf::Joint::PRISMATIC || 
      _jt->type == urdf::Joint::CONTINUOUS ) {
    
    // Revolute
    if(   _jt->type == urdf::Joint::REVOLUTE ) {  
      if( _jt->axis.x == 1 /*&& _jt->axis.y == 0 && _jt->axis.z == 0 */) {
	axis = GOLEM_ROLL; 
      } else if( _jt->axis.y == 1  /* && _jt->axis.x == 0 && _jt->axis.z == 0 */ ) {
	axis = GOLEM_PITCH; 
      } else if( _jt->axis.z == 1   /* && _jt->axis.x == 0 && _jt->axis.y == 0 */) {
	axis = GOLEM_YAW; 
      } else { 
	axis = GOLEM_ARBITRARY_ROTATION;
      }
      
      vmin = _jt->limits->lower;
      vmax = _jt->limits->upper;
    }

    // Prismatic
    else if( _jt->type == urdf::Joint::PRISMATIC ) {
      
      if( _jt->axis.x == 1 || _jt->axis.x == -1  ) { axis = GOLEM_X; }
      else if( _jt->axis.y == 1 || _jt->axis.y == -1  ) { axis = GOLEM_Y; }
      else if( _jt->axis.z == 1 || _jt->axis.z == -1 ) { axis = GOLEM_Z; }
      else { printf (" [ERROR] No axis defined for Prismatic DOF! \n"); }
      
      vmin = _jt->limits->lower;
      vmax = _jt->limits->upper;
    }
 

    // Continuous
    else if( _jt->type == urdf::Joint::CONTINUOUS ) {
      if( _jt->axis.x == 1 && _jt->axis.y == 0 && _jt->axis.z == 0 ) {
	axis = GOLEM_ROLL; 
      } else if( _jt->axis.y == 1 && _jt->axis.x == 0 && _jt->axis.z == 0  ) {
	axis = GOLEM_PITCH; 
      } else if( _jt->axis.z == 1 && _jt->axis.x == 0 && _jt->axis.y == 0 ) {
	axis = GOLEM_YAW; 
      } else { 
	axis = GOLEM_ARBITRARY_ROTATION;
      }
      // Define infinity limits
      vmin = -1*std::numeric_limits<double>::infinity();
      vmax = std::numeric_limits<double>::infinity();
    }
      
    add_DOF( _skel, joint, val, vmin, vmax, axis, _jt->axis.x, _jt->axis.y, _jt->axis.z );
     
  }
  
  // Fixed, do not add DOF
  else if( _jt->type == urdf::Joint::FIXED ) {
    if(debug) std::cout<<"[debug] Fixed joint: "<< jointName <<std::endl;  
  }
  
  // None of the above
  else {
    std::cout<<"[createDartJoint] ERROR: Parsing "<<  jointName <<" joint: No PRISMATIC or REVOLUTE or CONTINUOUS or FIXED \n";
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

  std::string lk_name = _lk->name;

  if( lk_name == "world" ) {
    std::cout << "[info] world is not parsed as a link" << std::endl;
    return NULL;
  }

  if(debug) std::cout<<"[debug] Creating dart node:"<< lk_name <<std::endl;
  
  dynamics::BodyNodeDynamics* node =  (dynamics::BodyNodeDynamics*) _skel->createBodyNode( lk_name.c_str() );
  
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
    node->setLocalCOM( localCOM );  // Temporary Hack: Do not set COM since DART uses that as the location of the mesh
    
    if( debug ) { std::cout<< "[debug] Mass is: "<< mass << std::endl; }
    if( debug ) { std::cout<< "[debug] Inertia is: \n"<< inertia << std::endl; }
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
