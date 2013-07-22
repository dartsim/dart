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
#include <kinematics/Shape.h>
#include <dynamics/BodyNodeDynamics.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/link.h>

// For continuous joint limit
#include <limits>


/**
 * @function createDartRootJoint
 * @brief Create defined rootJoint between world and rootNode
 */
kinematics::Joint* DartLoader::createDartRootJoint( boost::shared_ptr<urdf::Joint> _jt,
						    dynamics::SkeletonDynamics* _skel ) {

    // Currently we support 2 root joints: Fixed and floating
    // other type of joints (revolute and prismatic) appear as fixed so don't use them by the time being
    // (joints located on nodes other than root can be anything)
  
    double x, y, z;
    double roll, pitch, yaw;
    double val;
    double vmin, vmax;
    int axis;
    double xAxis, yAxis, zAxis;
  
    // Parent is NULL (World).
    if( strcmp( (_jt->parent_link_name).c_str(), "world") != 0 ) {
        std::cout<< "Error, createDartRootJoint should receive a joint with world as parent. No creating joint" << std::endl;
        return NULL;
    }

    // Get child body Nodes
    kinematics::Joint* joint;
    std::string jointName = ( _jt->name);
    std::string childName = (_jt->child_link_name);
    dynamics::BodyNodeDynamics* child = getNode( childName );

    joint = new kinematics::Joint( NULL, child, jointName.c_str() );

  
    // For FLOATING root joint we do NOT use createDartJoint since it automatically adds a transform xyzRPY (fixed)
    // before the DOF. This would make the root joint to behave as a fixed joint with the current setup
    val = 0;
    if( _jt->type == urdf::Joint::FLOATING ) {
    
        axis = GOLEM_FLOATING;

        // Define infinity limits
        vmin = -1*std::numeric_limits<double>::infinity();
        vmax = std::numeric_limits<double>::infinity();
        xAxis = 0; yAxis = 0; zAxis = 0;
    
        add_DOF( _skel, joint, val, vmin, vmax, axis, xAxis, yAxis, zAxis );
        return joint;
    }
  
    else {
        return createDartJoint( _jt, _skel );
    }

}


/**
 * @function createNewDartRootJoint
 * @brief Create a new floating joint if no joint is defined between world and rootNode
 */
kinematics::Joint* DartLoader::createNewDartRootJoint( dynamics::BodyNodeDynamics* _node,
						       dynamics::SkeletonDynamics* _skel ) {

  
  // Parent will be NULL.   
  kinematics::Joint* rootJoint;
  if(debug) std::cout<<"[debug] Creating joint for root node: "<< _node->getName() <<std::endl;

  // This joint connects with the world 
  rootJoint = new kinematics::Joint( NULL, _node, "worldJoint" );
   
  // This will be a FREEEULER joint type. We have 3 DOF for rotation and 3 DOF for translation
  add_DOF( _skel, rootJoint, 0, 0, 0, GOLEM_FLOATING );

 
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
  double xAxis, yAxis, zAxis;
 
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
      _jt->type == urdf::Joint::CONTINUOUS ||
      _jt->type == urdf::Joint::FLOATING ) {
    
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

      xAxis = _jt->axis.x; yAxis = _jt->axis.y; zAxis = _jt->axis.z;
    }

    // Prismatic
    else if( _jt->type == urdf::Joint::PRISMATIC ) {
      
      if( _jt->axis.x == 1 || _jt->axis.x == -1  ) { axis = GOLEM_X; }
      else if( _jt->axis.y == 1 || _jt->axis.y == -1  ) { axis = GOLEM_Y; }
      else if( _jt->axis.z == 1 || _jt->axis.z == -1 ) { axis = GOLEM_Z; }
      else { printf (" [ERROR] No axis defined for Prismatic DOF! \n"); }
      
      vmin = _jt->limits->lower;
      vmax = _jt->limits->upper;

      xAxis = _jt->axis.x; yAxis = _jt->axis.y; zAxis = _jt->axis.z;
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

      xAxis = _jt->axis.x; yAxis = _jt->axis.y; zAxis = _jt->axis.z;
    }

    // Floating
    else if( _jt->type == urdf::Joint::FLOATING ) {

      axis = GOLEM_FLOATING;

      // Define infinity limits
      vmin = 0;
      vmax = 0;

      xAxis = 0; yAxis = 0; zAxis = 0;
    }
      
    add_DOF( _skel, joint, val, vmin, vmax, axis, xAxis, yAxis, zAxis );
    
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
							dynamics::SkeletonDynamics* _skel,
							std::string _rootToSkelPath ) {

  std::string lk_name = _lk->name;

  if( lk_name == "world" ) {
    std::cout << "[info] world is not parsed as a link" << std::endl;
    return NULL;
  }

  if(debug) std::cout<<"[debug] Creating dart node:"<< lk_name <<std::endl;
  
  dynamics::BodyNodeDynamics* node =  (dynamics::BodyNodeDynamics*) _skel->createBodyNode( lk_name.c_str() );
  
  // Mesh Loading
  //FIXME: Shouldn't mass and inertia default to 0?
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
    node->setLocalCOM( localCOM );
    
    if( debug ) { std::cout<< "[debug] Mass is: "<< mass << std::endl; }
    if( debug ) { std::cout<< "[debug] Inertia is: \n"<< inertia << std::endl; }
  }

  // Set inertial information
  node->setMass(mass);
  node->setLocalInertia(inertia);

  // Set visual information (at least there is one visual element)
  if( _lk->visual ) {
 
    // Read all visual groups
    std::map<std::string, boost::shared_ptr<std::vector<boost::shared_ptr<urdf::Visual> > > >::iterator iter;    
    for( iter = _lk->visual_groups.begin(); iter != _lk->visual_groups.end(); iter++ ) {
      std::vector<boost::shared_ptr<urdf::Visual> > visualGroup;
      visualGroup = *(iter->second);
      for( int j = 0; j < visualGroup.size(); ++j ) {
        if(kinematics::Shape* shape = createShape(visualGroup[j], _rootToSkelPath)) {
          node->addVisualizationShape(shape);
        }
        else {
          std::cout << "Error loading VizShape" << std::endl;
          return NULL;
        }
      }
    }
 
  }
  else if(debug) {
    std::cout << "No Visualization tag defined for node " << node->getName() << "." << std::endl;
  }

      
  // Set collision information (at least there is one collision element)
  if( _lk->collision ) {

    // Read all collision groups
    std::map<std::string, boost::shared_ptr<std::vector<boost::shared_ptr<urdf::Collision> > > >::iterator iter;    
    for( iter = _lk->collision_groups.begin(); iter != _lk->collision_groups.end(); iter++ ) {
      std::vector<boost::shared_ptr<urdf::Collision> > collisionGroup;
      collisionGroup = *(iter->second);
      for( int j = 0; j < collisionGroup.size(); ++j ) {
        if(kinematics::Shape* shape = createShape(collisionGroup[j], _rootToSkelPath)) {
          node->addCollisionShape(shape);
        }
        else {
          std::cout << "Error loading ColShape" << std::endl;
          return NULL;
        }
      }
    }
    
  }
  else if(debug) {
    std::cout << "No Collision tag defined for node " << node->getName() << "." << std::endl;
  }

  return node;
}
