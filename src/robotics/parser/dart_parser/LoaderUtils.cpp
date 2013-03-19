/**
 * @file LoaderUtils.cpp
 * @brief Utils to load Dart objects
 */

#include "DartLoader.h"
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/TrfmRotateEuler.h>
#include <kinematics/TrfmRotateAxis.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <kinematics/Shape.h>
#include <kinematics/ShapeMesh.h>
#include <kinematics/ShapeBox.h>
#include <kinematics/ShapeCylinder.h>
#include <kinematics/ShapeEllipsoid.h>
#include <dynamics/BodyNodeDynamics.h>
#include <iostream>


/**
 * @function add_XyzRpy
 */
void DartLoader::add_XyzRpy( kinematics::Joint* _joint, 
			     double _x, double _y, double _z, 
			     double _rr, double _rp, double _ry ) {

  kinematics::Transformation* trans;
  
  trans = new kinematics::TrfmTranslate(new kinematics::Dof(_x), new kinematics::Dof(_y), new kinematics::Dof(_z));
  _joint->addTransform( trans, false );
  
  trans = new kinematics::TrfmRotateEulerZ(new ::kinematics::Dof(_ry ));
  _joint->addTransform(trans, false);
  
  trans = new kinematics::TrfmRotateEulerY(new ::kinematics::Dof( _rp ));
  _joint->addTransform(trans, false);
  
  trans = new kinematics::TrfmRotateEulerX(new ::kinematics::Dof( _rr ));
  _joint->addTransform(trans, false);
}

/**
 * @function add_DOF
 */
void DartLoader::add_DOF( dynamics::SkeletonDynamics* _skel, 
			  kinematics::Joint* _joint, 
			  double _val, double _min, double _max,
			  int _DOF_TYPE,
			  double _x, double _y, double _z  ) {

  kinematics::Transformation* trans;
  
  if(_DOF_TYPE == GOLEM_X) {
    trans = new kinematics::TrfmTranslateX(new kinematics::Dof(0, _joint->getName() ), "T_dof");
    _joint->addTransform(trans, true);
    _joint->getDof(0)->setMin(_min);
    _joint->getDof(0)->setMax(_max);
    _skel->addTransform(trans);
  }
  else if(_DOF_TYPE == GOLEM_Y) {
    trans = new kinematics::TrfmTranslateY(new kinematics::Dof(0, _joint->getName() ), "T_dof");
    _joint->addTransform(trans, true);
    _joint->getDof(0)->setMin(_min);
    _joint->getDof(0)->setMax(_max);
    _skel->addTransform(trans);
  }
  else if(_DOF_TYPE == GOLEM_Z) {
    trans = new kinematics::TrfmTranslateZ(new kinematics::Dof(0, _joint->getName() ), "T_dof");
    _joint->addTransform(trans, true);
    _joint->getDof(0)->setMin(_min);
    _joint->getDof(0)->setMax(_max);
    _skel->addTransform(trans);
  }
  else if(_DOF_TYPE == GOLEM_YAW) {
    trans = new kinematics::TrfmRotateEulerZ(new kinematics::Dof(0, _joint->getName() ), "T_dof");
    _joint->addTransform(trans, true);
    _joint->getDof(0)->setMin(_min);
    _joint->getDof(0)->setMax(_max);
    _skel->addTransform(trans);
  }
  else if(_DOF_TYPE == GOLEM_PITCH) {
    trans = new kinematics::TrfmRotateEulerY(new kinematics::Dof(0, _joint->getName() ), "T_dof");
    _joint->addTransform(trans, true);
    _joint->getDof(0)->setMin(_min);
    _joint->getDof(0)->setMax(_max);
    _skel->addTransform(trans);
  }
  else if(_DOF_TYPE == GOLEM_ROLL) {
    trans = new kinematics::TrfmRotateEulerX(new kinematics::Dof(0,  _joint->getName() ), "T_dof");
    _joint->addTransform(trans, true);
    _joint->getDof(0)->setMin(_min);
    _joint->getDof(0)->setMax(_max);
    _skel->addTransform(trans);
  }
  else if( _DOF_TYPE == GOLEM_ARBITRARY_ROTATION ) {
    Eigen::Vector3d axis; axis << _x, _y, _z;
    trans = new kinematics::TrfmRotateAxis( axis, new kinematics::Dof(0,  _joint->getName() ), "T_dof");
    _joint->addTransform(trans, true);
    _joint->getDof(0)->setMin(_min);
    _joint->getDof(0)->setMax(_max);
    _skel->addTransform(trans);
  }
  else if( _DOF_TYPE == GOLEM_FLOATING ) {

    trans = new kinematics::TrfmTranslate(new kinematics::Dof(0, "floatingX"),
                                          new kinematics::Dof(0, "floatingY"),
                                          new kinematics::Dof(0, "floatingZ"), "Tfxyz");
    _joint->addTransform( trans, true );
    _skel->addTransform( trans );
   
    trans = new kinematics::TrfmRotateEulerZ( new kinematics::Dof( 0, "floatingYaw" ), "Tfry" );
    _joint->addTransform( trans, true );
    _skel->addTransform( trans );
 
    trans = new kinematics::TrfmRotateEulerY( new kinematics::Dof( 0, "floatingPitch" ), "Tfrp" );
    _joint->addTransform( trans, true );
    _skel->addTransform( trans );

    trans = new kinematics::TrfmRotateEulerX( new kinematics::Dof( 0, "floatingRoll" ), "Tfrr" );
    _joint->addTransform( trans, true );
    _skel->addTransform( trans );

  }

  else {
    if(debug) std::cerr << " WATCH OUT! THIS SHOULD NOT HAPPEN, NO DOF SET" << std::endl;
  }
  
}

/**
 * @function add_VizShape
 */
bool  DartLoader::add_VizShape( dynamics::BodyNodeDynamics* _node,
				boost::shared_ptr<urdf::Visual> _viz,
				std::string _rootToSkelPath ) {
  // Variables to use
  kinematics::Shape* shape;
  const aiScene* model;
  urdf::Pose pose;

  // Origin
  pose = _viz->origin;  

  if(debug) { std::cout<< "Loading vizShape for node "<< _node->getName() << std::endl; }  

  // Type of Geometry

  //-- SPHERE
  if( _viz->geometry->type == urdf::Geometry::SPHERE ) {
    
    boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>( _viz->geometry );
    shape = new kinematics::ShapeEllipsoid(2.0 * sphere->radius * Eigen::Vector3d::Ones());
    
    // Set its pose
    Eigen::Affine3d transform;
    transform = pose2Affine3d( pose );
    
    // Set it into shape
    shape->setTransform( transform );
    
    // Set color
    if( _viz->material ) {
      if( (_viz->material)->color.r != 0  &&  
	  (_viz->material)->color.g != 0 &&  
	  (_viz->material)->color.b != 0 ) {
	Eigen::Vector3d color;
	color << _viz->material->color.r, _viz->material->color.g, _viz->material->color.b;
	shape->setColor(color);
      }
    }
    
    // Set in node
    _node->setVisualizationShape(shape);
    if(debug) { std::cout<< "Loading a sphere vizMesh of radius:" << sphere->radius << std::endl; }
  }

  //-- BOX
  else if( _viz->geometry->type == urdf::Geometry::BOX ) {

    boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>( _viz->geometry );
    Eigen::Vector3d dim; dim<< box->dim.x, box->dim.y, box->dim.z;
    shape = new kinematics::ShapeBox( dim );
    
    // Set its pose
    Eigen::Affine3d transform;
    transform = pose2Affine3d( pose );
    
    // Set it into shape
    shape->setTransform( transform );
    
    // Set color
    if( _viz->material ) {
      if( (_viz->material)->color.r != 0  &&  
	  (_viz->material)->color.g != 0 &&  
	  (_viz->material)->color.b != 0 ) {
	Eigen::Vector3d color;
	color << _viz->material->color.r, _viz->material->color.g, _viz->material->color.b;
	shape->setColor(color);
      }
    }
    

    // Set in node
    _node->setVisualizationShape(shape);

    if(debug) { std::cout<< "Loading a box vizMesh of dim:" << dim.transpose() << std::endl; }
  }

  //-- CYLINDER
  else if( _viz->geometry->type == urdf::Geometry::CYLINDER ) {

    boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>( _viz->geometry );
    shape = new kinematics::ShapeCylinder( cylinder->radius, cylinder->length );
    
    // Set its pose
    Eigen::Affine3d transform;
    transform = pose2Affine3d( pose );
    
    // Set it into shape
    shape->setTransform( transform );
    
    // Set color
    if( _viz->material ) {
      if( (_viz->material)->color.r != 0  &&  
	  (_viz->material)->color.g != 0 &&  
	  (_viz->material)->color.b != 0 ) {
	Eigen::Vector3d color;
	color << _viz->material->color.r, _viz->material->color.g, _viz->material->color.b;
	shape->setColor(color);
      }
    }
    
    // Set in node
    _node->setVisualizationShape(shape);

    if(debug) { std::cout<< "Loading a cylinder vizMesh of radius:" << cylinder->radius<<" and length: "<<cylinder->length<< std::endl; }
  }

  //-- Mesh : Save the path
  else if( _viz->geometry->type == urdf::Geometry::MESH ) {

    boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>( _viz->geometry );
    std::string fullPath = _rootToSkelPath;
    fullPath.append( mesh->filename );

    // Load aiScene visualization
    model = kinematics::ShapeMesh::loadMesh( fullPath );    
    
    if( model == NULL ) {
      std::cout<< "[add_VizShape] [ERROR] Not loading model "<< fullPath<<" (NULL) \n";
      return false;  
    }
    
    // Set shape as mesh
    shape = new kinematics::ShapeMesh( Eigen::Vector3d( 1, 1, 1), model );
    
    // Set its pose
    Eigen::Affine3d transform;
    transform = pose2Affine3d( pose );
    
    // Set it into shape
    shape->setTransform( transform );
    
    if(debug) std::cerr << "[debug] Loading visual model: " << fullPath << std::endl;
    
    // Set in node
    _node->setVisualizationShape(shape);
    
  } // end if (mesh)

  else {
    std::cout<< "[set_VizShape] No MESH, BOX, CYLINDER OR SPHERE! Exiting"<<std::endl;
    return false;
  }

  return true;
}

/**
 * @function add_ColShape
 */
bool  DartLoader::add_ColShape( dynamics::BodyNodeDynamics* _node,
				boost::shared_ptr<urdf::Collision> _col,
				std::string _rootToSkelPath ) {

  // Variables to use
  kinematics::Shape* shape;
  const aiScene* model;
  urdf::Pose pose;

  // Origin
  pose = _col->origin;  

  if(debug) { std::cout<< "Loading colShape for node "<< _node->getName() << std::endl; }
  
  // Type of Geometry
  
  //-- SPHERE
  if( _col->geometry->type == urdf::Geometry::SPHERE ) {
    
    boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>( _col->geometry );
    shape = new kinematics::ShapeEllipsoid(2.0 * sphere->radius * Eigen::Vector3d::Ones());
    
    // Set its pose
    Eigen::Affine3d transform;
    transform = pose2Affine3d( pose );
    
    // Set it into shape
    shape->setTransform( transform );
    
    // Set in node
    _node->setCollisionShape(shape);
    if(debug) { std::cout<< "Loading a sphere colMesh of radius:" << sphere->radius << std::endl; }

  }

  //-- BOX
  else if( _col->geometry->type == urdf::Geometry::BOX ) {

    boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>( _col->geometry );
    Eigen::Vector3d dim; dim<< box->dim.x, box->dim.y, box->dim.z;
    shape = new kinematics::ShapeBox( dim );
    
    // Set its pose
    Eigen::Affine3d transform;
    transform = pose2Affine3d( pose );
    
    // Set it into shape
    shape->setTransform( transform );
    
    // Set in node
    _node->setCollisionShape(shape);
    if(debug) { std::cout<< "Loading a box colMesh of dim:" << dim.transpose() << std::endl; }

  }

  //-- CYLINDER
  else if( _col->geometry->type == urdf::Geometry::CYLINDER ) {

    boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>( _col->geometry );
    shape = new kinematics::ShapeCylinder( cylinder->radius, cylinder->length );
    
    // Set its pose
    Eigen::Affine3d transform;
    transform = pose2Affine3d( pose );
    
    // Set it into shape
    shape->setTransform( transform );
    
    // Set in node
    _node->setCollisionShape(shape);
    if(debug) { std::cout<< "Loading a cylinder colMesh of radius:" << cylinder->radius<<" and length: "<<cylinder->length<< std::endl; }
  }
  
  //-- Mesh
  else if( _col->geometry->type == urdf::Geometry::MESH ) {

    boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>( _col->geometry );
    std::string fullPath = _rootToSkelPath;
    fullPath.append( mesh->filename );

    // Load aiScene visualization
    model = kinematics::ShapeMesh::loadMesh( fullPath );    
    
    if( model == NULL ) {
      std::cout<< "[add_ColShape] [ERROR] Not loading model "<< fullPath<<" (NULL) \n";
      return false;  
    }
    
    // Set shape as mesh
    shape = new kinematics::ShapeMesh( Eigen::Vector3d( 1, 1, 1), model );
    
    // Set its pose
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    // Set xyz
    Eigen::Vector3d t;
    t[0] = pose.position.x;
    t[1] = pose.position.y;
    t[2] = pose.position.z;
    transform.translation() = t;
    // Set rpy
    double roll, pitch, yaw;
    pose.rotation.getRPY( roll, pitch, yaw );
    Eigen::Matrix3d rot;
    rot  = Eigen::AngleAxisd( yaw, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd( pitch, Eigen::Vector3d::UnitY())* Eigen::AngleAxisd( roll, Eigen::Vector3d::UnitX() );
    transform.matrix().block(0,0,3,3) = rot;
    
    // Set it into shape
    shape->setTransform( transform );
    
    if(debug) std::cerr << "[debug] Loading visual model: " << fullPath << std::endl;
    
    // Set in node
    _node->setCollisionShape(shape);
    
  } // end if (mesh)

  else {
    std::cout<< "[set_ColShape] No MESH, BOX, CYLINDER OR SPHERE! Exiting"<<std::endl;
    return false;
  }

  return true;

}

/**
 * @function pose2Affine3d
 */
Eigen::Affine3d DartLoader::pose2Affine3d( urdf::Pose _pose ) {
    Eigen::Quaterniond quat;
    _pose.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
    Eigen::Affine3d transform(quat);
    transform.translation() = Eigen::Vector3d(_pose.position.x, _pose.position.y, _pose.position.z);
    return transform;
}
