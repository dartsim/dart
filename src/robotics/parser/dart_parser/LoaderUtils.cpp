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
#include <dynamics/BodyNodeDynamics.h>
#include <iostream>


/**
 * @function add_XyzRpy
 */
void DartLoader::add_XyzRpy( kinematics::Joint* _joint, 
			     double _x, double _y, double _z, 
			     double _rr, double _rp, double _ry ) {

  kinematics::Transformation* trans;
  
  trans = new kinematics::TrfmTranslateX( new kinematics::Dof(_x) );
  _joint->addTransform( trans, false );

  trans = new kinematics::TrfmTranslateY( new kinematics::Dof(_y) );
  _joint->addTransform( trans, false );

  trans = new kinematics::TrfmTranslateZ( new kinematics::Dof(_z) );
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
  else {
    if(debug) std::cerr << " WATCH OUT! THIS SHOULD NOT HAPPEN, NO DOF SET" << std::endl;
  }
  
}

/**
 * @function add_Shape
 */
void DartLoader::add_Shape( dynamics::BodyNodeDynamics* _node, 
			    double _mass,
			    Eigen::Matrix3d _inertiaMatrix ) {

  kinematics::Shape* shape;
  shape = new kinematics::Shape();
  _node->setVizShape(shape);
  _node->setColShape(shape);

}


/**
 * @function add_ShapeMesh
 */
void DartLoader::add_ShapeMesh( dynamics::BodyNodeDynamics* _node, 
				const char *_meshPath, 
				double _mass,
				Eigen::Matrix3d _inertiaMatrix,
				const char *_collisionMeshPath,
				urdf::Pose _pose ) {
  
  kinematics::Shape* vizShape, *colShape;
  
  // Load aiScene visualization
  const aiScene* model = kinematics::ShapeMesh::loadMesh( _meshPath );
  
  // Load collision model
  const aiScene* collisionModel = kinematics::ShapeMesh::loadMesh( _collisionMeshPath );
  
  if( model == NULL ) {
    std::cout<< "[add_Shape] [ERROR] Not loading model "<<_meshPath<<" (NULL) \n";
    return;  
  }
  else {
    vizShape = new kinematics::ShapeMesh( Eigen::Vector3d( 1, 1, 1), model );
    colShape = new kinematics::ShapeMesh(Eigen::Vector3d(1,1,1), collisionModel);

    // Use colShape only if collision model exists
    if(!collisionModel) {
    	delete colShape;
    	colShape = vizShape;
    }
    
    // Set the visPose
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    // Set xyz
    Eigen::Vector3d t;
    t[0] = _pose.position.x;
    t[1] = _pose.position.y;
    t[2] = _pose.position.z;
    transform.translation() = t;
    // Set rpy
    double roll, pitch, yaw;
    _pose.rotation.getRPY( roll, pitch, yaw );
    Eigen::Matrix3d rot;
    rot  = Eigen::AngleAxisd( yaw, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd( pitch, Eigen::Vector3d::UnitY())* Eigen::AngleAxisd( roll, Eigen::Vector3d::UnitX() );
    transform.matrix().block(0,0,3,3) = rot;

    // Set into the shape
    vizShape->setTransform( transform );
    colShape->setTransform(transform);
    
    if(debug) std::cerr << "** Loading visual model: " << _meshPath << std::endl;
    if(debug) std::cerr << "** Loading collision model: " <<  _collisionMeshPath << std::endl ;
    
    // Set in node
    _node->setVizShape(vizShape);
    _node->setColShape(colShape);
    _node->setMass(_mass);
  } 
  
}
