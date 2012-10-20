/**
 * @file LoaderUtils.cpp
 * @brief Utils to load Dart objects
 */

#include "DartLoader.h"

/**
 * @function add_XyzRpy
 */
void DartLoader::add_XyzRpy( kinematics::Joint* _joint, 
			     double _x, double _y, double _z, 
			     double _rr, double _rp, double _ry ) {

  kinematics::Transformation* trans;
  
  trans = new kinematics::TrfmTranslate( new kinematics::Dof(_x), new kinematics::Dof(_y), new kinematics::Dof(_z),
					 "Translate" );
  _joint->addTransform(trans, false);
  
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
			  int _DOF_TYPE ) {

  kinematics::Transformation* trans;
  
  if(_DOF_TYPE == GOLEM_X) {
    trans = new kinematics::TrfmTranslateX(new kinematics::Dof(0, "rootX"), "Tx");
    _joint->addTransform(trans, true);
    _joint->getDof(0)->setMin(_min);
    _joint->getDof(0)->setMax(_max);
    _skel->addTransform(trans);
  }
  else if(_DOF_TYPE == GOLEM_Y) {
    trans = new kinematics::TrfmTranslateY(new kinematics::Dof(0, "rootY"), "Ty");
    _joint->addTransform(trans, true);
    _joint->getDof(0)->setMin(_min);
    _joint->getDof(0)->setMax(_max);
    _skel->addTransform(trans);
  }
  else if(_DOF_TYPE == GOLEM_Z) {
    trans = new kinematics::TrfmTranslateZ(new kinematics::Dof(0, "rootZ"), "Tz");
    _joint->addTransform(trans, true);
    _joint->getDof(0)->setMin(_min);
    _joint->getDof(0)->setMax(_max);
    _skel->addTransform(trans);
  }
  else if(_DOF_TYPE == GOLEM_YAW) {
    trans = new kinematics::TrfmRotateEulerZ(new kinematics::Dof(0, "rootYaw"), "Try");
    _joint->addTransform(trans, true);
    _joint->getDof(0)->setMin(_min);
    _joint->getDof(0)->setMax(_max);
    _skel->addTransform(trans);
  }
  else if(_DOF_TYPE == GOLEM_PITCH) {
    trans = new kinematics::TrfmRotateEulerY(new kinematics::Dof(0, "rootPitch"), "Trp");
    _joint->addTransform(trans, true);
    _joint->getDof(0)->setMin(_min);
    _joint->getDof(0)->setMax(_max);
    _skel->addTransform(trans);
  }
  else if(_DOF_TYPE == GOLEM_ROLL) {
    trans = new kinematics::TrfmRotateEulerX(new kinematics::Dof(0, "rootRoll"), "Trr");
    _joint->addTransform(trans, true);
    _joint->getDof(0)->setMin(_min);
    _joint->getDof(0)->setMax(_max);
    _skel->addTransform(trans);
  }
  else {
    printf(" WATCH OUT! THIS SHOULD NOT HAPPEN, NO DOF SET \n");
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
	_node->setShape(shape);

}


/**
 * @function add_ShapeMesh
 */
void DartLoader::add_ShapeMesh( dynamics::BodyNodeDynamics* _node, 
				const char *_meshPath, 
				double _mass,
				Eigen::Matrix3d _inertiaMatrix,
				const char *_collisionMeshPath  ) {
  
  kinematics::Shape* shape;

  // Load aiScene visualization
  const aiScene* model = kinematics::ShapeMesh::loadMesh( _meshPath );

  // Load collision model
  const aiScene* collisionModel = kinematics::ShapeMesh::loadMesh( _collisionMeshPath );

  if( model == NULL ) {
    printf("[add_Shape] [ERROR] Not loading model %s (NULL) \n", _meshPath);
    return;  
  }
  else {
    shape = new kinematics::ShapeMesh( Eigen::Vector3d( 1, 1, 1),
				       _mass,
				       model ); 	 
    shape->setInertia( _inertiaMatrix );
			printf("** Loading visual model: %s \n", _meshPath );
    shape->setVizMesh( model ); 

    // Check if we have got a collision model
    if( !collisionModel ) {
      shape->setCollisionMesh( model );
    } else {
			printf("** Loading collision model: %s \n", _collisionMeshPath );
      shape->setCollisionMesh( collisionModel );
    }

    // Set in node
    _node->setShape( shape );
  } 
  
}
