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

void setMaterial(kinematics::Shape* _shape, const urdf::Visual* _viz) {
  if(_viz->material) {
    _shape->setColor(Eigen::Vector3d(_viz->material->color.r, _viz->material->color.g, _viz->material->color.b));
  }
}

void setMaterial(kinematics::Shape* _shape, const urdf::Collision* _col) {
}

/**
 * @function createShape
 */
template <class VisualOrCollision>
kinematics::Shape* DartLoader::createShape(boost::shared_ptr<VisualOrCollision> _vizOrCol,
                                           std::string _rootToSkelPath)
{
  kinematics::Shape* shape = NULL;

  // Sphere
  if(urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(_vizOrCol->geometry.get())) {
    shape = new kinematics::ShapeEllipsoid(2.0 * sphere->radius * Eigen::Vector3d::Ones());
    if(debug) std::cout << "Loading a sphere of radius:" << sphere->radius << std::endl;
  }

  // Box
  else if(urdf::Box* box = dynamic_cast<urdf::Box*>(_vizOrCol->geometry.get())) {
    shape = new kinematics::ShapeBox(Eigen::Vector3d(box->dim.x, box->dim.y, box->dim.z));
    if(debug) std::cout << "Loading a box of dim:" << box->dim.x << ", " << box->dim.y << ", " << box->dim.z << std::endl;
  }

  // Cylinder
  else if(urdf::Cylinder* cylinder = dynamic_cast<urdf::Cylinder*>(_vizOrCol->geometry.get())) {
    shape = new kinematics::ShapeCylinder(cylinder->radius, cylinder->length);
    if(debug) std::cout << "Loading a cylinder of radius:" << cylinder->radius << " and length: " << cylinder->length << std::endl;
  }

  // Mesh
  else if(urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(_vizOrCol->geometry.get())) {
    std::string fullPath = _rootToSkelPath + mesh->filename;
    const aiScene* model = kinematics::ShapeMesh::loadMesh( fullPath );
    
    if(!model) {
      std::cout<< "[add_Shape] [ERROR] Not loading model " << fullPath << " (NULL) \n";
    } 
    else {
      shape = new kinematics::ShapeMesh(Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z), model);
      if(debug) std::cout << "[debug] Loading visual model: " << fullPath << std::endl;
    }
  }

  // Unknown geometry type
  else {
    std::cout << "[add_Shape] No MESH, BOX, CYLINDER OR SPHERE! Exiting" << std::endl;
    return NULL;
  }

  shape->setTransform(pose2Affine3d(_vizOrCol->origin));
  setMaterial(shape, _vizOrCol.get());
  return shape;
}

template kinematics::Shape* DartLoader::createShape<urdf::Visual>(boost::shared_ptr<urdf::Visual> _vizOrCol,
                                                                  std::string _rootToSkelPath);
template kinematics::Shape* DartLoader::createShape<urdf::Collision>(boost::shared_ptr<urdf::Collision> _vizOrCol,
                                                                     std::string _rootToSkelPath);

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
