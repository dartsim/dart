/**
 * @file DartLoader.h
 */

#ifndef DART_LOADER_H
#define DART_LOADER_H

#include "../urdf_parser/urdf_parser.h"

#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "robotics/World.h"
#include "robotics/Robot.h"
#include "robotics/Object.h"
#include "utils/Paths.h"

// To load Mesh and Skel
#include  <kinematics/Joint.h>
#include <kinematics/ShapeMesh.h>
#include <kinematics/Transformation.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/TrfmRotateEuler.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Dof.h>
#include <robotics/Constants.h>


#include <string>

// Type of DOF Enum
enum TypeOfDOF {
  GOLEM_X, GOLEM_Y, GOLEM_Z, 
  GOLEM_ROLL, GOLEM_PITCH, GOLEM_YAW
};

/**
 * @class DartLoader
 */
class DartLoader {

 public:
  DartLoader();
  ~DartLoader();

  dynamics::SkeletonDynamics* parseSkeleton( std::string _urdfFile );

  robotics::Robot* parseRobot( std::string _urdfFile );
  robotics::Object* parseObject( std::string _urdfFile );
  robotics::World* parseWorld( std::string _urdfFile );

  dynamics::SkeletonDynamics* modelInterfaceToSkeleton( boost::shared_ptr<urdf::ModelInterface> _model );
  robotics::Robot* modelInterfaceToRobot( boost::shared_ptr<urdf::ModelInterface> _model );
  robotics::Object* modelInterfaceToObject( boost::shared_ptr<urdf::ModelInterface> _model );

  // Utilities
  dynamics::BodyNodeDynamics* getNode( std::string _nodeName );
  std::string readXmlToString( std::string _xmlFile );

  // Loader utils
  void add_XyzRpy(kinematics::Joint* _joint, 
		  double _x, double _y, double _z, 
		  double _rr, double _rp, double _ry );

  void add_DOF(dynamics::SkeletonDynamics* _skel, 
	       kinematics::Joint* _joint, 
	       double _val, double _min, double _max,
	       int _DOF_TYPE );

  void add_ShapeMesh( dynamics::BodyNodeDynamics* _node, 
		      const char *_meshPath, 
		      double _mass = 1.0,
		      Eigen::Matrix3d _inertiaMatrix = Eigen::MatrixXd::Identity(3,3),
		      const char *_collisionMeshPath = NULL );
  
  void add_Shape( dynamics::BodyNodeDynamics* _node, 
		  double _mass = 1.0,
		  Eigen::Matrix3d _inertiaMatrix = Eigen::MatrixXd::Identity(3,3) );
  
  // ToDart utils
  kinematics::Joint* createDartRootJoint( boost::shared_ptr<const urdf::Link> _rootLink,
					  dynamics::SkeletonDynamics* _skel,
					  bool _createdDummyRoot = false );
  kinematics::Joint* createDartJoint( boost::shared_ptr<urdf::Joint> _jt,
				      dynamics::SkeletonDynamics* _skel );
  dynamics::BodyNodeDynamics* createDartNode( boost::shared_ptr<urdf::Link> _lk,
					      dynamics::SkeletonDynamics* _skel ); 
  
  // Member variables
  std::vector<dynamics::BodyNodeDynamics*> mNodes;
  std::vector<kinematics::Joint*> mJoints;
  
  std::string mWorldPath;
  std::string mPath;
  
};


#endif /** DART_LOADER_H */
