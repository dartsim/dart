/**
 * @file DartLoader.h
 */

#ifndef DART_LOADER_H
#define DART_LOADER_H

#include <Eigen/Core>
#include <vector>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <robotics/parser/urdfdom_headers/urdf_model/pose.h>

const bool debug = false;

namespace dynamics {
	class SkeletonDynamics;
	class BodyNodeDynamics;
}
namespace robotics {
	class World;
	class Robot;
	class Object;
}
namespace kinematics {
	class Joint;
}
namespace urdf {
	class ModelInterface;
	class Link;
	class Joint;
}


// Type of DOF Enum
enum TypeOfDOF {
  GOLEM_X, GOLEM_Y, GOLEM_Z, 
  GOLEM_ROLL, GOLEM_PITCH, GOLEM_YAW,
  GOLEM_ARBITRARY_ROTATION
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
  robotics::Robot* modelInterfaceToRobot( boost::shared_ptr<urdf::ModelInterface> _model,
					  std::string _rootToRobotPath = NULL );
  robotics::Object* modelInterfaceToObject( boost::shared_ptr<urdf::ModelInterface> _model,
					    std::string _rootToObjectPath = NULL );

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
	       int _DOF_TYPE,
	       double _x = 0, double _y = 0, double _z = 0 );

  void add_ShapeMesh( dynamics::BodyNodeDynamics* _node, 
		      const char *_meshPath, 
		      double _mass = 1.0,
		      Eigen::Matrix3d _inertiaMatrix = Eigen::MatrixXd::Identity(3,3),
		      const char *_collisionMeshPath = NULL,
		      urdf::Pose _pose = urdf::Pose() );
  
  void add_Shape( dynamics::BodyNodeDynamics* _node, 
		  double _mass = 1.0,
		  Eigen::Matrix3d _inertiaMatrix = Eigen::MatrixXd::Identity(3,3) );
  
  // ToDart utils
  kinematics::Joint* createDartRootJoint( dynamics::BodyNodeDynamics* _node,
					  dynamics::SkeletonDynamics* _skel );
  kinematics::Joint* createDartJoint( boost::shared_ptr<urdf::Joint> _jt,
				      dynamics::SkeletonDynamics* _skel );
  dynamics::BodyNodeDynamics* createDartNode( boost::shared_ptr<urdf::Link> _lk,
					      dynamics::SkeletonDynamics* _skel,
					      std::string _rootToSkelPath = NULL ); 
  
  // Member variables
  std::vector<dynamics::BodyNodeDynamics*> mNodes;
  std::vector<kinematics::Joint*> mJoints;
  
  std::string mRoot_To_World_Path;
  std::map<std::string, std::string> mWorld_To_Entity_Paths;
  void parseWorldToEntityPaths( const std::string &_xml_string );
  
};


#endif /** DART_LOADER_H */
