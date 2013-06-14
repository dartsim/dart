/**
 * @file DartLoader.h
 */

#ifndef DART_LOADER_H
#define DART_LOADER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <robotics/parser/urdfdom_headers/urdf_model/pose.h>
#include <robotics/parser/urdfdom_headers/urdf_model/link.h>
#include <robotics/parser/urdfdom_headers/urdf_model/color.h>

const bool debug = false;

namespace dynamics {
	class SkeletonDynamics;
	class BodyNodeDynamics;
}
namespace simulation {
	class World;
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
  GOLEM_ARBITRARY_ROTATION,
  GOLEM_FLOATING
};

/**
 * @class DartLoader
 */
class DartLoader {
  
 public:
  DartLoader();
  ~DartLoader();
  
  dynamics::SkeletonDynamics* parseSkeleton( std::string _urdfFile,
					     std::string _rootToSkelPath = NULL );
  
  simulation::World* parseWorld( std::string _urdfFile );
  
  void parseWorldToEntityPaths( const std::string &_xml_string );

  dynamics::SkeletonDynamics* modelInterfaceToSkeleton( boost::shared_ptr<urdf::ModelInterface> _model,
							std::string _rootToSkelPath = NULL );
  
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

  bool add_VizShape( dynamics::BodyNodeDynamics* _node,
		     boost::shared_ptr<urdf::Visual> _viz,
		     std::string  _rootToSkelPath );
  bool add_ColShape( dynamics::BodyNodeDynamics* _node,
		     boost::shared_ptr<urdf::Collision> _col,
		     std::string _rootToSkelPath );

  // ToDart utils
  kinematics::Joint* createDartRootJoint( boost::shared_ptr<urdf::Joint> _jt,
					  dynamics::SkeletonDynamics* _skel ); 
  kinematics::Joint* createNewDartRootJoint( dynamics::BodyNodeDynamics* _node,
					     dynamics::SkeletonDynamics* _skel );


  kinematics::Joint* createDartJoint( boost::shared_ptr<urdf::Joint> _jt,
				      dynamics::SkeletonDynamics* _skel );
  dynamics::BodyNodeDynamics* createDartNode( boost::shared_ptr<urdf::Link> _lk,
					      dynamics::SkeletonDynamics* _skel,
					      std::string _rootToSkelPath = NULL ); 

  // Useful helpers
  Eigen::Affine3d pose2Affine3d( urdf::Pose _pose );

  // Member variables
  std::vector<dynamics::BodyNodeDynamics*> mNodes;
  std::vector<kinematics::Joint*> mJoints;
  
  std::string mRoot_To_World_Path;
  std::map<std::string, std::string> mWorld_To_Entity_Paths;
  
};


#endif /** DART_LOADER_H */
