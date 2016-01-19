#ifndef KIDO_UTILS_SDFPARSER_H
#define KIDO_UTILS_SDFPARSER_H

#include <map>
#include <string>
#include <functional>
#include <Eigen/Dense>
#include <Eigen/StdVector>
// TinyXML-2 Library
// http://www.grinninglizard.com/tinyxml2/index.html
#include <tinyxml2.h>

#include "kido/common/Deprecated.hpp"
#include "kido/utils/XmlHelpers.hpp"
#include "kido/common/ResourceRetriever.hpp"
#include "kido/dynamics/BodyNode.hpp"
#include "kido/dynamics/WeldJoint.hpp"
#include "kido/dynamics/RevoluteJoint.hpp"
#include "kido/dynamics/PrismaticJoint.hpp"
#include "kido/dynamics/ScrewJoint.hpp"
#include "kido/dynamics/UniversalJoint.hpp"
#include "kido/dynamics/BallJoint.hpp"
#include "kido/dynamics/TranslationalJoint.hpp"
#include "kido/dynamics/FreeJoint.hpp"
#include "kido/simulation/World.hpp"

namespace kido {

namespace dynamics {
class Skeleton;
class BodyNode;
class Shape;
class Joint;
class WeldJoint;
class PrismaticJoint;
class RevoluteJoint;
class ScrewJoint;
class UniversalJoint;
class BallJoint;
class EulerXYZJoint;
class EulerJoint;
class TranslationalJoint;
class FreeJoint;
}
namespace simulation {
class World;
}

namespace utils {

class SdfParser
{
public:
  // TODO: Make common::ResourceRetriever optional.
  static kido::simulation::WorldPtr readSdfFile(
      const common::Uri& _fileUri,
      const common::ResourceRetrieverPtr& _retriever = nullptr);

  static simulation::WorldPtr readSdfFile(
      const common::Uri& _fileUri,
      const common::ResourceRetrieverPtr& _retriever,
      std::function<simulation::WorldPtr (
        tinyxml2::XMLElement*, const std::string&,
        const common::ResourceRetrieverPtr&)> xmlReader);

  // TODO: Make common::ResourceRetriever optional.
  static dynamics::SkeletonPtr readSkeleton(
      const common::Uri& _fileUri,
      const common::ResourceRetrieverPtr& _retriever = nullptr);

  static dynamics::SkeletonPtr readSkeleton(
      const common::Uri& _fileUri,
      const common::ResourceRetrieverPtr& _retriever,
      std::function<dynamics::SkeletonPtr(
        tinyxml2::XMLElement*, const std::string&,
        const common::ResourceRetrieverPtr&)> xmlReader);

    typedef std::shared_ptr<dynamics::BodyNode::Properties> BodyPropPtr;

    struct SDFBodyNode
    {
        BodyPropPtr properties;
        Eigen::Isometry3d initTransform;
        std::string type;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    typedef std::shared_ptr<dynamics::Joint::Properties> JointPropPtr;

    struct SDFJoint
    {
      JointPropPtr properties;
      std::string parentName;
      std::string childName;
      std::string type;
    };

    // Maps the name of a BodyNode to its properties
    typedef Eigen::aligned_map<std::string, SDFBodyNode> BodyMap;

    // Maps a child BodyNode to the properties of its parent Joint
    typedef std::map<std::string, SDFJoint> JointMap;

    static simulation::WorldPtr readWorld(
        tinyxml2::XMLElement* _worldElement,
        const std::string& _skelPath,
        const common::ResourceRetrieverPtr& _retriever);

    /// \brief
    static simulation::WorldPtr readWorld(
        tinyxml2::XMLElement* _worldElement,
        const std::string& _skelPath,
        const common::ResourceRetrieverPtr& _retriever,
        std::function<dynamics::SkeletonPtr(
          tinyxml2::XMLElement*, const std::string&,
          const common::ResourceRetrieverPtr&)> skeletonReader);

    /// \brief
    static void readPhysics(tinyxml2::XMLElement* _physicsElement,
            simulation::WorldPtr _world);

    /// \brief
    static dynamics::SkeletonPtr readSkeleton(
        tinyxml2::XMLElement* _skeletonElement,
        const std::string& _skelPath,
        const common::ResourceRetrieverPtr& _retriever);

    static dynamics::SkeletonPtr readSkeleton(
        tinyxml2::XMLElement* _skeletonElement,
        const std::string& _skelPath,
        const common::ResourceRetrieverPtr& _retriever,

        std::function<SDFBodyNode (tinyxml2::XMLElement*,
                                   const Eigen::Isometry3d&,
                                   const std::string&,
                                   const common::ResourceRetrieverPtr&)> bodyReader,

        std::function<bool(dynamics::SkeletonPtr,
                           dynamics::BodyNode*,
                           const SDFJoint&,
                           const SDFBodyNode&)> pairCreator);

    static bool createPair(dynamics::SkeletonPtr skeleton,
                           dynamics::BodyNode* parent,
                           const SDFJoint& newJoint,
                           const SDFBodyNode& newBody);

    enum NextResult
    {
      VALID,
      CONTINUE,
      BREAK,
      CREATE_FREEJOINT_ROOT
    };

    static NextResult getNextJointAndNodePair(BodyMap::iterator& body,
        JointMap::const_iterator& parentJoint,
        dynamics::BodyNode*& parentBody,
        const dynamics::SkeletonPtr skeleton,
        BodyMap& sdfBodyNodes,
        const JointMap& sdfJoints);

    /// \brief
    static dynamics::SkeletonPtr makeSkeleton(
        tinyxml2::XMLElement* _skeletonElement,
        Eigen::Isometry3d& skeletonFrame);

    /// \brief
    template <class NodeType>
    static std::pair<dynamics::Joint*,dynamics::BodyNode*> createJointAndNodePair(
        dynamics::SkeletonPtr skeleton,
        dynamics::BodyNode* parent,
        const SDFJoint& joint,
        const SDFBodyNode& node)
    {
      const std::string& type = joint.type;

      if (std::string("prismatic") == type)
        return skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(parent,
              static_cast<const dynamics::PrismaticJoint::Properties&>(*joint.properties),
              static_cast<const typename NodeType::Properties&>(*node.properties));
      else if (std::string("revolute") == type)
        return skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(parent,
              static_cast<const dynamics::RevoluteJoint::Properties&>(*joint.properties),
              static_cast<const typename NodeType::Properties&>(*node.properties));
      else if (std::string("screw") == type)
        return skeleton->createJointAndBodyNodePair<dynamics::ScrewJoint>(parent,
              static_cast<const dynamics::ScrewJoint::Properties&>(*joint.properties),
              static_cast<const typename NodeType::Properties&>(*node.properties));
      else if (std::string("revolute2") == type)
        return skeleton->createJointAndBodyNodePair<dynamics::UniversalJoint>(parent,
              static_cast<const dynamics::UniversalJoint::Properties&>(*joint.properties),
              static_cast<const typename NodeType::Properties&>(*node.properties));
      else if (std::string("ball") == type)
        return skeleton->createJointAndBodyNodePair<dynamics::BallJoint>(parent,
              static_cast<const dynamics::BallJoint::Properties&>(*joint.properties),
              static_cast<const typename NodeType::Properties&>(*node.properties));
      else if (std::string("free") == type)
        return skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(parent,
              static_cast<const dynamics::FreeJoint::Properties&>(*joint.properties),
              static_cast<const typename NodeType::Properties&>(*node.properties));

      dterr << "[SdfParser::createJointAndNodePair] Unsupported Joint type encountered: "
            << type << ". Please report this as a bug! We will now quit parsing.\n";
      return std::pair<dynamics::Joint*, dynamics::BodyNode*>(nullptr, nullptr);
    }

    /// \brief
    static BodyMap readAllBodyNodes(
        tinyxml2::XMLElement* _skeletonElement,
        const std::string& _skelPath,
        const common::ResourceRetrieverPtr& _retriever, 
        const Eigen::Isometry3d& skeletonFrame);

    static BodyMap readAllBodyNodes(
        tinyxml2::XMLElement* _skeletonElement,
        const std::string& _skelPath,
        const common::ResourceRetrieverPtr& _retriever, 
        const Eigen::Isometry3d& skeletonFrame,

        std::function<SDFBodyNode(tinyxml2::XMLElement*,
                                  const Eigen::Isometry3d&,
                                  const std::string&,
                                  const common::ResourceRetrieverPtr&)> bodyReader);

    /// \brief
    static SDFBodyNode readBodyNode(
        tinyxml2::XMLElement* _bodyNodeElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const std::string& _skelPath,
        const common::ResourceRetrieverPtr& _retriever);

    /// \brief
    static dynamics::ShapePtr readShape(
            tinyxml2::XMLElement* _shapelement,
            const std::string& _skelPath,
            const common::ResourceRetrieverPtr& _retriever);

    /// \brief
    static JointMap readAllJoints(
        tinyxml2::XMLElement* _skeletonElement,
        const Eigen::Isometry3d& skeletonFrame,
        const BodyMap& sdfBodyNodes);

    /// \brief
    static SDFJoint readJoint(tinyxml2::XMLElement* _jointElement,
        const BodyMap& _bodies,
        const Eigen::Isometry3d& _skeletonFrame);

    static kido::dynamics::WeldJoint::Properties readWeldJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _parentModelFrame,
        const std::string& _name);

    static dynamics::RevoluteJoint::Properties readRevoluteJoint(
        tinyxml2::XMLElement* _revoluteJointElement,
        const Eigen::Isometry3d& _parentModelFrame,
        const std::string& _name);

    static dynamics::PrismaticJoint::Properties readPrismaticJoint(
        tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _parentModelFrame,
        const std::string& _name);

    static dynamics::ScrewJoint::Properties readScrewJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _parentModelFrame,
        const std::string& _name);

    static dynamics::UniversalJoint::Properties readUniversalJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _parentModelFrame,
        const std::string& _name);

    static dynamics::BallJoint::Properties readBallJoint(
        tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _parentModelFrame,
        const std::string& _name);

    static kido::dynamics::EulerJoint* readEulerJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _parentModelFrame,
        const std::string& _name);

    static kido::dynamics::TranslationalJoint::Properties readTranslationalJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _parentModelFrame,
        const std::string& _name);

    static dynamics::FreeJoint::Properties readFreeJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _parentModelFrame,
        const std::string& _name);

    static common::ResourceRetrieverPtr getResourceRetriever(
        const common::ResourceRetrieverPtr& _retriever);
};

} // namespace utils
} // namespace kido

#endif // #ifndef KIDO_UTILS_SDFPARSER_H
