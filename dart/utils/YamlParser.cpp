/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/utils/YamlParser.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <yaml-cpp/yaml.h>

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/common/Console.hpp"
#include "dart/config.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/dynamics/BallJoint.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/EulerJoint.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Marker.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/MultiSphereConvexHullShape.hpp"
#include "dart/dynamics/PlanarJoint.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/ScrewJoint.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/TranslationalJoint.hpp"
#include "dart/dynamics/UniversalJoint.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/utils/CompositeResourceRetriever.hpp"
#include "dart/utils/DartResourceRetriever.hpp"
#include "dart/utils/XmlHelpers.hpp"

namespace dart {
namespace {

//==============================================================================
template <typename MatrixType, bool IsVectorAtCompileTime>
struct encode_impl
{
  // Nothing defined. This class should be always specialized.

  // The reason this exists is so we can use template specialization to switch
  // between serializing vectors and serializing matrices based on the
  // IsVectorAtCompileTime flag. This is a "nice to have" that lets us generate
  // [1, 2, 3] instead of the syntactic travesty like [[1], [2] ,[3]] when
  // serializing an Eigen vector.
};

//==============================================================================
// Specialization for vector type
template <typename MatrixType>
struct encode_impl<MatrixType, true>
{
  static YAML::Node encode(const MatrixType& matrix)
  {
    using Index = typename MatrixType::Index;

    YAML::Node node;
    for (Index i = 0; i < matrix.size(); ++i)
      node.push_back(YAML::Node(matrix[i]));

    return node;
  }
};

//==============================================================================
// Specialization for matrix type
template <typename MatrixType>
struct encode_impl<MatrixType, false>
{
  static YAML::Node encode(const MatrixType& matrix)
  {
    using Index = typename MatrixType::Index;

    YAML::Node node;
    for (Index r = 0; r < matrix.rows(); ++r)
    {
      YAML::Node row(YAML::NodeType::Sequence);

      for (Index c = 0; c < matrix.cols(); ++c)
        row.push_back(matrix(r, c));

      node.push_back(row);
    }

    return node;
  }
};

//==============================================================================
inline YAML::Mark getMark(const YAML::Node& node)
{
#ifdef YAMLCPP_NODE_HAS_MARK
  return node.Mark();
#else
  DART_UNUSED(node);
  return YAML::Mark::null_mark();
#endif
  // We need this because Node::Mark() was introduced since yaml-cpp 0.5.3.
  // By using null_mark(), the error message wouldn't tell the exact location
  // (i.e., line number and column in the yaml file) where the error occurred.
}

//==============================================================================
common::ResourceRetrieverPtr
getRetriever(const common::ResourceRetrieverPtr& retriever)
{
  if (retriever)
  {
    return retriever;
  }
  else
  {
    auto newRetriever = std::make_shared<utils::CompositeResourceRetriever>();
    newRetriever->addSchemaRetriever(
        "file", std::make_shared<common::LocalResourceRetriever>());
    newRetriever->addSchemaRetriever("dart", DartResourceRetriever::create());
    return newRetriever;
  }
}

//==============================================================================
simulation::WorldPtr readWorld(const YAML::Node& worldNode)
{
  if (worldNode.Type() != YAML::NodeType::Map)
    return nullptr;

  auto world = simulation::World::create();

  if (auto nameNode = worldNode["name"])
    world->setName(nameNode.as<std::string>());

  if (auto physicsNode = worldNode["physics"])
  {
    if (auto timeStepNode = physicsNode["time_step"])
      world->setTimeStep(timeStepNode.as<double>());

    if (auto gravityNode = physicsNode["gravity"])
      world->setGravity(gravityNode.as<Eigen::Vector3d>());
  }

  return world;
}

//==============================================================================
std::vector<simulation::WorldPtr> readWorlds(const YAML::Node& node)
{
  std::vector<simulation::WorldPtr> worlds;

  if (node.Type() != YAML::NodeType::Sequence)
    return worlds;

  worlds.reserve(node.size());

  for (const auto& item : node)
    worlds.emplace_back(std::move(readWorld(item)));

  return worlds;
}

} // anonymous namespace
} // namespace dart

namespace YAML {

//==============================================================================
// Specialization for Eigen::Matrix<...>. This enables to use YAML::Node
// with std::unordered_map as `Node(Eigen::Matrix3d::Identity())` and
// `node.as<Eigen::Matrix3d>()`.
template <typename _Scalar,
          int _Rows,
          int _Cols,
          int _Options,
          int _MaxRows,
          int _MaxCols>
struct convert<Eigen::
                   Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>>
{
  using MatrixType
      = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>;
  using Index = typename MatrixType::Index;

  static Node encode(const MatrixType& matrix)
  {
    using aikido::io::detail::encode_impl;
    return encode_impl<MatrixType, MatrixType::IsVectorAtCompileTime>::encode(
        matrix);
  }

  /// Reads a YAML::Node that encodes a vector or a matrix; and stores it into
  /// `matrix`. If `matrix` is dynamic size Eigen object, resizes it
  /// accordingly.
  static bool decode(const YAML::Node& node, MatrixType& matrix)
  {
    using dart::utils::detail::getMark;

    if (node.Type() != YAML::NodeType::Sequence)
    {
      throw YAML::RepresentationException(
          getMark(node), "Matrix or vector must be a sequence.");
    }

    const Index rows = node.size();
    if (MatrixType::RowsAtCompileTime != Eigen::Dynamic
        && rows != MatrixType::RowsAtCompileTime)
    {
      std::stringstream ss;
      ss << "Matrix has incorrect number of rows: expected "
         << MatrixType::RowsAtCompileTime << "; got " << rows << ".";
      throw YAML::RepresentationException(getMark(node), ss.str());
    }

    if (node.size() > 0 && node[0].Type() == YAML::NodeType::Sequence)
    {
      // Matrix case

      const auto cols = node[0].size();

      if (MatrixType::ColsAtCompileTime != Eigen::Dynamic
          && cols != static_cast<std::size_t>(MatrixType::ColsAtCompileTime))
      {
        std::stringstream ss;
        ss << "Matrix has incorrect number of cols: expected "
           << MatrixType::ColsAtCompileTime << "; got " << cols << ".";
        throw YAML::RepresentationException(getMark(node), ss.str());
      }

      if (cols == 0u)
        matrix.resize(0, 0);
      else
        matrix.resize(rows, cols);

      for (auto r = 0u; r < node.size(); ++r)
      {
        if (node[r].Type() != YAML::NodeType::Sequence)
        {
          std::stringstream ss;
          ss << "Row " << r << " of the matrix must be a sequence.";
          throw YAML::RepresentationException(getMark(node), ss.str());
        }
        else if (node[r].size() != cols)
        {
          std::stringstream ss;
          ss << "Expected row " << r << " to have " << cols << " columns; got "
             << node[r].size() << ".";
          throw YAML::RepresentationException(getMark(node), ss.str());
        }

        for (auto c = 0u; c < cols; ++c)
        {
          if (node[r][c].Type() != YAML::NodeType::Scalar)
          {
            std::stringstream ss;
            ss << "Matrix has a non-scalar element at (" << r << ", " << c
               << ") component (1-based numbering).";
            throw YAML::RepresentationException(getMark(node), ss.str());
          }

          matrix(r, c) = node[r][c].template as<_Scalar>();
        }
      }
    }
    else
    {
      // Vector case

      if (MatrixType::ColsAtCompileTime != Eigen::Dynamic
          && 1 != MatrixType::ColsAtCompileTime)
      {
        std::stringstream ss;
        ss << "Matrix has incorrect number of cols: expected "
           << MatrixType::ColsAtCompileTime << "; got " << 1 << ".";
        throw YAML::RepresentationException(getMark(node), ss.str());
      }

      matrix.resize(rows, 1);
      for (Index i = 0; i < rows; ++i)
      {
        if (node[i].Type() != YAML::NodeType::Scalar)
        {
          std::stringstream ss;
          ss << "Vector has a non-scalar element at the " << i + 1
             << "-th component (1-based numbering).";
          throw YAML::RepresentationException(getMark(node), ss.str());
        }

        matrix(i, 0) = node[i].template as<_Scalar>();
      }
    }

    return true;
  }
};

//==============================================================================
// Specialization for Eigen::Isometry<...>. This enables to use YAML::Node
// with std::unordered_map as `Node(Eigen::Isometry3d::Identity())` and
// `node.as<Eigen::Isometry3d>()`.
template <typename _Scalar, int _Dim, int _Mode, int _Options>
struct convert<Eigen::Transform<_Scalar, _Dim, _Mode, _Options>>
{
  using TransformType = Eigen::Transform<_Scalar, _Dim, _Mode, _Options>;
  using MatrixType = typename TransformType::MatrixType;

  static Node encode(const TransformType& transform)
  {
    return convert<MatrixType>::encode(transform.matrix());
  }

  static bool decode(const Node& node, TransformType& transform)
  {
    return convert<MatrixType>::decode(node, transform.matrix());
  }
};

//==============================================================================
// Specialization for std::unordered_map<...>. This enables to use YAML::Node
// with std::unordered_map as `Node(std::unorderd_map<...>())` and
// `node.as<std::unordered_map<...>>()`.
template <typename _Key,
          typename _Tp,
          typename _Hash,
          typename _Pred,
          typename _Alloc>
struct convert<std::unordered_map<_Key, _Tp, _Hash, _Pred, _Alloc>>
{
  using UnorderedMap = std::unordered_map<_Key, _Tp, _Hash, _Pred, _Alloc>;

  static Node encode(const UnorderedMap& map)
  {
    Node node(NodeType::Map);

    for (const auto& it : map)
      node.force_insert(it.first, it.second);

    return node;
  }

  static bool decode(const Node& node, UnorderedMap& map)
  {
    if (!node.IsMap())
      return false;

    map.clear();
    map.reserve(node.size());

    for (const auto& it : node)
      map[it.first.as<_Key>()] = it.second.as<_Tp>();

    return true;
  }
};

} // namespace YAML

namespace dart {
namespace utils {
namespace YamlParser {

//==============================================================================
std::vector<simulation::WorldPtr> read(
    const common::Uri& uri, const common::ResourceRetrieverPtr& nullOrRetriever)
{
  const common::ResourceRetrieverPtr retriever = getRetriever(nullOrRetriever);

  auto content = retriever->readAll(uri);

  YAML::Node node = YAML::Load(content);

  std::vector<simulation::WorldPtr> worlds;

  YAML::Node worldsNode = node["worlds"];
  if (worldsNode.Type() == YAML::NodeType::Sequence)
    worlds = readWorlds(worldsNode);

  return worlds;
}

} // namespace YamlParser
} // namespace utils
} // namespace dart
