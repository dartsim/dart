/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/detail/basic_node_manager.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/marker.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

using namespace dart::dynamics;

namespace {

class DummyNode final : public Node
{
public:
  explicit DummyNode(BodyNode* bodyNode) : Node(bodyNode) {}

  const std::string& setName(const std::string& newName) override
  {
    mName = newName;
    return mName;
  }

  const std::string& getName() const override
  {
    return mName;
  }

protected:
  Node* cloneNode(BodyNode*) const override
  {
    return nullptr;
  }

private:
  std::string mName{"dummy"};
};

class DummyNodeManager : public detail::BasicNodeManagerForSkeleton
{
public:
  using detail::BasicNodeManagerForSkeleton::getNode;
  using detail::BasicNodeManagerForSkeleton::getNumNodes;

  void addTree()
  {
    mTreeNodeMaps.emplace_back();
    mTreeNodeMaps.back()[typeid(DummyNode)] = std::vector<Node*>();
  }

  void addNode(std::size_t treeIndex, DummyNode* node, const std::string& name)
  {
    mTreeNodeMaps[treeIndex][typeid(DummyNode)].push_back(node);

    auto& manager = mNodeNameMgrMap[typeid(DummyNode)];
    manager.setManagerName("DummyNode");
    manager.addName(name, node);
  }
};

} // namespace

//=============================================================================
TEST(BasicNodeManager, BodyNodeAndSkeletonAccessors)
{
  auto skeleton = Skeleton::create("node_manager");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  (void)joint;

  EXPECT_EQ(body->getNumNodes<Marker>(), 0u);

  auto shape = std::make_shared<EllipsoidShape>(Eigen::Vector3d::Ones());
  auto* shapeNode
      = body->createShapeNodeWith<VisualAspect>(shape, "shape_node");
  Marker::BasicProperties markerProps;
  markerProps.mName = "marker1";
  auto* marker = body->createMarker(markerProps);
  auto* endEffector = body->createEndEffector("ee1");
  (void)marker;
  (void)endEffector;

  EXPECT_EQ(body->getNumNodes<ShapeNode>(), 1u);
  EXPECT_EQ(body->getNode<ShapeNode>(0), shapeNode);
  EXPECT_EQ(body->getNode<Marker>(0), marker);

  const BodyNode* constBody = body;
  EXPECT_EQ(constBody->getNode<ShapeNode>(0), shapeNode);

  DummyNodeManager manager;
  DummyNode dummy(body);
  manager.addTree();
  manager.addNode(0, &dummy, "dummy");

  EXPECT_EQ(manager.getNumNodes<DummyNode>(0), 1u);
  EXPECT_EQ(manager.getNode<DummyNode>(0, 0), &dummy);
  EXPECT_EQ(manager.getNode<DummyNode>("dummy"), &dummy);

  const DummyNodeManager& constManager = manager;
  EXPECT_EQ(constManager.getNode<DummyNode>(0, 0), &dummy);
  EXPECT_EQ(constManager.getNode<DummyNode>("dummy"), &dummy);
}
