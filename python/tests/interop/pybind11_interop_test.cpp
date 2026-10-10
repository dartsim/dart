
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

// A pybind11 extension that registers no DART classes; every DART value
// crosses through dartpy's C API (see dartpy/pybind11_interop.hpp).
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>

#include <dartpy/pybind11_interop.hpp>
#include <pybind11/eigen.h>

#include <string>

namespace py = pybind11;
using dart::dynamics::BodyNode;
using dart::dynamics::BoxShape;
using dart::dynamics::Entity;
using dart::dynamics::Frame;
using dart::dynamics::FreeJoint;
using dart::dynamics::JacobianNode;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::ShapeNode;
using dart::dynamics::SimpleFrame;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::VisualAspect;
using dart::simulation::World;

namespace {

struct Attachment
{
  BodyNode* body = nullptr;
};

// Owns a frame that Python can borrow with either reference policy.
struct FrameOwner
{
  std::shared_ptr<SimpleFrame> frame
      = SimpleFrame::createShared(Frame::World(), "owned");
};

// Owns a frame until release() hands its last owner to Python.
struct FrameHolder
{
  std::shared_ptr<SimpleFrame> frame
      = SimpleFrame::createShared(Frame::World(), "held");
};

std::weak_ptr<SimpleFrame> releasedFrame;

} // namespace

PYBIND11_MODULE(dartpy_pybind11_interop_test, m)
{
  // Validate the C API at import rather than on the first conversion.
  dartpy::pybind11_interop::api();

  m.def("skeleton_name", [](const SkeletonPtr& skeleton) {
    return skeleton->getName();
  });
  m.def("same_skeleton", [](SkeletonPtr skeleton) { return skeleton; });
  m.def("const_skeleton", [](const std::shared_ptr<const Skeleton>& skeleton) {
    return skeleton;
  });
  m.def("make_skeleton", [](const std::string& name) {
    auto skeleton = Skeleton::create(name);
    skeleton->createJointAndBodyNodePair<FreeJoint>();
    return skeleton;
  });
  m.def(
      "body",
      [](const SkeletonPtr& skeleton, std::size_t index) {
        return skeleton->getBodyNode(index);
      },
      py::return_value_policy::reference);
  m.def(
      "joint",
      [](const SkeletonPtr& skeleton, std::size_t index) {
        return skeleton->getJoint(index);
      },
      py::return_value_policy::reference);
  m.def(
      "dof",
      [](const SkeletonPtr& skeleton, std::size_t index) {
        return skeleton->getDof(index);
      },
      py::return_value_policy::reference);
  m.def(
      "owned_body",
      [](const SkeletonPtr& skeleton) { return skeleton->getBodyNode(0); },
      py::return_value_policy::take_ownership);
  m.def("automatic_body", [](const SkeletonPtr& skeleton) {
    return skeleton->getBodyNode(0);
  });
  m.def(
      "copied_body",
      [](const SkeletonPtr& skeleton) -> const BodyNode& {
        return *skeleton->getBodyNode(0);
      },
      py::return_value_policy::copy);
  m.def(
      "add_shape_node",
      [](BodyNode* body) {
        return body->createShapeNodeWith<VisualAspect>(
            std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
      },
      py::return_value_policy::reference);
  m.def("jacobian_node_name", [](const JacobianNode* node) {
    return node->getName();
  });
  m.def(
      "shape_node_name", [](const ShapeNode& node) { return node.getName(); });
  m.def("meta_skeleton_dofs", [](const std::shared_ptr<MetaSkeleton>& meta) {
    return meta->getNumDofs();
  });
  m.def("frame_name", [](const Frame* frame) {
    return frame ? frame->getName() : std::string();
  });
  m.def("entity_name", [](Entity& entity) { return entity.getName(); });
  m.def(
      "call_with_body",
      [](const py::function& callback,
         const SkeletonPtr& skeleton,
         std::size_t index) { callback(skeleton->getBodyNode(index)); });
  m.def("same_world", [](std::shared_ptr<World> world) { return world; });
  m.def(
      "add_skeleton",
      [](const std::shared_ptr<World>& world, const SkeletonPtr& skeleton) {
        world->addSkeleton(skeleton);
      });
  m.def(
      "translate",
      [](const Eigen::Isometry3d& transform, const Eigen::Vector3d& offset) {
        Eigen::Isometry3d result = transform;
        result.translation() += offset;
        return result;
      });
  py::class_<Attachment>(m, "Attachment")
      .def(py::init<>())
      .def_readwrite("body", &Attachment::body);
  py::class_<FrameHolder>(m, "FrameHolder")
      .def(py::init<>())
      .def(
          "borrow",
          [](FrameHolder& holder) { return holder.frame.get(); },
          py::return_value_policy::reference)
      .def("release", [](FrameHolder& holder) {
        releasedFrame = holder.frame;
        return std::move(holder.frame);
      });
  m.def("released_frame_alive", [] { return !releasedFrame.expired(); });
  py::class_<FrameOwner>(m, "FrameOwner")
      .def(py::init<>())
      .def(
          "borrow",
          [](FrameOwner& owner) { return owner.frame.get(); },
          py::return_value_policy::reference)
      .def(
          "internal",
          [](FrameOwner& owner) { return owner.frame.get(); },
          py::return_value_policy::reference_internal);
}
