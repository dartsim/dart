#include "detail/dart_nb.hpp"

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

#include "gui/osg/ownership.hpp"

#include <dart/gui/osg/detail/CameraModeCallback.hpp>

#include <osg/NodeVisitor>

#include <unordered_map>

namespace dartnb::gui {
namespace {
std::unordered_map<::osg::Referenced*, std::weak_ptr<Ownership>> owners;
std::shared_ptr<Ownership> existing(PyObject* self)
{
  if (!nb::inst_ready(self))
    return nullptr;
  auto* view = nb::cast<osgViewer::View*>(nb::handle(self));
  auto it = owners.find(view);
  return it == owners.end() ? nullptr : it->second.lock();
}
} // namespace
Ownership::~Ownership()
{
  owners.erase(owner);
  std::vector<std::shared_ptr<Ownership>> remaining;
  for (const auto& entry : owners)
    if (auto state = entry.second.lock())
      remaining.push_back(std::move(state));
  for (const auto& state : remaining)
    state->prune();
}
void Ownership::prune()
{
  for (auto it = retired.begin(); it != retired.end();) {
    if (it->pointer->referenceCount() == 1)
      it = retired.erase(it);
    else
      ++it;
  }
}
void Ownership::clear()
{
  for (const auto& edge : active)
    edge.detach();
  if (auto* viewer = dynamic_cast<dart::gui::osg::Viewer*>(owner)) {
    const auto& root = viewer->getRootGroup();
    for (auto* callback = root->getUpdateCallback(); callback;
         callback = callback->getNestedCallback()) {
      if (auto* camera
          = dynamic_cast<dart::gui::osg::detail::CameraModeCallback*>(
              callback)) {
        ::osg::ref_ptr<::osg::Group> empty = new ::osg::Group;
        camera->setSceneData(empty);
        ::osg::NodeVisitor visitor(::osg::NodeVisitor::TRAVERSE_NONE);
        (*camera)(root.get(), &visitor);
      }
    }
  }
  active.clear();
  retired.clear();
}
std::shared_ptr<Ownership> ownership(::osg::Referenced* owner)
{
  auto& weak = owners[owner];
  auto result = weak.lock();
  if (!result) {
    result = std::make_shared<Ownership>(owner);
    weak = result;
  }
  return result;
}
void retainWrapper(
    ::osg::Referenced* owner,
    ::osg::Referenced* child,
    nb::object wrapper,
    std::function<void()> detach)
{
  if (!child)
    return;
  auto state = ownership(owner);
  for (const auto& edge : state->active)
    if (edge.pointer == child)
      return;
  for (auto it = state->retired.begin(); it != state->retired.end();) {
    if (it->pointer == child)
      it = state->retired.erase(it);
    else
      ++it;
  }
  state->active.push_back({child, std::move(wrapper), std::move(detach)});
  state->prune();
}
void retire(::osg::Referenced* owner, ::osg::Referenced* child)
{
  auto state = ownership(owner);
  for (auto it = state->active.begin(); it != state->active.end(); ++it) {
    if (it->pointer == child) {
      if (child->referenceCount() > 1)
        state->retired.push_back(std::move(*it));
      state->active.erase(it);
      break;
    }
  }
  state->prune();
}
int traverse(PyObject* self, visitproc visit, void* arg)
{
  Py_VISIT(Py_TYPE(self));
  if (auto state = existing(self)) {
    for (const auto& edge : state->active)
      Py_VISIT(edge.wrapper.ptr());
    for (const auto& edge : state->retired)
      Py_VISIT(edge.wrapper.ptr());
  }
  return 0;
}
int clear(PyObject* self)
{
  if (auto state = existing(self))
    state->clear();
  return 0;
}
const PyType_Slot* slots()
{
  static const PyType_Slot result[]
      = {{Py_tp_traverse, reinterpret_cast<void*>(traverse)},
         {Py_tp_clear, reinterpret_cast<void*>(clear)},
         {0, nullptr}};
  return result;
}
namespace {
GcEdges nodeEdges(PyObject* self)
{
  GcEdges edges;
  if (nb::inst_ready(self)) {
    auto* node = nb::cast<dart::gui::osg::WorldNode*>(nb::handle(self));
    edges.add(node->getWorld(), [node] { node->setWorld(nullptr); });
  }
  return edges;
}
int nodeTraverse(PyObject* self, visitproc visit, void* arg)
{
  Py_VISIT(Py_TYPE(self));
  return nodeEdges(self).traverse(visit, arg);
}
int nodeClear(PyObject* self)
{
  nodeEdges(self).clear();
  return 0;
}
} // namespace
const PyType_Slot* nodeSlots()
{
  static const PyType_Slot result[]
      = {{Py_tp_traverse, reinterpret_cast<void*>(nodeTraverse)},
         {Py_tp_clear, reinterpret_cast<void*>(nodeClear)},
         {0, nullptr}};
  return result;
}
} // namespace dartnb::gui
