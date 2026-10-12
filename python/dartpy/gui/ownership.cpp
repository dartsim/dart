// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include "gui/ownership.hpp"

#include <dart/simulation/World.hpp>

#include <unordered_map>

namespace dartnb::gui {
namespace {
std::unordered_map<dart::gui::Viewer*, std::weak_ptr<Ownership>> owners;

std::shared_ptr<Ownership> existing(PyObject* self)
{
  if (!nb::inst_ready(self))
    return nullptr;
  auto* viewer = nb::cast<dart::gui::Viewer*>(nb::handle(self));
  const auto it = owners.find(viewer);
  return it == owners.end() ? nullptr : it->second.lock();
}

GcEdges nativeEdges(const Ownership& state)
{
  GcEdges edges;
  for (const auto& edge : state.active) {
    if (auto value = edge.native.lock()) {
      // The alias retains the original deleter, including a PythonPin.
      if (auto* pin = std::get_deleter<PythonPin>(value))
        edges.edges.push_back({std::move(value), pin, edge.detach});
    }
  }
  return edges;
}

int traverse(PyObject* self, visitproc visit, void* arg)
{
  Py_VISIT(Py_TYPE(self));
  if (auto state = existing(self)) {
    for (const auto& edge : state->active)
      Py_VISIT(edge.wrapper.ptr());
    return nativeEdges(*state).traverse(visit, arg);
  }
  return 0;
}

int clear(PyObject* self) noexcept
{
  try {
    if (auto state = existing(self))
      state->clear(self);
    return 0;
  } catch (nb::python_error& error) {
    error.restore();
  } catch (const std::exception& error) {
    PyErr_SetString(PyExc_RuntimeError, error.what());
  } catch (...) {
    PyErr_SetString(PyExc_RuntimeError, "GUI ownership clearing failed");
  }
  return -1;
}

GcEdges nodeEdges(PyObject* self)
{
  GcEdges edges;
  if (nb::inst_ready(self)) {
    auto* node = nb::cast<dart::gui::WorldNode*>(nb::handle(self));
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

// Python-owned picked frames and shapes can reference their PickInfo.
GcEdges pickEdges(PyObject* self)
{
  GcEdges edges;
  if (nb::inst_ready(self)) {
    auto* pick = nb::cast<dart::gui::PickInfo*>(nb::handle(self));
    edges.add(pick->getFrameOwner(), [pick] { pick->setFrameOwner(nullptr); });
    edges.add(pick->shape, [pick] { pick->shape.reset(); });
  }
  return edges;
}

int pickTraverse(PyObject* self, visitproc visit, void* arg)
{
  Py_VISIT(Py_TYPE(self));
  return pickEdges(self).traverse(visit, arg);
}

int pickClear(PyObject* self)
{
  pickEdges(self).clear();
  return 0;
}
} // namespace

Ownership::~Ownership()
{
  owners.erase(owner);
  if (nb::detail::cleanup_guard guard{})
    active.clear();
  else
    // The interpreter is finalizing; intentionally leak the Python handles.
    for (auto& edge : active)
      edge.wrapper.release();
}

void Ownership::clear(PyObject* self)
{
  // Detachment can invoke Python callbacks that modify the registry.
  auto edges = std::move(active);
  active.clear();
  for (const auto& edge : edges) {
    try {
      edge.detach();
    } catch (nb::python_error& error) {
      error.restore();
      PyErr_WriteUnraisable(self);
    } catch (const std::exception& error) {
      PyErr_SetString(PyExc_RuntimeError, error.what());
      PyErr_WriteUnraisable(self);
    } catch (...) {
      PyErr_SetString(PyExc_RuntimeError, "GUI attachment teardown failed");
      PyErr_WriteUnraisable(self);
    }
  }
}

std::shared_ptr<Ownership> ownership(dart::gui::Viewer* owner)
{
  auto& weak = owners[owner];
  auto result = weak.lock();
  if (!result) {
    result = std::make_shared<Ownership>(owner);
    weak = result;
    auto wrapper = nb::cast(owner, nb::rv_policy::reference);
    auto retained = std::make_unique<std::shared_ptr<Ownership>>(result);
    nb::keep_alive_cb(wrapper, retained.get(), [](void* payload) noexcept {
      delete static_cast<std::shared_ptr<Ownership>*>(payload);
    });
    retained.release();
  }
  return result;
}

void retire(dart::gui::Viewer* owner, void* child)
{
  auto state = ownership(owner);
  for (auto it = state->active.begin(); it != state->active.end(); ++it) {
    if (it->pointer == child) {
      state->active.erase(it);
      return;
    }
  }
}

std::vector<std::shared_ptr<dart::gui::WorldNode>> nodes(
    dart::gui::Viewer* owner)
{
  std::vector<std::shared_ptr<dart::gui::WorldNode>> result;
  for (const auto& edge : ownership(owner)->active)
    if (auto node = edge.node.lock())
      result.push_back(std::move(node));
  return result;
}

const PyType_Slot* slots()
{
  static const PyType_Slot result[]
      = {{Py_tp_traverse, reinterpret_cast<void*>(traverse)},
         {Py_tp_clear, reinterpret_cast<void*>(clear)},
         {0, nullptr}};
  return result;
}

const PyType_Slot* nodeSlots()
{
  static const PyType_Slot result[]
      = {{Py_tp_traverse, reinterpret_cast<void*>(nodeTraverse)},
         {Py_tp_clear, reinterpret_cast<void*>(nodeClear)},
         {0, nullptr}};
  return result;
}

const PyType_Slot* pickSlots()
{
  static const PyType_Slot result[]
      = {{Py_tp_traverse, reinterpret_cast<void*>(pickTraverse)},
         {Py_tp_clear, reinterpret_cast<void*>(pickClear)},
         {0, nullptr}};
  return result;
}
} // namespace dartnb::gui
