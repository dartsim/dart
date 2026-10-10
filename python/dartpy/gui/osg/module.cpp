#include "detail/dart_nb.hpp"
#include "gui/osg/ownership.hpp"

#include <osgGA/GUIActionAdapter>

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

namespace dart {
namespace python {

void bindWorldNode(nb::module_& sm);
void bindRealTimeWorldNode(nb::module_& sm);

void GUIEventHandler(nb::module_& sm);

void InteractiveFrame(nb::module_& sm);

void ImGuiHandler(nb::module_& sm);
void ImGuiWidget(nb::module_& sm);

void Viewer(nb::module_& sm);
void ImGuiViewer(nb::module_& sm);
void ViewerAttachment(nb::module_& sm);
void GridVisual(nb::module_& sm);
void DebugOverlay(nb::module_& sm);

void DragAndDrop(nb::module_& sm);

void ShadowTechnique(nb::module_& sm);

void dart_gui_osg(nb::module_& m)
{
  auto sm = m.def_submodule("osg");

  bindWorldNode(sm);
  bindRealTimeWorldNode(sm);

  GUIEventHandler(sm);

  InteractiveFrame(sm);

  ImGuiHandler(sm);
  ImGuiWidget(sm);

  Viewer(sm);
  ImGuiViewer(sm);
  ViewerAttachment(sm);
  GridVisual(sm);
  DebugOverlay(sm);

  DragAndDrop(sm);

  ShadowTechnique(sm);
  sm.attr("__dict__")["__builtins__"]
      = nb::module_::import_("builtins").attr("__dict__");
  nb::exec(
      R"(
import enum as _enum
import operator as _operator

def _mask_missing(cls, value):
    try:
        value = _operator.index(value)
    except TypeError:
        return None
    result = int.__new__(cls, value)
    result._name_ = '???'
    result._value_ = value
    return result

def _enum_str(self):
    return type(self).__name__ + '.' + self.name

def _enum_repr(self):
    return '<' + str(self) + ': ' + str(int(self)) + '>'

_seen_enums = set()
for _owner in [GUIEventAdapter, Viewer, DragAndDrop, InteractiveTool, GridVisual, globals()]:
    for _value in (list(_owner.values()) if isinstance(_owner, dict)
                   else list(vars(_owner).values())):
        if (isinstance(_value, type) and issubclass(_value, _enum.Enum)
                and _value not in _seen_enums):
            _seen_enums.add(_value)
            _canonical_names = {}
            for _name, _member in _value.__members__.items():
                _member._name_ = _canonical_names.setdefault(int(_member), _name)
            _value.__str__ = _enum_str
            _value.__repr__ = _enum_repr
for _enum_class in list(vars(GUIEventAdapter).values()):
    if isinstance(_enum_class, type) and issubclass(_enum_class, _enum.Enum):
        for _name, _member in _enum_class.__members__.items():
            setattr(GUIEventAdapter, _name, _member)
for _mask in [GUIEventAdapter.MouseButtonMask, GUIEventAdapter.ModKeyMask]:
    _mask._missing_ = classmethod(_mask_missing)
)",
      sm.attr("__dict__"));
}

} // namespace python
} // namespace dart
