#pragma once

#include <dart/gui/fwd.hpp>

#include <nanobind/nanobind.h>

namespace dart::python_nb {

void defGuiPanels(nanobind::module_& m);

dart::gui::Panel makeGuiPanelFromPython(nanobind::handle panelLike);

} // namespace dart::python_nb
