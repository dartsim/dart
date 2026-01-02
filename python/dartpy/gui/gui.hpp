#pragma once

#include <nanobind/nanobind.h>

namespace dart::python_nb {

void defGuiEventHandler(nanobind::module_& m);
void defWorldNode(nanobind::module_& m);
void defRealTimeWorldNode(nanobind::module_& m);
void defInteractiveFrame(nanobind::module_& m);
void defViewerAttachment(nanobind::module_& m);
void defGridVisual(nanobind::module_& m);
void defPolyhedronVisual(nanobind::module_& m);
void defSupportPolygonVisual(nanobind::module_& m);
void defDragAndDrop(nanobind::module_& m);
void defShadowTechnique(nanobind::module_& m);
void defViewer(nanobind::module_& m);
void defImGuiApi(nanobind::module_& m);
void defImGuiWidget(nanobind::module_& m);
void defImGuiHandler(nanobind::module_& m);
void defImGuiViewer(nanobind::module_& m);

} // namespace dart::python_nb
