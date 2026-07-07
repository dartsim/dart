"""Consolidated dartpy demo runner.

Mirrors the C++ ``examples/demos`` catalog taxonomy where Python bindings can
support the same scene. The runner is a plain keyboard-driven ``Viewer``
instead of an ``ImGuiViewer`` workspace because dartpy has no usable ImGui
bindings, so there is no Python-side equivalent to the C++ host's panel
chrome.
"""
