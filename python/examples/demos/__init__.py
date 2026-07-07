"""Consolidated dartpy demo runner.

Mirrors the C++ ``examples/demos`` catalog (see
``docs/dev_tasks/dart6_consolidated_demos/PLAN.md``) for the taxonomy this
package follows, and ``EVIDENCE-dartpy-bindings.md`` in the same folder for
why this runner is a plain keyboard-driven ``Viewer`` instead of an
``ImGuiViewer`` workspace: dartpy has no usable ImGui bindings, so there is no
Python-side equivalent to the C++ host's panel chrome.
"""
