# Python ImGui widget example

This example shows how to subclass `dartpy.gui.ImGuiWidget` in Python and
drive the UI from Python code while DART owns the Dear ImGui rendering loop.

## Requirements

- `dartpy` built with GUI support (`DART_BUILD_GUI=ON`) and the Python bindings
- The bundled `dartpy.gui.imgui` module (no extra Python ImGui package needed)

## Run

```bash
pixi run py-ex imgui_python
```

Or, without pixi:

```bash
PYTHONPATH=build/<env>/cpp/<build_type>/python \
python python/examples/imgui_python/main.py
```

You should see a small ImGui overlay with play/pause and gravity controls rendered on top of the viewer.
