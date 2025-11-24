# Python ImGui widget example

This example shows how to subclass `dartpy.gui.ImGuiWidget` in Python and
drive the UI from Python code while DART owns the Dear ImGui rendering loop.

## Requirements

- `dartpy` built with GUI support (`DART_BUILD_GUI=ON`) and the Python bindings
- A Python Dear ImGui package built against a compatible ImGui version, e.g.:

  ```bash
  pip install "imgui>=2.0.0"
  ```

## Run

```bash
PYTHONPATH=build/<env>/cpp/<build_type>/python \
python python/examples/imgui_python/main.py
```

You should see a small ImGui overlay with play/pause and gravity controls rendered on top of the viewer.
