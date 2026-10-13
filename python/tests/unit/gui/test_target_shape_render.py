"""DART 7.0 target shape, headless render.

The PR that removes a marker must make its test exercise the full contract in
docs/design/dart7_python_api.md.

Rendering needs a graphics device, so test-gui runs this module and test-py
leaves it out. The device-free target-shape tests live in
python/tests/acceptance/.
"""

import dartpy as dart
import numpy as np
import pytest


@pytest.mark.xfail(
    strict=True, reason="M6b/M5: dart.render scene RGB capture and io.load_world are pending"
)
def test_renderer_rgb(tmp_path):
    camera = dart.render.Camera.look_at(eye=(2, 0, 1), target=(0, 0, 0.5))
    background = dart.render.Renderer(dart.simulation.World()).render(camera)
    path = tmp_path / "box.xml"
    path.write_text(
        '<mujoco><worldbody><geom type="box" size="0.3 0.3 0.3" '
        'pos="0 0 0.5" rgba="1 0 0 1"/></worldbody></mujoco>'
    )
    world = dart.io.load_world(path)
    rgb = dart.render.Renderer(world).render(camera)
    assert rgb.shape == (480, 640, 3)
    assert rgb.dtype == np.uint8
    # The camera targets the box center, so its projection covers this region.
    changed = np.any(rgb[200:280, 280:360] != background[200:280, 280:360], axis=2)
    assert np.count_nonzero(changed) > 128
