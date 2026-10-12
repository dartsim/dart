"""DART 7.0 target shape, headless render.

Rendering needs a graphics device, so test-gui runs this module and test-py
leaves it out. The device-free target-shape tests live in
python/tests/acceptance/.
"""

import dartpy as dart
import numpy as np
import pytest


@pytest.mark.xfail(
    strict=True, reason="M6b: dart.render.Renderer and Camera RGB capture are pending"
)
def test_renderer_rgb():
    rgb = dart.render.Renderer(dart.simulation.World()).render(
        dart.render.Camera.look_at(eye=(2, 0, 1), target=(0, 0, 0.5))
    )
    assert rgb.shape == (480, 640, 3)
    assert rgb.dtype == np.uint8
