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
    strict=True, reason="M6b/M5: dart.render RGB, metric depth, segmentation and io.load_world are pending"
)
def test_renderer_outputs(tmp_path):
    camera = dart.render.Camera.look_at(eye=(2, 0, 0.5), target=(0, 0, 0.5))
    empty = dart.render.Renderer(dart.simulation.World())
    background = empty.render(camera)
    path = tmp_path / "box.xml"
    path.write_text(
        '<mujoco><worldbody><body name="box" pos="0 0 0.5">'
        '<geom type="box" size="0.3 0.3 0.3" rgba="1 0 0 1"/>'
        '</body></worldbody></mujoco>'
    )
    world = dart.io.load_world(path)
    renderer = dart.render.Renderer(world)
    rgb = renderer.render(camera)
    assert rgb.shape == (480, 640, 3)
    assert rgb.dtype == np.uint8
    # The camera targets the box center, so its projection covers this region.
    changed = np.any(rgb[200:280, 280:360] != background[200:280, 280:360], axis=2)
    assert np.count_nonzero(changed) > 128
    # M6b settles the export selector spelling; all three outputs and lookup
    # must work before this acceptance marker can be removed.
    depth = renderer.render(camera, output="depth")
    segmentation = renderer.render(camera, output="segmentation")
    background_ids = empty.render(camera, output="segmentation")
    assert depth.shape == segmentation.shape == background_ids.shape == (480, 640)
    assert np.issubdtype(depth.dtype, np.floating)
    assert np.issubdtype(segmentation.dtype, np.integer)
    # The center ray meets the x=0.3 face, 1.7 meters from the camera.
    assert np.isfinite(depth[240, 320])
    assert depth[240, 320] == pytest.approx(1.7, abs=0.03)
    identifier = int(segmentation[240, 320])
    assert identifier != int(background_ids[240, 320])
    owner = renderer.lookup_segmentation(identifier)
    scene_objects = []
    for model in world.multibodies:
        scene_objects.append(model)
        for link in model.links:
            scene_objects.append(link)
            scene_objects.extend(link.visuals)
    assert any(owner is item for item in scene_objects)
