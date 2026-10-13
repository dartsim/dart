"""DART 7.0 target shape, headless render.

The PR that removes a marker must make its test exercise the full contract in
docs/design/dart7_python_api.md.
M6b must add moving frame-attached-camera qualification before removing its
marker; the free-camera smoke scene does not replace that required gate.

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
    # M6b settles look_at's explicit-up spelling. Looking along -X with +Z up
    # maps world +Y to image right and world +Z to image up.
    camera = dart.render.Camera.look_at(eye=(2, 0, 0.5), target=(0, 0, 0.5), up=(0, 0, 1))
    empty = dart.render.Renderer(dart.World())
    background = empty.render(camera)
    path = tmp_path / "box.xml"
    path.write_text(
        '<mujoco><worldbody><body name="box" pos="0 0 0.5">'
        '<geom type="box" size="0.3 0.3 0.3" rgba="1 0 0 1"/>'
        '</body><body name="upper_right" pos="0 0.45 0.8">'
        '<geom type="box" size="0.08 0.08 0.08" rgba="1 0 0 1"/>'
        '</body><body name="lower_left" pos="0 -0.45 0.2">'
        '<geom type="box" size="0.08 0.08 0.08" rgba="0 0 1 1"/>'
        '</body></worldbody></mujoco>'
    )
    world = dart.io.load_world(path)
    upper = world.multibodies["upper_right"]
    upper.links[0].visuals[0].material = dart.Material(base_color=(0, 1, 0, 1))
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
    assert any(owner == item for item in scene_objects)
    # Segmentation links the asymmetric geometry to its pixels, independent
    # of RGB lighting. Distinct quadrants reject either mirrored camera axis.
    def pixels_for(model):
        objects = [model]
        for link in model.links:
            objects.append(link)
            objects.extend(link.visuals)
        foreground = np.setdiff1d(np.unique(segmentation), np.unique(background_ids))
        identifiers = [int(value) for value in foreground
                       if any(renderer.lookup_segmentation(int(value)) == item for item in objects)]
        rows, columns = np.nonzero(np.isin(segmentation, identifiers))
        assert len(rows) > 32
        return rows, columns

    for name, quadrant, channel in (("upper_right", (-1, 1), 1), ("lower_left", (1, -1), 2)):
        rows, columns = pixels_for(world.multibodies[name])
        assert quadrant[0] * (np.mean(rows) - 240) > 0
        assert quadrant[1] * (np.mean(columns) - 320) > 0
        # Interior pixel colors tolerate shading but must preserve the named
        # dominant channel. Green overrides the asset's red legacy RGBA;
        # blue uses the legacy RGBA when no explicit Material is assigned.
        color = np.median(rgb[rows, columns], axis=0)
        assert color[channel] > 32
        assert color[channel] > max(np.delete(color, channel)) + 16


@pytest.mark.xfail(strict=True, reason="M6b: final passive open and managed launch viewer entry points are pending")
def test_viewer_entry_points():
    world = dart.World()
    # M6b settles lifetime and bounded-run options. The provisional shape uses
    # the current viewer's headless bounded-step test strategy, avoiding an
    # interactive run while requiring construction through both final names.
    with dart.viewer.open(world, headless=True) as viewer:
        world.step()
        count = world.step_count
        viewer.sync()
        assert world.step_count == count
    dart.viewer.launch(world, headless=True, steps=2)
    assert world.step_count == count + 2
