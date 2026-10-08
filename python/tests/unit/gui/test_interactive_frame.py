import subprocess
import sys
import textwrap


def test_interactive_frame_shape_frames_keep_owner_alive():
    script = textwrap.dedent(
        """
        import gc
        import dartpy as dart

        frame = dart.gui.osg.InteractiveFrame(dart.dynamics.Frame.World())
        shapes = frame.getShapeFrames()
        assert isinstance(shapes, list)
        assert shapes
        del frame
        gc.collect()
        assert all(shape.getName() for shape in shapes)
        """
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
