from __future__ import annotations

import dartpy as dart


def test_viewer_threading_model_roundtrip() -> None:
    viewer = dart.gui.Viewer()
    viewer.set_threading_model(dart.gui.Viewer.ThreadingModel.SingleThreaded)
    assert (
        viewer.get_threading_model()
        == dart.gui.Viewer.ThreadingModel.SingleThreaded
    )
