from __future__ import annotations

import dartpy as dart


def test_imgui_widget_shared_ptr_roundtrip() -> None:
    class TestWidget(dart.gui.ImGuiWidget):
        def __init__(self) -> None:
            super().__init__()
            self.render_calls = 0

        def render(self) -> None:
            self.render_calls += 1

    handler = dart.gui.ImGuiHandler()
    widget = TestWidget()

    assert handler.has_widget(widget) is False
    handler.add_widget(widget, True)
    assert handler.has_widget(widget) is True
    assert widget.is_visible() is True
    handler.remove_widget(widget)
    assert handler.has_widget(widget) is False
