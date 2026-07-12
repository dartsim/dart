"""Headless smoke support for the Python demos.

Provides a faithful in-process fake of the C++ ``dart.gui.PanelBuilder`` /
``PanelContext`` API plus :func:`exercise_panels`, so the no-crash smoke can run
each scene's ``ScenePanel.build`` callbacks without the real Filament viewer.
Panels are a real live-viewer crash source that building + stepping a scene does
not touch.

The fake mirrors the binding's method signatures and return shapes exactly (see
``dart.gui.PanelBuilder``): input widgets return ``(changed=False, value)`` so a
single exercise pass does not trigger scene state mutations, while containers
(`collapsing_header`, `begin_table`, `table_next_column`) return ``True`` so the
panel body is traversed for maximum coverage. Pure Python — no dartpy import — so
it is safe to import anywhere.
"""

from __future__ import annotations

from typing import Any, Sequence


class FakePanelBuilder:
    """In-process stand-in for ``dart.gui.PanelBuilder`` (renderer-neutral)."""

    # --- void widgets -----------------------------------------------------
    def text(self, text: str) -> None:
        pass

    def item_tooltip(self, text: str) -> None:
        pass

    def separator(self) -> None:
        pass

    def same_line(self) -> None:
        pass

    def indent(self, width: float = 12.0) -> None:
        pass

    def unindent(self, width: float = 12.0) -> None:
        pass

    def plot_lines(self, label: str, values: Sequence[float]) -> None:
        pass

    def color_swatch(self, label: str, rgba: Any) -> None:
        pass

    def block_grid(
        self,
        label: str,
        colors: Sequence[Any],
        tooltips: Sequence[str] = (),
        preferred_columns: int = 32,
    ) -> None:
        pass

    # --- input widgets: report "no change", echo current value ------------
    def slider(
        self, label: str, value: float, minimum: float, maximum: float
    ) -> tuple[bool, float]:
        return (False, value)

    def select(
        self, label: str, selected_index: int, choices: Sequence[str]
    ) -> tuple[bool, int]:
        return (False, selected_index)

    def checkbox(self, label: str, value: bool) -> tuple[bool, bool]:
        return (False, value)

    def text_input(self, label: str, value: str) -> tuple[bool, str]:
        return (False, value)

    def color_edit(self, label: str, rgba: Any) -> tuple[bool, Any]:
        return (False, rgba)

    def timeline(
        self,
        label: str,
        value: float,
        minimum: float,
        maximum: float,
        value_track: Sequence[float] = (),
        marker_track: Sequence[float] = (),
        cursor_track: Sequence[float] = (),
        value_track_label: str = "Values",
    ) -> tuple[bool, float]:
        return (False, value)

    def button(self, label: str) -> bool:
        return False

    def selectable(self, label: str, selected: bool = False) -> bool:
        return False

    # --- containers: report "open/visible" so the body is traversed -------
    def collapsing_header(self, label: str, default_open: bool = False) -> bool:
        return True

    def begin_table(self, label: str, columns: Sequence[str]) -> bool:
        return True

    def end_table(self) -> None:
        pass

    def table_next_row(self) -> None:
        pass

    def table_next_column(self) -> bool:
        return True


class FakePanelContext:
    """In-process stand-in for ``dart.gui.PanelContext`` (read-only snapshot)."""

    def __init__(self) -> None:
        self.paused = False
        self.contact_count = 0
        self.display_size = (1280.0, 720.0)
        self.font_size = 13.0
        self.frame_output_enabled = False
        self.rendered_frames = 0
        self.skipped_frames = 0
        self.simulation_time = 0.0
        self.selected_label = ""
        self.selected_point = (0.0, 0.0, 0.0)
        self.selected_normal = (0.0, 0.0, 1.0)

    def set_paused(self, paused: bool) -> None:
        self.paused = bool(paused)

    def request_single_step(self) -> None:
        pass

    def request_scene_replay(self, scene_id: str) -> None:
        pass

    def request_scene_switch(self, scene_id: str) -> None:
        pass


def exercise_panels(setup: Any) -> None:
    """Call every scene-owned ``ScenePanel.build`` once with the fakes.

    Raises whatever the panel code raises, so callers can attribute crashes to
    the scene. No-op for scenes without panels.
    """

    panels = getattr(setup, "panels", None) or []
    if not panels:
        return
    builder = FakePanelBuilder()
    context = FakePanelContext()
    for panel in panels:
        panel.build(builder, context)
