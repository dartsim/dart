from __future__ import annotations

from typing import Iterable

from imgui_bundle import imgui

from demo_hub.gui.primitives import Segment2D


def draw_topdown(name: str, segments: Iterable[Segment2D], size=(0, 0)) -> None:
    """Render a simple top-down 2D view using ImGui draw lists."""
    imgui.begin(name)
    canvas_pos = imgui.get_window_pos()
    canvas_size = imgui.get_window_size()
    width, height = canvas_size.x, canvas_size.y

    segs = list(segments)
    if not segs:
        imgui.end()
        return

    # Compute bounds
    xs = [p for seg in segs for p in (seg.start[0], seg.end[0])]
    zs = [p for seg in segs for p in (seg.start[1], seg.end[1])]
    min_x, max_x = min(xs), max(xs)
    min_z, max_z = min(zs), max(zs)
    pad = 0.2
    span_x = max(max_x - min_x, 1e-3) + pad
    span_z = max(max_z - min_z, 1e-3) + pad

    scale_x = (width - 20) / span_x
    scale_z = (height - 20) / span_z
    scale = min(scale_x, scale_z)

    draw_list = imgui.get_window_draw_list()

    # Center content
    offset_x = canvas_pos.x + width * 0.5
    offset_z = canvas_pos.y + height * 0.5
    center_x = (min_x + max_x) * 0.5
    center_z = (min_z + max_z) * 0.5

    def project(pt):
        x, z = pt
        return offset_x + (x - center_x) * scale, offset_z - (z - center_z) * scale

    for seg in segs:
        c = seg.color
        col = imgui.get_color_u32(imgui.ImVec4(c[0], c[1], c[2], 1.0))
        p0 = project(seg.start)
        p1 = project(seg.end)
        draw_list.add_line(p0[0], p0[1], p1[0], p1[1], col, thickness=2.0)

    imgui.end()
