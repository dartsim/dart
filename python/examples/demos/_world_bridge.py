"""Compatibility re-export: ``WorldRenderBridge`` now lives in the dartpy package.

The render bridge graduated from demo-local infrastructure into a supported
dartpy helper (``dartpy.gui.WorldRenderBridge``). It mirrors an ECS
``dartpy.World`` into a parallel classic render World that the C++
Filament viewer draws. This thin module re-exports it so existing demo scenes
that ``from .._world_bridge import WorldRenderBridge`` keep working; new code
should import ``dartpy.gui.WorldRenderBridge`` directly.
"""

from __future__ import annotations

from dartpy.gui import WorldRenderBridge

__all__ = ["WorldRenderBridge"]
