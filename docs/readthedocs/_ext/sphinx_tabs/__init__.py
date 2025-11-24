"""Patched sphinx-tabs shim to avoid deprecated pkg_resources usage."""

from sphinx_tabs.tabs import setup  # noqa: F401
