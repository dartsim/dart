"""
Python package entry point for dartpy.

This module re-exports the pybind11 extension located in
``dartpy/dartpy.cpython-*.so`` so users can simply ``import dartpy``.
It also exposes the installed package version via ``dartpy.__version__`` so
wheel tests and downstream tooling can confirm compatibility.
"""

from importlib import metadata

try:
    __version__ = metadata.version("dartpy")
except metadata.PackageNotFoundError:  # pragma: no cover
    __version__ = ""

from .dartpy import *  # noqa: F401,F403
