"""
Python package entry point for dartpy.

This module re-exports the pybind11 extension located in
``dartpy/dartpy.cpython-*.so`` so users can simply ``import dartpy``.
"""

from .dartpy import *  # noqa: F401,F403
