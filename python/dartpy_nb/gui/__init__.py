# Temporary shim to provide dartpy_nb.gui via the legacy dartpy GUI bindings.
# Once GUI bindings are ported to nanobind, this package should be replaced.
from . import osg  # noqa: F401

__all__ = ["osg"]
