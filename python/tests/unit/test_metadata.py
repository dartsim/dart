"""Metadata exposed by the built Python module."""

import dartpy as dart


def test_module_version_is_nonempty():
    assert isinstance(dart.__version__, str)
    assert dart.__version__
