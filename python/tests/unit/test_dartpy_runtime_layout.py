from __future__ import annotations

import warnings


def test_legacy_module_dunder_metadata_is_introspection_safe():
    import dartpy

    with warnings.catch_warnings():
        warnings.simplefilter("error", DeprecationWarning)

        getattr(dartpy.common, "__file__", None)
        getattr(dartpy.common, "__spec__", None)
        hasattr(dartpy.common, "__file__")
        hasattr(dartpy.common, "__spec__")


def test_legacy_module_public_access_is_quiet_by_default(monkeypatch):
    import dartpy
    import dartpy._layout as layout

    monkeypatch.setattr(layout, "WARN_ON_LEGACY_MODULES", False)
    layout._WARNED.discard("dartpy.common->dartpy")

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", DeprecationWarning)
        getattr(dartpy.common, "Uri")

    assert not any("dartpy.common" in str(warning.message) for warning in caught)


def test_legacy_module_public_access_can_warn_when_enabled(monkeypatch):
    import dartpy
    import dartpy._layout as layout

    monkeypatch.setattr(layout, "WARN_ON_LEGACY_MODULES", True)
    layout._WARNED.discard("dartpy.common->dartpy")

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", DeprecationWarning)
        getattr(dartpy.common, "Uri")

    assert any("dartpy.common" in str(warning.message) for warning in caught)
