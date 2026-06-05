import dartpy as dart
import dartpy.common as common
import pytest


def test_common_profile_helpers_are_available_from_flat_namespace():
    assert callable(common.isProfileEnabled)
    assert callable(common.isTextProfileEnabled)
    assert callable(common.markProfileFrame)
    assert callable(common.resetProfile)
    assert callable(common.getProfileSummaryText)

    assert not hasattr(dart, "isProfileEnabled")
    assert not hasattr(dart, "isTextProfileEnabled")
    assert not hasattr(dart, "markProfileFrame")
    assert not hasattr(dart, "resetProfile")
    assert not hasattr(dart, "getProfileSummaryText")

    assert isinstance(dart.is_profile_enabled(), bool)
    assert isinstance(dart.is_text_profile_enabled(), bool)

    dart.reset_profile()
    dart.mark_profile_frame()

    summary = dart.get_profile_summary_text()
    assert isinstance(summary, str)

    if dart.is_text_profile_enabled():
        assert "DART profiler (text" in summary
    else:
        assert summary == ""


if __name__ == "__main__":
    pytest.main()
