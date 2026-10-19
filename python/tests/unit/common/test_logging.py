import dartpy as dart
import pytest


def test_basics():
    dart.common.trace("trace log")
    dart.common.debug("debug log")
    dart.common.info("info log")
    dart.common.warn("warn log")
    dart.common.error("error log")
    dart.common.fatal("fatal log")


def test_arguments():
    val = 10
    dart.common.info("Log with param '{}' and '{}'".format(1, val))


@pytest.mark.parametrize("level", ["info", "warn", "error", "fatal"])
def test_user_text_is_literal(level, capfd):
    text = "literal {} {broken} {{braces}}"
    getattr(dart.common, level)(text)
    out, err = capfd.readouterr()
    assert text in (out if level == "info" else err)
    assert "log format error" not in out + err


if __name__ == "__main__":
    pytest.main()
