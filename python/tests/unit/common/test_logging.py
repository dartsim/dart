import dartpy as dart
import pytest


def test_basics(capsys):
    dart.common.trace("trace log")
    captured = capsys.readouterr()
    assert "trace log" in captured.out or "trace log" in captured.err

    dart.common.debug("debug log")
    captured = capsys.readouterr()
    assert "debug log" in captured.out or "debug log" in captured.err

    dart.common.info("info log")
    captured = capsys.readouterr()
    assert "info log" in captured.out or "info log" in captured.err

    dart.common.warn("warn log")
    captured = capsys.readouterr()
    assert "warn log" in captured.out or "warn log" in captured.err

    dart.common.error("error log")
    captured = capsys.readouterr()
    assert "error log" in captured.out or "error log" in captured.err

    dart.common.fatal("fatal log")
    captured = capsys.readouterr()
    assert "fatal log" in captured.out or "fatal log" in captured.err


def test_arguments(capsys):
    val = 10
    dart.common.info("Log with param '{}' and '{}'".format(1, val))
    captured = capsys.readouterr()
    assert "Log with param '1' and '10'" in captured.out or "Log with param '1' and '10'" in captured.err


if __name__ == "__main__":
    pytest.main()
