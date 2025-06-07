import dartpy as dart
import pytest


def assert_logged(capsys, *messages: str) -> None:
    """Read captured logs and ensure *messages* appear in the output."""
    captured = capsys.readouterr()
    output = captured.out + captured.err
    for msg in messages:
        assert msg in output, f"{msg!r} not found in captured output"


def test_basics(capsys):
    dart.common.trace("trace log")
    dart.common.debug("debug log")
    dart.common.info("info log")
    dart.common.warn("warn log")
    dart.common.error("error log")
    dart.common.fatal("fatal log")

    assert_logged(
        capsys,
        "trace log",
        "debug log",
        "info log",
        "warn log",
        "error log",
        "fatal log",
    )


def test_arguments(capsys):
    val = 10
    dart.common.info("Log with param '{}' and '{}'".format(1, val))

    assert_logged(capsys, "Log with param '1' and '10'")


if __name__ == "__main__":
    pytest.main()
