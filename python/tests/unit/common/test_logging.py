import logging
import os
import re
import signal
import subprocess
import sys
import threading
import time

import dartpy as dart
import pytest

QUEUE_CAPACITY = 1024  # dartnb::Queue::kCapacity
RECORDS_PER_MESSAGE = 3  # _testing.log_from_threads: spdlog, native, console
CONSOLE_PREFIX = re.compile(r"^\[module\.cpp:\d+\] ")
NATIVE_WARNING = re.compile(r"^SphereShape::setRadius: Invalid radius '-\d+'\.")

# dartpy._testing is compiled in with BUILD_TESTING and left out of wheels.
_testing = getattr(dart, "_testing", None)
needs_testing_hooks = pytest.mark.skipif(
    _testing is None, reason="dartpy was built without its testing hooks"
)


def _records(caplog):
    return [record for record in caplog.records if record.name == "dartpy"]


def _console_messages(threads, messages):
    # dartpy._testing.log_from_threads formats each console line with
    # per-thread setw/setfill and hex manipulators.
    return sorted(
        f"worker {t:0{2 + t}d} console {i:x}"
        for t in range(threads)
        for i in range(messages)
    )


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


def test_records_reach_python_logging(caplog):
    with caplog.at_level(logging.DEBUG, logger="dartpy"):
        dart.common.info("info log")
        dart.common.warn("warn log")
        dart.common.error("error log")
        dart.common.fatal("fatal log")
        dart.common.flush_logs()
    assert [(r.levelname, r.getMessage()) for r in _records(caplog)] == [
        ("INFO", "info log"),
        ("WARNING", "warn log"),
        ("ERROR", "error log"),
        ("CRITICAL", "fatal log"),
    ]


def test_records_arrive_without_an_explicit_flush(caplog):
    with caplog.at_level(logging.WARNING, logger="dartpy"):
        dart.common.warn("auto")
        for _ in range(100):
            if _records(caplog):
                break
            time.sleep(0.01)
    assert [r.getMessage() for r in _records(caplog)] == ["auto"]


def test_logging_configuration_silences_dart(caplog):
    with caplog.at_level(logging.ERROR, logger="dartpy"):
        dart.common.warn("silenced")
        dart.common.flush_logs()
    assert _records(caplog) == []


def test_native_diagnostics_reach_python_logging(caplog, capfd):
    # A diagnostic emitted inside libdart, not by code compiled into the
    # extension: with a header-only spdlog each library has its own registry.
    shape = dart.dynamics.SphereShape(1.0)
    with caplog.at_level(logging.WARNING, logger="dartpy"):
        shape.setRadius(-1.0)
        dart.common.flush_logs()
    records = _records(caplog)
    assert [r.levelname for r in records] == ["WARNING"]
    assert records[0].getMessage().startswith(
        "SphereShape::setRadius: Invalid radius '-1'."
    )
    assert "Invalid radius" not in capfd.readouterr().err


@needs_testing_hooks
@pytest.mark.parametrize("hold_gil", [True, False])
def test_worker_threads_log_while_python_holds_the_gil(caplog, hold_gil):
    threads, messages = 4, 25
    stop = threading.Event()

    def spin():
        # Keeps taking the GIL on a Python thread while the C++ workers log.
        while not stop.is_set():
            pass

    spinner = threading.Thread(target=spin)
    spinner.start()
    try:
        with caplog.at_level(logging.WARNING, logger="dartpy"):
            # hold_gil=True joins the workers with the GIL held on this thread.
            _testing.log_from_threads(
                threads=threads, messages=messages, hold_gil=hold_gil
            )
            dart.common.flush_logs()
    finally:
        stop.set()
        spinner.join()
    records = _records(caplog)
    assert len(records) == threads * messages * RECORDS_PER_MESSAGE
    assert {r.levelname for r in records} == {"WARNING"}
    spdlog = sorted(r.getMessage() for r in records if "spdlog" in r.getMessage())
    native = [r.getMessage() for r in records if NATIVE_WARNING.match(r.getMessage())]
    console = [r.getMessage() for r in records if "console" in r.getMessage()]
    assert spdlog == sorted(
        f"worker {t} spdlog {i}" for t in range(threads) for i in range(messages)
    )
    assert len(native) == threads * messages
    assert all(CONSOLE_PREFIX.match(message) for message in console)
    assert sorted(CONSOLE_PREFIX.sub("", m) for m in console) == _console_messages(
        threads, messages
    )


def test_flush_logs_delivers_on_the_calling_thread(caplog):
    flushed = {}

    def worker():
        dart.common.warn("from worker")
        flushed["count"] = dart.common.flush_logs()

    thread = threading.Thread(target=worker)
    with caplog.at_level(logging.WARNING, logger="dartpy"):
        thread.start()
        thread.join()
    records = _records(caplog)
    assert flushed == {"count": 1}
    assert [(r.getMessage(), r.thread) for r in records] == [
        ("from worker", thread.ident)
    ]


def test_a_raising_filter_is_unraisable_and_does_not_stop_delivery(
    caplog, monkeypatch
):
    unraisable = []
    monkeypatch.setattr(sys, "unraisablehook", unraisable.append)

    def explode(record):
        if record.getMessage() == "boom":
            raise RuntimeError("filter failed")
        return True

    logger = logging.getLogger("dartpy")
    logger.addFilter(explode)
    try:
        with caplog.at_level(logging.WARNING, logger="dartpy"):
            # The pending call delivers each record as soon as the call returns.
            dart.common.warn("boom")
            dart.common.warn("after")
            dart.common.flush_logs()
    finally:
        logger.removeFilter(explode)
    assert [r.getMessage() for r in _records(caplog)] == ["after"]
    assert len(unraisable) == 1
    assert isinstance(unraisable[0].exc_value, RuntimeError)
    assert str(unraisable[0].exc_value) == "filter failed"


@needs_testing_hooks
def test_queue_overflow_drops_the_oldest_and_reports_the_count(caplog):
    messages = QUEUE_CAPACITY // RECORDS_PER_MESSAGE + 88
    dropped = RECORDS_PER_MESSAGE * messages - QUEUE_CAPACITY
    with caplog.at_level(logging.WARNING, logger="dartpy"):
        # The GIL stays held, so nothing drains until the call returns.
        _testing.log_from_threads(threads=1, messages=messages, hold_gil=True)
        assert dart.common.flush_logs() == QUEUE_CAPACITY + 1
    records = _records(caplog)
    assert len(records) == QUEUE_CAPACITY + 1  # flush_logs counts the notice
    # One thread pushes spdlog, native, console per message, so the oldest
    # surviving record is item `dropped` of that sequence.
    first = [
        f"worker 0 spdlog {dropped // RECORDS_PER_MESSAGE}",
        f"SphereShape::setRadius: Invalid radius '-{dropped // RECORDS_PER_MESSAGE + 1}'.",
        f"worker 00 console {dropped // RECORDS_PER_MESSAGE:x}",
    ][dropped % RECORDS_PER_MESSAGE]
    assert CONSOLE_PREFIX.sub("", records[0].getMessage()).startswith(first)
    assert records[-1].getMessage().startswith(
        f"dartpy dropped {dropped} log records"
    )


@needs_testing_hooks
def test_a_thread_local_destroyed_after_the_console_stream_still_logs(caplog):
    with caplog.at_level(logging.WARNING, logger="dartpy"):
        _testing.log_from_late_thread_local_destructor()
        dart.common.flush_logs()
    assert [CONSOLE_PREFIX.sub("", r.getMessage()) for r in _records(caplog)] == [
        "early thread_local console",
        "late thread_local console",
        "late thread_local spdlog",
    ]


@needs_testing_hooks
def test_a_static_destructor_logs_to_stderr_after_python_shuts_down():
    code = (
        "import dartpy as dart\n"
        "from dartpy import _testing\n"
        "_testing.log_from_static_destructor()\n"
        "dart.common.flush_logs()\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        check=True,
        env=dict(os.environ),
    )
    # The main thread's thread-local console is gone by then, and so is Python:
    # both paths fall through to the pass-through console.
    assert re.search(
        r"\[dartpy warning\] \[module\.cpp:\d+\] static destructor console",
        result.stderr,
    )
    assert "[dartpy warning] static destructor spdlog" in result.stderr


@needs_testing_hooks
def test_warnings_reach_stderr_without_logging_configuration():
    code = (
        "import atexit\n"
        # atexit runs handlers last-in first-out: registering before the
        # import runs this after dartpy has switched to pass-through.
        "atexit.register(lambda: dart.common.warn('after-atexit'))\n"
        "import dartpy as dart\n"
        "from dartpy import _testing\n"
        "dart.common.info('info-line')\n"
        "dart.common.warn('warn-line')\n"
        "_testing.log_from_threads(threads=1, messages=1, hold_gil=True)\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        check=True,
        env=dict(os.environ),
    )
    # logging's last-resort handler prints WARNING and above; the exit flush
    # delivers the worker-thread records.
    assert "warn-line" in result.stderr
    assert "worker 0 spdlog 0" in result.stderr
    assert "[module.cpp:" in result.stderr
    assert "[dartpy warning] after-atexit" in result.stderr
    assert "info-line" not in result.stderr + result.stdout


def _wait_for_child(pid, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        done, status = os.waitpid(pid, os.WNOHANG)
        if done:
            return os.waitstatus_to_exitcode(status)
        time.sleep(0.005)
    os.kill(pid, signal.SIGKILL)
    os.waitpid(pid, 0)
    pytest.fail(f"forked child hung for {timeout}s")


@pytest.mark.skipif(not hasattr(os, "fork"), reason="needs os.fork")
# Other test modules may leave native threads behind; none of them logs.
@pytest.mark.filterwarnings("ignore::DeprecationWarning")
def test_child_of_a_fork_can_log_and_flush(caplog):
    # The supported case is a fork from a single-threaded process; forking a
    # process with other running threads is documented as unsupported. Native
    # threads other tests leave behind are idle and take no logging locks.
    with caplog.at_level(logging.WARNING, logger="dartpy"):
        dart.common.warn("parent")  # may still be queued at the fork
        pid = os.fork()
        if pid == 0:  # child
            try:
                delivered = []
                handler = logging.Handler()
                handler.emit = lambda record: delivered.append(record.getMessage())
                logging.getLogger("dartpy").addHandler(handler)
                dart.dynamics.SphereShape(1.0).setRadius(-1.0)
                dart.common.warn("child")
                dart.common.flush_logs()
                # The child starts with an empty queue: its own records
                # arrive, and none the parent had queued.
                status = (
                    0
                    if len(delivered) == 2
                    and NATIVE_WARNING.match(delivered[0])
                    and delivered[1] == "child"
                    else 3
                )
            except BaseException:
                status = 2
            os._exit(status)
        assert _wait_for_child(pid, timeout=10) == 0
        dart.common.flush_logs()
    assert [r.getMessage() for r in _records(caplog)] == ["parent"]


if __name__ == "__main__":
    pytest.main()
