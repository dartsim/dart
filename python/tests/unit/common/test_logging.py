import logging
import threading

import dartpy as dart

import pytest

from ..bindings._support import run_isolated


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


@pytest.fixture
def bridge():
    dart.common.enable_logging(level=logging.DEBUG)
    try:
        yield
    finally:
        dart.common.disable_logging()


def test_bridge_levels_and_logger_name(bridge, caplog):
    caplog.set_level(logging.DEBUG, logger="dartpy")
    names = ["trace", "debug", "info", "warn", "error", "fatal"]
    for name in names:
        getattr(dart.common, name)(f"{name} 100% %s {{}} {{broken}}")
    assert not caplog.records
    assert dart.common.flush_logs() == 6
    assert [r.name for r in caplog.records] == ["dartpy"] * 6
    assert [r.levelno for r in caplog.records] == [10, 10, 20, 30, 40, 50]
    assert [r.message for r in caplog.records] == [
        f"{name} 100% %s {{}} {{broken}}" for name in names
    ]
    assert dart.common.flush_logs() == 0


def test_bridge_filters_at_native_and_python_thresholds(bridge, caplog):
    dart.common.enable_logging(level=logging.WARNING)
    caplog.set_level(logging.ERROR, logger="dartpy")
    dart.common.info("native filtered")
    dart.common.warn("python filtered")
    dart.common.error("delivered")
    assert dart.common.flush_logs() == 2
    assert [r.message for r in caplog.records] == ["delivered"]


def test_bridge_drops_oldest_records_and_reports_overflow_once(bridge, caplog):
    caplog.set_level(logging.INFO, logger="dartpy")
    for i in range(100007):
        dart.common.info(str(i))
    assert dart.common.flush_logs() == 100000
    assert caplog.records[0].levelno == logging.WARNING
    assert caplog.records[0].message == "dartpy: 7 log records dropped"
    assert [r.message for r in caplog.records[1:]] == [
        str(i) for i in range(7, 100007)
    ]
    assert dart.common.flush_logs() == 0
    dart.common.info("after overflow")
    assert dart.common.flush_logs() == 1
    assert len(caplog.records) == 100002
    assert caplog.records[-1].message == "after overflow"


@pytest.mark.parametrize(
    "level, minimum",
    [
        (-1, 10), (0, 10), (9, 10), (10, 10),
        (11, 20), (19, 20), (20, 20),
        (21, 30), (29, 30), (30, 30),
        (31, 40), (39, 40), (40, 40),
        (41, 50), (49, 50), (50, 50), (51, 60),
    ],
)
def test_native_level_bucket_edges(bridge, caplog, level, minimum):
    dart.common.enable_logging(level=level)
    caplog.set_level(logging.DEBUG, logger="dartpy")
    for name in ["trace", "debug", "info", "warn", "error", "fatal"]:
        getattr(dart.common, name)(name)
    expected = [severity for severity in [10, 10, 20, 30, 40, 50]
                if severity >= minimum]
    assert dart.common.flush_logs() == len(expected)
    assert [record.levelno for record in caplog.records] == expected


def test_disable_drains_pending_and_restores_console(bridge, caplog, capfd):
    caplog.set_level(logging.DEBUG, logger="dartpy")
    dart.common.info("pending")
    assert dart.common.disable_logging() == 1
    dart.common.info("console restored")
    dart.common.debug("default threshold restored")
    assert [r.message for r in caplog.records] == ["pending"]
    out, err = capfd.readouterr()
    assert "console restored" in out
    assert "default threshold restored" not in out + err
    assert dart.common.disable_logging() == 0


@pytest.mark.skipif(not hasattr(dart, "viewer"), reason="GUI app host is unavailable")
def test_bridge_can_outlive_the_gui_app_capture():
    result = run_isolated(
        """
        import logging

        messages = []
        class Handler(logging.Handler):
            def emit(self, record):
                messages.append(record.getMessage())

        logger = logging.getLogger("dartpy")
        logger.setLevel(logging.INFO)
        logger.addHandler(Handler())

        def build():
            dart.common.enable_logging()
            dart.common.info("queued in App")
            return dart.viewer.SceneSetup(dart.simulation.World())

        scene = dart.viewer.Scene("logs", title="Logs", category="Tests", factory=build)
        assert dart.viewer.run([scene], ["test", "--cycle-scenes", "--frames", "1"]) == 0
        assert dart._probe.has_custom_log_sink()
        dart.common.info("between App exit and disable")
        assert dart.common.disable_logging() >= 2
        assert not dart._probe.has_custom_log_sink()
        assert "queued in App" in messages
        assert "between App exit and disable" in messages
        dart.common.info("after App and bridge shutdown")
        assert dart.common.flush_logs() == 0
        """
    )
    assert "between App exit and disable" not in result.stdout
    assert "after App and bridge shutdown" in result.stdout


@pytest.mark.skipif(not hasattr(dart, "viewer"), reason="GUI app host is unavailable")
def test_bridge_disabled_inside_app_keeps_forwarding_after_app_exit():
    result = run_isolated(
        """
        dart.common.enable_logging()

        def build():
            dart.common.disable_logging()
            assert dart._probe.has_custom_log_sink()
            dart.common.info("between disable and App exit")
            assert dart.common.flush_logs() == 0
            return dart.viewer.SceneSetup(dart.simulation.World())

        scene = dart.viewer.Scene("logs", title="Logs", category="Tests", factory=build)
        assert dart.viewer.run([scene], ["test", "--cycle-scenes", "--frames", "1"]) == 0
        assert not dart._probe.has_custom_log_sink()
        dart.common.info("after App exit with disabled bridge")
        assert dart.common.flush_logs() == 0
        # Re-enabling must not reactivate a stale wrapper in the previous chain.
        dart.common.enable_logging()
        dart.common.info("one new record")
        assert dart.common.disable_logging() == 1
        assert not dart._probe.has_custom_log_sink()
        dart.common.info("after re-enable and disable")
        assert dart.common.flush_logs() == 0
        """
    )
    assert "between disable and App exit" in result.stdout
    assert "after App exit with disabled bridge" in result.stdout
    assert "after re-enable and disable" in result.stdout


def test_concurrent_producers_preserve_per_thread_order(bridge, caplog):
    caplog.set_level(logging.INFO, logger="dartpy")

    def produce(worker):
        for i in range(100):
            dart.common.info(f"worker {worker}:{i}")

    threads = [threading.Thread(target=produce, args=(i,)) for i in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=10)
        assert not thread.is_alive()
    assert dart.common.flush_logs() == 400
    messages = [r.message for r in caplog.records]
    assert len(set(messages)) == 400
    for worker in range(4):
        assert [m for m in messages if m.startswith(f"worker {worker}:")] == [
            f"worker {worker}:{i}" for i in range(100)
        ]


@pytest.mark.parametrize("threads", [1, 4])
def test_native_step_workers_queue_in_order_until_drained(threads):
    # A subprocess bounds deadlocks if a sink incorrectly tries to acquire the GIL.
    run_isolated(
        f"""
        import logging
        import threading

        records = []
        class Handler(logging.Handler):
            def emit(self, record):
                assert threading.current_thread() is threading.main_thread()
                records.append(record)

        logger = logging.getLogger("dartpy")
        logger.setLevel(logging.INFO)
        logger.addHandler(Handler())
        world = dart.simulation.World()
        world.setNumSimulationThreads({threads})
        # World uses its native executor at 128 skeletons and above.
        for i in range(128):
            skel = dart.dynamics.Skeleton(f"worker-{{i}}")
            joint, _ = skel.createRevoluteJointAndBodyNodePair()
            joint.setName(f"joint-{{i}}")
            # Infinite stiffness is supported and exercises the existing
            # non-finite acceleration diagnostic during forward dynamics.
            joint.setSpringStiffness(0, float("inf"))
            world.addSkeleton(skel)
        dart.common.enable_logging()
        for step in range(3):
            dart.common.info(f"begin {{step}}")
            world.step()
            dart.common.info(f"end {{step}}")
        assert not records
        assert dart.common.flush_logs() == 3 * (128 + 2)
        messages = [record.getMessage() for record in records]
        for step in range(3):
            block = messages[step * 130:(step + 1) * 130]
            assert block[0] == f"begin {{step}}"
            assert block[-1] == f"end {{step}}"
            warnings = block[1:-1]
            assert all("updateAccelerationDynamic" in text for text in warnings)
            indices = [
                int(text.split("joint [joint-")[1].split("]")[0])
                for text in warnings
            ]
            assert set(indices) == set(range(128))
            # Each native executor participant traverses one ascending chunk.
            chunk_size = 128 // {threads}
            for first in range(0, 128, chunk_size):
                expected = list(range(first, first + chunk_size))
                assert [i for i in indices if first <= i < first + chunk_size] == expected
            assert all(record.levelno == logging.WARNING
                       for record in records[step * 130 + 1:(step + 1) * 130 - 1])
        assert dart.common.flush_logs() == 0
        dart.common.disable_logging()
        """
    )


def test_handler_exception_leaves_remaining_records_queued(bridge, caplog):
    class FailingHandler(logging.Handler):
        def emit(self, record):
            raise RuntimeError("handler failed")

    logger = logging.getLogger("dartpy")
    handler = FailingHandler()
    logger.addHandler(handler)
    caplog.set_level(logging.INFO, logger="dartpy")
    try:
        dart.common.info("failed record")
        dart.common.info("remaining record")
        with pytest.raises(RuntimeError, match="handler failed"):
            dart.common.flush_logs()
    finally:
        logger.removeHandler(handler)
    assert dart.common.flush_logs() == 1


def test_handler_can_log_and_flush_without_deadlock():
    run_isolated(
        """
        import logging
        import threading

        messages = []
        class Handler(logging.Handler):
            def emit(self, record):
                assert threading.current_thread() is threading.main_thread()
                messages.append(record.getMessage())
                if record.getMessage() == "outer":
                    dart.common.info("nested")
                    assert dart.common.flush_logs() == 0
                    worker = threading.Thread(
                        target=lambda: dart.common.info("handler worker")
                    )
                    worker.start()
                    worker.join(timeout=5)
                    assert not worker.is_alive()

        logger = logging.getLogger("dartpy")
        logger.setLevel(logging.DEBUG)
        logger.addHandler(Handler())
        dart.common.enable_logging()
        dart.common.info("outer")
        assert dart.common.flush_logs() == 1
        assert messages == ["outer"]
        assert dart.common.flush_logs() == 2
        assert messages == ["outer", "nested", "handler worker"]
        dart.common.disable_logging()
        """
    )


def test_concurrent_flushes_wait_and_deliver_each_record_once():
    run_isolated(
        """
        import logging
        import threading

        entered = threading.Event()
        release = threading.Event()
        waiting = threading.Event()
        returned = threading.Event()
        messages = []
        counts = []
        class Handler(logging.Handler):
            def emit(self, record):
                messages.append(record.getMessage())
                if record.getMessage() == "first":
                    entered.set()
                    assert release.wait(5)  # Releases the GIL inside a handler.

        logger = logging.getLogger("dartpy")
        logger.setLevel(logging.INFO)
        logger.addHandler(Handler())
        dart.common.enable_logging()
        dart.common.info("first")
        dart.common.info("second")
        first = threading.Thread(target=lambda: counts.append(dart.common.flush_logs()))
        def flush_again():
            waiting.set()
            counts.append(dart.common.flush_logs())
            returned.set()
        second = threading.Thread(target=flush_again)
        first.start()
        assert entered.wait(5)
        second.start()
        assert waiting.wait(5)
        try:
            assert not returned.wait(0.1)
            dart.common.info("late")
        finally:
            release.set()
        for thread in (first, second):
            thread.join(timeout=5)
            assert not thread.is_alive()
        assert counts == [2, 1], counts
        assert messages == ["first", "second", "late"], messages
        assert dart.common.disable_logging() == 0
        """
    )


def test_handler_disable_restores_sink_and_outer_flush_drains_remainder():
    result = run_isolated(
        """
        import logging

        messages = []
        class Handler(logging.Handler):
            def emit(self, record):
                messages.append(record.getMessage())
                if record.getMessage() == "outer":
                    dart.common.info("nested")
                    assert dart.common.disable_logging() == 0
                    dart.common.info("restored console")

        logger = logging.getLogger("dartpy")
        logger.setLevel(logging.INFO)
        logger.addHandler(Handler())
        dart.common.enable_logging()
        dart.common.info("outer")
        dart.common.info("pending")
        assert dart.common.flush_logs() == 3
        assert messages == ["outer", "pending", "nested"], messages
        assert dart.common.flush_logs() == 0
        assert dart.common.disable_logging() == 0
        """
    )
    assert "restored console" in result.stdout


def test_interpreter_exit_drains_pending_records():
    result = run_isolated(
        """
        dart.common.enable_logging()
        import logging
        import sys

        logger = logging.getLogger("dartpy")
        logger.setLevel(logging.INFO)
        logger.addHandler(logging.StreamHandler(sys.stdout))
        dart.common.enable_logging()
        dart.common.info("queued until interpreter exit")
        """
    )
    assert result.stdout.count("queued until interpreter exit") == 1


if __name__ == "__main__":
    pytest.main()
