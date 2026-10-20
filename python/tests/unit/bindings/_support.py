"""Run native ownership regressions through interpreter shutdown."""

import os
import signal
import subprocess
import sys
import textwrap
from pathlib import Path

import dartpy as dart


# abort() ends a POSIX child with SIGABRT and an MSVC runtime child with exit 3.
ABORT_CODES = {-signal.SIGABRT, 3}


def run_isolated(script, allow_leaks=False, assertion=None):
    # Native lifetime failures must not prevent the remaining tests from running.
    # With `assertion`, a build with native assertions may stop at the
    # DART_ASSERT that follows that reported misuse; any other failure counts.
    source = "import gc\nimport dartpy as dart\nimport numpy as np\ndef main():\n"
    source += textwrap.indent(textwrap.dedent(script).strip(), "    ")
    source += '\nmain()\ngc.collect()\nprint("done")\n'
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join(
        [str(Path(dart.__file__).resolve().parents[1]), *filter(None, sys.path)]
    )
    result = subprocess.run(
        [sys.executable, "-c", source],
        env=environment,
        capture_output=True,
        text=True,
        timeout=120,
    )
    if result.returncode != 0 and assertion is not None:
        assert result.returncode in ABORT_CODES and assertion in result.stderr, (
            f"subprocess exited with {result.returncode}\n"
            f"stderr: {result.stderr[-4000:]}"
        )
        return result
    assert result.returncode == 0, (
        f"subprocess exited with {result.returncode}\n"
        f"stdout: {result.stdout[-2000:]}\nstderr: {result.stderr[-4000:]}"
    )
    assert "done" in result.stdout
    if not allow_leaks:
        assert "nanobind: leaked" not in result.stderr, result.stderr[-4000:]
    if result.stderr:
        print(result.stderr[-4000:])
    return result
