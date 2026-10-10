"""Run native ownership regressions through interpreter shutdown."""

import os
import subprocess
import sys
import textwrap
from pathlib import Path

import dartpy as dart



def run_isolated(script):
    # Native lifetime failures must not prevent the remaining tests from running.
    source = "import gc\nimport dartpy as dart\nimport numpy as np\ndef main():\n"
    source += textwrap.indent(textwrap.dedent(script).strip(), "    ")
    source += '\nmain()\ngc.collect()\nprint("done")\n'
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join(
        [str(Path(dart.__file__).resolve().parent), *filter(None, sys.path)]
    )
    result = subprocess.run(
        [sys.executable, "-c", source],
        env=environment,
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert result.returncode == 0, (
        f"subprocess exited with {result.returncode}\n"
        f"stdout: {result.stdout[-2000:]}\nstderr: {result.stderr[-4000:]}"
    )
    assert "done" in result.stdout
    assert "nanobind: leaked" not in result.stderr, result.stderr[-4000:]
    if result.stderr:
        print(result.stderr[-4000:])
