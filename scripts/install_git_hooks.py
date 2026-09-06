#!/usr/bin/env python3
"""Install DART's fast ``pre-commit`` and evidence-checking ``pre-push`` hooks.

Idempotently writes ``<git-hooks-dir>/pre-commit`` so every ``git commit`` runs
``scripts/check_agent_hook.py --profile staged`` with a compatible Python
interpreter first. Behaviour:

* Managed hooks carry sentinel lines; reinstallation publishes complete files
  atomically, keeping the old checker effective while its replacement is written.
* If a *foreign* (non-DART) hook already exists it is preserved,
  not clobbered: it is moved to the corresponding ``.local`` with its mode unchanged
  and chained from the managed hook when executable. If that ``.local``
  is already present the installer refuses with a clear message rather than
  lose an existing local hook.
* Worktrees are handled via ``git rev-parse --git-path hooks``, which resolves
  to the shared common hooks directory, so a single install covers all linked
  worktrees of the repository.
* A repository or user with ``core.hooksPath`` set manages hooks elsewhere;
  the installer refuses rather than write into a shared personal hooks
  directory.
* Emergency bypass at commit time: ``DART_SKIP_HOOKS=1 git commit ...``.
* Verification aid: ``DART_HOOK_DRY_RUN=1`` makes the installed hook print the
  command it *would* run instead of running it, so tests and manual checks can
  confirm wiring without invoking the full lint (see
  ``tests/test_install_git_hooks.py``).

Runnable with plain ``python3`` — no third-party imports.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from review_gate import pre_push_hook

SENTINEL = "DART-MANAGED-HOOK"
HOOK_VERSION = "7"

# POSIX sh hook body. Kept dependency-free. It prefers the repository Pixi
# interpreter, then a compatible PATH python3. In an older linked worktree or
# without Python 3.11+, it safely falls back to Git's staged whitespace check.
HOOK_TEMPLATE = f"""\
#!/bin/sh
# DART pre-commit hook — installed by scripts/install_git_hooks.py
# {SENTINEL} v{HOOK_VERSION}  (sentinel line: do not edit; the installer keys on it)
#
# Runs the fast staged-file gate (`scripts/check_agent_hook.py --profile staged`) before every commit.
# Emergency bypass: DART_SKIP_HOOKS=1 git commit ...

if [ "${{DART_SKIP_HOOKS:-0}}" = "1" ]; then
    echo "DART pre-commit: skipped (DART_SKIP_HOOKS=1)" >&2
    exit 0
fi

repo_root=$(git rev-parse --show-toplevel) || exit 1

select_hook_python() {{
    for candidate in \
        "${{DART_HOOK_PYTHON:-}}" \
        "$repo_root/.pixi/envs/default/bin/python" \
        "$repo_root/.pixi/envs/default/python.exe" \
        python3
    do
        [ -n "$candidate" ] || continue
        if [ -x "$candidate" ] || command -v "$candidate" >/dev/null 2>&1; then
            if "$candidate" -c 'import tomllib' >/dev/null 2>&1; then
                printf '%s\n' "$candidate"
                return 0
            fi
        fi
    done
    return 1
}}

python_cmd=$(select_hook_python) || python_cmd=

if [ -n "${{DART_HOOK_DRY_RUN:-}}" ]; then
    echo "DART pre-commit (dry run): would run selected Python: scripts/check_agent_hook.py --profile staged" >&2
    exit 0
fi

# Chain to a foreign hook preserved at install time, if any.
hooks_dir=$(git rev-parse --git-path hooks)
if [ -x "$hooks_dir/pre-commit.local" ]; then
    "$hooks_dir/pre-commit.local" "$@" || exit $?
fi

cd "$repo_root" || exit 1

if [ ! -f scripts/check_agent_hook.py ] \
    || [ -z "$python_cmd" ]; then
    echo "DART pre-commit: full agent gate unavailable in this worktree; running staged diff fallback..." >&2
    if ! git -c core.whitespace=cr-at-eol diff --cached --check; then
        echo "DART pre-commit: staged diff check FAILED — commit blocked." >&2
        exit 1
    fi
    exit 0
fi

echo "DART pre-commit: running fast agent gate ($python_cmd scripts/check_agent_hook.py --profile staged)..." >&2
if ! "$python_cmd" scripts/check_agent_hook.py --profile staged; then
    echo "" >&2
    echo "DART pre-commit: agent hook FAILED — commit blocked." >&2
    echo "  Fix with: pixi run lint   (then re-stage and commit)" >&2
    echo "  Emergency bypass: DART_SKIP_HOOKS=1 git commit ..." >&2
    exit 1
fi
"""


def run_git(args: list[str]) -> str:
    """Run a git command from the current directory and return stripped stdout."""
    result = subprocess.run(
        ["git", *args],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        sys.exit(
            f"error: `git {' '.join(args)}` failed: {result.stderr.strip()}\n"
            "  (run this from inside the DART git repository)"
        )
    return result.stdout.strip()


def resolve_hooks_dir() -> Path:
    """Resolve the git hooks directory, honoring worktrees.

    Refuses when ``core.hooksPath`` is set: ``git rev-parse --git-path hooks``
    would then resolve to a personal or global hooks directory shared by other
    repositories, and installing (or relocating a foreign hook) there would
    affect every repo that uses it.
    """
    hooks_path = subprocess.run(
        ["git", "config", "--get", "core.hooksPath"],
        capture_output=True,
        text=True,
    ).stdout.strip()
    if hooks_path:
        sys.exit(
            "error: core.hooksPath is set "
            f"({hooks_path}); refusing to install into a custom hooks\n"
            "  directory that may be shared across repositories. Add the gate "
            "to your own\n"
            "  hook manager (see docs/onboarding/ai-tools.md for both pre-commit "
            "and pre-push integration), "
            "or unset\n"
            "  core.hooksPath and re-run `pixi run install-hooks`."
        )
    raw = Path(run_git(["rev-parse", "--git-path", "hooks"]))
    if not raw.is_absolute():
        raw = (Path.cwd() / raw).resolve()
    return raw


def publish_file(path: Path, content: bytes, executable: bool = False) -> None:
    """Readers see a complete old or new file, including during reinstall."""
    descriptor, temporary = tempfile.mkstemp(prefix=".dart-hook-", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as output:
            output.write(content)
        Path(temporary).chmod(0o755 if executable else 0o644)
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def write_hook(path: Path, template: str = HOOK_TEMPLATE) -> None:
    publish_file(path, template.encode("utf-8"), executable=True)


def foreign_hook(path: Path, sentinel: str) -> bool:
    return path.is_symlink() or (
        path.exists() and sentinel not in path.read_text(errors="replace")
    )


def install(hooks_dir: Path) -> int:
    checker = Path(__file__).with_name("review_gate.py").read_bytes()
    definitions = (
        ("pre-commit", SENTINEL, HOOK_TEMPLATE),
        ("pre-push", "DART-MANAGED-PRE-PUSH v1", pre_push_hook(checker)),
    )
    # Check both preservation boundaries before replacing either hook.
    for name, sentinel, _ in definitions:
        hook = hooks_dir / name
        local = hooks_dir / f"{name}.local"
        if foreign_hook(hook, sentinel) and os.path.lexists(local):
            sys.exit(
                f"error: refusing to overwrite an existing {name} hook.\n"
                f"  A foreign hook exists at {hook} AND {local} is already\n"
                "  present, so the foreign hook cannot be backed up without loss.\n"
                f"  Resolve manually: fold your hook logic into {name}.local,\n"
                f"  remove {name}, then re-run `pixi run install-hooks`."
            )
    # Keep the installed checker independent of the currently checked-out tree.
    publish_file(hooks_dir / "dart-review-gate.py", checker)
    for name, sentinel, template in definitions:
        hook = hooks_dir / name
        if foreign_hook(hook, sentinel):
            local = hooks_dir / f"{name}.local"
            shutil.move(str(hook), str(local))
            print(
                f"Preserved existing {name} hook as {local} (chained from the DART hook)."
            )
        write_hook(hook, template)
        print(f"Installed DART {name} hook: {hook}")
    print(
        "  Pre-push requires recorded local reviews; see docs/onboarding/ai-reviews.md."
    )
    print("  DART_SKIP_HOOKS applies only to the existing commit guard, not pre-push.")
    return 0


def main() -> int:
    hooks_dir = resolve_hooks_dir()
    hooks_dir.mkdir(parents=True, exist_ok=True)
    lock = hooks_dir / ".dart-install-lock"
    try:
        lock.mkdir()
    except FileExistsError:
        sys.exit(
            f"error: another installer holds {lock}; after a crash verify it stopped before removing the lock"
        )
    try:
        return install(hooks_dir)
    finally:
        lock.rmdir()


if __name__ == "__main__":
    raise SystemExit(main())
