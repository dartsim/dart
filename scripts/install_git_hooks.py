#!/usr/bin/env python3
"""Install DART's managed ``pre-commit``, ``commit-msg`` and ``pre-push`` hooks.

Idempotently writes hooks so every ``git commit`` runs the fast staged
gate (``scripts/check_agent_hook.py --profile staged``) and scans its message
(``scripts/check_local_paths.py --commit-msg-file "$1"``) with compatible
Python interpreter selection for each gate. The message scan derives cleanup
from the parent Git command, independent of the editor template's language.
Strip cleanup skips configured comments; whitespace cleanup scans every line.
Scissors cleanup or verbose mode excludes text after Git's scissors line. No
``pre-merge-commit`` hook is installed: a conflicted merge ends with
``git commit``, and ``pre-push`` checks commits created by merges, cherry-picks,
rebases, reverts, imports and sequencer operations before publication. It reads
Git's ref updates from stdin, skips deletions, and scans messages and changes
using ``check_local_paths.py --commit-range``. Existing refs use the remote SHA;
new refs use the merge base with the remote's default branch. Missing base
objects are fetched without changing refs or FETCH_HEAD. An empty remote or
unrelated history scans the whole local history; lookup or scan errors block
the push. Foreign pre-push hooks receive the same stdin. Behaviour:

* Each managed hook carries a sentinel line (``DART-MANAGED-HOOK``); re-running
  this installer detects it and rewrites the hook in place, so the command is
  safe to run any number of times.
* If a *foreign* (non-DART) hook already exists it is preserved,
  not clobbered: it is moved to ``<hook>.local`` with its mode unchanged
  and chained from the managed hook when executable. If a ``<hook>.local``
  is already present the installer refuses with a clear message rather than
  lose an existing local hook.
* Worktrees are handled via ``git rev-parse --git-path hooks``, which resolves
  to the shared common hooks directory, so a single install covers all linked
  worktrees of the repository.
* A repository or user with ``core.hooksPath`` set manages hooks elsewhere;
  the installer refuses rather than write into a shared personal hooks
  directory.
* Emergency bypass: ``DART_SKIP_HOOKS=1 git commit ...`` or
  ``DART_SKIP_HOOKS=1 git push ...``.
* Verification aid: ``DART_HOOK_DRY_RUN=1`` makes each installed hook print the
  command it *would* run instead of running it, so tests and manual checks can
  confirm wiring without invoking the full lint (see
  ``tests/test_install_git_hooks.py``).

Runnable with plain ``python3`` — no third-party imports.
"""

from __future__ import annotations

import shutil
import stat
import subprocess
import sys
from pathlib import Path

SENTINEL = "DART-MANAGED-HOOK"
HOOK_VERSION = "11"

PRE_PUSH_SCAN = """\
printf '%s\\n' "$push_updates" | while read -r local_ref local_sha remote_ref remote_sha; do
    # A zero local SHA denotes deletion; no commit is being published.
    case "$local_sha" in
        *[!0]*) ;;
        *) continue ;;
    esac
    if [ ! -f scripts/check_local_paths.py ] || [ -z "$python_cmd" ]; then
        echo "DART pre-push: local-path gate unavailable — push blocked." >&2
        exit 1
    fi
    base_sha=$remote_sha
    new_ref=0
    case "$remote_sha" in
        *[!0]*) ;;
        *)
            new_ref=1
            remote_head=$(git ls-remote --symref "$2" HEAD) || exit 1
            base_sha=$(printf '%s\\n' "$remote_head" | awk '$2 == "HEAD" && $1 != "ref:" {print $1}')
            ;;
    esac
    if [ -n "$base_sha" ]; then
        if ! git cat-file -e "$base_sha^{commit}" 2>/dev/null; then
            git fetch --no-tags --no-write-fetch-head "$2" "$base_sha" || exit 1
        fi
        if [ "$new_ref" = 1 ]; then
            base_sha=$(git merge-base "$base_sha" "$local_sha")
            status=$?
            # Status 1 means unrelated histories; other errors must block.
            [ "$status" -le 1 ] || exit "$status"
        fi
    fi
    commit_range=$local_sha
    [ -z "$base_sha" ] || commit_range="$base_sha..$local_sha"
    echo "DART pre-push: scanning $local_ref ($commit_range)..." >&2
    if ! "$python_cmd" scripts/check_local_paths.py --commit-range "$commit_range"; then
        echo "DART pre-push: local-path hook FAILED — push blocked." >&2
        echo "  Fix: remove local paths from pushed commits." >&2
        echo "  Emergency bypass: DART_SKIP_HOOKS=1 git push ..." >&2
        exit 1
    fi
done
"""


def hook_template(name: str) -> str:
    script = "check_agent_hook.py" if name == "pre-commit" else "check_local_paths.py"
    arguments = (
        "--profile staged"
        if name == "pre-commit"
        else (
            '--commit-range "$commit_range"'
            if name == "pre-push"
            else '--commit-msg-file "$1" --git-pid "$PPID"'
        )
    )
    command = f"scripts/{script} {arguments}"
    display_command = command.replace('"', '\\"')
    gate = "agent" if name == "pre-commit" else "local-path"
    compatibility_check = (
        "import tomllib"
        if name == "pre-commit"
        else "import sys; sys.exit(sys.version_info < (3, 9))"
    )
    fix = (
        "pixi run lint   (then re-stage and commit)"
        if name == "pre-commit"
        else "remove local paths from the commit message"
    )
    if name == "pre-commit":
        fallback = """\
    echo "DART pre-commit: full agent gate unavailable in this worktree; running staged diff fallback..." >&2
    if ! git -c core.whitespace=cr-at-eol diff --cached --check; then
        echo "DART pre-commit: staged diff check FAILED — commit blocked." >&2
        exit 1
    fi
"""
    else:
        fallback = '    echo "DART commit-msg: local-path gate unavailable in this worktree; skipping message scan." >&2\n'
    operation = "push" if name == "pre-push" else "commit"
    capture = ""
    chain = f'    "$hooks_dir/{name}.local" "$@" || exit $?'
    run_gate = f"""\
if [ ! -f scripts/{script} ] \
    || [ -z "$python_cmd" ]; then
{fallback}    exit 0
fi

echo "DART {name}: running fast {gate} gate ($python_cmd {display_command})..." >&2
if ! "$python_cmd" {command}; then
    echo "" >&2
    echo "DART {name}: {gate} hook FAILED — commit blocked." >&2
    echo "  Fix with: {fix}" >&2
    echo "  Emergency bypass: DART_SKIP_HOOKS=1 git commit ..." >&2
    exit 1
fi
"""
    if name == "pre-push":
        capture = "push_updates=$(cat) || exit 1\n"
        chain = '    printf \'%s\\n\' "$push_updates" | "$hooks_dir/pre-push.local" "$@" || exit $?'
        run_gate = PRE_PUSH_SCAN
    # All hooks prefer Pixi Python, then a compatible PATH python3.
    return f"""\
#!/bin/sh
# DART {name} hook — installed by scripts/install_git_hooks.py
# {SENTINEL} v{HOOK_VERSION}  (sentinel line: do not edit; the installer keys on it)
#
# Runs `{command}` before every {operation}.
# Emergency bypass: DART_SKIP_HOOKS=1 git {operation} ...

if [ "${{DART_SKIP_HOOKS:-0}}" = "1" ]; then
    echo "DART {name}: skipped (DART_SKIP_HOOKS=1)" >&2
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
            if "$candidate" -c '{compatibility_check}' >/dev/null 2>&1; then
                printf '%s\n' "$candidate"
                return 0
            fi
        fi
    done
    return 1
}}

python_cmd=$(select_hook_python) || python_cmd=

if [ -n "${{DART_HOOK_DRY_RUN:-}}" ]; then
    echo "DART {name} (dry run): would run selected Python: {display_command}" >&2
    exit 0
fi

# Chain to a foreign hook preserved at install time, if any.
{capture}hooks_dir=$(git rev-parse --git-path hooks)
if [ -x "$hooks_dir/{name}.local" ]; then
{chain}
fi

cd "$repo_root" || exit 1

{run_gate}
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
    )
    if hooks_path.returncode == 0:
        sys.exit(
            "error: core.hooksPath is set "
            f"({hooks_path.stdout.strip()!r}); refusing to install into a custom hooks\n"
            "  directory that may be shared across repositories. Add the gate "
            "to your own\n"
            '  hook manager (run `python3 scripts/check_agent_hook.py --profile staged` from pre-commit and `python3 scripts/check_local_paths.py --commit-msg-file "$1"` from commit-msg), '
            "include the pre-push ref-range scan too, or unset\n"
            "  core.hooksPath and re-run `pixi run install-hooks`."
        )
    if hooks_path.returncode != 1:
        sys.exit(f"error: cannot read core.hooksPath: {hooks_path.stderr.strip()}")
    raw = Path(run_git(["rev-parse", "--git-path", "hooks"]))
    if not raw.is_absolute():
        raw = (Path.cwd() / raw).resolve()
    return raw


def write_hook(path: Path) -> None:
    path.write_text(hook_template(path.name))
    mode = path.stat().st_mode
    path.chmod(mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def main() -> int:
    hooks_dir = resolve_hooks_dir()
    hooks_dir.mkdir(parents=True, exist_ok=True)

    hooks = [hooks_dir / name for name in ("pre-commit", "commit-msg", "pre-push")]
    # Check all backups before changing any hook.
    for hook in hooks:
        local = hook.with_name(f"{hook.name}.local")
        if (
            hook.is_symlink()
            or hook.exists()
            and SENTINEL not in hook.read_text(errors="replace")
        ) and (local.exists() or local.is_symlink()):
            sys.exit(
                f"error: refusing to overwrite an existing {hook.name} hook.\n"
                f"  A foreign hook exists at {hook} AND {local} is already\n"
                "  present, so the foreign hook cannot be backed up without loss.\n"
                f"  Resolve manually: fold your hook logic into {hook.name}.local,\n"
                f"  remove {hook.name}, then re-run `pixi run install-hooks`."
            )

    for hook in hooks:
        if (
            hook.is_symlink()
            or hook.exists()
            and SENTINEL not in hook.read_text(errors="replace")
        ):
            local = hook.with_name(f"{hook.name}.local")
            shutil.move(str(hook), str(local))
            print(
                f"Preserved existing {hook.name} hook as {local} (chained from the DART hook)."
            )
        write_hook(hook)
        print(f"Installed/refreshed DART {hook.name} hook: {hook}")
    print("  All gates use the repository Pixi Python when available.")
    print("  Emergency bypass: DART_SKIP_HOOKS=1 git commit/push ...")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
