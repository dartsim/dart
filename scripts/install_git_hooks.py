#!/usr/bin/env python3
"""Install DART's fast ``pre-commit`` and evidence-checking ``pre-push`` hooks.

Idempotently writes ``<git-hooks-dir>/pre-commit`` so every ``git commit`` runs
``scripts/check_agent_hook.py --profile staged`` with a compatible Python
interpreter first. Behaviour:

* Managed hooks carry sentinel lines; reinstallation publishes complete files
  atomically, keeping the old checker effective while its replacement is written.
* If a *foreign* (non-DART) hook already exists it is preserved,
  not clobbered: it is copied to the corresponding ``.local`` with its mode unchanged
  and chained from the managed hook when executable. If that ``.local``
  is already present the installer refuses with a clear message rather than
  lose an existing local hook.
* Worktrees are handled via ``git rev-parse --git-path hooks``, which resolves
  to the shared common hooks directory, so a single install covers all linked
  worktrees of the repository.
* A repository or user with ``core.hooksPath`` set manages hooks elsewhere;
  the installer refuses rather than write into a shared personal hooks
  directory. ``--custom-manager`` instead exports three named runtime files
  directly into the Git common directory for explicit chaining.
* Emergency bypass at commit time: ``DART_SKIP_HOOKS=1 git commit ...``.
* Verification aid: ``DART_HOOK_DRY_RUN=1`` makes the installed hook print the
  command it *would* run instead of running it, so tests and manual checks can
  confirm wiring without invoking the full lint (see
  ``tests/test_install_git_hooks.py``).

Runnable with plain ``python3`` — no third-party imports.
"""

from __future__ import annotations

import argparse
import os
import shlex
import shutil
import stat
import sys
import tempfile
from pathlib import Path

from review_gate import GateError, git, hooks_path_configuration, pre_push_hook

SENTINEL = "DART-MANAGED-HOOK"
HOOK_VERSION = "8"

# POSIX sh hook body. Kept dependency-free. It prefers the repository Pixi
# interpreter, then a compatible PATH python3. In an older linked worktree or
# without Python 3.11+, it safely falls back to Git's staged whitespace check.
_HOOK_TEMPLATE = f"""\
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

# Protect path newlines, then remove exactly Git's LF and the sentinel.
repo_root=$(git rev-parse --show-toplevel && printf '.') || exit 1
repo_root=${{repo_root%??}}

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
                python_cmd="$candidate"
                return 0
            fi
        fi
    done
    return 1
}}

python_cmd=
select_hook_python || python_cmd=

if [ -n "${{DART_HOOK_DRY_RUN:-}}" ]; then
    echo "DART pre-commit (dry run): would run selected Python: scripts/check_agent_hook.py --profile staged" >&2
    exit 0
fi

@LOCAL_COMMIT_HOOK@

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


def pre_commit_hook(*, chain_local: bool = True) -> str:
    """Share commit behavior while leaving custom-manager chaining to its owner."""
    local_hook = """\
# Chain to a foreign hook preserved at install time, if any.
local_hook=$(git rev-parse --git-path hooks/pre-commit.local) || exit 1
if [ -x "$local_hook" ]; then
    "$local_hook" "$@" || exit $?
fi"""
    return _HOOK_TEMPLATE.replace(
        "@LOCAL_COMMIT_HOOK@", local_hook if chain_local else ""
    )


HOOK_TEMPLATE = pre_commit_hook()


def run_git(args: list[str]) -> str:
    """Read Git's path output without trimming significant path whitespace."""
    try:
        return git(Path.cwd(), *args)
    except GateError as error:
        sys.exit(f"error: {error}\n" "  (run this from inside the DART git repository)")


def resolve_hooks_dir() -> Path:
    """Resolve the git hooks directory, honoring worktrees.

    Refuses when ``core.hooksPath`` is set: ``git rev-parse --git-path hooks``
    would then resolve to a personal or global hooks directory shared by other
    repositories, and installing (or relocating a foreign hook) there would
    affect every repo that uses it.
    """
    try:
        configured, hooks_path = hooks_path_configuration(Path.cwd())
    except GateError as error:
        sys.exit(f"error: {error}; refusing to install hooks")
    if configured:
        sys.exit(
            "error: core.hooksPath is set "
            f"({hooks_path!r}); refusing to install into a custom hooks\n"
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


def resolve_export_root() -> Path:
    """Use Git's canonical metadata root without an aliasable child directory."""
    common = Path(
        run_git(["rev-parse", "--path-format=absolute", "--git-common-dir"])
    ).resolve()
    configured = Path(
        run_git(["rev-parse", "--path-format=absolute", "--git-path", "hooks"])
    ).resolve()
    if common in (configured, (common / "hooks").resolve()):
        sys.exit(
            "error: Git common directory is also a hooks directory; "
            "refusing to export into the hook manager"
        )
    return common


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


def preserve_hook(path: Path, local: Path) -> None:
    """Keep the original active until its managed replacement is published.

    Exclusive creation also refuses a backup created after preflight. If the
    copy or later publication fails, the active foreign hook remains intact.
    """
    if path.is_symlink():
        local.symlink_to(os.readlink(path), target_is_directory=path.is_dir())
    else:
        with path.open("rb") as original, local.open("xb") as backup:
            shutil.copyfileobj(original, backup)
            local.chmod(stat.S_IMODE(os.fstat(original.fileno()).st_mode))


def require_owned_file(path: Path, sentinel: str, *, recovery: bool = False) -> None:
    """Refuse ambiguous outputs before any installer publication or adoption."""
    if path.is_symlink() or (path.exists() and not path.is_file()):
        sys.exit(f"error: refusing to replace an aliased or non-file output: {path}")
    if path.exists() and not recovery and foreign_hook(path, sentinel):
        sys.exit(f"error: refusing to replace an unmanaged output: {path}")


def require_owned_checker(runtime: Path, launcher: Path) -> None:
    owned_launcher = (
        launcher.is_file()
        and not launcher.is_symlink()
        and not foreign_hook(launcher, "DART-MANAGED-PRE-PUSH v1")
    )
    # An owned push launcher permits recovery of its damaged checker. A
    # checker-only interrupted installation carries its own stable marker.
    require_owned_file(runtime, "# DART-REVIEW-CHECKER v1", recovery=owned_launcher)


def install(hooks_dir: Path) -> int:
    checker = Path(__file__).with_name("review_gate.py").read_bytes()
    definitions = (
        ("pre-commit", SENTINEL, HOOK_TEMPLATE),
        ("pre-push", "DART-MANAGED-PRE-PUSH v1", pre_push_hook(checker)),
    )
    require_owned_checker(hooks_dir / "dart-review-gate.py", hooks_dir / "pre-push")
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
            preserve_hook(hook, local)
            print(
                f"Preserved existing {name} hook as {local} (chained from the DART hook)."
            )
        write_hook(hook, template)
        print(f"Installed DART {name} hook: {hook}")
    print(
        "  Pre-push requires recorded local reviews; see docs/onboarding/ai-reviews.md."
    )
    print(
        "  Installed evidence CLI: python3 -I "
        '"$(git rev-parse --git-path hooks/dart-review-gate.py)" --help'
    )
    print("  DART_SKIP_HOOKS applies only to the existing commit guard, not pre-push.")
    return 0


def export_review_runtime(common: Path) -> int:
    """Publish only reserved leaves, never adopt or change manager handlers."""
    launcher = common / "dart-review-pre-push"
    commit_hook = common / "dart-review-pre-commit"
    runtime = common / "dart-review-gate.py"
    require_owned_file(launcher, "DART-MANAGED-PRE-PUSH v1")
    require_owned_file(commit_hook, SENTINEL)
    require_owned_checker(runtime, launcher)
    checker = Path(__file__).with_name("review_gate.py").read_bytes()
    publish_file(runtime, checker)
    write_hook(commit_hook, pre_commit_hook(chain_local=False))
    write_hook(launcher, pre_push_hook(checker, chain_local=False))
    print(f"Exported DART review launcher: {launcher}")
    print(
        "  Chain dart-review-pre-commit and dart-review-pre-push from your manager; "
        "its configuration and handlers were not changed."
    )
    print("  Installed evidence CLI: python3 -I " f"{shlex.quote(str(runtime))} --help")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--custom-manager",
        action="store_true",
        help="export both Git gates without changing the hook manager",
    )
    args = parser.parse_args(argv)
    if args.custom_manager:
        hooks_dir = resolve_export_root()
        lock_name = ".dart-review-export-lock"
    else:
        hooks_dir = resolve_hooks_dir()
        lock_name = ".dart-install-lock"
    hooks_dir.mkdir(parents=True, exist_ok=True)
    lock = hooks_dir / lock_name
    try:
        lock.mkdir()
    except FileExistsError:
        sys.exit(
            f"error: another installer holds {lock}; after a crash verify it stopped before removing the lock"
        )
    try:
        if args.custom_manager:
            return export_review_runtime(hooks_dir)
        return install(hooks_dir)
    finally:
        lock.rmdir()


if __name__ == "__main__":
    raise SystemExit(main())
