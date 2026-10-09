#!/bin/sh
# DART Claude Code/Codex PreToolUse guard, shared by both hook configurations.
# Reads .tool_input.command or .cmd from JSON stdin; exit 2 blocks the tool call.
#
# Three paths:
#   * Raw command without both case-insensitive words "git" and "commit": allow.
#   * Simple commands joined by &&, ; or newlines: inspect every Git commit,
#     retaining message extraction, plain-cd tracking, foreign-repo skipping,
#     child-shell checks and allowlisted delegation to current managed hooks.
#   * Other commands containing both words: block and ask for a simple commit
#     command; execution order and commit-time content cannot be inspected.
#
# Simple commits without managed enforcement must supply inspectable messages,
# stage files beforehand, and avoid multiple commits after content/index changes.
# Git aliases, am/applypatch imports and alternate GIT_INDEX_FILE indexes remain
# out of scope; PR Text scans every resulting commit with the base checker.
# DART_SKIP_HOOKS=1 in the environment bypasses the guard; simple command prefixes
# retain their existing emergency bypass. DART_HOOK_DRY_RUN prints the simple-chain gate only.
# Missing Python prints a notice and allows; an injected interpreter fails closed.

input=$(cat)

# Only Unicode escapes can hide ASCII "commit" in JSON; decode those below.
case "$input" in
    *[cC][oO][mM][mM][iI][tT]*|*'\u'*) ;;
    *) exit 0 ;;
esac

python_cmd=${DART_HOOK_PYTHON:-}
if [ -z "$python_cmd" ]; then
    hook_project_dir=${CLAUDE_PROJECT_DIR:-${CODEX_PROJECT_DIR:-}}
    if [ -z "$hook_project_dir" ]; then
        hook_project_dir=$(git rev-parse --show-toplevel 2>/dev/null || true)
    fi
    for candidate in \
        "$hook_project_dir/.pixi/envs/default/bin/python" \
        "$hook_project_dir/.pixi/envs/default/python.exe"
    do
        if [ -n "$hook_project_dir" ] && [ -x "$candidate" ]; then
            python_cmd=$candidate
            break
        fi
    done
    if [ -z "$python_cmd" ] && command -v python3 >/dev/null 2>&1; then
        python_cmd=python3
    fi
fi
# Without a working classifier, block anything that may commit and allow the rest.
may_commit() {
    printf '%s' "$input" | grep -qiw git && printf '%s' "$input" | grep -qiw commit
}
disable_guard() {
    if [ -n "${DART_HOOK_PYTHON:-}" ] || may_commit; then
        echo "DART guard: cannot inspect this commit; blocking it" >&2
        exit 2
    fi
    echo "DART guard: guard disabled for this call" >&2
    exit 0
}

if [ -z "$python_cmd" ]; then
    echo "DART guard: python3 unavailable" >&2
    disable_guard
fi

# Decode once, then allow, inspect a precise simple chain, or block.
guard_program="$(dirname "$0")/pre-commit-guard.py"
guard_result=$(printf '%s' "$input" | "$python_cmd" "$guard_program")
guard_status=$?
if [ "$guard_status" -ne 0 ]; then
    echo "DART guard: commit detection failed" >&2
    disable_guard
fi

verdict=$(printf '%s\n' "$guard_result" | sed -n '1p' | tr -d '\r')
target_repo_root=$(printf '%s\n' "$guard_result" | sed -n '2p' | tr -d '\r')

if [ "$verdict" != "commit" ] \
    && [ "$verdict" != "commit-uninspectable" ] \
    && [ "$verdict" != "commit-unsafe-chain" ] \
    && [ "$verdict" != "commit-stages-content" ] \
    && [ "$verdict" != "commit-uninspectable-shell" ] \
    && [ "$verdict" != "commit-complex-shell" ]; then
    if [ "$verdict" = "skip" ]; then
        exit 0
    fi
    echo "DART guard: invalid commit-detection result" >&2
    disable_guard
fi

repo_root="${target_repo_root:-${CLAUDE_PROJECT_DIR:-${CODEX_PROJECT_DIR:-$(git rev-parse --show-toplevel 2>/dev/null || pwd)}}}"

if [ "${DART_SKIP_HOOKS:-0}" = "1" ]; then
    exit 0
fi

if [ "$verdict" = "commit-complex-shell" ]; then
    echo "DART guard: complex command cannot be inspected — commit blocked." >&2
    echo "  Run git commit as its own simple command, optionally after cd/git add joined by &&." >&2
    echo "  Avoid pipes, ||, groups, subshells, conditionals, loops, functions and background jobs." >&2
    exit 2
fi

if [ -n "${DART_HOOK_DRY_RUN:-}" ]; then
    echo "DART guard (dry run): would run 'python3 scripts/check_agent_hook.py --profile staged' in $repo_root" >&2
    exit 0
fi

if [ "$verdict" = "commit-uninspectable-shell" ]; then
    echo "DART guard: shell script cannot be inspected — commit blocked." >&2
    exit 2
fi

if [ "$verdict" = "commit-unsafe-chain" ]; then
    echo "DART guard: chained commits may change staged content — commit blocked." >&2
    echo "  Split commits into separate tool calls, or let the hooks run." >&2
    exit 2
fi

if [ "$verdict" = "commit-stages-content" ]; then
    echo "DART guard: commit-time staging cannot be inspected — commit blocked." >&2
    echo "  Stage the files first, or let the hooks run." >&2
    exit 2
fi

if [ "$verdict" = "commit-uninspectable" ]; then
    echo "DART guard: commit message cannot be inspected — commit blocked." >&2
    echo "  Use -m or -F <file> without reused/autosquash sources, or let the hooks run." >&2
    exit 2
fi

if [ -f "$repo_root/scripts/check_local_paths.py" ]; then
    if ! printf '%s\n' "$guard_result" | sed -n '3p' | "$python_cmd" -c '
import json
import subprocess
import sys

message = json.load(sys.stdin)
if message:
    sys.exit(subprocess.run(
        [sys.executable, sys.argv[1], "--stdin"], input=message, text=True,
        stdout=sys.stderr,
    ).returncode)
' "$repo_root/scripts/check_local_paths.py"; then
        echo "DART guard: local-path check of supplied commit message FAILED — commit blocked." >&2
        echo "  Remove local paths from the commit message, then retry the commit." >&2
        exit 2
    fi
fi

if [ ! -f "$repo_root/scripts/check_agent_hook.py" ]; then
    echo "DART guard: full agent gate unavailable in this worktree; running staged diff fallback" >&2
    if ! git -C "$repo_root" -c core.whitespace=cr-at-eol diff --cached --check >&2; then
        echo "DART guard: staged diff check FAILED — commit blocked." >&2
        exit 2
    fi
    exit 0
fi

# Alternate GIT_INDEX_FILE indexes are out of scope; PR Text scans the resulting
# commits with the base checker instead of relying on this staged gate.
if ! "$python_cmd" -c 'import tomllib' >/dev/null 2>&1; then
    echo "DART guard: compatible Python unavailable; running staged diff fallback" >&2
    if ! git -C "$repo_root" -c core.whitespace=cr-at-eol diff --cached --check >&2; then
        echo "DART guard: staged diff check FAILED — commit blocked." >&2
        exit 2
    fi
    exit 0
fi

if ! (cd "$repo_root" && "$python_cmd" scripts/check_agent_hook.py --profile staged >&2); then
    echo "" >&2
    echo "DART guard: 'python3 scripts/check_agent_hook.py --profile staged' FAILED — commit blocked." >&2
    echo "  Run 'pixi run lint', re-stage, then retry the commit." >&2
    echo "  One-time install of the git hook: pixi run install-hooks" >&2
    exit 2
fi

exit 0
