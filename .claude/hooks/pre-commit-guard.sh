#!/bin/sh
# DART Claude Code PreToolUse guard for Bash `git commit` calls.
#
# Wired via .claude/settings.json (PreToolUse, matcher "Bash"). It enforces the
# Tier-0 lint gate for agent sessions even before `pixi run install-hooks` has
# been run, so an agent cannot commit past the gate by forgetting it.
#
# Contract:
#   * reads the hook JSON from stdin, extracts .tool_input.command
#   * exits 0 fast for anything that is not a `git commit` invocation
#   * for a git commit:
#       - if the git pre-commit hook is installed, exit 0 (that hook enforces;
#         avoid running the gate twice)
#       - if DART_SKIP_HOOKS=1, exit 0 (emergency bypass)
#       - otherwise run `pixi run check-lint-quick`; on failure exit 2 with a
#         concise message (exit 2 blocks the tool call and surfaces stderr)
#   * DART_HOOK_DRY_RUN=1 prints the command it would run and exits 0 (test aid)
#
# JSON is parsed with python3 (no jq dependency). No heavy subprocess runs
# unless the command is an actual git commit.

input=$(cat)

# Extract the command and decide whether it is a `git commit` in one python3
# pass. Handles env-var prefixes, `git -C dir commit`, `git -c k=v commit`, and
# && / || / ; / | command chains; ignores "commit" as a substring elsewhere.
verdict=$(printf '%s' "$input" | python3 -c '
import sys, json, re

try:
    data = json.load(sys.stdin)
except Exception:
    print("skip")
    sys.exit(0)

cmd = ((data.get("tool_input") or {}).get("command") or "")

OPTS_WITH_ARG = {"-C", "-c", "--git-dir", "--work-tree", "--namespace", "--exec-path"}

def is_git_commit(text):
    for part in re.split(r"&&|\|\||[;|\n]", text):
        tokens = part.split()
        i = 0
        skipped = False
        while i < len(tokens) and re.match(r"^[A-Za-z_][A-Za-z0-9_]*=", tokens[i]):
            if tokens[i] == "DART_SKIP_HOOKS=1":
                skipped = True  # command-level bypass, same as the git hook
            i += 1  # skip VAR=value env prefixes
        if skipped:
            continue
        if i >= len(tokens) or tokens[i] != "git":
            continue
        i += 1
        while i < len(tokens):  # skip git global options and their args
            t = tokens[i]
            if t in OPTS_WITH_ARG:
                i += 2
                continue
            if t.startswith("-"):
                i += 1
                continue
            break
        if i < len(tokens) and tokens[i] == "commit":
            return True
    return False

print("commit" if is_git_commit(cmd) else "skip")
')

if [ "$verdict" != "commit" ]; then
    exit 0
fi

repo_root="${CLAUDE_PROJECT_DIR:-$(git rev-parse --show-toplevel 2>/dev/null || pwd)}"

# If the git pre-commit hook is installed, let it enforce; don't double-run.
hook_path=$(git -C "$repo_root" rev-parse --git-path hooks/pre-commit 2>/dev/null)
if [ -n "$hook_path" ]; then
    case "$hook_path" in
        /*) : ;;
        *) hook_path="$repo_root/$hook_path" ;;
    esac
    if [ -f "$hook_path" ]; then
        exit 0
    fi
fi

if [ "${DART_SKIP_HOOKS:-0}" = "1" ]; then
    exit 0
fi

if [ -n "${DART_HOOK_DRY_RUN:-}" ]; then
    echo "DART guard (dry run): would run 'pixi run check-lint-quick' in $repo_root" >&2
    exit 0
fi

if ! (cd "$repo_root" && pixi run check-lint-quick >&2); then
    echo "" >&2
    echo "DART guard: 'pixi run check-lint-quick' FAILED — commit blocked." >&2
    echo "  Run 'pixi run lint' to auto-fix, then retry the commit." >&2
    echo "  One-time install of the git hook: pixi run install-hooks" >&2
    echo "  Emergency bypass: set DART_SKIP_HOOKS=1." >&2
    exit 2
fi

exit 0
