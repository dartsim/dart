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
#       - if the executable git pre-commit hook is DART-managed and the commit
#         is not using --no-verify/-n or a core.hooksPath override, exit 0
#         (that hook enforces; avoid running the gate twice)
#       - if DART_SKIP_HOOKS=1 (in the environment or as a command prefix),
#         exit 0 (emergency bypass, same as the git hook)
#       - if the commit targets another repository (`git -C /other/repo
#         commit`), exit 0 (not this gate's business)
#       - otherwise run `pixi run check-lint-quick`; on failure exit 2 with a
#         concise message (exit 2 blocks the tool call and surfaces stderr)
#   * DART_HOOK_DRY_RUN=1 prints the command it would run and exits 0 (test aid)
#   * if python3 is unavailable the guard prints a one-line notice and exits 0
#     (fail-open: a lint guard must never brick every Bash call)
#
# JSON is parsed with python3 (no jq dependency). No heavy subprocess runs
# unless the command is an actual git commit.

input=$(cat)

if ! command -v python3 >/dev/null 2>&1; then
    echo "DART guard: python3 unavailable; commit guard disabled" >&2
    exit 0
fi

# Extract the command and decide whether it is a `git commit` against THIS
# repository, in one python3 pass. Handles env-var prefixes (quote-aware),
# wrapper prefixes (command/exec/time/nice/nohup/env), subshell and brace-group
# openers, backslash-escaped git, `git -C dir commit`, `git -c k=v commit`,
# hook overrides via `--config-env`, and && / || / ; / | command chains; ignores
# "commit" as a substring elsewhere.
verdict=$(printf '%s' "$input" | python3 -c '
import json
import os
import re
import subprocess
import sys

try:
    data = json.load(sys.stdin)
except Exception:
    print("skip")
    sys.exit(0)

cmd = ((data.get("tool_input") or {}).get("command") or "")

OPTS_WITH_ARG = {
    "-C",
    "-c",
    "--config-env",
    "--git-dir",
    "--work-tree",
    "--namespace",
    "--exec-path",
}
CONFIG_ENV_PREFIX = "--config-env="
WRAPPERS = {"command", "exec", "time", "nice", "nohup"}
ENV_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*=(?P<value>.*)$")


def skip_env_prefix(tokens, i):
    """Skip VAR=value prefixes (quote-aware); return (i, dart_skip_seen)."""
    bypass = False
    while i < len(tokens):
        m = ENV_RE.match(tokens[i])
        if not m:
            break
        if tokens[i] == "DART_SKIP_HOOKS=1":
            bypass = True
        value = m.group("value")
        for quote in ("\"", "'\''"):
            if value.startswith(quote) and not (
                len(value) > 1 and value.endswith(quote)
            ):
                # quoted value with spaces spans tokens: consume to close quote
                i += 1
                while i < len(tokens) and not tokens[i].endswith(quote):
                    i += 1
                break
        i += 1
    return i, bypass


def strip_outer_quotes(value):
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'\''":
        return value[1:-1], value[0]
    return value, ""


def shell_expand_path_token(value):
    path, quote = strip_outer_quotes(value)
    if quote != "'\''":
        path = os.path.expandvars(path)
    return os.path.expanduser(path)


def is_git_commit(text):
    for part in re.split(r"&&|\|\||[;|\n]", text):
        part = part.strip().lstrip("({").strip()
        tokens = part.split()
        i, bypass = skip_env_prefix(tokens, 0)
        if bypass:
            continue  # command-level bypass, same as the git hook
        # unwrap common wrappers: command git commit, time git commit, env X=1 git commit
        while i < len(tokens):
            head = tokens[i]
            if head in WRAPPERS:
                i += 1
                continue
            if head == "env":
                i += 1
                while i < len(tokens) and (
                    tokens[i].startswith("-") or ENV_RE.match(tokens[i])
                ):
                    i += 1
                continue
            break
        if i >= len(tokens):
            continue
        if tokens[i].lstrip("\\") != "git":
            continue
        i += 1
        target_dir = None
        hooks_path_override = False
        while i < len(tokens):
            t = tokens[i]
            if t.startswith(CONFIG_ENV_PREFIX):
                option, _ = strip_outer_quotes(t[len(CONFIG_ENV_PREFIX) :])
                if option.startswith("core.hooksPath="):
                    hooks_path_override = True
                i += 1
                continue
            if t in OPTS_WITH_ARG:
                if t == "-C" and i + 1 < len(tokens):
                    target_dir = shell_expand_path_token(tokens[i + 1])
                if t == "-c" and i + 1 < len(tokens):
                    option, _ = strip_outer_quotes(tokens[i + 1])
                    if option.startswith("core.hooksPath="):
                        hooks_path_override = True
                if t == "--config-env" and i + 1 < len(tokens):
                    option, _ = strip_outer_quotes(tokens[i + 1])
                    if option.startswith("core.hooksPath="):
                        hooks_path_override = True
                i += 2
                continue
            if t.startswith("-"):
                i += 1
                continue
            break
        if i < len(tokens) and tokens[i].rstrip(")}") == "commit":
            no_verify = any(
                t.strip(")}") in {"--no-verify", "-n"} for t in tokens[i + 1 :]
            )
            if target_dir:
                project = os.environ.get("CLAUDE_PROJECT_DIR")
                try:
                    target_top = subprocess.run(
                        ["git", "-C", target_dir, "rev-parse", "--show-toplevel"],
                        capture_output=True,
                        text=True,
                    )
                    project_top = subprocess.run(
                        ["git", "-C", project, "rev-parse", "--show-toplevel"],
                        capture_output=True,
                        text=True,
                    ) if project else None
                    if (
                        project
                        and target_top.returncode == 0
                        and project_top
                        and project_top.returncode == 0
                        and os.path.realpath(target_top.stdout.strip())
                        != os.path.realpath(project_top.stdout.strip())
                    ):
                        continue  # commit into another repository
                    if (
                        project
                        and target_top.returncode != 0
                        and not os.path.realpath(target_dir).startswith(
                            os.path.realpath(project) + os.sep
                        )
                    ):
                        continue  # non-repo path outside this project
                except OSError:
                    pass
            if no_verify:
                return "commit-no-verify"
            if hooks_path_override:
                return "commit-hooks-override"
            return "commit"
    return "skip"


print(is_git_commit(cmd))
')

if [ "$verdict" != "commit" ] \
    && [ "$verdict" != "commit-no-verify" ] \
    && [ "$verdict" != "commit-hooks-override" ]; then
    if [ -z "$verdict" ]; then
        echo "DART guard: commit detection failed; guard disabled for this call" >&2
    fi
    exit 0
fi

repo_root="${CLAUDE_PROJECT_DIR:-$(git rev-parse --show-toplevel 2>/dev/null || pwd)}"

if [ "$verdict" = "commit" ]; then
    # If the executable git pre-commit hook is DART-managed, let it enforce;
    # don't double-run. Foreign hooks are not guaranteed to include DART's lint
    # gate, and --no-verify/-n plus core.hooksPath overrides bypass even a
    # DART-managed hook.
    hook_path=$(git -C "$repo_root" rev-parse --git-path hooks/pre-commit 2>/dev/null)
    if [ -n "$hook_path" ]; then
        case "$hook_path" in
            /*) : ;;
            *) hook_path="$repo_root/$hook_path" ;;
        esac
        if [ -x "$hook_path" ] && grep -q "DART-MANAGED-HOOK" "$hook_path" 2>/dev/null; then
            exit 0
        fi
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
