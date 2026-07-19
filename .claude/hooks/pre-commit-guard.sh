#!/bin/sh
# DART PreToolUse guard for Codex/Claude Bash `git commit` calls.
#
# Wired via `.codex/hooks.json` and `.claude/settings.json`. It enforces the
# fast Tier-0 gate for agent sessions even before `pixi run install-hooks` has
# been run, so an agent cannot commit past the gate by forgetting it.
#
# Contract:
#   * reads the hook JSON from stdin, extracts .tool_input.command or .cmd
#   * exits 0 fast for anything that is not a `git commit` invocation
#   * for a git commit:
#       - if the executable git pre-commit hook is DART-managed and the commit
#         is not using --no-verify/-n or a core.hooksPath override, exit 0
#         (that hook enforces; avoid running the gate twice)
#       - if DART_SKIP_HOOKS=1 (in the environment or as a command prefix),
#         exit 0 (emergency bypass, same as the git hook)
#       - if the commit targets another repository (`git -C /other/repo
#         commit`), exit 0 (not this gate's business)
#       - otherwise run the selected Python interpreter with
#         `scripts/check_agent_hook.py --profile staged`; on failure exit 2 with a
#         concise message (exit 2 blocks the tool call and surfaces stderr)
#   * DART_HOOK_DRY_RUN=1 prints the command it would run and exits 0 (test aid)
#   * without an injected interpreter, unavailable python3 prints a notice and
#     exits 0; an injected interpreter fails closed
#
# JSON is parsed with the selected Python interpreter (no jq dependency). No heavy subprocess runs
# unless the command is an actual git commit.

input=$(cat)

# Fast path: ordinary non-git/non-commit Bash calls can be skipped before
# spawning the Python tokenizer (measured ~70 ms per spawn, paid on every Bash
# tool call). A real git commit may spell the subcommand through shell quote or
# backslash removal (`git com\mit`, `git com"mit"`), so the conservative sentinel
# also falls through when the raw hook JSON contains g-i-t followed by c-o-m-m-i-t
# in order. False positives still reach the tokenizer, which classifies them.
case "$input" in
    *commit*|*g*i*t*c*o*m*m*i*t*) ;;
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
if [ -z "$python_cmd" ]; then
    echo "DART guard: python3 unavailable; commit guard disabled" >&2
    exit 0
fi

# Extract the command and decide whether it is a `git commit` against THIS
# repository, in one Python pass. Handles env-var prefixes (quote-aware),
# wrapper prefixes (command/exec/time/nice/nohup/env), subshell and brace-group
# openers, backslash-escaped/path-qualified git, `git -C dir commit`,
# `git -c k=v commit`, hook overrides via `--config-env` or command-scoped Git
# config file environment, and && / || / & / ; / | command chains; ignores "commit"
# as a substring elsewhere.
guard_result=$(printf '%s' "$input" | "$python_cmd" -c '
import json
import os
import re
import shlex
import subprocess
import sys

try:
    data = json.load(sys.stdin)
except Exception:
    print("skip")
    sys.exit(0)

tool_input = data.get("tool_input") or {}
cmd = tool_input.get("command") or tool_input.get("cmd") or ""

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
SHELL_CONTROL_PREFIXES = {"!", "if", "then", "else", "elif", "while", "until", "do"}
SHELL_GROUP_OPENERS = {"(", "{"}
CWD_UNCERTAIN_PREFIXES = {"then", "else", "elif", "do"}
EXEC_ALWAYS = "always"
EXEC_NEVER = "never"
EXEC_MAYBE = "maybe"
ENV_OPTS_WITH_ARG = {
    "-C",
    "--chdir",
    "-f",
    "--file",
    "-u",
    "--unset",
    "-S",
    "--split-string",
    "-a",
    "--argv0",
}
ENV_OPTS_WITH_ARG_PREFIXES = (
    "--chdir=",
    "--file=",
    "--unset=",
    "--split-string=",
    "--argv0=",
)
ENV_CHDIR_PREFIX = "--chdir="
ENV_OPTS_NO_ARG = {
    "-",
    "-i",
    "--ignore-environment",
    "-v",
    "--debug",
    "--ignore-signal",
    "--default-signal",
    "--block-signal",
}
ENV_OPTS_NO_ARG_PREFIXES = (
    "--ignore-signal=",
    "--default-signal=",
    "--block-signal=",
)
COMMIT_SHORT_OPTS_WITH_ATTACHED_ARG = {"m", "F", "c", "C", "S", "t", "u", "U"}
GIT_CONFIG_FILE_ENV = {
    "GIT_CONFIG_GLOBAL",
    "GIT_CONFIG_SYSTEM",
    "GIT_CONFIG_NOSYSTEM",
    "HOME",
    "XDG_CONFIG_HOME",
}
ENV_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*=(?P<value>.*)$")


def record_env_assignment(env, token):
    m = ENV_RE.match(token)
    if not m:
        return
    name, _, value = token.partition("=")
    value, _ = strip_outer_quotes(value)
    env[name] = value


def is_dart_skip_assignment(token):
    m = ENV_RE.match(token)
    if not m or not token.startswith("DART_SKIP_HOOKS="):
        return False
    value = m.group("value")
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'\''":
        value = value[1:-1]
    return value == "1"


def skip_env_prefix(tokens, i, env=None):
    """Skip VAR=value prefixes (quote-aware); return (i, dart_skip_seen)."""
    bypass = False
    while i < len(tokens):
        m = ENV_RE.match(tokens[i])
        if not m:
            break
        if env is not None:
            record_env_assignment(env, tokens[i])
        if is_dart_skip_assignment(tokens[i]):
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


def is_hooks_path_override(option):
    key, sep, _ = option.partition("=")
    return bool(sep) and key.lower() == "core.hookspath"


def command_word(token):
    return token.lstrip("({").lstrip("\\")


def command_basename(token):
    return os.path.basename(command_word(token))


def is_git_executable(token):
    return command_basename(token).lower() in {"git", "git.exe"}


def skip_shell_prefixes(tokens, i):
    while i < len(tokens):
        if tokens[i] in SHELL_GROUP_OPENERS:
            i += 1
            continue
        if command_word(tokens[i]) in SHELL_CONTROL_PREFIXES:
            i += 1
            continue
        break
    return i


def segment_allows_cwd_update(tokens):
    for token in tokens:
        if token in SHELL_GROUP_OPENERS:
            continue
        return command_word(token) not in CWD_UNCERTAIN_PREFIXES
    return True


def heredoc_delimiters(line):
    delimiters = []
    i = 0
    quote = ""
    while i < len(line):
        ch = line[i]
        if quote:
            if quote == "\"" and ch == "\\":
                i += 2
                continue
            if ch == quote:
                quote = ""
            i += 1
            continue
        if ch in "\"'\''":
            quote = ch
            i += 1
            continue
        if ch == "\\":
            i += 2
            continue
        if not line.startswith("<<", i) or line.startswith("<<<", i):
            i += 1
            continue
        i += 2
        if i < len(line) and line[i] == "-":
            i += 1
        while i < len(line) and line[i].isspace():
            i += 1
        if i >= len(line):
            break
        if line[i] in "\"'\''":
            delimiter_quote = line[i]
            i += 1
            start = i
            while i < len(line) and line[i] != delimiter_quote:
                i += 1
            word = line[start:i]
            if i < len(line):
                i += 1
        else:
            if line[i] == "\\":
                i += 1
            start = i
            while i < len(line) and not line[i].isspace() and line[i] not in ";&|":
                i += 1
            word = line[start:i]
        if word:
            delimiters.append(word)
    return delimiters


def strip_heredoc_bodies(text):
    stripped = []
    pending = []
    for line in text.splitlines():
        if pending:
            if line.strip() == pending[0]:
                pending.pop(0)
            continue
        stripped.append(line)
        pending.extend(heredoc_delimiters(line))
    return "\n".join(stripped)


def split_shell_segments(text):
    text = strip_heredoc_bodies(text)
    part = []
    quote = ""
    contexts = []
    part_isolated = False
    i = 0
    while i < len(text):
        ch = text[i]
        if quote:
            if quote == "\"" and ch == "\\":
                part.append(ch)
                if i + 1 < len(text):
                    part.append(text[i + 1])
                    i += 2
                else:
                    i += 1
                continue
            if quote == "\"" and text.startswith("$(", i):
                contexts.append(("command-substitution", ")", quote, part))
                part = []
                part_isolated = True
                quote = ""
                i += 2
                continue
            if quote == "\"" and ch == "`":
                contexts.append(("command-substitution", "`", quote, part))
                part = []
                part_isolated = True
                quote = ""
                i += 1
                continue
            part.append(ch)
            if ch == quote:
                quote = ""
            i += 1
            continue
        if ch == "\\":
            part.append(ch)
            if i + 1 < len(text):
                part.append(text[i + 1])
                i += 2
            else:
                i += 1
            continue
        if ch in "\"'\''":
            quote = ch
            part.append(ch)
            i += 1
            continue
        if ch == "`":
            if contexts and contexts[-1][1] == "`":
                if part:
                    yield "".join(part), "", True, True
                _, _, restore_quote, outer_part = contexts.pop()
                part = outer_part
                quote = restore_quote
            else:
                contexts.append(("command-substitution", "`", "", part))
                part = []
                part_isolated = True
            i += 1
            continue
        if text.startswith("$(", i):
            contexts.append(("command-substitution", ")", "", part))
            part = []
            part_isolated = True
            i += 2
            continue
        if ch == "(":
            if re.search(
                r"(?:^|\s)(?:function\s+)?[A-Za-z_][A-Za-z0-9_]*\s*$",
                "".join(part),
            ) and re.match(r"\(\s*\)", text[i:]):
                part.append(ch)
                i += 1
                continue
            contexts.append(("subshell", ")", "", part))
            part = []
            part_isolated = True
            i += 1
            continue
        if ch == "{" and (
            (contexts and contexts[-1][0] == "function-body")
            or re.search(
                r"(?:^|\s)(?:[A-Za-z_][A-Za-z0-9_]*\s*\(\s*\)"
                r"|function\s+[A-Za-z_][A-Za-z0-9_]*)\s*$",
                "".join(part),
            )
        ):
            contexts.append(("function-body", "}", "", part))
            part = []
            part_isolated = True
            i += 1
            continue
        if contexts and ch == contexts[-1][1]:
            context_kind = contexts[-1][0]
            if part and context_kind != "function-body":
                yield "".join(part), "", True, True
            _, _, restore_quote, outer_part = contexts.pop()
            part = outer_part
            quote = restore_quote
            part_isolated = True
            i += 1
            continue
        separator = ""
        if text.startswith("&&", i) or text.startswith("||", i):
            separator = text[i : i + 2]
        elif ch in ";&|\n":
            separator = ch
        if separator:
            segment = (
                ""
                if any(context[0] == "function-body" for context in contexts)
                else "".join(part)
            )
            yield (
                segment,
                separator,
                part_isolated or bool(contexts),
                bool(contexts),
            )
            part = []
            part_isolated = bool(contexts)
            i += len(separator)
            continue
        part.append(ch)
        i += 1
    yield "".join(part), "", part_isolated or bool(contexts), bool(contexts)


def env_config_has_hooks_path_override(env):
    try:
        count = int(env.get("GIT_CONFIG_COUNT", "0"))
    except ValueError:
        return False
    for index in range(count):
        key = env.get(f"GIT_CONFIG_KEY_{index}", "")
        if key.lower() == "core.hookspath":
            return True
    return False


def env_may_load_hookspath_config(env):
    return any(name in env for name in GIT_CONFIG_FILE_ENV)


def split_env_split_string(value):
    try:
        return shlex.split(value)
    except ValueError:
        if "git" in value and "commit" in value:
            return ["git", "commit"]
        return []


def shell_expand_path_token(value, base_cwd=None):
    path, quote = strip_outer_quotes(value)
    if quote != "'\''":
        path = os.path.expandvars(path)
    path = os.path.expanduser(path)
    if not path:
        return None
    if not os.path.isabs(path) and base_cwd is None:
        return None
    if base_cwd and not os.path.isabs(path):
        path = os.path.join(base_cwd, path)
    return os.path.normpath(path)


def shell_cd_target(tokens, i, current_cwd):
    if i >= len(tokens) or command_word(tokens[i]) != "cd":
        return None
    args = tokens[i + 1 :]
    if args and args[0] == "--":
        args = args[1:]
    if len(args) > 1:
        return None
    if not args:
        target = os.environ.get("HOME")
        if not target:
            return None
    else:
        target = args[0]
    if target == "-":
        return None
    return shell_expand_path_token(target, current_cwd)


def known_shell_status(tokens, current_cwd):
    """Return a segment status only when it can be determined statically."""
    i, _ = skip_env_prefix(tokens, 0)
    while i < len(tokens):
        if tokens[i] in SHELL_GROUP_OPENERS:
            i += 1
            continue
        head = command_word(tokens[i])
        if head == "!":
            return None
        if head in SHELL_CONTROL_PREFIXES:
            i += 1
            continue
        break
    i, _ = skip_env_prefix(tokens, i)
    if i >= len(tokens):
        return None
    head = command_word(tokens[i]).rstrip(")}")
    if head in {":", "true"}:
        return True
    if head == "false":
        return False
    if head != "cd":
        return None
    target = shell_cd_target(tokens, i, current_cwd)
    if target is None:
        return None
    return os.path.isdir(target)


def next_segment_execution(separator, current_execution, status):
    if separator == "&&":
        if current_execution == EXEC_ALWAYS and status is True:
            return EXEC_ALWAYS
        if current_execution == EXEC_ALWAYS and status is False:
            return EXEC_NEVER
        return EXEC_MAYBE
    if separator == "||":
        if current_execution == EXEC_ALWAYS and status is False:
            return EXEC_ALWAYS
        if current_execution == EXEC_ALWAYS and status is True:
            return EXEC_NEVER
        return EXEC_MAYBE
    return EXEC_ALWAYS


def maybe_update_shell_cwd(
    tokens, i, current_cwd, separator, subshell_like, execution, mutation_policy
):
    if i >= len(tokens) or command_word(tokens[i]) != "cd":
        return current_cwd
    if execution == EXEC_NEVER or subshell_like or separator in {"&", "|"}:
        return current_cwd
    if mutation_policy != "allow":
        return None
    target = shell_cd_target(tokens, i, current_cwd)
    if target is not None and not os.path.isdir(target):
        return current_cwd
    if execution != EXEC_ALWAYS or target is None:
        return None
    return target


def commit_args_disable_hooks(args):
    for token in args:
        t = token.strip(")}")
        if t == "--":
            return False
        if t == "--no-verify" or t == "-n":
            return True
        if t.startswith("--") or not t.startswith("-") or t == "-":
            continue
        for option in t[1:]:
            if option == "n":
                return True
            if option in COMMIT_SHORT_OPTS_WITH_ATTACHED_ARG:
                break
    return False


def unwrap_wrapper(tokens, i, head):
    """Return the wrapped command index, or None when the wrapper only queries."""
    i += 1
    if head == "command":
        while i < len(tokens):
            token = tokens[i]
            if token == "--":
                return i + 1
            if token == "-p" or (
                token.startswith("-")
                and token != "-"
                and set(token[1:]) == {"p"}
            ):
                i += 1
                continue
            if token in {"-v", "-V"} or (
                token.startswith("-")
                and token != "-"
                and any(option in token[1:] for option in "vV")
            ):
                return None
            return i
        return i
    if head == "exec":
        while i < len(tokens):
            token = tokens[i]
            if token == "--":
                return i + 1
            if token in {"-a", "--argv0"}:
                i += 2
                continue
            if token.startswith("--argv0=") or token in {"-c", "-l"}:
                i += 1
                continue
            if token.startswith("-") and token != "-" and set(token[1:]) <= {
                "c",
                "l",
            }:
                i += 1
                continue
            return i
        return i
    if head == "time":
        while i < len(tokens):
            token = tokens[i]
            if token == "--":
                return i + 1
            if token in {"--help", "--version"}:
                return None
            if token in {"-f", "--format", "-o", "--output"}:
                i += 2
                continue
            if token.startswith(("--format=", "--output=")):
                i += 1
                continue
            if token in {
                "-a",
                "--append",
                "-p",
                "--portability",
                "-q",
                "--quiet",
                "-v",
                "--verbose",
            }:
                i += 1
                continue
            return i
        return i
    if head == "nice":
        while i < len(tokens):
            token = tokens[i]
            if token == "--":
                return i + 1
            if token in {"--help", "--version"}:
                return None
            if token in {"-n", "--adjustment"}:
                i += 2
                continue
            if token.startswith("--adjustment=") or re.fullmatch(r"[-+]\d+", token):
                i += 1
                continue
            return i
        return i
    if head == "nohup":
        if i < len(tokens) and tokens[i] == "--":
            return i + 1
        if i < len(tokens) and tokens[i] in {"--help", "--version"}:
            return None
        return i
    return i


def is_git_commit(text):
    current_cwd = os.getcwd()
    segment_execution = EXEC_ALWAYS
    previous_separator = ""
    for (
        part,
        separator,
        isolated_context,
        separator_isolated,
    ) in split_shell_segments(text):
        raw_part = part.strip()
        subshell_like = isolated_context or previous_separator == "|"
        part = raw_part.lstrip("({").strip()
        try:
            tokens = shlex.split(part)
        except ValueError:
            tokens = part.split()
        cwd_execution = segment_execution
        if not segment_allows_cwd_update(tokens) and cwd_execution != EXEC_NEVER:
            cwd_execution = EXEC_MAYBE
        if not separator_isolated:
            status = (
                None
                if isolated_context
                else known_shell_status(tokens, current_cwd)
            )
            segment_execution = next_segment_execution(
                separator, cwd_execution, status
            )
        previous_separator = separator
        command_env = {}
        i, bypass = skip_env_prefix(tokens, 0, command_env)
        i = skip_shell_prefixes(tokens, i)
        next_i, env_bypass = skip_env_prefix(tokens, i, command_env)
        bypass = bypass or env_bypass
        i = next_i
        command_cwd = None
        cwd_mutation_policy = "allow"
        if bypass:
            continue  # command-level bypass, same as the git hook
        # unwrap common wrappers: command git commit, time git commit, env X=1 git commit
        while i < len(tokens):
            i = skip_shell_prefixes(tokens, i)
            next_i, env_bypass = skip_env_prefix(tokens, i, command_env)
            bypass = bypass or env_bypass
            i = next_i
            if bypass or i >= len(tokens):
                break
            head = command_basename(tokens[i])
            if head == "builtin" and command_word(tokens[i]) == "builtin":
                builtin_i = i + 1
                if builtin_i < len(tokens) and tokens[builtin_i] == "--":
                    builtin_i += 1
                if builtin_i < len(tokens) and command_word(tokens[builtin_i]) in {
                    "cd",
                    "eval",
                    "source",
                    ".",
                }:
                    i = builtin_i
                    continue
                break
            if head in WRAPPERS:
                if not (
                    head == "command" and command_word(tokens[i]) == "command"
                ):
                    cwd_mutation_policy = "unknown"
                i = unwrap_wrapper(tokens, i, head)
                if i is None:
                    i = len(tokens)
                    break
                continue
            if head == "env":
                cwd_mutation_policy = "unknown"
                i += 1
                while i < len(tokens):
                    t = tokens[i]
                    if t == "--":
                        i += 1
                        break
                    if t in {"-S", "--split-string"} and i + 1 < len(tokens):
                        split_tokens = split_env_split_string(tokens[i + 1])
                        tokens = tokens[:i] + split_tokens + tokens[i + 2 :]
                        continue
                    if t.startswith("--split-string="):
                        split_tokens = split_env_split_string(
                            t[len("--split-string=") :]
                        )
                        tokens = tokens[:i] + split_tokens + tokens[i + 1 :]
                        continue
                    if t in ENV_OPTS_WITH_ARG:
                        if t in {"-C", "--chdir"} and i + 1 < len(tokens):
                            command_cwd = shell_expand_path_token(
                                tokens[i + 1], current_cwd
                            )
                        i += 2
                        continue
                    if t.startswith(ENV_CHDIR_PREFIX):
                        command_cwd = shell_expand_path_token(
                            t[len(ENV_CHDIR_PREFIX) :], current_cwd
                        )
                        i += 1
                        continue
                    if any(
                        t.startswith(prefix)
                        for prefix in ENV_OPTS_WITH_ARG_PREFIXES
                    ):
                        i += 1
                        continue
                    if t in ENV_OPTS_NO_ARG or any(
                        t.startswith(prefix)
                        for prefix in ENV_OPTS_NO_ARG_PREFIXES
                    ):
                        i += 1
                        continue
                    next_i, env_bypass = skip_env_prefix(tokens, i, command_env)
                    if next_i == i:
                        break
                    bypass = bypass or env_bypass
                    i = next_i
                if bypass:
                    break
                continue
            break
        if bypass:
            continue
        if i >= len(tokens):
            continue
        if not is_git_executable(tokens[i]):
            current_cwd = maybe_update_shell_cwd(
                tokens,
                i,
                current_cwd,
                separator,
                subshell_like,
                cwd_execution,
                cwd_mutation_policy,
            )
            if (
                cwd_execution != EXEC_NEVER
                and not subshell_like
                and separator not in {"&", "|"}
                and i < len(tokens)
                and command_word(tokens[i]).rstrip(")}")
                in {"eval", "source", "."}
            ):
                current_cwd = None
            continue
        i += 1
        target_dir = None
        hooks_path_override = False
        while i < len(tokens):
            t = tokens[i]
            if t.startswith(CONFIG_ENV_PREFIX):
                option, _ = strip_outer_quotes(t[len(CONFIG_ENV_PREFIX) :])
                if is_hooks_path_override(option):
                    hooks_path_override = True
                i += 1
                continue
            if t in OPTS_WITH_ARG:
                if t == "-C" and i + 1 < len(tokens):
                    target_dir = shell_expand_path_token(
                        tokens[i + 1], target_dir or command_cwd or current_cwd
                    )
                if t == "-c" and i + 1 < len(tokens):
                    option, _ = strip_outer_quotes(tokens[i + 1])
                    if is_hooks_path_override(option):
                        hooks_path_override = True
                if t == "--config-env" and i + 1 < len(tokens):
                    option, _ = strip_outer_quotes(tokens[i + 1])
                    if is_hooks_path_override(option):
                        hooks_path_override = True
                i += 2
                continue
            if t.startswith("-"):
                i += 1
                continue
            break
        if not target_dir:
            target_dir = command_cwd or current_cwd
        if i < len(tokens) and command_word(tokens[i]).rstrip(")}") == "commit":
            if env_config_has_hooks_path_override(command_env) or (
                env_may_load_hookspath_config(command_env)
            ):
                hooks_path_override = True
            no_verify = commit_args_disable_hooks(tokens[i + 1 :])
            target_repo_root = ""
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
                    if target_top.returncode == 0:
                        target_repo_root = target_top.stdout.strip()
                    if (
                        project
                        and target_top.returncode == 0
                        and project_top
                        and project_top.returncode == 0
                        and os.path.realpath(target_top.stdout.strip())
                        != os.path.realpath(project_top.stdout.strip())
                    ):
                        target_common = subprocess.run(
                            [
                                "git",
                                "-C",
                                target_dir,
                                "rev-parse",
                                "--path-format=absolute",
                                "--git-common-dir",
                            ],
                            capture_output=True,
                            text=True,
                        )
                        project_common = subprocess.run(
                            [
                                "git",
                                "-C",
                                project,
                                "rev-parse",
                                "--path-format=absolute",
                                "--git-common-dir",
                            ],
                            capture_output=True,
                            text=True,
                        )
                        if not (
                            target_common.returncode == 0
                            and project_common.returncode == 0
                            and os.path.realpath(target_common.stdout.strip())
                            == os.path.realpath(project_common.stdout.strip())
                        ):
                            continue  # commit into an unrelated repository
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
                return "commit-no-verify", target_repo_root
            if hooks_path_override:
                return "commit-hooks-override", target_repo_root
            return "commit", target_repo_root
    return "skip", ""


verdict, target_repo_root = is_git_commit(cmd)
print(verdict)
print(target_repo_root)
')
guard_status=$?
if [ "$guard_status" -ne 0 ]; then
    echo "DART guard: commit detection failed" >&2
    if [ -n "${DART_HOOK_PYTHON:-}" ]; then
        exit 2
    fi
    echo "DART guard: guard disabled for this call" >&2
    exit 0
fi

verdict=$(printf '%s\n' "$guard_result" | sed -n '1p' | tr -d '\r')
target_repo_root=$(printf '%s\n' "$guard_result" | sed -n '2p' | tr -d '\r')

if [ "$verdict" != "commit" ] \
    && [ "$verdict" != "commit-no-verify" ] \
    && [ "$verdict" != "commit-hooks-override" ]; then
    if [ "$verdict" = "skip" ]; then
        exit 0
    fi
    echo "DART guard: invalid commit-detection result" >&2
    if [ -n "${DART_HOOK_PYTHON:-}" ]; then
        exit 2
    fi
    echo "DART guard: guard disabled for this call" >&2
    exit 0
fi

repo_root="${target_repo_root:-${CLAUDE_PROJECT_DIR:-$(git rev-parse --show-toplevel 2>/dev/null || pwd)}}"

if [ "$verdict" = "commit" ]; then
    # If the executable git pre-commit hook is the current DART-managed hook,
    # let it enforce; don't double-run. Stale or incomplete managed hooks and
    # foreign hooks are not guaranteed to include DART's staged gate, while
    # --no-verify/-n and core.hooksPath overrides bypass even a current hook.
    hook_path=$(git -C "$repo_root" rev-parse --git-path hooks/pre-commit 2>/dev/null)
    if [ -n "$hook_path" ]; then
        case "$hook_path" in
            /*) : ;;
            *) hook_path="$repo_root/$hook_path" ;;
        esac
        if [ -x "$hook_path" ] \
            && grep -Fq "DART-MANAGED-HOOK v5  (sentinel line: do not edit; the installer keys on it)" "$hook_path" 2>/dev/null \
            && grep -Fq 'if ! "$python_cmd" scripts/check_agent_hook.py --profile staged; then' "$hook_path" 2>/dev/null; then
            exit 0
        fi
    fi
fi

if [ "${DART_SKIP_HOOKS:-0}" = "1" ]; then
    exit 0
fi

if [ -n "${DART_HOOK_DRY_RUN:-}" ]; then
    echo "DART guard (dry run): would run 'python3 scripts/check_agent_hook.py --profile staged' in $repo_root" >&2
    exit 0
fi

if [ ! -f "$repo_root/scripts/check_agent_hook.py" ]; then
    echo "DART guard: full agent gate unavailable in this worktree; running staged diff fallback" >&2
    if ! git -C "$repo_root" diff --cached --check >&2; then
        echo "DART guard: staged diff check FAILED — commit blocked." >&2
        exit 2
    fi
    exit 0
fi

if ! "$python_cmd" -c 'import tomllib' >/dev/null 2>&1; then
    echo "DART guard: compatible Python unavailable; running staged diff fallback" >&2
    if ! git -C "$repo_root" diff --cached --check >&2; then
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
    echo "  Emergency bypass: set DART_SKIP_HOOKS=1." >&2
    exit 2
fi

exit 0
