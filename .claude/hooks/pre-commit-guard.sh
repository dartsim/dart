#!/bin/sh
# DART Claude Code/Codex PreToolUse guard for Bash `git commit` calls.
#
# Wired via .claude/settings.json and .codex/hooks.json. It enforces the
# fast staged-file gate for agent sessions even before `pixi run install-hooks` has
# been run, so an agent cannot commit past the gate by forgetting it.
#
# Scope:
#   Git aliases expanding to commit, git am/applypatch imports, and alternate
#   GIT_INDEX_FILE indexes are not inspected by the local hooks; PR Text checks
#   every PR commit's messages, names and added lines with the base checker.
#
# Contract:
#   * reads the hook JSON from stdin, extracts .tool_input.command or .cmd
#   * exits 0 fast for anything that is not a `git commit` invocation
#   * evaluates every git commit split by the shell tokenizer:
#       - if all executable git hooks are current DART-managed hooks and the
#         commit is not using --no-verify/-n (including accepted abbreviations)
#         or a core.hooksPath override, its arguments cannot expand, and all
#         earlier segments are allowlisted with only safe redirections, skip
#         that invocation (the hooks enforce; avoid running the gate twice)
#       - if DART_SKIP_HOOKS=1 (in the environment or as a command prefix),
#         skip the affected invocations (emergency bypass, same as the git hook)
#       - if the commit certainly targets another repository (`git -C /other/repo
#         commit`), skip that invocation; uncertain cwd uses the project gates
#       - otherwise require inspectable -m or -F <file> messages; block stdin,
#         reused, autosquash and editor-only messages when managed hooks cannot enforce them
#       - block commit-time staging when managed hooks will not run; the staged
#         scan cannot inspect files that the commit command has yet to stage
#       - block multiple unhooked commits after commands that can change files
#         or the index; a single pre-tool staged scan cannot cover those changes
#       - scan all supplied messages needing enforcement with
#         `scripts/check_local_paths.py --stdin` and run the staged gate once:
#         `scripts/check_agent_hook.py --profile staged`; on failure
#         exit 2 with a concise message (exit 2 blocks the tool call and surfaces stderr)
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
    *commit*|*[gG]*[iI]*[tT]*[cC]*[oO]*[mM]*[mM]*[iI]*[tT]*|*'$'*|*'`'*) ;;
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
WRAPPERS = {"command", "exec", "time", "nice", "nohup", "timeout"}
SHELLS = {"bash", "sh", "zsh", "dash", "ksh"}
NON_POSIX_SHELLS = {"powershell", "pwsh", "cmd"}
MAX_SHELL_DEPTH = 8


class UninspectableShellScript(Exception):
    pass


READ_ONLY_GIT_COMMANDS = {
    "status", "diff", "log", "show", "rev-parse", "ls-files", "ls-tree",
}
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
SHELL_OPERATOR_RE = re.compile(
    r"&>>|<<<|<<-|;;&|&&|\|\||\|&|;;|;&|<<|>>|<&|>&|<>|>\||&>|[<>&|;(){}]"
)


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
        if line.startswith("<<<", i):
            i += 3
            continue
        if not line.startswith("<<", i):
            i += 1
            continue
        start_redirect = i
        i += 2
        strip_tabs = i < len(line) and line[i] == "-"
        if strip_tabs:
            i += 1
        while i < len(line) and line[i].isspace():
            i += 1
        if i >= len(line):
            break
        expands = line[i] not in "\"'\''\\"
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
            delimiters.append((start_redirect, i, word, strip_tabs, expands))
    return delimiters


def strip_heredoc_bodies(text, heredocs):
    lines = iter(text.splitlines(keepends=True))
    stripped = []
    for line in lines:
        redirects = heredoc_delimiters(line)
        replacements = []
        for start, end, delimiter, strip_tabs, expands in redirects:
            body = []
            complete = False
            for body_line in lines:
                if strip_tabs:
                    body_line = body_line.lstrip("\t")
                if body_line.rstrip("\r\n") == delimiter:
                    complete = True
                    break
                body.append(body_line)
            marker = f"__DART_HEREDOC_{len(heredocs)}__"
            heredocs[marker] = ("".join(body), expands, complete)
            replacements.append((start, end, marker))
        for start, end, marker in reversed(replacements):
            line = line[:start] + " <<< " + marker + " " + line[end:]
        stripped.append(line)
    return "".join(stripped)


def has_shell_expansion(text, respect_quotes=True, globs=False):
    quote = ""
    i = 0
    while i < len(text):
        ch = text[i]
        if ch == "\\" and quote != "'\''":
            i += 2
            continue
        if respect_quotes and ch in "\"'\''" and (not quote or ch == quote):
            quote = "" if quote else ch
        elif quote != "'\''" and (
            ch == "`" or ch == "$" and i + 1 < len(text)
            and (text[i + 1].isalnum() or text[i + 1] in "_({@*#?-$!")
            or globs and not quote and ch in "*?["
        ):
            return True
        i += 1
    return False


def argument_has_expansion(part, value, globs=False):
    words = re.findall(
        r"(?:[^\s\\\"'\'']|\\[\s\S]|\"(?:\\[\s\S]|[^\"\\])*\"|'\''[^'\'']*'\'')+",
        part,
    )
    matched = False
    for word in words:
        try:
            if shlex.split(word) == [value]:
                matched = True
                if has_shell_expansion(word, globs=globs):
                    return True
        except ValueError:
            return True
    return not matched and has_shell_expansion(part, globs=globs)


def text_has_commit(text):
    return re.search(r"\bgit(?:\.exe)?\b[^\n]*\bcommit\b", text) is not None


def child_shell_script(tokens, i, raw_part, heredocs, parsed):
    script = None
    dynamic = not parsed
    j = i + 1
    while j < len(tokens):
        token = tokens[j]
        if token == "<<<" and j + 1 < len(tokens):
            script = tokens[j + 1]
            dynamic |= argument_has_expansion(raw_part, script)
            if script in heredocs:
                script, expands, complete = heredocs[script]
                dynamic = not complete or expands and has_shell_expansion(
                    script, respect_quotes=False
                )
            break
        if token.startswith("-") and not token.startswith("--") and "c" in token:
            if j + 1 < len(tokens):
                script = tokens[j + 1]
                dynamic |= argument_has_expansion(raw_part, script)
            break
        if token in {"-o", "+o", "-O", "+O"}:
            j += 2
            continue
        if token == "--":
            j += 1
            if j < len(tokens) and tokens[j] != "<<<":
                break
            continue
        if not token.startswith(("-", "+")):
            break
        j += 1
    return script, dynamic


def shell_tokens(text, i=0):
    """Yield raw words and operator tokens, retaining quotes and escapes."""
    while i < len(text):
        if text[i] in " \t\r\n":
            i += 1
            continue
        start = i
        operator = SHELL_OPERATOR_RE.match(text, i)
        if operator:
            i += len(operator[0])
            yield operator[0], True, i
            continue
        quote = ""
        while i < len(text):
            ch = text[i]
            if ch == "\\" and quote != "'\''":
                i += 2
                continue
            if ch in "\"'\''" and (not quote or ch == quote):
                quote = "" if quote else ch
            elif not quote and ch in " \t\r\n<>&|;(){}":
                break
            i += 1
        yield text[start:i], False, i


def split_shell_segments(text, heredocs):
    # Child shell syntax belongs to extracted arguments; this layer is Bash.
    text = strip_heredoc_bodies(text, heredocs)
    part = []
    part_start = 0
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
                part.append("$DART_DYNAMIC_SCRIPT")
                contexts.append(("command-substitution", ")", quote, part, part_start))
                part = []
                part_start = i + 2
                part_isolated = True
                quote = ""
                i += 2
                continue
            if quote == "\"" and ch == "`":
                part.append("$DART_DYNAMIC_SCRIPT")
                contexts.append(("command-substitution", "`", quote, part, part_start))
                part = []
                part_start = i + 1
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
                    yield "".join(part), "", True, True, text[part_start:i]
                _, _, restore_quote, outer_part, part_start = contexts.pop()
                part = outer_part
                quote = restore_quote
            else:
                part.append("$DART_DYNAMIC_SCRIPT")
                contexts.append(("command-substitution", "`", "", part, part_start))
                part = []
                part_start = i + 1
                part_isolated = True
            i += 1
            continue
        if text.startswith("$(", i):
            part.append("$DART_DYNAMIC_SCRIPT")
            contexts.append(("command-substitution", ")", "", part, part_start))
            part = []
            part_start = i + 2
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
            contexts.append(("subshell", ")", "", part, part_start))
            part = []
            part_start = i + 1
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
            contexts.append(("function-body", "}", "", part, part_start))
            part = []
            part_start = i + 1
            part_isolated = True
            i += 1
            continue
        if (
            ch == "{"
            and (i == 0 or text[i - 1].isspace() or text[i - 1] in ";|&()")
            and (i + 1 == len(text) or text[i + 1].isspace())
        ):
            contexts.append(("command-group", "}", "", part, part_start))
            part = []
            part_start = i + 1
            part_isolated = True
            i += 1
            continue
        if contexts and ch == contexts[-1][1]:
            context_kind = contexts[-1][0]
            if part and context_kind != "function-body":
                yield "".join(part), "", True, True, text[part_start:i]
            _, _, restore_quote, outer_part, part_start = contexts.pop()
            part = outer_part
            quote = restore_quote
            part_isolated = True
            i += 1
            continue
        separator = ""
        if ch in "<>&|;":
            operator, is_operator, end = next(shell_tokens(text, i))
            if is_operator and operator in {
                "<", ">", "<<", "<<-", "<<<", ">>", "<&", ">&", "<>", ">|", "&>", "&>>",
            }:
                part.append(" <<< " if operator == "<<<" else operator)
                i = end
                continue
            separator = operator
        elif ch == "\n":
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
                text[part_start:i],
            )
            part = []
            part_start = i + len(separator)
            part_isolated = bool(contexts)
            i += len(separator)
            continue
        part.append(ch)
        i += 1
    yield (
        "".join(part), "", part_isolated or bool(contexts), bool(contexts),
        text[part_start:],
    )


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


def env_may_load_hookspath_config(env, target_dir):
    if not any(name in env for name in GIT_CONFIG_FILE_ENV):
        return False
    if any(
        name in env and env[name] != os.environ.get(name)
        for name in GIT_CONFIG_FILE_ENV
    ):
        return True
    # Ordinary inherited HOME/config settings need not disable managed hooks.
    result = subprocess.run(
        ["git", "-C", target_dir or os.getcwd(), "config", "--get", "core.hooksPath"],
        env=env, capture_output=True,
    )
    return result.returncode != 1


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
    if execution == EXEC_NEVER:
        return current_cwd
    # Context-dependent cd must never leave a foreign cwd falsely certain.
    if subshell_like or separator in {"&", "|", "|&"}:
        return None
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
        # Git accepts unique long-option prefixes; "--no-ver" is ambiguous.
        if t == "-n" or (t.startswith("--no-veri") and "--no-verify".startswith(t)):
            return True
        if t.startswith("--") or not t.startswith("-") or t == "-":
            continue
        for option in t[1:]:
            if option == "n":
                return True
            if option in COMMIT_SHORT_OPTS_WITH_ATTACHED_ARG:
                break
    return False


def supplied_commit_message(args, cwd, inspect_message=True, raw_part=""):
    # Track the message source separately: trailers alone still need an editor.
    messages = []
    supplied = False
    uninspectable = False
    stages_content = False
    i = 0
    while i < len(args):
        token = args[i]
        value_word = token
        i += 1
        if token == "--":
            stages_content |= bool(args[i:])
            break
        option, sep, value = token.partition("=")
        for name in ("--fixup", "--squash"):
            if option.startswith("--") and name.startswith(option):
                option = name
                break
        if (
            token.startswith("--")
            and any(name.startswith(option) for name in ("--all", "--include", "--only"))
        ):
            stages_content = True
            continue
        if option in {
            "--message",
            "--file",
            "--reuse-message",
            "--reedit-message",
            "--author",
            "--date",
            "--cleanup",
            "--template",
            "--fixup",
            "--squash",
            "--trailer",
            "--pathspec-from-file",
        }:
            if not sep:
                if i >= len(args):
                    continue
                value = args[i]
                value_word = args[i]
                i += 1
        elif token.startswith("-") and not token.startswith("--"):
            option = ""
            for offset, short_option in enumerate(token[1:], 2):
                if short_option in {"a", "i", "o"}:
                    stages_content = True
                if short_option in COMMIT_SHORT_OPTS_WITH_ATTACHED_ARG:
                    option = "-" + short_option
                    value = token[offset:]
                    if (
                        not value
                        and short_option not in {"S", "u", "U"}
                        and i < len(args)
                    ):
                        value = args[i]
                        value_word = args[i]
                        i += 1
                    break
        else:
            stages_content |= (
                not token.startswith("-") and bool(token.rstrip(")}"))
                and not argument_has_expansion(raw_part, token, globs=True)
            )
            continue
        if option == "--pathspec-from-file":
            stages_content = True
        if not inspect_message:
            continue
        if option in {"-m", "--message", "--trailer", "-F", "--file"} and argument_has_expansion(
            raw_part, value_word, globs=True
        ):
            uninspectable = True
            continue
        if option in {"-m", "--message", "--trailer"}:
            messages.append(value)
            supplied |= option != "--trailer"
        elif option in {
            "-C", "-c", "--reuse-message", "--reedit-message", "--fixup", "--squash",
        }:
            # Autosquash can reuse more than a subject (amend/reword bodies).
            uninspectable = True
        elif option in {"-F", "--file"}:
            supplied = True
            if value == "-":
                uninspectable = True
                continue
            path = shell_expand_path_token(value, cwd)
            if path:
                try:
                    with open(path, encoding="utf-8", errors="replace") as message_file:
                        messages.append(message_file.read())
                except OSError:
                    uninspectable = True
            else:
                uninspectable = True
    return "\n\n".join(messages), supplied and not uninspectable, stages_content


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
            if token in {"-a", "--append", "-p", "--portability", "-q", "--quiet", "-v", "--verbose"}:
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
    if head == "timeout":
        while i < len(tokens) and tokens[i].startswith("-"):
            if tokens[i] == "--":
                i += 1
                break
            if tokens[i] in {"--help", "--version"}:
                return None
            i += 2 if tokens[i] in {"-k", "--kill-after", "-s", "--signal"} else 1
        return i + 1  # duration precedes the wrapped command
    if head == "nohup":
        if i < len(tokens) and tokens[i] == "--":
            return i + 1
        if i < len(tokens) and tokens[i] in {"--help", "--version"}:
            return None
        return i
    return i


def command_prefix(tokens, current_cwd, inherited_env=None):
    command_env = dict(os.environ if inherited_env is None else inherited_env)
    i, bypass = skip_env_prefix(tokens, 0, command_env)
    i = skip_shell_prefixes(tokens, i)
    next_i, env_bypass = skip_env_prefix(tokens, i, command_env)
    bypass = bypass or env_bypass
    i = next_i
    command_cwd = None
    cwd_mutation_policy = "allow"
    if bypass:
        return tokens, i, bypass, command_env, command_cwd, cwd_mutation_policy
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
    return tokens, i, bypass, command_env, command_cwd, cwd_mutation_policy


def git_common_dir(path, options=(), env=None):
    result = subprocess.run(
        ["git", "-C", path, *options, "rev-parse", "--git-common-dir"],
        env=env, capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return None
    common = result.stdout.strip()
    if not os.path.isabs(common):
        common = os.path.join(path, common)
    return os.path.realpath(common)


def git_worktree_root(path, options=(), env=None):
    result = subprocess.run(
        ["git", "-C", path, *options, "rev-parse", "--show-toplevel"],
        env=env, capture_output=True,
        text=True,
    )
    return os.path.realpath(result.stdout.strip()) if result.returncode == 0 else ""


def preceding_segment_allowed(raw_part):
    """Only known commands and harmless output redirections preserve the fast path."""
    if has_shell_expansion(raw_part, globs=True):
        return False
    try:
        words = [
            (word if is_operator else shlex.split(word)[0], is_operator)
            for word, is_operator, _ in shell_tokens(raw_part)
        ]
    except ValueError:
        return False
    tokens = []
    i = 0
    while i < len(words):
        word, is_operator = words[i]
        if is_operator:
            if i + 1 >= len(words):
                return False
            target, target_is_operator = words[i + 1]
            if target_is_operator:
                return False
            if not (
                word in {">", ">>", "&>", "&>>"} and target == "/dev/null"
                or word == ">&" and target.isascii() and target.isdecimal()
            ):
                return False
            if len(tokens) > 1 and tokens[-1].isascii() and tokens[-1].isdecimal():
                tokens.pop()  # Optional output file descriptor.
            i += 2
            continue
        tokens.append(word)
        i += 1
    if not tokens:
        return not raw_part.strip()
    if tokens[0] in {"cd", "pushd", "popd", "pwd", "true", ":"}:
        return True
    if tokens[0] not in {"git", "git.exe"}:
        return False
    i = 1
    while i < len(tokens) and tokens[i].startswith("-"):
        if tokens[i] in OPTS_WITH_ARG:
            if i + 1 >= len(tokens):
                return False
            i += 2
        else:
            i += 1
    return i < len(tokens) and tokens[i] in READ_ONLY_GIT_COMMANDS | {"add", "rm", "mv"}


def git_commits(text, current_cwd=os.getcwd(), inherited_env=None,
                content_may_change=False, depth=0, fast_path_disabled=False):
    heredocs = {}
    segment_execution = EXEC_ALWAYS
    previous_separator = ""
    for (
        part,
        separator,
        isolated_context,
        separator_isolated,
        source_part,
    ) in split_shell_segments(text, heredocs):
        raw_part = part.strip()
        subshell_like = isolated_context or previous_separator in {"|", "|&"}
        part = raw_part.lstrip("({").strip()
        parsed = True
        try:
            tokens = shlex.split(part)
        except ValueError:
            parsed = False
            tokens = part.split()
        if depth and not parsed and text_has_commit(part):
            raise UninspectableShellScript
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
        (
            tokens, i, bypass, command_env, command_cwd, cwd_mutation_policy,
        ) = command_prefix(tokens, current_cwd, inherited_env)
        segment_allowed = parsed and preceding_segment_allowed(source_part)
        if not parsed:
            fast_path_disabled = True
        if bypass:
            content_may_change = True
            fast_path_disabled = True
            continue
        if i >= len(tokens):
            content_may_change |= bool(tokens)
            fast_path_disabled |= not segment_allowed
            continue
        shell = command_basename(tokens[i]).lower().removesuffix(".exe")
        if shell in NON_POSIX_SHELLS:
            # These scripts use syntax the POSIX tokenizer cannot inspect.
            script = " ".join(
                heredocs[token][0] if token in heredocs else token
                for token in tokens[i + 1 :]
            )
            if "$DART_DYNAMIC_SCRIPT" in script:
                script += "\n" + source_part
            escape = "^" if shell == "cmd" else "`"
            script = (
                script.replace(escape + "\r\n", "")
                .replace(escape + "\n", "")
                .replace(escape, "")
            )
            if text_has_commit(script.lower()):
                raise UninspectableShellScript
            # Preserve dynamic Git detection inside opaque script arguments.
            for argument in tokens[i + 1 :]:
                argument = heredocs[argument][0] if argument in heredocs else argument
                if has_shell_expansion(argument) and (
                    depth >= MAX_SHELL_DEPTH or any(git_commits(
                        argument, command_cwd or current_cwd, command_env,
                        content_may_change, depth + 1, fast_path_disabled,
                    ))
                ):
                    raise UninspectableShellScript
            content_may_change = True
            fast_path_disabled = True
            continue
        if command_basename(tokens[i]) in SHELLS:
            fast_path_disabled |= not segment_allowed
            script, dynamic = child_shell_script(
                tokens, i, raw_part, heredocs, parsed
            )
            if dynamic and text_has_commit(
                text if not parsed or "$DART_DYNAMIC_SCRIPT" in (script or "")
                else script or ""
            ):
                raise UninspectableShellScript
            if script is not None:
                if depth >= MAX_SHELL_DEPTH:
                    if text_has_commit(script):
                        raise UninspectableShellScript
                    content_may_change = True
                    fast_path_disabled = True
                else:
                    content_may_change, fast_path_disabled = yield from git_commits(
                        script, command_cwd or current_cwd, command_env,
                        content_may_change, depth + 1, fast_path_disabled,
                    )
            else:
                content_may_change = True
                fast_path_disabled = True
            continue
        expanded_executable = argument_has_expansion(raw_part, tokens[i])
        if not is_git_executable(tokens[i]) and not expanded_executable:
            content_may_change = True
            fast_path_disabled |= not segment_allowed
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
                and separator not in {"&", "|", "|&"}
                and i < len(tokens)
                and command_word(tokens[i]).rstrip(")}")
                in {"eval", "source", "."}
            ):
                current_cwd = None
            continue
        i += 1
        target_dir = None
        repository_paths = {}
        hooks_path_override = False
        while i < len(tokens):
            t = tokens[i]
            if t.startswith(CONFIG_ENV_PREFIX):
                option, _ = strip_outer_quotes(t[len(CONFIG_ENV_PREFIX) :])
                if is_hooks_path_override(option):
                    hooks_path_override = True
                i += 1
                continue
            option, sep, value = t.partition("=")
            if option in {"--git-dir", "--work-tree"}:
                if not sep and i + 1 < len(tokens):
                    i += 1
                    value = tokens[i]
                repository_paths[option] = value
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
        subcommand = command_word(tokens[i]).rstrip(")}") if i < len(tokens) else ""
        expanded_subcommand = bool(subcommand) and argument_has_expansion(raw_part, tokens[i])
        possible_commit = subcommand == "commit" or expanded_subcommand
        dynamic_command = expanded_executable or expanded_subcommand
        changed_before_commit = content_may_change
        fast_path_disabled_before_commit = fast_path_disabled or target_dir is None
        fast_path_disabled |= not segment_allowed
        if possible_commit:
            _, _, stages_content = supplied_commit_message(
                tokens[i + 1 :], target_dir, inspect_message=False
            )
            content_may_change |= stages_content
        elif subcommand not in READ_ONLY_GIT_COMMANDS:
            content_may_change = True
        # Git aliases and git am/applypatch imports are out of scope; PR Text
        # scans every resulting PR commit with the base checker.
        if possible_commit:
            repository_options = []
            for option, value in repository_paths.items():
                path = shell_expand_path_token(value, target_dir)
                if path:
                    repository_options.append(f"{option}={path}")
            for name in ("GIT_DIR", "GIT_WORK_TREE"):
                if name in command_env:
                    path = shell_expand_path_token(command_env[name], target_dir)
                    if path:
                        command_env[name] = path
            if env_config_has_hooks_path_override(command_env) or (
                env_may_load_hookspath_config(command_env, target_dir)
            ):
                hooks_path_override = True
            no_verify = commit_args_disable_hooks(tokens[i + 1 :])
            expanded_args = any(
                argument_has_expansion(raw_part, token, globs=True)
                for token in tokens[i + 1 :]
            )
            if dynamic_command and expanded_args:
                raise UninspectableShellScript
            project = (
                os.environ.get("CLAUDE_PROJECT_DIR")
                or os.environ.get("CODEX_PROJECT_DIR")
                or os.getcwd()
            )
            target_common = None
            project_common = git_common_dir(project) if project else None
            if target_dir:
                try:
                    target_common = git_common_dir(target_dir, repository_options, command_env)
                    if (
                        project
                        and target_common
                        and project_common
                        and target_common != project_common
                    ):
                        continue  # commit into another repository
                    if (
                        project
                        and target_common is None
                        and not os.path.realpath(target_dir).startswith(
                            os.path.realpath(project) + os.sep
                        )
                    ):
                        continue  # non-repo path outside this project
                except OSError:
                    pass
            gate_target_dir = target_dir or project
            target_root = git_worktree_root(
                gate_target_dir, repository_options, command_env
            )
            # An explicit Git directory can name this index from a foreign cwd.
            if (
                target_root and target_common and target_common == project_common
                and (repository_options or "GIT_DIR" in command_env)
                and git_common_dir(target_root) != target_common
            ):
                target_root = git_worktree_root(project)
            yield (
                no_verify or hooks_path_override or expanded_args or dynamic_command,
                target_root,
                tokens[i + 1 :],
                target_dir,
                changed_before_commit,
                fast_path_disabled_before_commit,
                raw_part,
            )
    return content_may_change, fast_path_disabled


def managed_hooks_current(root):
    result = subprocess.run(
        ["git", "-C", root, "rev-parse", "--git-path", "hooks/pre-commit"],
        capture_output=True, text=True,
    )
    if result.returncode != 0 or not result.stdout.strip():
        return False
    hook_path = result.stdout.strip()
    if not os.path.isabs(hook_path):
        hook_path = os.path.join(root, hook_path)
    for name, command in (
        ("pre-commit", "scripts/check_agent_hook.py --profile staged"),
        ("commit-msg", "scripts/check_local_paths.py --commit-msg-file \"$1\" --git-pid \"$PPID\""),
        ("pre-push", "scripts/check_local_paths.py --commit-range \"$commit_range\""),
    ):
        path = os.path.join(os.path.dirname(hook_path), name)
        try:
            with open(path, encoding="utf-8", errors="replace") as hook:
                content = hook.read()
        except OSError:
            return False
        if (
            not os.access(path, os.X_OK)
            or "DART-MANAGED-HOOK v12  (sentinel line: do not edit; the installer keys on it)"
            not in content
            or "if ! \"$python_cmd\" " + command + "; then" not in content
        ):
            return False
    return True


verdict, target_repo_root = "skip", ""
messages = []
unhooked_commits = 0
unsafe_chain = False
try:
    for bypassed, root, args, cwd, changed_before_commit, fast_path_disabled_before_commit, raw_part in git_commits(cmd):
        root = (
            root
            or os.environ.get("CLAUDE_PROJECT_DIR")
            or os.environ.get("CODEX_PROJECT_DIR")
            or os.getcwd()
        )
        if not bypassed and not fast_path_disabled_before_commit and managed_hooks_current(root):
            continue
        unhooked_commits += 1
        unsafe_chain |= unhooked_commits > 1 and changed_before_commit
        message, inspectable, stages_content = supplied_commit_message(args, cwd, raw_part=raw_part)
        if verdict == "skip":
            verdict, target_repo_root = "commit", root
        if stages_content:
            verdict = "commit-stages-content"
        elif not inspectable and verdict != "commit-stages-content":
            verdict = "commit-uninspectable"
        messages.append(message)
except UninspectableShellScript:
    verdict = "commit-uninspectable-shell"
if unsafe_chain and verdict != "commit-uninspectable-shell":
    verdict = "commit-unsafe-chain"
print(verdict)
print(target_repo_root)
print(json.dumps("\n\n".join(messages)))
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
    && [ "$verdict" != "commit-uninspectable" ] \
    && [ "$verdict" != "commit-unsafe-chain" ] \
    && [ "$verdict" != "commit-stages-content" ] \
    && [ "$verdict" != "commit-uninspectable-shell" ]; then
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

repo_root="${target_repo_root:-${CLAUDE_PROJECT_DIR:-${CODEX_PROJECT_DIR:-$(git rev-parse --show-toplevel 2>/dev/null || pwd)}}}"

if [ "${DART_SKIP_HOOKS:-0}" = "1" ]; then
    exit 0
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
