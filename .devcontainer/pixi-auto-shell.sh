# Auto-enter the pixi environment for interactive shells.
# Skips root, non-interactive sessions, and re-entry loops.

case "$-" in
    *i*) ;;
    *) return 0 ;;
esac

[ "$(id -u)" -eq 0 ] && return 0
[ -n "$PIXI_AUTO_SHELL" ] && return 0
command -v pixi >/dev/null 2>&1 || return 0
[ -f /workspaces/dart/pixi.toml ] || return 0

export PIXI_AUTO_SHELL=1
cd /workspaces/dart
exec pixi shell
