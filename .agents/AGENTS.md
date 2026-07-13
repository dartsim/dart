# .agents/

Current Codex skill discovery surface for DART 6.20.

- Treat only paths listed by `skills/.dart-generated.json`, plus the manifest
  itself, as DART-generated output.
- Edit workflow and domain-skill sources under `.claude/`, then run
  `pixi run sync-ai-commands` and `pixi run check-ai-commands`.
- Preserve unrelated skills in this shared directory. The sync tool may remove
  only paths owned by the DART-generated manifest.
- Keep release-specific C++17, pybind11, `dart::utils`, OSG, and Gazebo
  assumptions; do not import DART 7-only workflows.
