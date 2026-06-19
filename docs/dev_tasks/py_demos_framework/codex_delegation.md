# Codex Delegation Playbook — py-demos Framework

This task runs as **orchestrator (this session) + Codex (implementer)**. The
orchestrator diagnoses, scopes, and verifies; Codex executes laser-focused,
well-defined units in this same working tree (`task_8`).

## When to delegate vs. do inline

- **Delegate to Codex:** a single, well-specified fix with a clear acceptance
  check — e.g. "scene X raises `<error>` at build/step; make it build + step for
  3 frames without raising, without changing unrelated scenes."
- **Keep inline (orchestrator):** diagnosis, triage, cross-scene decisions,
  registry/runner contract changes, anything ambiguous, and **all verification**.

## Invocation

```bash
codex exec -s workspace-write -C /home/jeongseok/dev/jslee02/dartsim/dart/task_8 \
  -o /tmp/codex_<slug>.out "<prompt>"
```

- `-s workspace-write`: Codex may edit the repo but not escape the workspace.
- `-o <file>`: capture Codex's final message. **Do not trust this summary** —
  verify artifacts directly (see below). Run several independent fixes by
  launching multiple `codex exec` calls (different `-o` files) in parallel.

## Prompt template (one fix per call)

```
You are fixing ONE DART Python demo scene in this repo. Work only in
python/examples/demos/ unless the fix demands a shared change (flag it if so).

Scene id: <id>   (file: python/examples/demos/scenes/<file>.py)
Symptom (from the no-crash smoke):
<paste the exact traceback / exit status>

Requirements:
- Make this scene build, step 3 frames, and run its renderable/debug providers
  WITHOUT raising, under both CPU and CUDA.
- Do NOT change any other scene, the registry, or the runner contract unless
  strictly required; if required, explain why in your final message.
- Match the surrounding scene's style and the SceneSetup/ScenePanel contract.
- If the scene genuinely cannot run yet (missing engine feature), do NOT fake
  it: say so and leave it for quarantine instead.

Verify before finishing:
  PYTHONPATH=build/cuda/cpp/Release-docking/python:python \
    .pixi/envs/cuda/bin/python scripts/py_demos_smoke.py --only <id> --frames 3
Report the exact command output in your final message.
```

## Mandatory orchestrator verification (after every Codex run)

Never accept Codex's word. Re-run the check yourself:

```bash
PYTHONPATH=build/cuda/cpp/Release-docking/python:python \
  .pixi/envs/cuda/bin/python scripts/py_demos_smoke.py --only <id> --frames 3
git -C . diff --stat        # confirm the change is scoped to the claimed files
```

A fix is accepted only when: (1) the smoke reports `ok` for the scene, (2) the
diff touches only the intended file(s), and (3) no other scene regressed
(re-run the full smoke before concluding a batch).

## Batch discipline

1. Group failures by root cause; one Codex call per independent fix.
2. Launch independent fixes in parallel; collect outputs.
3. Re-run the **full** smoke after a batch — a per-scene fix can regress a peer
   if it touched a shared helper.
4. Record accepted/deferred outcomes in `RESUME.md`.
