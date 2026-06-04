# Cloud Dartpy Tutorials

## Operating State

See `PLAN-012` in [`dashboard.md`](dashboard.md) for priority, state, horizon,
next step, and gate.

## Outcome

DART 7 should have official cloud-runnable `dartpy` tutorial notebooks that let
new users try the Python API without a local install. The first public target is
Google Colab because it can open notebooks directly from GitHub and matches the
MuJoCo tutorial pattern, but the notebook source should stay standard Jupyter
so the same material can run locally and on other hosted notebook services.

The tutorial path is successful when a user can open a DART-owned notebook from
GitHub, run the setup cells in a managed cloud runtime, import DART 7 `dartpy`,
step a small simulation, and display Filament-backed output inline as an image
or video. The public API should remain DART-owned; notebooks must not expose
Filament, GLFW, Dear ImGui, OpenGL, Vulkan, Metal, OSG, or Raylib as user-facing
concepts.

## Current Evidence

- [`README.md`](../../README.md#installation) says the quick-start snippets
  target `main` and DART 7, but package managers may still resolve DART 6.17
  until DART 7 package artifacts are published.
- The [PyPI `dartpy` page](https://pypi.org/project/dartpy/) listed 6.16.7 as
  the current release on 2026-05-21, with 7.0.0 yanked. Stable cloud notebooks
  therefore cannot use an unqualified `pip install dartpy` yet.
- The [Colab FAQ](https://research.google.com/colaboratory/intl/en-GB/faq.html)
  describes Colab as a hosted Jupyter service with no local setup, says
  notebooks can be loaded from GitHub, and notes that the runtime VM and custom
  installed libraries are not shared with a notebook. Tutorial notebooks need
  explicit setup cells.
- The [Colab runtime FAQ](https://research.google.com/colaboratory/runtime-version-faq.html)
  recommends using the latest runtime plus notebook-installed package versions.
  The 2026.04 runtime lists Ubuntu 22.04.5 LTS and Python 3.12.13, which should
  drive the first DART 7 wheel target for Colab proof.
- The [MuJoCo Python tutorial notebook](https://github.com/google-deepmind/mujoco/blob/main/python/tutorial.ipynb)
  demonstrates the user experience target: GitHub-hosted notebook, package
  install cell, GPU/EGL setup for rendering, and inline media output.
- `origin/feature/filament-gui-full-execution` is the relevant renderer branch.
  At inspected head `3871046d40d`, it promotes the Filament-backed GUI under
  `DART_BUILD_GUI`, documents `dartpy.gui` as the public Python GUI surface,
  and carries headless screenshot paths through the `dartsim` app and restored
  GUI examples. It does not yet prove Colab execution.
- Current `main` still tracks Filament through the active dev-task notes under
  [`docs/dev_tasks/filament_gui/`](../dev_tasks/filament_gui/) and records
  headless smoke evidence in
  [`05-testing.md`](../dev_tasks/filament_gui/05-testing.md).

## Platform Decision

Use Colab as the first public hosted tutorial target, not as the only supported
runtime.

Colab is the best first target because it needs no local setup, can load
notebooks from GitHub, is familiar to machine-learning and robotics users, and
has a successful precedent in MuJoCo's Python tutorial. It also gives DART a
simple launch URL for README and documentation pages.

Colab should not be the sole support claim because runtime resources are not
guaranteed, runtimes are ephemeral, Python and system packages change over
time, and installed libraries are not part of the shared notebook. DART should
own portable notebooks plus explicit Colab smoke evidence. If the Colab proof
becomes unreliable or too expensive, keep the same notebook source and add
secondary launch paths such as local Jupyter, GitHub Codespaces, Kaggle, Binder,
or a hosted documentation-rendered notebook.

## Workstreams

### 1. Artifact And Runtime Gate

- Publish or stage a DART 7 `dartpy` wheel that supports the current Colab
  Python runtime on Linux x86_64. The first proof should target Python 3.12
  because Colab's 2026.04 runtime reports Python 3.12.13.
- Keep the notebook setup cell version-explicit. It must fail fast if the
  imported package major version is not 7 or if `dartpy.gui` is missing.
- Do not publish a stable Colab link that silently falls back to DART 6. Until
  DART 7 is on production PyPI, use a clearly labeled release-candidate,
  TestPyPI, direct wheel, or CI artifact path for maintainer-only validation.
- Verify the wheel bundles or resolves the Filament-backed GUI dependencies
  needed for headless rendering in the cloud runtime.

### 2. Notebook Source Shape

- Add notebooks as tracked Jupyter `.ipynb` files under a DART-owned tutorial
  surface, preferably near Python tutorials, with README and documentation links
  added only after the runtime gate passes.
- Keep setup cells idempotent: install the pinned DART 7 artifact, install only
  small notebook helpers such as `matplotlib` or `mediapy` when missing, print
  the `dartpy` version, and assert the expected major version.
- Keep cells usable outside Colab by detecting Colab-specific helpers instead
  of importing `google.colab` unconditionally.
- Store tutorial assets in the repository or load them through DART package
  resource APIs. Avoid Google Drive dependencies for first-run notebooks.

### 3. Tutorial Sequence

- Start with one "first simulation" notebook that mirrors README concepts:
  import `dartpy`, create or load a small model, step the world, inspect
  positions, and show a compact plot or table.
- Add a rendering section only after the Filament-backed `dartpy.gui` path is
  available from the DART 7 artifact. The notebook should request headless
  rendering and show the resulting frame or video inline.
- Add a model-loading tutorial after the first notebook passes, using bundled
  or downloadable sample assets that work in a clean runtime.
- Keep interactive native windows out of the Colab promise. Colab success is
  inline image or video rendering from a bounded headless run.

### 4. Filament GUI Cloud Proof

- Use the Filament branch as the implementation base, but require the promoted
  API to stay backend-hidden and match the durable GUI guidance after the branch
  lands on `main`.
- Define the Python API needed by notebooks. If `dartpy.gui` only exposes
  descriptors and run-state helpers, add a small DART-owned Python rendering
  helper or documented app invocation path before writing user-facing notebook
  cells.
- Prove a nonblank image or video in Colab with a pixel/content check similar
  to the existing headless smoke analyzer, not just command success.
- Prefer software or headless rendering that works on standard Colab CPU
  runtimes. GPU-specific EGL setup can be a separate advanced notebook if it is
  required for performance.

### 5. Verification And Publication

- Add an automated local notebook smoke using a Jupyter runner such as
  `nbclient` once notebook dependencies are introduced to the Pixi environment.
  The smoke should execute the notebook setup, import, simulation, and
  rendering cells against a temporary kernel.
- Add a lightweight notebook lint/check that rejects stale DART 6 install
  commands, missing major-version assertions, unchecked `google.colab` imports,
  and saved heavyweight outputs.
- Record a manual Colab smoke before publishing each stable launch link:
  runtime version, Python version, wheel source, install command, DART version,
  import result, simulation result, and rendered image/video evidence.
- Publish links from README and Read the Docs only after the DART 7 wheel and
  Filament cloud rendering gates pass. Documentation should say that Colab is a
  cloud trial path, while local development and reproducible source builds still
  use the existing Pixi workflow.

## Open Decisions

- Whether DART 7 preview notebooks should install from TestPyPI, a GitHub
  Actions artifact, or a direct wheel URL before production PyPI publication.
- Whether the first rendering notebook should depend on a Python rendering
  helper, invoke the `dartsim` app in headless mode, or expose a more direct
  `dartpy.gui` frame-capture API.
- Whether Colab CPU software rendering is reliable enough for the first public
  rendering tutorial, or whether the first published rendering notebook should
  require a GPU runtime.
- Whether notebooks should live under `python/tutorials/`, `docs/readthedocs/`,
  or a new top-level `notebooks/` directory once implementation begins.

## Revision Triggers

- DART 7 `dartpy` wheels are published or the package source changes.
- The Filament GUI branch lands, is substantially redesigned, or exposes a
  different Python rendering surface.
- Colab changes its default Python, Ubuntu image, GPU availability, or package
  install behavior.
- A Colab smoke fails for reasons not reproducible in local Jupyter execution.
- Maintainers decide that another hosted notebook service is the primary public
  path.
