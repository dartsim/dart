# User Guide

```{raw} html
<div class="ug-hero">
  <span class="ug-hero__kicker">DART 7 · Python-first</span>
  <h2 style="color:#ffffff;border:0;margin-top:0;">Build a simulation, step by step</h2>
  <p>
    This guide takes you from a one-file "hello, DART" simulation to building
    articulated robots, tuning contact, and visualizing results. Pages are
    ordered to be read start to finish, but each stands on its own once you know
    the basics.
  </p>
</div>
```

DART 7 is the in-progress redesign of the Dynamic Animation and Robotics
Toolkit. It keeps DART's transparent, research-grade dynamics while exposing a
smaller, Python-first API built around a single `World` object. This guide uses
that API throughout.

```{admonition} DART 7 is under active development
:class: warning

The DART 7 API shown here is still evolving and is **not yet recommended for
production**. Names and behavior can change between releases. For production
work, use [DART 6 LTS](https://dart.readthedocs.io/en/stable/). If something in
this guide does not match your build, please
[open an issue](https://github.com/dartsim/dart/issues).
```

## Where to start

```{raw} html
<div class="ug-card-grid">
  <div class="ug-card">
    <span class="ug-card__step">Start here</span>
    <div class="ug-card__title"><a href="getting_started/installation.html">Install DART</a></div>
    <p>Add <code>dartpy</code> to a Python environment, or build from source for the newest DART 7 surface.</p>
  </div>
  <div class="ug-card">
    <span class="ug-card__step">First steps</span>
    <div class="ug-card__title"><a href="getting_started/hello_dart.html">Hello, DART</a></div>
    <p>Drop a box onto the ground in a dozen lines and read back its motion.</p>
  </div>
  <div class="ug-card">
    <span class="ug-card__step">First steps</span>
    <div class="ug-card__title"><a href="getting_started/simulation_loop.html">The simulation loop</a></div>
    <p>Time steps, gravity, and what actually happens on each <code>world.step()</code>.</p>
  </div>
  <div class="ug-card">
    <span class="ug-card__step">Core concepts</span>
    <div class="ug-card__title"><a href="concepts/world.html">The World</a></div>
    <p>The single entry point that owns bodies, time, and the step pipeline.</p>
  </div>
  <div class="ug-card">
    <span class="ug-card__step">Core concepts</span>
    <div class="ug-card__title"><a href="concepts/rigid_bodies.html">Rigid bodies &amp; shapes</a></div>
    <p>Mass, pose, collision shapes, and surface material for single bodies.</p>
  </div>
  <div class="ug-card">
    <span class="ug-card__step">Core concepts</span>
    <div class="ug-card__title"><a href="concepts/articulated_systems.html">Articulated systems</a></div>
    <p>Build multibodies from links and joints — the heart of robot modeling.</p>
  </div>
  <div class="ug-card">
    <span class="ug-card__step">Going further</span>
    <div class="ug-card__title"><a href="interaction/collisions_and_contacts.html">Collisions &amp; contacts</a></div>
    <p>How DART finds contacts and resolves them so bodies don't interpenetrate.</p>
  </div>
  <div class="ug-card">
    <span class="ug-card__step">Going further</span>
    <div class="ug-card__title"><a href="interaction/solvers.html">Choosing solvers</a></div>
    <p>Pick the integration and contact methods that fit your accuracy and speed needs.</p>
  </div>
  <div class="ug-card">
    <span class="ug-card__step">Going further</span>
    <div class="ug-card__title"><a href="visualization.html">Visualization</a></div>
    <p>See your scene in the interactive viewer and capture frames headlessly.</p>
  </div>
</div>
```

## How this guide is organized

- **Getting started** — installation, your first simulation, and the loop that
  drives every DART program.
- **Core concepts** — the `World`, rigid bodies, and articulated systems you
  compose into a scene.
- **Going further** — collisions and contacts, solver choices, and
  visualization.
- **{doc}`Next steps <next_steps>`** — deeper topics, runnable examples, and API
  references.

```{toctree}
:hidden:
:caption: Getting started
:maxdepth: 1

getting_started/installation
getting_started/hello_dart
getting_started/simulation_loop
```

```{toctree}
:hidden:
:caption: Core concepts
:maxdepth: 1

concepts/world
concepts/rigid_bodies
concepts/articulated_systems
```

```{toctree}
:hidden:
:caption: Going further
:maxdepth: 1

interaction/collisions_and_contacts
interaction/solvers
visualization
next_steps
```
