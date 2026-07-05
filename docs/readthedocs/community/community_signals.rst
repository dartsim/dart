Community Signals
=================

DART's public activity signals are summarized in a small dashboard that is
refreshed from GitHub Actions. Read these as project activity and adoption
signals, not as a de-duplicated user count.

Open the dashboard directly:
`https://dartsim.github.io/dart/community-signals/
<https://dartsim.github.io/dart/community-signals/>`_

The live dashboard is embedded below. If it does not load, open it directly
with the link above.

.. raw:: html

   <iframe src="https://dartsim.github.io/dart/community-signals/"
           title="DART community signals dashboard"
           width="100%" height="1100" loading="lazy"
           style="border: 1px solid #d0d7de; border-radius: 6px;"></iframe>

The dashboard is published from GitHub Actions to GitHub Pages. New or updated
dashboard content appears after the publishing workflow and Pages rebuild have
completed.

What it tracks
--------------

* **Repository activity**: releases, pushes, stars, forks, contributors,
  issue/PR backlog, and recent PR activity.
* **Repository traffic**: clone and view windows when the scheduled collector
  has permission to read GitHub traffic.
* **Package reach**: conda-forge, dartpy/PyPI/PePy, and Homebrew download or
  install events.
* **Research visibility**: indexed citation counts for the DART JOSS paper.
* **Downstream evidence**: public projects such as Gazebo, RobotDART, Aikido,
  Nimble Physics, MASS, SimBenchmark, gym-dart, and pydart2 with freshness
  status.

Boundary
--------

The dashboard intentionally reports activity signals rather than a single user
total. Clones, downloads, installs, stars, citations, and downstream references
can overlap and are not unique people.
