Who Uses DART?
==============

Software
--------

DART serves as the backend physics engine for several software projects,
including:

* `Gazebo <https://gazebosim.org/>`_: Gazebo simulates multiple robots in a 3D
  environment, with extensive dynamic integration between objects. Gazebo
  supports multiple physics engines: ODE, Bullet, DART, and Simbody. website,
  source on bitbucket
* `Aikido <https://github.com/personalrobotics/aikido>`_: a C++ library,
  complete with Python bindings, for solving robotic
  motion planning and decision making problems. This library is tightly
  integrated with DART for kinematic/dynamics calculations and OMPL for motion
  planning.
* `robot_dart <https://github.com/resibots/robot_dart>`_: A generic and
  lightweight wrapper over DART simulator for fast and flexible robot
  simulations.

Research
--------

DART has been used in various research domains, including robotics,
biomechanics, computer graphics, animation, and physics-based simulation.
Notable institutions, universities, or companies that have used DART in their
research include Georgia Tech, Oxford University, MIT, Disney Research, and
Toyota Research Institute.

DART has been utilized in research areas such as:

* Development of black-box priors for model-based policy search for robotics
* Bayesian optimization with automatic prior selection
* Alternating optimization and quadrature for robust control
* Reset-free trial-and-error learning for robot damage recovery
* Data-driven approach to simulating realistic human joint constraints
* Multi-task learning with gradient-guided policy specialization
* Learning human behaviors for robot-assisted dressing
* Expanding motor skills through relay neural networks
* Learning to navigate cloth using haptics
* Simulation-based design of dynamic controllers for humanoid balancing
* Humanoid manipulation planning using backward-forward search
* Evolutionary optimization for parameterized whole-body dynamic motor skills
* Dexterous manipulation of cloth
* Multiple contact planning for minimizing damage of humanoid falls
* Animating human dressing
* Coupling cloth and rigid bodies for dexterous manipulation
* Orienting in mid-air through configuration changes to achieve a rolling
  landing for reducing impact after a fall
* Dexterous manipulation using both palm and fingers
* Several conferences and journals where DART has been prominently featured
  include the International Conference on Robotics and Automation (ICRA), the
  AAAI Conference on Artificial Intelligence, IEEE Transactions on Evolutionary
  Computation, Computer Graphics Forum (Eurographics), and ACM Transactions on
  Graphics (presented at SIGGRAPH Asia).

As of February 2023, DART had over 777 stars on GitHub, and DART has been cited
over 236 times, indicating its widespread adoption and use in the research
community.

More research papers cited DART can be found at `Google Scholar`_.

.. note::

    If you are using DART in your project and would like to be listed here,
    please send a pull request to the GitHub repository.

.. _Google Scholar: https://scholar.google.com/scholar?oi=bibs&hl=en&cites=3727458449064418084&as_sdt=5
