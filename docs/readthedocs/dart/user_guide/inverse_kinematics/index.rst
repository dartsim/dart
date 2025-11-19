Inverse Kinematics
===================

DART ships with a modular inverse kinematics (IK) framework that works the same
way across C++ and dartpy. Every end effector can own an
:class:`dart::dynamics::InverseKinematics` instance that exposes:

* A *target* frame (typically a :class:`dart::dynamics::SimpleFrame`) describing
  the desired pose
* Optional hierarchy levels so you can stack multiple objectives
* A *gradient method* implementation that numerically optimizes the joint
  vector or an *analytical* solver that produces closed-form solutions

The IK module updates the owning skeleton directly, so you can wire it into UI
handlers, controllers, or headless loops without writing glue code.

Workflow overview
-----------------

1. Create or locate a :class:`dart::dynamics::EndEffector`.
2. Call :func:`EndEffector::createIK()` to attach an IK object.
3. Configure the target, hierarchy level, error bounds, and solver tolerances.
4. Choose a gradient method (default GD, damped least squares, etc.) *or* plug
   in an analytical solver such as IkFast.
5. Call :func:`InverseKinematics::solve()` whenever you need to update the
   skeleton to satisfy the target.

For a full walkthrough see :doc:`tutorials/wholebody-ik`, which pairs the
API with runnable Atlas examples in C++ and Python.

Gradient methods
----------------

Gradient methods iterate numerically until the desired pose is met. DART
exposes multiple built-in strategies (Jacobians with pseudo-inverse, damped
least squares, biasing functions, joint limit soft constraints, etc.) that you
can mix via :func:`InverseKinematics::setGradientMethod`. The "Whole-Body IK"
tutorial shows how to:

* Clamp translation/rotation error bounds to tighten convergence
* Bias floating-base DOFs to keep the robot balanced
* Stack multiple objectives (hands, head, torso) using hierarchy levels
* Drive the solver headlessly for scripted trajectories

Analytical methods
------------------

Analytical IK solvers emit closed-form joint configurations without
iteratively optimizing a cost function. DART exposes them through
:class:`dart::dynamics::InverseKinematics::Analytical` and lets you mix them
with gradient objectives―for example, you can solve a 6-DOF manipulator
analytically while leaving the rest of the skeleton to the gradient backend.

The most common implementation is IkFast; consult the page below for current
upstream status, generation instructions, and DART's support policy.

.. toctree::
   :maxdepth: 1

   IkFast <ikfast>
