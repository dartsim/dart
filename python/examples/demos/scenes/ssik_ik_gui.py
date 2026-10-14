"""Browse ssik analytical IK branches for 19 arm models."""

import importlib

import dartpy as dart
import numpy as np

from ..models.arm_skeleton import (
    ArmSkeleton,
    JointSpec,
    get_ssik_arm_specs,
    make_ssik_target_transform,
    ssik_target_controls,
)


def isometry(matrix):
    """Convert an ssik pose to DART's native transform."""
    matrix = np.asarray(matrix, dtype=float).reshape((4, 4))
    transform = dart.math.Isometry3()
    transform.set_rotation(matrix[:3, :3])
    transform.set_translation(matrix[:3, 3])
    return transform


def arm_chain(module):
    """Describe ssik's product-of-exponentials chain for the DART model."""
    if not all(
        hasattr(module, name)
        for name in ("_JOINT_T_LEFTS", "_JOINT_T_RIGHTS", "_JOINT_AXES", "_JOINT_TYPES")
    ):
        return []
    return [
        JointSpec(
            np.asarray(module._JOINT_T_LEFTS[i], dtype=float).reshape((4, 4)),
            np.asarray(module._JOINT_T_RIGHTS[i], dtype=float).reshape((4, 4)),
            np.asarray(module._JOINT_AXES[i], dtype=float),
            prismatic=(
                "pris" in str(module._JOINT_TYPES[i])
                or "Pris" in str(module._JOINT_TYPES[i])
            ),
        )
        for i in range(len(module._JOINT_T_LEFTS))
    ]


class _SsikState:
    def __init__(self, world, target, solved_frame):
        self.world = world
        self.target = target
        self.solved_frame = solved_frame
        self.arm = ArmSkeleton(world)
        self.arms = get_ssik_arm_specs()
        self.module = None
        self.home = target.getTransform()
        self.seed = np.zeros(0)
        self.solutions = []
        self.fk_poses = []
        self.selected = 0
        self.applied = np.eye(4)
        self.target_xyz = np.zeros(3)
        self.target_rpy = np.zeros(3)

        panel = dart.viewer.Panel()
        self.panel = panel
        self.arm_choice = panel.add_choice(
            "Arm", [arm.label for arm in self.arms], combo=True
        )
        self.arm_choice.tooltip = (
            "Robot arm to control. Each entry is an ssik prebuilt analytical IK "
            "module; switching rebuilds the arm and re-solves immediately."
        )
        self.arm_choice.on_change = self.load_arm
        self.module_text = panel.add_text("")
        self.module_text.tooltip = (
            "The Python module providing this arm's closed-form IK and kinematics."
        )
        self.chain_text = panel.add_text("")
        self.chain_text.tooltip = (
            "Kinematic chain solved: from the base link to the end-effector (tip) link."
        )
        self.dof_text = panel.add_text("")
        self.dof_text.tooltip = "Degrees of freedom: the number of independently actuated joints in the arm."
        self.unavailable = panel.add_text("ssik is not available")
        self.unavailable.color = [1.0, 0.78, 0.20, 1.0]
        self.load_error = panel.add_text("", wrap=True)
        self.install_hint = panel.add_text(
            "Install into this Pixi environment with: pixi run python -m pip install ssik",
            wrap=True,
        )
        panel.add_separator()
        self.position = panel.add_vec3("Position", [0, 0, 0], speed=0.005)
        self.position.tooltip = (
            "Desired end-effector position (x, y, z) in meters, in the arm's base "
            "frame. Drag a value to edit, or drag the gizmo in the 3D view."
        )
        self.rpy = panel.add_vec3("RPY deg", [0, 0, 0], speed=0.5)
        self.rpy.tooltip = (
            "Desired end-effector orientation as roll, pitch, yaw (degrees) "
            "about the X, Y, Z axes."
        )
        self.position.on_change = self.edit_target
        self.rpy.on_change = self.edit_target
        home = panel.add_button("Home target")
        home.tooltip = "Reset the target to the arm's home (rest) pose."
        home.on_change = self.home_target
        read = panel.add_button("Read dragged")
        read.same_line = True
        read.tooltip = "Copy the pose of the draggable gizmo in the 3D view back into these fields."
        read.on_change = self.read_target
        panel.add_separator()
        self.auto_solve = panel.add_checkbox("Auto solve", True)
        self.auto_solve.tooltip = (
            "Re-run the IK automatically whenever the target or an option changes "
            "(including while dragging the gizmo)."
        )
        self.respect_limits = panel.add_checkbox("Respect limits", True)
        self.respect_limits.tooltip = (
            "Discard solutions that violate the arm's joint limits. Turn off to "
            "also see out-of-limit branches."
        )
        self.allow_refinement = panel.add_checkbox("Allow refinement", False)
        self.allow_refinement.tooltip = (
            "Let ssik numerically refine near-singular solutions for extra "
            "accuracy, at some extra cost."
        )
        self.use_seed = panel.add_checkbox("Use q seed", False)
        self.use_seed.tooltip = (
            "Provide a seed (reference) joint configuration so solutions are "
            "ranked by closeness to it, keeping motion continuous."
        )
        self.max_solutions = panel.add_slider(
            "Max solutions", 0, min=0, max=256, integer=True
        )
        self.max_solutions.tooltip = (
            "Maximum number of IK branches to return (0 = all). Analytical IK "
            "yields several distinct elbow/wrist configurations for one target."
        )
        self.seed_metric = panel.add_choice(
            "Seed metric", ["wrap_linf", "wrap_l2"], combo=True
        )
        self.seed_metric.tooltip = (
            "How distance to the seed is measured when ranking solutions: "
            "wrap_linf = largest single-joint angular distance; wrap_l2 = "
            "Euclidean distance across joints."
        )
        for control in (
            self.auto_solve,
            self.respect_limits,
            self.allow_refinement,
            self.use_seed,
            self.max_solutions,
            self.seed_metric,
        ):
            control.on_change = self.solve_if_auto
        solve = panel.add_button("Solve")
        solve.tooltip = "Run the analytical IK once for the current target."
        solve.on_change = self.solve
        seed = panel.add_button("Seed from selection")
        seed.same_line = True
        seed.tooltip = (
            "Use the currently selected solution below as the seed configuration "
            "and enable 'Use q seed'."
        )
        seed.on_change = self.seed_from_selection
        panel.add_separator()
        self.status = panel.add_text("", wrap=True)
        self.solution_choice = panel.add_slider(
            "Solution", 0, min=0, max=0, integer=True
        )
        self.solution_choice.tooltip = (
            "Browse the IK solutions: distinct joint configurations that all "
            "reach the same target. The selected one is opaque; the others are "
            "shown translucent."
        )
        self.solution_choice.on_change = self.select_solution
        self.residual = panel.add_text("")
        self.residual.tooltip = (
            "Forward-kinematics residual: how far this solution's actual "
            "end-effector pose is from the target (smaller is better)."
        )
        self.refinement = panel.add_text("")
        self.refinement.tooltip = "Whether ssik numerically refined this solution."
        self.q_text = panel.add_text("", wrap=True)
        self.q_text.tooltip = "Joint angles of this solution in radians, one per DOF."
        self.fk_text = panel.add_text("")
        self.fk_text.tooltip = (
            "End-effector position computed by forward kinematics from q; should "
            "match the target position."
        )
        self.refresh_text()

    def target_controls(self, transform):
        xyz, rpy = ssik_target_controls(transform)
        self.target_xyz = xyz
        self.target_rpy = rpy
        self.position.value = xyz
        self.rpy.value = rpy

    def sync_target(self):
        self.target.setTransform(
            make_ssik_target_transform(self.position.value, self.rpy.value)
        )
        self.applied = self.target.getTransform().matrix().copy()

    def edit_target(self):
        with np.errstate(over="ignore"):
            xyz = np.asarray(self.position.value, dtype=np.float32)
            rpy = np.asarray(self.rpy.value, dtype=np.float32)
        changed = False
        if np.isfinite(xyz).all():
            changed |= not np.array_equal(xyz, self.target_xyz)
            self.target_xyz = xyz
        if np.isfinite(rpy).all():
            changed |= not np.array_equal(rpy, self.target_rpy)
            self.target_rpy = rpy
        self.position.value = self.target_xyz
        self.rpy.value = self.target_rpy
        if changed:
            self.sync_target()
            self.solve_if_auto()

    def home_target(self):
        self.target_controls(self.home)
        self.sync_target()
        self.solve_if_auto()

    def read_target(self):
        self.target_controls(self.target.getTransform())
        self.solve_if_auto()

    def solve_if_auto(self):
        if self.auto_solve.value:
            self.solve()

    def load_arm(self):
        self.module = None
        self.solutions = []
        self.fk_poses = []
        self.selected = 0
        arm = self.arms[self.arm_choice.value]
        self.status.label = "Loading ssik.prebuilt." + arm.module
        try:
            module = importlib.import_module("ssik.prebuilt." + arm.module)
            self.home = isometry(module.T_HOME)
            dof = int(getattr(module, "DOF", 0))
            if dof <= 0:
                raise ValueError("ssik prebuilt module reported a non-positive DOF")
            self.seed = np.zeros(dof)
            self.arm.build(arm_chain(module))
            self.module = module
        except Exception as error:  # noqa: BLE001 - mirror the native load status
            self.status.label = (
                "Install ssik in this Python environment to use this scene: "
                + str(error)
            )
            self.home = dart.math.Isometry3()
            self.home.set_translation([0.45, 0.0, 0.35])
            self.arm.build([])
            self.target_controls(self.home)
            self.sync_target()
            failed_pose = dart.math.Isometry3(self.home)
            failed_pose.set_translation(self.home.translation() + [0.0, 0.25, 0.0])
            self.solved_frame.setTransform(failed_pose)
            self.refresh_text()
            return
        self.target_controls(self.home)
        self.sync_target()
        self.solved_frame.setTransform(self.home)
        self.solve()

    def solve(self):
        if self.module is None:
            return
        dof = int(getattr(self.module, "DOF", 0))
        if self.seed.size != dof:
            self.seed = np.zeros(dof)
        kwargs = {
            "max_solutions": self.max_solutions.value or None,
            "respect_limits": self.respect_limits.value,
            "allow_refinement": self.allow_refinement.value,
            "seed_metric": self.seed_metric.options[self.seed_metric.value],
        }
        if self.use_seed.value:
            kwargs["q_seed"] = self.seed
        try:
            self.solutions = list(
                self.module.solve(self.target.getTransform().matrix(), **kwargs)
            )
            self.fk_poses = [
                (
                    isometry(self.module.fk(solution.q))
                    if hasattr(self.module, "fk")
                    else None
                )
                for solution in self.solutions
            ]
            if self.solutions:
                self.selected = min(self.selected, len(self.solutions) - 1)
            self.update_solution()
        except Exception as error:  # noqa: BLE001 - mirror the native solve status
            self.solutions = []
            self.fk_poses = []
            self.status.label = "Solve failed: " + str(error)
            self.arm.show_solutions([], -1)
            self.refresh_text()
            return
        self.status.label = f"Solutions: {len(self.solutions)}"
        if self.max_solutions.value == 0:
            self.status.label += " (all branches)"
        self.refresh_text()

    def seed_from_selection(self):
        if 0 <= self.selected < len(self.solutions):
            self.seed = np.asarray(self.solutions[self.selected].q, dtype=float).copy()
            self.use_seed.value = True
            self.solve_if_auto()

    def select_solution(self):
        self.selected = self.solution_choice.value
        self.update_solution()

    def update_solution(self):
        self.arm.show_solutions(
            [solution.q for solution in self.solutions],
            self.selected if self.solutions else -1,
        )
        if 0 <= self.selected < len(self.fk_poses):
            pose = self.fk_poses[self.selected]
            if pose is not None:
                self.solved_frame.setTransform(pose)
        self.refresh_text()

    def refresh_text(self):
        loaded = self.module is not None
        for control in (self.module_text, self.chain_text, self.dof_text):
            control.visible = loaded
        for control in (self.unavailable, self.load_error, self.install_hint):
            control.visible = not loaded
        self.load_error.label = self.status.label
        if loaded:
            self.module_text.label = (
                "Module: ssik.prebuilt." + self.arms[self.arm_choice.value].module
            )
            self.chain_text.label = (
                f"Chain: {getattr(self.module, 'BASE_LINK', '')} -> "
                f"{getattr(self.module, 'EE_LINK', '')}"
            )
            self.dof_text.label = f"DOF: {getattr(self.module, 'DOF', 0)}"
        self.solution_choice.visible = len(self.solutions) > 1
        self.solution_choice.set_range(0, max(0, len(self.solutions) - 1))
        self.solution_choice.value = self.selected
        has_solution = bool(self.solutions)
        for control in (self.residual, self.refinement, self.q_text):
            control.visible = has_solution
        self.fk_text.visible = has_solution and hasattr(self.module, "fk")
        if has_solution:
            solution = self.solutions[self.selected]
            self.residual.label = (
                f"fk residual: {getattr(solution, 'fk_residual', float('nan')):.3e}"
            )
            self.refinement.label = (
                f"refinement: {getattr(solution, 'refinement_used', '')}"
            )
            self.q_text.label = "q: " + ", ".join(f"{q:.4f}" for q in solution.q)
            if self.fk_text.visible:
                xyz = self.solved_frame.getTransform().translation()
                self.fk_text.label = "FK xyz: " + ", ".join(
                    f"{value:.3f}" for value in xyz
                )

    def pre_refresh(self):
        transform = self.target.getTransform()
        matrix = transform.matrix()
        tolerance = 1e-9 * min(np.linalg.norm(matrix), np.linalg.norm(self.applied))
        if np.linalg.norm(matrix - self.applied) > tolerance:
            self.target_controls(transform)
            self.applied = matrix.copy()
            self.solve_if_auto()
        self.refresh_text()

    def on_activate(self, ctx):
        grid = dart.gui.GridVisual()
        grid.setPlaneType(dart.gui.GridVisual.PlaneType.XY)
        ctx.add_attachment(grid)
        ctx.enable_drag_and_drop(self.target)
        self.load_arm()


def build():
    try:
        importlib.import_module("ssik")
    except ImportError as error:
        raise dart.viewer.AssetsNotPrefetchedError(
            "Install ssik in this Python environment with: "
            "pixi run python -m pip install ssik"
        ) from error
    world = dart.simulation.World()
    world.setGravity([0.0, 0.0, 0.0])
    default_target = dart.math.Isometry3()
    default_target.set_translation([0.45, 0.0, 0.35])
    target = dart.gui.InteractiveFrame(
        dart.dynamics.Frame.World(), "ssik_target", default_target, 0.25
    )
    world.addSimpleFrame(target)
    solved_pose = dart.math.Isometry3(default_target)
    solved_pose.set_translation([0.45, 0.25, 0.35])
    solved_frame = dart.dynamics.SimpleFrame(
        dart.dynamics.Frame.World(), "ssik_fk_solution", solved_pose
    )
    solved_frame.setShape(dart.dynamics.SphereShape(0.05))
    solved_frame.createVisualAspect().setColor([0.15, 0.85, 1.0, 0.85])
    world.addSimpleFrame(solved_frame)
    state = _SsikState(world, target, solved_frame)
    return dart.viewer.SceneSetup(
        world,
        panel=state.panel,
        pre_refresh=state.pre_refresh,
        on_activate=state.on_activate,
        allow_simulation=False,
        camera_home=dart.viewer.CameraHome(
            eye=[2.6, -3.0, 1.6], center=[0.1, 0.0, 0.55], up=[0.0, 0.0, 1.0]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "ssik_ik_gui",
            title="ssik Analytical IK",
            category="Control & IK",
            summary="Analytical IK against 19 prebuilt robot arms (optional ssik package).",
            factory=build,
        )
    ]
