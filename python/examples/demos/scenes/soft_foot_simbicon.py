"""Y-up SIMBICON push recovery with rigid or deformable Atlas feet."""

import math
import os
import re

import dartpy as dart

from ..models import soft_foot_simbicon as _model


def _atoi(text):
    # The C++ environment knobs accept a numeric prefix, as std::atoi does.
    match = re.match(r"[ \t\n\r\v\f]*([+-]?[0-9]+)", text)
    return int(match[1]) if match else 0


def _atof(text):
    match = re.match(
        r"[ \t\n\r\v\f]*([+-]?(?:"
        r"0x(?:[0-9a-f]+(?:\.[0-9a-f]*)?|\.[0-9a-f]+)(?:p[+-]?[0-9]+)?|"
        r"(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:e[+-]?[0-9]+)?|"
        r"infinity|inf|nan(?:\([a-z0-9_]*\))?))",
        text,
        re.IGNORECASE,
    )
    if match is None:
        return 0.0
    number = match[1].lower()
    if "nan" in number:
        return float("nan")
    if "0x" in number:
        try:
            return float.fromhex(number)
        except OverflowError:
            return math.copysign(math.inf, -1 if number.startswith("-") else 1)
    return float(number)


def _feet_label(feet):
    return (
        "Soft (SoftBodyNode)"
        if feet == _model.Feet.SOFT
        else "Rigid (rest-mesh control)"
    )


class _SoftFootSimbicon:
    def __init__(self, feet_selection, scripted_push_step, scripted_push_magnitude):
        self.feet_selection = feet_selection
        self.scripted_push_step = scripted_push_step
        self.scripted_push_magnitude = scripted_push_magnitude
        self.model = _model.create_model(feet_selection[0])
        self.finite = True

        self.panel = dart.viewer.Panel()
        self.feet_text = self.panel.add_text("")
        self.pending_text = self.panel.add_text("")
        self.pending_text.color = [1.0, 0.80, 0.25, 1.0]
        self.panel.add_separator()
        self.gait_text = self.panel.add_text("")
        self.height_text = self.panel.add_text("")
        self.foot_contacts_text = self.panel.add_text("")
        self.contacts_text = self.panel.add_text("")
        self.upright_text = self.panel.add_text("")
        self.push_text = self.panel.add_text("")
        self.finite_text = self.panel.add_text("State is non-finite")
        self.finite_text.color = [1.0, 0.25, 0.2, 1.0]
        self.panel.add_separator()
        self.panel.add_text(
            f"a/s push the pelvis forward/back, d/f push left/right "
            f"({_model.default_push_magnitude:.0f} N for {_model.push_steps} steps). "
            "'r' restarts the biped in place, keeping the current "
            "feet. 't' selects the other foot geometry, which needs a new world "
            "and so takes effect on the host's Reset or Rebuild.\n\n"
            "Compare the two. Both foot types collide as the same rest-pose "
            "mesh with the same rest inertia; only the soft one deforms. Soft "
            "feet spread far more contact points (about 51 vs 16 over a "
            "settled window, gate-enforced at 1.5x) and recover from larger "
            "pushes (measured 18000 N soft vs 8000 N rigid, gate-asserted "
            "soft >= rigid) -- both Jain/Liu 2011 results.",
            wrap=True,
        )
        self.pre_refresh()

    def pre_step(self):
        if (
            self.scripted_push_step > 0
            and self.model.world.getSimFrames() == self.scripted_push_step
        ):
            _model.apply_push(
                self.model,
                [0.0, 0.0, self.scripted_push_magnitude],
                _model.push_steps,
            )
        if self.finite:
            _model.prepare_step(self.model)

    def post_step(self):
        self.finite = _model.is_finite(self.model)

    def push(self, fx, fz):
        _model.apply_push(
            self.model,
            [
                fx * _model.default_push_magnitude,
                0.0,
                fz * _model.default_push_magnitude,
            ],
            _model.push_steps,
        )

    def restart(self):
        _model.reset_model(self.model)
        self.finite = True

    def toggle_feet(self):
        self.feet_selection[0] = (
            _model.Feet.RIGID
            if self.feet_selection[0] == _model.Feet.SOFT
            else _model.Feet.SOFT
        )

    def pre_refresh(self):
        model = self.model
        self.feet_text.label = f"Feet in world: {_feet_label(model.feet)}"
        self.pending_text.visible = self.feet_selection[0] != model.feet
        self.pending_text.label = (
            f"Pending: {_feet_label(self.feet_selection[0])} -- "
            "press the host's Reset or Rebuild to apply"
        )
        controller = model.controller
        self.gait_text.label = f"Gait: {controller.current_state_machine} / state {controller.current_state}"
        self.height_text.label = (
            f"Pelvis height Y: {_model.pelvis_height_y(model):.3f} m"
        )
        self.foot_contacts_text.label = (
            f"Foot contacts: {_model.foot_contact_count(model)}"
        )
        self.contacts_text.label = (
            f"Total contacts: {model.world.getLastCollisionResult().getNumContacts()}"
        )
        self.upright_text.label = (
            f"Upright: {'yes' if _model.is_upright(model) else 'NO (fallen)'}"
        )
        self.push_text.visible = model.force_duration > 0
        self.push_text.label = f"Pushing: {model.force_duration} steps left"
        self.finite_text.visible = not self.finite


def scenes():
    feet_selection = [
        (
            _model.Feet.RIGID
            if os.getenv("DART_DEMO_SOFT_FOOT_FEET") == "rigid"
            else _model.Feet.SOFT
        )
    ]
    scripted_push_step = _atoi(os.getenv("DART_DEMO_SOFT_FOOT_PUSH_STEP", "0"))
    magnitude_env = os.getenv("DART_DEMO_SOFT_FOOT_PUSH_N")
    scripted_push_magnitude = (
        _model.default_push_magnitude if magnitude_env is None else _atof(magnitude_env)
    )

    def build():
        state = _SoftFootSimbicon(
            feet_selection, scripted_push_step, scripted_push_magnitude
        )
        return dart.viewer.SceneSetup(
            state.model.world,
            pre_step=state.pre_step,
            post_step=state.post_step,
            pre_refresh=state.pre_refresh,
            key_actions=[
                dart.viewer.KeyAction(
                    "a", "Push forward (+X)", lambda: state.push(1.0, 0.0)
                ),
                dart.viewer.KeyAction(
                    "s", "Push backward (-X)", lambda: state.push(-1.0, 0.0)
                ),
                dart.viewer.KeyAction(
                    "d", "Push left (+Z)", lambda: state.push(0.0, 1.0)
                ),
                dart.viewer.KeyAction(
                    "f", "Push right (-Z)", lambda: state.push(0.0, -1.0)
                ),
                dart.viewer.KeyAction(
                    "r", "Restart biped (keeps current feet)", state.restart
                ),
                dart.viewer.KeyAction(
                    "t",
                    "Toggle feet (applies on host Reset/Rebuild)",
                    state.toggle_feet,
                ),
            ],
            panel=state.panel,
            camera_home=dart.viewer.CameraHome(
                eye=[5.14 * 1.6, 3.28 * 1.6, 6.28 * 1.6],
                center=[0.0, 0.0, 0.0],
                up=[0.0, 0.1, 0.0],
            ),
        )

    return [
        dart.viewer.Scene(
            "soft_foot_simbicon",
            title="Soft-Foot SIMBICON",
            category="Soft Bodies",
            summary=(
                "A SIMBICON-balanced Atlas biped comparing soft vs rigid foot contact "
                "under pushes (Jain/Liu push recovery)."
            ),
            factory=build,
        )
    ]
