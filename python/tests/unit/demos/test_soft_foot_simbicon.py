"""Optional soft-foot SIMBICON models stay deterministic and own their handles."""

import gc
import weakref

import numpy as np
import pytest
from dartpy import _demos

soft_foot = getattr(_demos, "soft_foot_simbicon", None)
pytestmark = pytest.mark.skipif(
    soft_foot is None, reason="dart-utils and dart-utils-urdf were compiled out"
)


@pytest.mark.parametrize("feet_name", ["RIGID", "SOFT"])
def test_soft_foot_construction_push_steps_and_determinism(feet_name):
    feet = getattr(soft_foot.Feet, feet_name)
    lhs, rhs = [soft_foot.create_model(feet) for _ in range(2)]
    assert lhs.world.getNumSkeletons() == 2
    assert lhs.world.getTimeStep() == soft_foot.time_step
    assert lhs.feet == feet
    assert lhs.pelvis.getName() == "pelvis"
    assert lhs.left_foot.getName() == "l_foot"
    assert lhs.right_foot.getName() == "r_foot"
    assert soft_foot.is_upright(lhs)
    assert soft_foot.pelvis_height_y(lhs) - soft_foot.mean_foot_height_y(lhs) > (
        soft_foot.upright_gap
    )
    assert soft_foot.push_steps > 0
    assert soft_foot.default_push_magnitude > 0
    assert 0 < soft_foot.push_sweep_start < soft_foot.push_sweep_end
    assert soft_foot.push_sweep_step > 0
    assert callable(soft_foot.max_recoverable_push)
    initial_state = soft_foot.state_vector(lhs).copy()

    for model in (lhs, rhs):
        soft_foot.apply_push(model, [10.0, 0.0, 0.0], steps=3)
    for _ in range(8):
        soft_foot.step(lhs)
        soft_foot.prepare_step(rhs)
        rhs.world.step()
    assert lhs.force_duration == 0
    assert soft_foot.is_finite(lhs)
    assert soft_foot.is_finite(rhs)
    assert soft_foot.foot_contact_count(lhs) >= 0
    state = soft_foot.state_vector(lhs)
    assert state.ndim == 1
    assert state.dtype == np.dtype(np.float64)
    np.testing.assert_array_equal(state, soft_foot.state_vector(rhs))

    soft_foot.reset_model(lhs)
    assert lhs.world.getTime() == 0
    assert lhs.force_duration == 0
    np.testing.assert_array_equal(lhs.external_force, np.zeros(3))
    np.testing.assert_array_equal(soft_foot.state_vector(lhs), initial_state)


@pytest.mark.parametrize("property_name", ["pelvis", "left_foot", "right_foot"])
def test_soft_foot_borrowed_child_keeps_model_alive(property_name):
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    body_name = {"pelvis": "pelvis", "left_foot": "l_foot", "right_foot": "r_foot"}[
        property_name
    ]
    existing = model.atlas.getBodyNode(body_name)
    child = getattr(model, property_name)
    assert child is existing
    del existing
    reference = weakref.ref(model)
    del model
    gc.collect()
    assert reference() is not None
    assert child.getName()
    soft_foot.step(reference())
    assert soft_foot.is_finite(reference())
    del child
    gc.collect()
    assert reference() is None


@pytest.mark.parametrize("force, steps", [([np.nan, 0, 0], 10), ([1, 0, 0], -1)])
def test_apply_push_rejects_invalid_input(force, steps):
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    with pytest.raises(ValueError):
        soft_foot.apply_push(model, force, steps)
    assert model.force_duration == 0
