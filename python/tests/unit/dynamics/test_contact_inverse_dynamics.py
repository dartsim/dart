import dartpy as dart
import numpy as np
import pytest


def _create_floating_box(mass=2.0, size=None):
    if size is None:
        size = [0.4, 0.4, 0.2]
    skel = dart.dynamics.Skeleton("floating_box")
    [_, body] = skel.createFreeJointAndBodyNodePair()
    shape = dart.dynamics.BoxShape(size)
    body.createShapeNode(shape)
    inertia = dart.dynamics.Inertia()
    inertia.setMass(mass)
    inertia.setMoment(shape.computeInertia(mass))
    body.setInertia(inertia)
    return skel, body


def _bottom_contacts(body, friction_coeff=0.6):
    contacts = []
    for x in (0.2, -0.2):
        for y in (0.2, -0.2):
            contact = dart.dynamics.ContactInverseDynamics.Contact()
            contact.bodyNode = body
            contact.localOffset = [x, y, -0.1]
            contact.normal = [0.0, 0.0, 1.0]
            contact.frictionCoeff = friction_coeff
            contacts.append(contact)
    return contacts


def test_detects_free_root_joint_as_unactuated():
    skel, _ = _create_floating_box()
    solver = dart.dynamics.ContactInverseDynamics(skel)
    assert solver.getUnactuatedDofs() == [0, 1, 2, 3, 4, 5]


def test_static_box_contact_forces_balance_weight():
    mass = 2.0
    skel, body = _create_floating_box(mass=mass)

    solver = dart.dynamics.ContactInverseDynamics(skel)
    solver.setContacts(_bottom_contacts(body))
    result = solver.compute()

    assert result.feasible
    # The documented contract bounds the residual by tolerance * |target|_inf.
    bound = solver.getResidualTolerance() * mass * 9.81
    total = np.sum(result.contactForces, axis=0)
    np.testing.assert_allclose(total[:2], 0.0, atol=bound)
    np.testing.assert_allclose(total[2], mass * 9.81, atol=bound)
    assert np.linalg.norm(result.unactuatedResidual, ord=np.inf) <= bound


def test_infeasible_acceleration_is_reported():
    skel, body = _create_floating_box()
    accelerations = np.zeros(6)
    accelerations[3] = 3.0 * 0.2 * 9.81  # far beyond the friction limit
    skel.setAccelerations(accelerations)

    solver = dart.dynamics.ContactInverseDynamics(skel)
    solver.setContacts(_bottom_contacts(body, friction_coeff=0.2))
    result = solver.compute()

    assert not result.feasible
    assert np.linalg.norm(result.unactuatedResidual, ord=np.inf) > 1e-2


def test_round_trip_with_forward_dynamics():
    mass = 2.0
    skel, body = _create_floating_box(mass=mass)
    contacts = _bottom_contacts(body, friction_coeff=0.8)

    solver = dart.dynamics.ContactInverseDynamics(skel)
    solver.setContacts(contacts)
    result = solver.compute()
    assert result.feasible

    skel.setForces(result.jointForces)
    for contact, force in zip(contacts, result.contactForces):
        body.addExtForce(force, contact.localOffset, False, True)

    skel.computeForwardDynamics()
    recovered = skel.getAccelerations()
    np.testing.assert_allclose(recovered, np.zeros(6), atol=1e-6)


def test_contact_body_node_round_trip():
    skel, body = _create_floating_box()
    contact = dart.dynamics.ContactInverseDynamics.Contact()
    contact.bodyNode = body
    assert contact.bodyNode.getName() == body.getName()


def test_rejects_non_finite_contact_specs():
    skel, body = _create_floating_box()
    solver = dart.dynamics.ContactInverseDynamics(skel)

    contact = dart.dynamics.ContactInverseDynamics.Contact()
    contact.bodyNode = body
    contact.frictionCoeff = float("nan")
    solver.setContacts([contact])
    assert not solver.compute().feasible


def test_rejects_invalid_contacts():
    skel, body = _create_floating_box()
    other_skel, other_body = _create_floating_box()

    solver = dart.dynamics.ContactInverseDynamics(skel)
    contact = dart.dynamics.ContactInverseDynamics.Contact()
    contact.bodyNode = other_body
    solver.setContacts([contact])

    result = solver.compute()
    assert not result.feasible
    assert len(result.jointForces) == 0


if __name__ == "__main__":
    pytest.main()
