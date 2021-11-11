import pytest
import dartpy8 as dart


def _test_collide(engine_name: str):
    engine = dart.collision.Engine(engine_name)
    sphere1 = engine.create_sphere_object()
    sphere2 = engine.create_sphere_object()
    assert sphere1
    assert sphere2

    sphere1.set_position(-1, 0, 0)
    sphere2.set_position(+1, 0, 0)
    assert not dart.collision.collide(sphere1, sphere2)

    sphere1.set_position(-0.25, 0, 0)
    sphere2.set_position(+0.25, 0, 0)
    assert dart.collision.collide(sphere1, sphere2)


def test_collide():
    _test_collide('dart')
    _test_collide('fcl')


if __name__ == "__main__":
    pytest.main()
