# Copyright (c) 2011-2022, The DART development contributors

import pytest
import dartpy8 as dart


def test_constructors():
    pass
    engine = dart.collision.Engine('dart')
    scene = engine.create_scene()
    sphere_object = engine.create_sphere_object()


if __name__ == "__main__":
    pytest.main()
