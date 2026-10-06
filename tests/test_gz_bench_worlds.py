"""Tests for the Gazebo benchmark world generator."""

import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "gazebo" / "bench" / "make_worlds.py"


def _load():
    spec = importlib.util.spec_from_file_location("make_worlds", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


make_worlds = _load()


def _shapes_world(path, bodies=3003):
    """A world laid out like gz-sim's 3k_shapes.sdf."""
    models = []
    for i in range(bodies):
        shape = ("box", "cylinder", "sphere")[i % 3]
        models.append(
            f'<model name="{shape}_{i // 3}"><pose>{i * 1.5} 0 0.5 0 0 0</pose>'
            '<link name="link"/></model>'
        )
    path.write_text(
        '<?xml version="1.0" ?><sdf version="1.6"><world name="shapes">'
        "<physics><max_step_size>0.001</max_step_size>"
        "<real_time_factor>1.0</real_time_factor></physics>"
        '<model name="ground_plane"><static>true</static><link name="link"/>'
        "</model>" + "".join(models) + "</world></sdf>"
    )


def _world(path):
    return ET.parse(path).getroot().find("world")


def _poses(world):
    return {
        m.get("name"): [float(v) for v in m.find("pose").text.split()]
        for m in world.findall("model")
        if m.find("pose") is not None
    }


def test_writes_the_benchmark_worlds(tmp_path):
    source = tmp_path / "3k_shapes.sdf"
    _shapes_world(source)
    out = tmp_path / "worlds"
    assert make_worlds.main(["make_worlds.py", str(source), str(out)]) == 0

    for name in make_worlds.VARIANTS:
        assert _world(out / name).find("physics").findtext("real_time_factor") == "0"

    base = _poses(_world(out / "3k_shapes.sdf"))
    assert len(base) == 3003

    dropped = _poses(_world(out / "3k_shapes_drop5cm.sdf"))
    assert all(abs(dropped[n][2] - base[n][2] - 0.05) < 1e-12 for n in base)

    doubled = _poses(_world(out / "6k_shapes.sdf"))
    assert len(doubled) == 6006
    assert doubled["sphere_dup7"][1] == base["sphere_7"][1] + 4.5

    pendulum = _world(out / "3k_shapes_pendulum.sdf")
    joint = pendulum.find("model[@name='pendulum']/joint")
    assert joint.get("type") == "revolute" and joint.findtext("parent") == "world"


def test_rejects_an_unexpected_world(tmp_path, capsys):
    source = tmp_path / "other.sdf"
    _shapes_world(source, bodies=30)
    assert make_worlds.main(["make_worlds.py", str(source), str(tmp_path)]) == 1
    assert "expected 3003 bodies, found 30" in capsys.readouterr().err


@pytest.mark.parametrize("pose", ["0 0 nan 0 0 0", "0 inf 0 0 0 0", "0 0 0"])
def test_rejects_unusable_model_poses(tmp_path, capsys, pose):
    source = tmp_path / "3k_shapes.sdf"
    _shapes_world(source)
    tree = ET.parse(source)
    tree.getroot().find("world/model[@name='box_0']/pose").text = pose
    tree.write(source)
    assert make_worlds.main(["make_worlds.py", str(source), str(tmp_path)]) == 2
    assert "pose must contain six finite numbers" in capsys.readouterr().err
