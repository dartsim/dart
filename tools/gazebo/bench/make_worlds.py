#!/usr/bin/env python3
"""Write the Gazebo benchmark worlds derived from gz-sim's 3k_shapes.sdf.

usage: make_worlds.py <3k_shapes.sdf> <output directory>

Every world gets a real-time factor of 0, so a gz-sim server steps it as fast
as it can; the gz-physics driver ignores the factor.

    3k_shapes.sdf           the 3,003 boxes, cylinders and spheres at rest
    3k_shapes_drop5cm.sdf   the same bodies released 5 cm above the ground
    6k_shapes.sdf           a second copy of the bodies, 4.5 m further along +y
    3k_shapes_pendulum.sdf  plus one contact-free pendulum 500 m away, which
                            never rests
"""

import copy
import math
import pathlib
import sys
import xml.etree.ElementTree as ET

PENDULUM = """
<model name="pendulum">
  <pose>0 500 2 0 0 0</pose>
  <link name="bob">
    <inertial>
      <mass>1</mass>
      <inertia>
        <ixx>0.004</ixx><iyy>0.004</iyy><izz>0.004</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry><sphere><radius>0.1</radius></sphere></geometry>
    </collision>
  </link>
  <joint name="pivot" type="revolute">
    <parent>world</parent>
    <child>bob</child>
    <pose>-0.5 0 0 0 0 0</pose>
    <axis><xyz>0 1 0</xyz></axis>
  </joint>
</model>
"""


def bodies(world):
    return [m for m in world.findall("model") if m.findtext("static", "") != "true"]


def move(model, dy=0.0, dz=0.0):
    pose = model.find("pose")
    values = [float(v) for v in pose.text.split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError(f"{model.get('name')}: pose must contain six finite numbers")
    values[1] += dy
    values[2] += dz
    pose.text = " ".join(f"{v:.15g}" for v in values)


def drop(world):
    for model in bodies(world):
        move(model, dz=0.05)


def double(world):
    for model in bodies(world):
        twin = copy.deepcopy(model)
        shape, index = model.get("name").rsplit("_", 1)
        twin.set("name", f"{shape}_dup{index}")
        move(twin, dy=4.5)
        world.append(twin)


def add_pendulum(world):
    world.append(ET.fromstring(PENDULUM))


VARIANTS = {
    "3k_shapes.sdf": None,
    "3k_shapes_drop5cm.sdf": drop,
    "6k_shapes.sdf": double,
    "3k_shapes_pendulum.sdf": add_pendulum,
}


def main(argv):
    if len(argv) != 3:
        print(__doc__.split("\n\n")[1], file=sys.stderr)
        return 2
    source, out_dir = pathlib.Path(argv[1]), pathlib.Path(argv[2])
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, change in VARIANTS.items():
        tree = ET.parse(source)
        world = tree.getroot().find("world")
        world.find("physics").find("real_time_factor").text = "0"
        count = len(bodies(world))
        if count != 3003:
            print(f"{source}: expected 3003 bodies, found {count}", file=sys.stderr)
            return 1
        if change:
            try:
                change(world)
            except ValueError as error:
                print(f"{source}: {error}", file=sys.stderr)
                return 2
        tree.write(out_dir / name, xml_declaration=True, encoding="unicode")
        print(f"wrote {out_dir / name} ({len(bodies(world))} non-static models)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
