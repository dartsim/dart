#!/usr/bin/env python3
"""Qualify the pinned humanoid catalog through dartpy (explicit online gate)."""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import dartpy as dart
import numpy as np


def verify_model(model_id: str, cache_directory: str, offline: bool) -> dict:
    manifest_uri = f"dart://sample/robot_models/{model_id}.xml"
    manifest_bytes = dart.utils.DartResourceRetriever().readAll(manifest_uri).encode()
    manifest = ET.fromstring(manifest_bytes)
    base = f"model://{manifest.attrib['id']}/{manifest.attrib['revision']}/"
    models = dart.utils.ModelResourceRetriever(cache_directory, offline)
    entry_uri = base + manifest.attrib["entrypoint"]
    entry_path = models.getFilePath(entry_uri)
    assert entry_path, f"Failed to retrieve {entry_uri}"
    files = manifest.findall("file")
    for file in files:
        path = Path(models.getFilePath(base + file.attrib["path"]))
        assert path.is_file(), file.attrib["path"]
        data = path.read_bytes()
        assert len(data) == int(file.attrib["size"]), path
        assert hashlib.sha256(data).hexdigest() == file.attrib["sha256"], path
    description = ET.parse(entry_path).getroot()
    links = description.findall("link")
    results = []
    for fixed in (True, False):
        root_type = (
            dart.utils.DartLoader.RootJointType.FIXED
            if fixed
            else dart.utils.DartLoader.RootJointType.FLOATING
        )
        loader = dart.utils.DartLoader()
        loader.setOptions(dart.utils.DartLoader.Options(models, root_type))
        skeleton = loader.parseSkeleton(entry_uri)
        assert skeleton is not None, entry_uri
        assert (
            skeleton.getNumBodyNodes()
            == len(links)
            == (36 if model_id == "atlas-v5" else 38)
        )
        assert skeleton.getNumDofs() == (29 if fixed else 35)
        visual_shapes = collision_shapes = meshes = 0
        for link in links:
            body = skeleton.getBodyNode(link.attrib["name"])
            assert body is not None, link.attrib["name"]
            inertia = body.getInertia()
            moment = np.asarray(inertia.getMoment())
            assert np.isfinite(inertia.getMass()) and inertia.getMass() > 0
            assert np.isfinite(moment).all(), body.getName()
            assert np.isfinite(inertia.getLocalCOM()).all(), body.getName()
            declared = link.find("inertial")
            if declared is not None:
                mass = float(declared.find("mass").attrib["value"])
                assert np.isclose(inertia.getMass(), mass), body.getName()
                principal = np.linalg.eigvalsh(moment)
                assert principal.min() > 0, (body.getName(), principal)
                assert principal[-1] <= principal[:2].sum() + 1e-9, (
                    body.getName(),
                    principal,
                )
            nodes = body.getShapeNodes()
            assert sum(node.hasVisualAspect() for node in nodes) == len(
                link.findall("visual")
            ), body.getName()
            assert sum(node.hasCollisionAspect() for node in nodes) == len(
                link.findall("collision")
            ), body.getName()
            for node in nodes:
                visual_shapes += int(node.hasVisualAspect())
                collision_shapes += int(node.hasCollisionAspect())
                shape = node.getShape()
                assert shape is not None, body.getName()
                bounds = shape.getBoundingBox()
                assert np.isfinite(bounds.getMin()).all(), body.getName()
                assert np.isfinite(bounds.getMax()).all(), body.getName()
                if isinstance(shape, dart.dynamics.MeshShape):
                    meshes += 1
                    assert Path(shape.getMeshPath()).is_file(), shape.getMeshUri()
                    assert np.any(np.asarray(bounds.getMax()) > bounds.getMin())
        for joint in description.findall("joint"):
            if joint.attrib["type"] != "revolute":
                continue
            loaded = skeleton.getJoint(joint.attrib["name"])
            assert loaded is not None and loaded.getNumDofs() == 1
            limits = joint.find("limit").attrib
            dof = skeleton.getDof(loaded.getIndexInSkeleton(0))
            assert np.isclose(dof.getPositionLowerLimit(), float(limits["lower"]))
            assert np.isclose(dof.getPositionUpperLimit(), float(limits["upper"]))
            dof.setPosition(
                float(np.clip(0.0, float(limits["lower"]), float(limits["upper"])))
            )
        world = dart.simulation.World()
        world.setGravity([0, 0, -9.81])
        world.setTimeStep(0.001)
        world.addSkeleton(skeleton)
        for _ in range(100):
            world.step()
            assert np.isfinite(skeleton.getPositions()).all(), model_id
            assert np.isfinite(skeleton.getVelocities()).all(), model_id
        results.append(
            {
                "root": "fixed" if fixed else "floating",
                "bodies": skeleton.getNumBodyNodes(),
                "dofs": skeleton.getNumDofs(),
                "visual_shapes": visual_shapes,
                "collision_shapes": collision_shapes,
                "mesh_shapes": meshes,
                "finite_steps": 100,
            }
        )
    return {
        "model": model_id,
        "revision": manifest.attrib["revision"],
        "manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
        "verified_files": len(files),
        "verified_bytes": sum(int(file.attrib["size"]) for file in files),
        "results": results,
        "claim_boundary": "Model loading and finite short simulation; no controller or hardware fidelity claim.",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache-dir", default="")
    parser.add_argument("--offline", action="store_true")
    args = parser.parse_args()
    if not hasattr(dart.utils, "ModelResourceRetriever"):
        parser.error("Build DART with DART_BUILD_UTILS_ASSETS=ON")
    results = [
        verify_model(model, args.cache_dir, args.offline)
        for model in ("atlas-v5", "unitree-g1")
    ]
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
