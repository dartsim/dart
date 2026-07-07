from __future__ import annotations

import json
import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import image_ab_round2_packet
from _image_tools import read_image, write_png


def test_make_world_uses_half_extents_for_clean_controls(monkeypatch) -> None:
    shape_calls: list[tuple[float, float, float]] = []

    class FakeBody:
        def __init__(self, name: str, position: tuple[float, float, float]) -> None:
            self.name = name
            self.position = position
            self.is_static = False
            self.shape = None

        def set_collision_shape(self, shape) -> None:
            self.shape = shape

    class FakeWorld:
        def __init__(self, gravity: tuple[float, float, float]) -> None:
            self.gravity = gravity
            self.bodies: list[FakeBody] = []

        def add_rigid_body(
            self,
            name: str,
            *,
            position: tuple[float, float, float],
            mass: float | None = None,
        ) -> FakeBody:
            del mass
            body = FakeBody(name, position)
            self.bodies.append(body)
            return body

    class FakeCollisionShape:
        @staticmethod
        def box(half_extents: tuple[float, float, float]):
            shape_calls.append(half_extents)
            return ("box", half_extents)

    fake_dartpy = types.SimpleNamespace(
        World=FakeWorld,
        CollisionShape=FakeCollisionShape,
    )
    monkeypatch.setitem(sys.modules, "dartpy", fake_dartpy)

    world = image_ab_round2_packet._make_world(image_ab_round2_packet.CASES[0])

    assert shape_calls == [(0.8, 0.8, 0.04), (0.12, 0.12, 0.12)]
    ground = world.bodies[0]
    box = world.bodies[1]
    assert ground.position[2] + shape_calls[0][2] == 0.0
    assert box.position[2] - shape_calls[1][2] == 0.0


def test_judge_form_hides_case_variant_and_expected_labels() -> None:
    judge_form = {
        "schema_version": "dart.image_ab_round2_observations/v1",
        "study_id": "round2-test",
        "rows": [
            {
                "artifact_id": "r2_abc",
                "image": "images/r2_abc.png",
                "question": "Does this show a defect?",
                "choices": ["defect", "clean", "uncertain"],
            }
        ],
    }

    markdown = image_ab_round2_packet.render_judge_form_markdown(judge_form)

    assert "r2_abc" in markdown
    assert "images/r2_abc.png" in markdown
    assert "ground_penetration" not in markdown
    assert "multi_view" not in markdown
    assert "expected" not in markdown.lower()


def test_annotate_still_writes_overlay_pixels(tmp_path: Path) -> None:
    source = tmp_path / "source.png"
    target = tmp_path / "annotated.png"
    write_png(source, 96, 72, bytes((30, 30, 30)) * (96 * 72))

    image_ab_round2_packet.annotate_still(
        source,
        target,
        image_ab_round2_packet.CASES[0],
    )

    annotated = read_image(target)
    assert annotated.width == 96
    assert annotated.height == 72
    assert annotated.pixels != bytes((30, 30, 30)) * (96 * 72)
    assert b"\xff\xe6\x00" in annotated.pixels


def test_merge_observations_builds_reducer_manifest() -> None:
    answer_key = {
        "schema_version": "dart.image_ab_round2_packet/v1",
        "study_id": "round2-test",
        "rows": [
            {
                "artifact_id": "r2_a",
                "case": "clean",
                "variant": "single",
                "expected": "clean",
                "image": "images/r2_a.png",
            },
            {
                "artifact_id": "r2_b",
                "case": "penetration",
                "variant": "annotated",
                "expected": "defect",
                "image": "images/r2_b.png",
            },
        ],
    }
    observations = {
        "schema_version": "dart.image_ab_round2_observations/v1",
        "study_id": "round2-test",
        "rows": [
            {"artifact_id": "r2_a", "judge": "j1", "observed": "clean"},
            {"artifact_id": "r2_b", "judge": "j1", "observed": "defect"},
        ],
    }

    manifest = image_ab_round2_packet.merge_observations(answer_key, observations)

    assert manifest == {
        "schema_version": "dart.image_ab_study/v1",
        "study_id": "round2-test",
        "rows": [
            {
                "case": "clean",
                "variant": "single",
                "judge": "j1",
                "expected": "clean",
                "observed": "clean",
            },
            {
                "case": "penetration",
                "variant": "annotated",
                "judge": "j1",
                "expected": "defect",
                "observed": "defect",
            },
        ],
    }


def test_merge_observations_rejects_duplicate_judge_artifact_pairs() -> None:
    answer_key = {
        "schema_version": "dart.image_ab_round2_packet/v1",
        "study_id": "round2-test",
        "rows": [
            {
                "artifact_id": "r2_a",
                "case": "clean",
                "variant": "single",
                "expected": "clean",
                "image": "images/r2_a.png",
            }
        ],
    }
    observations = {
        "schema_version": "dart.image_ab_round2_observations/v1",
        "study_id": "round2-test",
        "rows": [
            {"artifact_id": "r2_a", "judge": "j1", "observed": "clean"},
            {"artifact_id": "r2_a", "judge": "j1", "observed": "defect"},
        ],
    }

    try:
        image_ab_round2_packet.merge_observations(answer_key, observations)
    except ValueError as exc:
        assert "duplicate observation" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_merge_observations_rejects_incomplete_judge_forms() -> None:
    answer_key = {
        "schema_version": "dart.image_ab_round2_packet/v1",
        "study_id": "round2-test",
        "rows": [
            {
                "artifact_id": "r2_a",
                "case": "clean",
                "variant": "single",
                "expected": "clean",
                "image": "images/r2_a.png",
            },
            {
                "artifact_id": "r2_b",
                "case": "penetration",
                "variant": "annotated",
                "expected": "defect",
                "image": "images/r2_b.png",
            },
        ],
    }
    observations = {
        "schema_version": "dart.image_ab_round2_observations/v1",
        "study_id": "round2-test",
        "rows": [
            {"artifact_id": "r2_a", "judge": "j1", "observed": "clean"},
        ],
    }

    try:
        image_ab_round2_packet.merge_observations(answer_key, observations)
    except ValueError as exc:
        assert "missing observations" in str(exc)
        assert "r2_b" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_score_observations_writes_manifest_and_results(tmp_path: Path) -> None:
    answer_key = {
        "schema_version": "dart.image_ab_round2_packet/v1",
        "study_id": "round2-test",
        "rows": [
            {
                "artifact_id": "r2_a",
                "case": "penetration",
                "variant": "single",
                "expected": "defect",
                "image": "images/r2_a.png",
            },
            {
                "artifact_id": "r2_b",
                "case": "penetration",
                "variant": "annotated",
                "expected": "defect",
                "image": "images/r2_b.png",
            },
        ],
    }
    observations = {
        "schema_version": "dart.image_ab_round2_observations/v1",
        "study_id": "round2-test",
        "rows": [
            {"artifact_id": "r2_a", "judge": "j1", "observed": "clean"},
            {"artifact_id": "r2_b", "judge": "j1", "observed": "defect"},
        ],
    }
    answer_key_path = tmp_path / "answer_key.json"
    observations_path = tmp_path / "observations.json"
    manifest_path = tmp_path / "manifest.json"
    results_path = tmp_path / "results.json"
    markdown_path = tmp_path / "results.md"
    answer_key_path.write_text(json.dumps(answer_key), encoding="utf-8")
    observations_path.write_text(json.dumps(observations), encoding="utf-8")

    results = image_ab_round2_packet.score_observations(
        answer_key_path=answer_key_path,
        observations_path=observations_path,
        out_path=manifest_path,
        results_path=results_path,
        markdown_path=markdown_path,
        baseline="single",
    )

    assert results["variants"]["single"]["detection_rate"] == 0.0
    assert results["variants"]["annotated"]["detection_rate"] == 1.0
    assert json.loads(manifest_path.read_text(encoding="utf-8"))[
        "schema_version"
    ] == "dart.image_ab_study/v1"
    assert json.loads(results_path.read_text(encoding="utf-8")) == results
    assert "| `annotated` |" in markdown_path.read_text(encoding="utf-8")


def test_parse_size_rejects_invalid_shape() -> None:
    try:
        image_ab_round2_packet._parse_size("640-by-480")
    except ValueError as exc:
        assert "WIDTHxHEIGHT" in str(exc)
    else:
        raise AssertionError("expected ValueError")
