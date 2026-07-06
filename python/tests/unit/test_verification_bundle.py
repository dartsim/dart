from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import verification_bundle
from _image_tools import write_png


def test_verification_bundle_copies_artifacts_and_writes_prompt(tmp_path: Path) -> None:
    scene = tmp_path / "scene.json"
    metrics = tmp_path / "metrics.json"
    image = tmp_path / "frame.png"
    grid = tmp_path / "grid.png"
    out = tmp_path / "bundle"
    scene.write_text('{"schema":"dart.scene/v0"}\n', encoding="utf-8")
    metrics.write_text('{"total_energy":1.0}\n', encoding="utf-8")
    write_png(image, 2, 2, bytes((32, 64, 96)) * 4)
    write_png(grid, 2, 2, bytes((96, 64, 32)) * 4)

    manifest = verification_bundle.build_bundle(
        out_dir=out,
        question="Does the box rest on the ground?",
        text=[scene, metrics],
        image=image,
        grid=grid,
        metadata={"scene": "box"},
    )

    assert manifest["schema_version"] == "dart.verification_bundle/v1"
    assert manifest["metadata"] == {"scene": "box"}
    assert {artifact["role"] for artifact in manifest["artifacts"]} == {
        "text-primary",
        "image-still",
        "image-grid",
        "review-prompt",
    }
    assert (out / "scene.json").is_file()
    assert (out / "metrics.json").is_file()
    assert (out / "frame.png").is_file()
    assert (out / "grid.png").is_file()
    prompt = (out / "vlm_prompt.md").read_text(encoding="utf-8")
    assert "Use the text artifacts as the primary oracle" in prompt
    assert "Does the box rest on the ground?" in prompt

    disk_manifest = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
    assert disk_manifest == manifest


def test_verification_bundle_cli(tmp_path: Path, capsys) -> None:
    scene = tmp_path / "scene.txt"
    image = tmp_path / "frame.png"
    out = tmp_path / "bundle"
    scene.write_text("world bodies=1\n", encoding="utf-8")
    write_png(image, 2, 2, bytes((32, 64, 96)) * 4)

    assert (
        verification_bundle.main(
            [
                "--out",
                str(out),
                "--question",
                "Check the scene.",
                "--text",
                str(scene),
                "--image",
                str(image),
            ]
        )
        == 0
    )
    assert "dart.verification_bundle/v1" in capsys.readouterr().out


def test_verification_bundle_rejects_duplicate_filenames(tmp_path: Path) -> None:
    one = tmp_path / "one" / "scene.json"
    two = tmp_path / "two" / "scene.json"
    image = tmp_path / "frame.png"
    one.parent.mkdir()
    two.parent.mkdir()
    one.write_text("{}\n", encoding="utf-8")
    two.write_text("{}\n", encoding="utf-8")
    write_png(image, 2, 2, bytes((32, 64, 96)) * 4)

    try:
        verification_bundle.build_bundle(
            out_dir=tmp_path / "bundle",
            question="Check duplicate handling.",
            text=[one, two],
            image=image,
        )
    except ValueError as exc:
        assert "duplicate artifact filename" in str(exc)
    else:
        raise AssertionError("expected ValueError")
