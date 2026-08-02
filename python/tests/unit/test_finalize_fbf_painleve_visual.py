import argparse
import importlib.util
import json
import math
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/finalize_fbf_painleve_visual.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "finalize_fbf_painleve_visual", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _trace_text(module, scenario):
    lines = [",".join(module.TRACE_COLUMNS)]
    for step in range(module.TOTAL_STEPS + 1):
        if scenario == "painleve_mu_0_5":
            final_x = 1.2980816997249069
            x = final_x * min(step, 120) / 120
            z = 0.49712057009207827
            up_z = 0.999998178998452
        else:
            final_x = 1.5430482150935836
            x = 1.2597198197697048 * min(step, 36) / 36
            if step > 36:
                x += (final_x - 1.2597198197697048) * min(step - 36, 84) / 84
            z = 0.5140937241172977 if step < 60 else 0.2660584153667479
            up_z = 0.996 if step < 36 else -0.054245797075879627
        row = {
            "step": str(step),
            "time": repr(step / 60),
            "scenario": scenario,
            "solver": "exact_fbf",
            "body": "painleve_box_body",
            "x": repr(x),
            "y": "0.0",
            "z": repr(z),
            "vx": "0.0" if step >= 120 else "0.1",
            "vy": "0.0",
            "vz": "0.001",
            "up_z": repr(up_z),
            "contacts": "3",
            "exact_solves": str(step),
            "warm_starts": str(max(0, step - 1)),
            "fallbacks": "0",
            "residual": "nan" if step == 0 else "9e-7",
            "status": "not_run" if step == 0 else "success",
        }
        lines.append(",".join(row[column] for column in module.TRACE_COLUMNS))
    return "\n".join(lines) + "\n"


def test_trace_pair_accepts_fixture_threshold_and_disambiguates_travel():
    module = _load_module()
    slide = module.parse_trace_text(
        _trace_text(module, "painleve_mu_0_5"), "painleve_mu_0_5"
    )
    tumble = module.parse_trace_text(
        _trace_text(module, "painleve_mu_0_55"), "painleve_mu_0_55"
    )

    summary = module.summarize_trace_pair([slide, tumble])

    assert summary["pass"] is True
    assert summary["paired_contract"]["mu055_first_tumble_step"] == 36
    assert math.isclose(
        summary["paired_contract"]["mu055_first_tumble_travel"],
        1.2597198197697048,
    )
    assert summary["paired_contract"]["shorter_by"] > 0
    assert tumble["final"]["x"] > slide["final"]["x"]


@pytest.mark.parametrize(
    ("scenario", "mutator", "message"),
    [
        (
            "painleve_mu_0_5",
            lambda lines: lines[:-1],
            "expected 151 rows",
        ),
        (
            "painleve_mu_0_5",
            lambda lines: [lines[0].replace("step,time", "time,step"), *lines[1:]],
            "unexpected trace columns",
        ),
        (
            "painleve_mu_0_5",
            lambda lines: [
                *lines[:2],
                lines[2].replace(",0,9e-7,success", ",1,9e-7,success"),
                *lines[3:],
            ],
            "fallback",
        ),
        (
            "painleve_mu_0_5",
            lambda lines: [
                *lines[:2],
                lines[2].replace(",9e-7,success", ",2e-6,success"),
                *lines[3:],
            ],
            "residual",
        ),
        (
            "painleve_mu_0_55",
            lambda lines: [
                line.replace(",0.2660584153667479,", ",0.5,").replace(
                    ",-0.05424579707587963,", ",0.9,"
                )
                for line in lines
            ],
            "never crossed",
        ),
    ],
)
def test_trace_contract_fails_closed(scenario, mutator, message):
    module = _load_module()
    lines = _trace_text(module, scenario).splitlines()
    mutated = "\n".join(mutator(lines)) + "\n"

    with pytest.raises(ValueError, match=message):
        module.parse_trace_text(mutated, scenario)


def test_trace_rejects_nonfinite_state_and_noncontiguous_time():
    module = _load_module()
    text = _trace_text(module, "painleve_mu_0_5")
    lines = text.splitlines()
    columns = lines[10].split(",")
    columns[module.TRACE_COLUMNS.index("x")] = "inf"
    lines[10] = ",".join(columns)
    with pytest.raises(ValueError, match="nonfinite x"):
        module.parse_trace_text("\n".join(lines) + "\n", "painleve_mu_0_5")

    lines = text.splitlines()
    columns = lines[10].split(",")
    columns[module.TRACE_COLUMNS.index("time")] = "1.0"
    lines[10] = ",".join(columns)
    with pytest.raises(ValueError, match="is not step/60"):
        module.parse_trace_text("\n".join(lines) + "\n", "painleve_mu_0_5")


def test_artifact_index_enforces_exact_membership_size_and_hash(tmp_path):
    module = _load_module()
    (tmp_path / "frames").mkdir()
    (tmp_path / "frames/a.png").write_bytes(b"a")
    (tmp_path / "REPORT.md").write_text("report\n", encoding="utf-8")
    (tmp_path / "metadata.json").write_text("{}\n", encoding="utf-8")
    index = module.artifact_index(tmp_path)
    module.write_json(tmp_path / "artifact-index.json", index)

    module.validate_artifact_index(
        tmp_path, json.loads((tmp_path / "artifact-index.json").read_text())
    )
    assert {item["path"] for item in index["artifacts"]} == {
        "REPORT.md",
        "frames/a.png",
    }

    (tmp_path / "frames/a.png").write_bytes(b"changed")
    with pytest.raises(ValueError, match="byte mismatch"):
        module.validate_artifact_index(tmp_path, index)

    (tmp_path / "frames/a.png").write_bytes(b"a")
    (tmp_path / "unexpected.txt").write_text("unexpected", encoding="utf-8")
    with pytest.raises(ValueError, match="membership mismatch"):
        module.validate_artifact_index(tmp_path, index)


def test_manual_inspection_binds_all_three_panels(tmp_path):
    module = _load_module()
    paths = [
        "painleve_mu05/panel.png",
        "painleve_mu055/panel.png",
        "groups/painleve/panel.png",
    ]
    images = []
    for index, relative in enumerate(paths):
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(f"panel-{index}".encode())
        images.append(
            {
                "path": relative,
                "sha256": module.sha256(path),
                "observation": "clear panel",
            }
        )
    module.write_json(
        tmp_path / "manual-inspection.json",
        {
            "schema_version": module.MANUAL_SCHEMA_VERSION,
            "manual_inspected": True,
            "pass": True,
            "verdicts": {
                "paired_proxy_outcome_supported": True,
                "mu05_upright_return_visible": True,
                "mu055_tumble_and_visually_settled_horizontal_visible": True,
                "shorter_travel_requires_physical_trace": True,
                "paper_parity": False,
                "external_solver_parity": False,
                "approved_source_golden": False,
                "timing_verdict": None,
                "realtime_verdict": None,
            },
            "representative_images": images,
        },
    )

    assert module.validate_manual_inspection(tmp_path)["pass"] is True
    (tmp_path / paths[1]).write_bytes(b"mutated")
    with pytest.raises(ValueError, match="image changed"):
        module.validate_manual_inspection(tmp_path)


def test_clean_checkout_membership_requires_exact_durable_bundle(tmp_path):
    module = _load_module()
    bundle = tmp_path / "bundle"
    for relative in module.EXPECTED_FINAL_PATHS:
        path = bundle / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(relative.encode())

    module._validate_bundle_paths(bundle, complete=True)
    assert module.EXPECTED_FINAL_PATHS.isdisjoint(module.STAGING_PATHS)
    assert all("/frames/" not in path for path in module.EXPECTED_FINAL_PATHS)
    assert all("/panel_frames/" not in path for path in module.EXPECTED_FINAL_PATHS)

    staging = bundle / "painleve_mu05/frames/step_000000.png"
    staging.parent.mkdir(parents=True)
    staging.write_bytes(b"ignored")
    with pytest.raises(ValueError, match="unexpected entries"):
        module._validate_bundle_paths(bundle, complete=True)
    staging.unlink()
    staging.parent.rmdir()

    durable_panel = bundle / "groups/painleve/panel.png"
    durable_panel.unlink()
    with pytest.raises(ValueError, match="sealed bundle is incomplete"):
        module._validate_bundle_paths(bundle, complete=True)
    durable_panel.write_bytes(b"restored")
    (bundle / "unexpected.txt").write_text("unexpected", encoding="utf-8")
    with pytest.raises(ValueError, match="unexpected entries"):
        module._validate_bundle_paths(bundle, complete=True)


def test_pruning_removes_disposable_staging_but_preserves_panels(tmp_path):
    module = _load_module()
    panel = tmp_path / "groups/painleve/panel.png"
    panel.parent.mkdir(parents=True)
    panel.write_bytes(b"durable panel")
    for relative in (
        "capture.stdout.txt",
        "capture.stderr.txt",
        "verification.stderr.txt",
        "painleve_mu05/video_frames.ffconcat",
        "painleve_mu05/frames/step_000000.png",
        "painleve_mu05/panel_frames/step_000000.png",
    ):
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"staging")

    module._prune_capture_staging(tmp_path)

    assert panel.read_bytes() == b"durable panel"
    assert not any((tmp_path / relative).exists() for relative in module.STAGING_PATHS)


def test_bundle_transaction_restores_original_bundle_on_failure(tmp_path):
    module = _load_module()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    original = bundle / "run-summary.json"
    original.write_text("original\n", encoding="utf-8")

    with pytest.raises(RuntimeError, match="forced failure"):
        with module._bundle_transaction(bundle):
            original.write_text("corrupted\n", encoding="utf-8")
            (bundle / "partial.txt").write_text("partial\n", encoding="utf-8")
            raise RuntimeError("forced failure")

    assert original.read_text(encoding="utf-8") == "original\n"
    assert not (bundle / "partial.txt").exists()


def test_bundle_transaction_restores_snapshot_after_root_symlink_swap(tmp_path):
    module = _load_module()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    (bundle / "run-summary.json").write_text("original\n", encoding="utf-8")
    outside = tmp_path / "outside"
    outside.mkdir()
    sentinel = outside / "sentinel.txt"
    sentinel.write_text("outside\n", encoding="utf-8")

    with pytest.raises(RuntimeError, match="forced failure"):
        with module._bundle_transaction(bundle):
            shutil.rmtree(bundle)
            bundle.symlink_to(outside, target_is_directory=True)
            raise RuntimeError("forced failure")

    assert bundle.is_dir()
    assert not bundle.is_symlink()
    assert (bundle / "run-summary.json").read_text(encoding="utf-8") == "original\n"
    assert sentinel.read_text(encoding="utf-8") == "outside\n"


@pytest.mark.parametrize("entrypoint", ["finalize", "verify_finalized"])
def test_entrypoints_reject_root_and_ancestor_symlinks(tmp_path, entrypoint):
    module = _load_module()

    def invoke(bundle):
        if entrypoint == "finalize":
            return module.finalize(argparse.Namespace(bundle=bundle))
        return module.verify_finalized(
            bundle,
            trace_binary=SCRIPT,
            demo=SCRIPT,
            runner=SCRIPT,
            ffmpeg=SCRIPT,
            ffprobe=SCRIPT,
        )

    target = tmp_path / "target"
    target.mkdir()
    root_link = tmp_path / "bundle-link"
    root_link.symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError, match="bundle root is a symlink"):
        invoke(root_link)

    real_parent = tmp_path / "real-parent"
    (real_parent / "bundle").mkdir(parents=True)
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(real_parent, target_is_directory=True)
    with pytest.raises(ValueError, match="passes through a symlink"):
        invoke(linked_parent / "bundle")


def test_capture_provenance_binds_durable_artifacts_and_disposal_hashes(tmp_path):
    module = _load_module()
    bundle = tmp_path / "bundle"
    for capture_id in module.CAPTURE_IDS:
        for name in ("metadata.json", "timeline.json", "panel.png", "clip.mp4"):
            path = bundle / capture_id / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(f"{capture_id}/{name}".encode())
    for name in ("metadata.json", "panel.png", "clip.mp4"):
        path = bundle / "groups/painleve" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(f"group/{name}".encode())
    module.write_json(bundle / "run-summary.json", {"pass": True})
    module.write_json(bundle / "verification.json", {"kind": "verification"})
    runner = demo = ffmpeg = ffprobe = SCRIPT
    verification = module.read_json(bundle / "verification.json")
    payload = {
        "schema_version": module.CAPTURE_PROVENANCE_SCHEMA_VERSION,
        "run_summary": {
            "path": "run-summary.json",
            "sha256": module.sha256(bundle / "run-summary.json"),
        },
        "verification": {
            "argv": module._verification_argv(runner, demo, bundle, ffmpeg, ffprobe),
            "returncode": 0,
            "path": "verification.json",
            "sha256": module.sha256(bundle / "verification.json"),
            "payload_sha256": module._payload_sha256(verification),
        },
        "verification_resources": module._verification_resource_identity(
            runner=runner,
            demo=demo,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        ),
        "verification_stdout_sha256": "1" * 64,
        "verification_stderr_sha256": "2" * 64,
        "durable_capture_records": module._durable_capture_records(bundle),
        "staging_pruned": True,
    }

    assert (
        module._validate_capture_provenance(
            payload,
            bundle=bundle,
            runner=runner,
            demo=demo,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        )
        == payload
    )

    (bundle / "painleve_mu05/panel.png").write_bytes(b"tampered")
    with pytest.raises(ValueError, match="capture provenance changed"):
        module._validate_capture_provenance(
            payload,
            bundle=bundle,
            runner=runner,
            demo=demo,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        )
    (bundle / "painleve_mu05/panel.png").unlink()
    with pytest.raises(ValueError, match="not a file"):
        module._validate_capture_provenance(
            payload,
            bundle=bundle,
            runner=runner,
            demo=demo,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        )
