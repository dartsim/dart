import importlib.util
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/finalize_fbf_arch101_failed_prefix.py"
VISUAL_RUNNER = ROOT / "scripts/run_fbf_visual_evidence.py"
FFMPEG = ROOT / ".pixi/envs/gazebo/bin/ffmpeg"
FFPROBE = ROOT / ".pixi/envs/gazebo/bin/ffprobe"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "finalize_fbf_arch101_failed_prefix", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_png(path, width, height, seed):
    sys.path.insert(0, str(ROOT / "scripts"))
    from _image_tools import write_png

    pixels = bytearray(width * height * 3)
    for y in range(height):
        for x in range(width):
            offset = (y * width + x) * 3
            pixels[offset : offset + 3] = bytes(
                (
                    (17 * x + seed) % 256,
                    (19 * y + 2 * seed) % 256,
                    (11 * (x + y) + 3 * seed) % 256,
                )
            )
    write_png(path, width, height, bytes(pixels))


def _terminal_signature(module):
    return dict(module.EXPECTED_TERMINAL_SIGNATURE)


def test_current_runner_schedule_and_freeze_mapping_are_pinned():
    module = _load_module()
    runner = module._load_runner(VISUAL_RUNNER)
    schedule = runner.SCHEDULES[module.SCHEDULE_ID]

    module._validate_schedule_contract(runner, schedule)
    plan = module._freeze_plan(schedule, module.EXPECTED_FAIL_STEP)

    assert plan["observed_steps"] == list(range(0, 209, 8))
    assert plan["observed_frame_count"] == 27
    assert plan["first_frozen_output_frame"] == 27
    assert plan["frozen_output_frame_count"] == 174
    assert plan["frozen_source_step"] == 208


def test_demo_identity_binds_every_resolved_build_dependency(tmp_path):
    module = _load_module()
    build_root = tmp_path / "build"
    core = build_root / "lib/libdart.so.6.19"
    renderer = build_root / "lib/libdart-gui-osg.so.6.19"
    external = tmp_path / "external/libc.so.6"
    core.parent.mkdir(parents=True)
    external.parent.mkdir(parents=True)
    core.write_bytes(b"core")
    renderer.write_bytes(b"renderer-v1")
    external.write_bytes(b"external")
    ldd_output = "\n".join(
        [
            f"libdart.so.6.19 => {core} (0x00000001)",
            f"libdart-gui-osg.so.6.19 => {renderer} (0x00000002)",
            f"libc.so.6 => {external} (0x00000003)",
        ]
    )

    before = module._linked_build_library_identities(ldd_output, build_root)
    renderer.write_bytes(b"renderer-v2")
    after = module._linked_build_library_identities(ldd_output, build_root)

    assert len(before) == len(after) == 2
    before_by_path = {item["resolved_path"]: item for item in before}
    after_by_path = {item["resolved_path"]: item for item in after}
    assert before_by_path[str(core)] == after_by_path[str(core)]
    assert (
        before_by_path[str(renderer)]["sha256"]
        != after_by_path[str(renderer)]["sha256"]
    )


def test_source_identity_binds_direct_validation_helpers_and_both_pythons(
    monkeypatch,
):
    module = _load_module()
    monkeypatch.setattr(
        module, "_file_identity", lambda path: {"path": str(Path(path))}
    )
    monkeypatch.setattr(
        module, "_demo_identity", lambda path: {"path": str(Path(path))}
    )
    monkeypatch.setattr(
        module, "_tool_identity", lambda path: {"path": str(Path(path))}
    )
    capture_python = Path("/tmp/capture-python")

    identity = module._source_identity(
        runner_path=module.DEFAULT_RUNNER,
        demo=module.DEFAULT_DEMO,
        ffmpeg=module.DEFAULT_FFMPEG,
        ffprobe=module.DEFAULT_FFPROBE,
        python=capture_python,
        font=module.DEFAULT_FONT,
    )

    assert {
        "image_tools_helper",
        "image_verdict_helper",
        "image_verdict_test",
    } <= set(identity["sources"])
    assert identity["finalizer_python_runtime"]["path"] == str(Path(sys.executable))
    assert identity["capture_python"]["path"] == str(capture_python)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("contacts", 207),
        ("iterations", 4999),
        ("residual", 1.3e-6),
        ("exact_attempts", 341),
        ("accepted_at_cap", 0),
        ("boxed_lcp_fallbacks", 1),
        ("contacts", True),
    ],
)
def test_terminal_signature_fails_closed(field, value):
    module = _load_module()
    signature = _terminal_signature(module)
    signature[field] = value

    with pytest.raises(module.EvidenceError, match="terminal signature changed"):
        module._require_terminal_signature(signature)


def test_freeze_plan_rejects_any_other_failure_step():
    module = _load_module()
    runner = module._load_runner(VISUAL_RUNNER)
    schedule = runner.SCHEDULES[module.SCHEDULE_ID]

    with pytest.raises(module.EvidenceError, match="fail step changed"):
        module._freeze_plan(schedule, 210)


def test_exact_frame_manifest_enforces_membership_and_motion(tmp_path):
    module = _load_module()
    runner = module._load_runner(VISUAL_RUNNER)
    output_dir = tmp_path / "exact"
    frames = output_dir / "frames"
    frames.mkdir(parents=True)
    _write_png(frames / "step_000000.png", 16, 12, 1)
    _write_png(frames / "step_000001.png", 16, 12, 2)
    schedule = SimpleNamespace(
        width=16,
        height=12,
        crop=(16, 12, 0, 0),
        expect_motion=True,
    )

    manifest = module._validate_frame_manifest(runner, schedule, output_dir, (0, 1))
    assert [item["step"] for item in manifest] == [0, 1]
    assert len({item["world_viewport"]["sha256"] for item in manifest}) == 2

    (frames / "unexpected.png").write_bytes(b"unexpected")
    with pytest.raises(module.EvidenceError, match="frame membership changed"):
        module._validate_frame_manifest(runner, schedule, output_dir, (0, 1))


def test_exact_frame_manifest_rejects_symlink(tmp_path):
    module = _load_module()
    runner = module._load_runner(VISUAL_RUNNER)
    output_dir = tmp_path / "exact"
    frames = output_dir / "frames"
    frames.mkdir(parents=True)
    target = tmp_path / "target.png"
    _write_png(target, 16, 12, 1)
    (frames / "step_000000.png").symlink_to(target)
    schedule = SimpleNamespace(
        width=16,
        height=12,
        crop=(16, 12, 0, 0),
        expect_motion=False,
    )

    with pytest.raises(module.EvidenceError, match="not a regular file"):
        module._validate_frame_manifest(runner, schedule, output_dir, (0,))


def _make_boxed_clip(module, destination, fps=5, frames=5):
    subprocess.run(
        [
            str(FFMPEG),
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "lavfi",
            "-i",
            f"testsrc=size=64x48:rate={fps}",
            "-frames:v",
            str(frames),
            *module._codec_arguments(),
            str(destination),
        ],
        check=True,
    )


def _decoded_frames(path, crop, width=64, height=48):
    completed = subprocess.run(
        [
            str(FFMPEG),
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            str(path),
            "-vf",
            crop,
            "-pix_fmt",
            "rgb24",
            "-f",
            "rawvideo",
            "-",
        ],
        check=True,
        capture_output=True,
    )
    frame_size = width * height * 3
    assert len(completed.stdout) % frame_size == 0
    return [
        completed.stdout[offset : offset + frame_size]
        for offset in range(0, len(completed.stdout), frame_size)
    ]


def _mean_absolute_difference(first, second):
    return sum(abs(a - b) for a, b in zip(first, second)) / len(first)


@pytest.mark.skipif(
    not FFMPEG.is_file() or not FFPROBE.is_file(),
    reason="Pixi ffmpeg tools are unavailable",
)
def test_composition_is_deterministic_and_freezes_only_exact_tail(tmp_path):
    module = _load_module()
    exact_frames = []
    for index in range(3):
        path = tmp_path / f"exact_{index}.png"
        _write_png(path, 64, 48, index + 1)
        exact_frames.append(path)
    boxed_final = tmp_path / "boxed_final.png"
    _write_png(boxed_final, 64, 48, 9)
    boxed_clip = tmp_path / "boxed.mp4"
    _make_boxed_clip(module, boxed_clip)
    spec = module.CompositionSpec(
        source_width=64,
        source_height=48,
        crop_width=64,
        crop_height=48,
        crop_x=0,
        crop_y=0,
        output_fps=5,
        output_frame_count=5,
        exact_steps=(0, 1, 2),
        boxed_label="BOXED TEST",
        font=module.DEFAULT_FONT,
    )

    first = module._compose_media(
        exact_frames=exact_frames,
        boxed_clip=boxed_clip,
        exact_last_frame=exact_frames[-1],
        boxed_final_frame=boxed_final,
        spec=spec,
        ffmpeg=FFMPEG,
        ffprobe=FFPROBE,
        destination=tmp_path / "first",
    )
    second = module._compose_media(
        exact_frames=exact_frames,
        boxed_clip=boxed_clip,
        exact_last_frame=exact_frames[-1],
        boxed_final_frame=boxed_final,
        spec=spec,
        ffmpeg=FFMPEG,
        ffprobe=FFPROBE,
        destination=tmp_path / "second",
    )

    assert first["clip"]["sha256"] == second["clip"]["sha256"]
    assert first["panel"]["sha256"] == second["panel"]["sha256"]
    assert {path.name for path in (tmp_path / "first").iterdir()} == {
        "clip.mp4",
        "panel.png",
    }
    left = _decoded_frames(
        tmp_path / "first/clip.mp4",
        f"crop=64:48:0:{module.LABEL_BAND_HEIGHT}",
    )
    right = _decoded_frames(
        tmp_path / "first/clip.mp4",
        f"crop=64:48:64:{module.LABEL_BAND_HEIGHT}",
    )
    assert len(left) == len(right) == 5
    assert _mean_absolute_difference(left[1], left[2]) > 1.0
    assert _mean_absolute_difference(left[2], left[3]) < 0.5
    assert _mean_absolute_difference(left[3], left[4]) < 0.5
    assert _mean_absolute_difference(right[2], right[3]) > 1.0
    assert _mean_absolute_difference(right[3], right[4]) > 1.0


def test_claim_boundary_rejects_promotion():
    module = _load_module()
    module._validate_claim_boundary(dict(module.CLAIM_BOUNDARY))
    promoted = dict(module.CLAIM_BOUNDARY)
    promoted["solver_superiority"] = True

    with pytest.raises(module.EvidenceError, match="claim boundary changed"):
        module._validate_claim_boundary(promoted)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("scene", "another_scene"),
        ("schedule_id", "another_schedule"),
        ("source_segment", "another_segment"),
        ("member_order", ["boxed", "exact"]),
    ],
)
def test_group_identity_rejects_provenance_field_mutation(field, value):
    module = _load_module()
    schedule = SimpleNamespace(
        id=module.SCHEDULE_ID,
        scene="fbf_author_masonry_arch_101_standing_current_source",
    )
    metadata = {
        "schema_version": module.SCHEMA_VERSION,
        "kind": module.KIND,
        "group_id": module.GROUP_ID,
        "scene": schedule.scene,
        "schedule_id": schedule.id,
        "source_segment": "masonry_arch_101",
        "member_order": [schedule.id, module.BOXED_SCHEDULE_ID],
        "pass": True,
        "actual_simulator": True,
        "generated_scene_imagery": False,
        "presentation_frame_repetition": True,
    }
    module._validate_group_identity(metadata, schedule)
    metadata[field] = value

    with pytest.raises(module.EvidenceError, match="metadata identity changed"):
        module._validate_group_identity(metadata, schedule)


@pytest.mark.parametrize("standing_outcome", [False, True])
def test_boxed_summary_accepts_complete_trace_regardless_of_standing_outcome(
    tmp_path, standing_outcome
):
    module = _load_module()
    boxed_dir = tmp_path / module.BOXED_SCHEDULE_ID
    boxed_dir.mkdir()
    metadata_path = boxed_dir / "metadata.json"
    timeline_path = boxed_dir / "timeline.json"
    clip_path = boxed_dir / "clip.mp4"
    module.write_json(
        metadata_path,
        {
            "schema_version": "dart.fbf_visual_evidence/v2",
            "runtime": {"scene_physics_provenance": {"sidecar_contract_match": True}},
        },
    )
    module.write_json(timeline_path, {"pass": True})
    clip_path.write_bytes(b"verified clip fixture")
    metrics = {
        "solver_lane": "boxed",
        "sample_count": 1601,
        "completed_substeps": 1600,
        "complete_horizon": True,
        "complete_trace_valid": True,
        "comparison_capture_valid": True,
        "standing_outcome_valid": standing_outcome,
    }

    class FakeRunner:
        CAPTURE_RESULT_SCHEMA_VERSION = "dart.fbf_visual_evidence/v2"

        @staticmethod
        def _derive_boxed_schedule(_schedule):
            return SimpleNamespace(
                id=module.BOXED_SCHEDULE_ID,
                scene="fbf_author_masonry_arch_101_standing_current_source",
            )

        @staticmethod
        def _verify_existing(*_args, **_kwargs):
            return {
                "timeline": {"author_masonry_arch_101_scene_state_metrics": metrics},
                "media": [{"kind": "mp4", "path": str(clip_path)}],
                "metadata_sha256": module.sha256(metadata_path),
            }

    summary = module._boxed_summary(
        FakeRunner(),
        SimpleNamespace(),
        output_root=tmp_path,
        demo=tmp_path / "demo",
        ffmpeg=tmp_path / "ffmpeg",
        ffprobe=tmp_path / "ffprobe",
    )

    assert summary["comparison_capture_valid"] is True
    assert summary["standing_outcome_valid"] is standing_outcome


def test_boxed_summary_rejects_incomplete_trace(tmp_path):
    module = _load_module()
    boxed_dir = tmp_path / module.BOXED_SCHEDULE_ID
    boxed_dir.mkdir()
    module.write_json(
        boxed_dir / "metadata.json",
        {
            "schema_version": "dart.fbf_visual_evidence/v2",
            "runtime": {"scene_physics_provenance": {"sidecar_contract_match": True}},
        },
    )
    module.write_json(boxed_dir / "timeline.json", {"pass": True})
    (boxed_dir / "clip.mp4").write_bytes(b"verified clip fixture")

    class FakeRunner:
        CAPTURE_RESULT_SCHEMA_VERSION = "dart.fbf_visual_evidence/v2"

        @staticmethod
        def _derive_boxed_schedule(_schedule):
            return SimpleNamespace(
                id=module.BOXED_SCHEDULE_ID,
                scene="fbf_author_masonry_arch_101_standing_current_source",
            )

        @staticmethod
        def _verify_existing(*_args, **_kwargs):
            return {
                "timeline": {
                    "author_masonry_arch_101_scene_state_metrics": {
                        "solver_lane": "boxed",
                        "sample_count": 1600,
                        "completed_substeps": 1599,
                        "complete_horizon": False,
                        "complete_trace_valid": False,
                        "comparison_capture_valid": False,
                        "standing_outcome_valid": False,
                    }
                },
                "media": [
                    {
                        "kind": "mp4",
                        "path": str(boxed_dir / "clip.mp4"),
                    }
                ],
                "metadata_sha256": module.sha256(boxed_dir / "metadata.json"),
            }

    with pytest.raises(module.EvidenceError, match="complete comparison trace"):
        module._boxed_summary(
            FakeRunner(),
            SimpleNamespace(),
            output_root=tmp_path,
            demo=tmp_path / "demo",
            ffmpeg=tmp_path / "ffmpeg",
            ffprobe=tmp_path / "ffprobe",
        )


def test_manual_inspection_binds_media_and_false_claims(tmp_path):
    module = _load_module()
    clip = tmp_path / "clip.mp4"
    panel = tmp_path / "panel.png"
    clip.write_bytes(b"clip")
    panel.write_bytes(b"panel")
    record = {
        "schema_version": module.MANUAL_SCHEMA_VERSION,
        "manual_inspected": True,
        "pass": True,
        "clip_sha256": module.sha256(clip),
        "panel_sha256": module.sha256(panel),
        "inspected_keyframe_seconds": [0.0, 0.9, 6.6],
        "observation": "Labels are legible and the exact tail is visibly frozen.",
        "verdicts": {
            "labels_legible": True,
            "no_gross_cropping_or_render_failure": True,
            "frozen_exact_tail_explicitly_disclosed": True,
            "complete_exact_trajectory": False,
            "solver_superiority": False,
            "fig08_parity": False,
            "paper_parity": False,
        },
    }
    path = tmp_path / "manual-inspection.json"
    module.write_json(path, record)

    assert module._validate_manual_inspection(
        path, clip=clip, panel=panel, duration_seconds=6.7
    )["pass"]
    record["verdicts"]["paper_parity"] = True
    module.write_json(path, record)
    with pytest.raises(module.EvidenceError, match="contract changed"):
        module._validate_manual_inspection(
            path, clip=clip, panel=panel, duration_seconds=6.7
        )


def test_seal_transitions_pending_metadata_after_explicit_inspection(
    tmp_path, monkeypatch
):
    module = _load_module()
    group = tmp_path / "groups" / module.GROUP_ID
    group.mkdir(parents=True)
    (group / "clip.mp4").write_bytes(b"clip")
    (group / "panel.png").write_bytes(b"panel")
    module.write_json(
        group / "metadata.json",
        {
            "status": "mechanically_valid_pending_manual_inspection",
            "manual_inspection": {"required": True, "status": "pending"},
            "upload_ready": False,
        },
    )
    calls = []

    def fake_verify(*_args, require_manual, **_kwargs):
        calls.append(require_manual)
        if require_manual:
            metadata = module.read_json(group / "metadata.json")
            assert metadata["upload_ready"] is True
            assert (group / "manual-inspection.json").is_file()
        return {"pass": True, "upload_ready": require_manual}

    monkeypatch.setattr(module, "verify_bundle", fake_verify)
    args = SimpleNamespace(
        output_root=tmp_path,
        demo=tmp_path / "demo",
        runner=tmp_path / "runner",
        ffmpeg=tmp_path / "ffmpeg",
        ffprobe=tmp_path / "ffprobe",
        python=tmp_path / "python",
        font=tmp_path / "font",
        observation="The labels and frozen exact tail are visually clear.",
        keyframe_second=[0.0, 0.9, 4.0, 6.6],
    )

    result = module.seal(args)

    assert result["upload_ready"] is True
    assert calls == [False, True]
    metadata = module.read_json(group / "metadata.json")
    assert metadata["status"] == "valid_current_dart_blocker_diagnostic"
    assert metadata["manual_inspection"]["status"] == "passed"


def test_seal_rolls_back_to_pending_when_final_verification_fails(
    tmp_path, monkeypatch
):
    module = _load_module()
    group = tmp_path / "groups" / module.GROUP_ID
    group.mkdir(parents=True)
    (group / "clip.mp4").write_bytes(b"clip")
    (group / "panel.png").write_bytes(b"panel")
    pending = {
        "status": "mechanically_valid_pending_manual_inspection",
        "manual_inspection": {"required": True, "status": "pending"},
        "upload_ready": False,
    }
    module.write_json(group / "metadata.json", pending)
    calls = []

    def fail_final_verify(*_args, require_manual, **_kwargs):
        calls.append(require_manual)
        if require_manual:
            raise module.EvidenceError("injected sealed verification failure")
        return {"pass": True}

    monkeypatch.setattr(module, "verify_bundle", fail_final_verify)
    args = SimpleNamespace(
        output_root=tmp_path,
        demo=tmp_path / "demo",
        runner=tmp_path / "runner",
        ffmpeg=tmp_path / "ffmpeg",
        ffprobe=tmp_path / "ffprobe",
        python=tmp_path / "python",
        font=tmp_path / "font",
        observation="The labels and frozen exact tail are visually clear.",
        keyframe_second=[0.0, 0.9, 4.0, 6.6],
    )

    with pytest.raises(module.EvidenceError, match="injected"):
        module.seal(args)

    assert calls == [False, True]
    assert module.read_json(group / "metadata.json") == pending
    assert not (group / "manual-inspection.json").exists()


@pytest.mark.parametrize(
    "keyframes",
    [
        [0.0],
        [0.0, 2.0, 6.6],
        [0.0, 0.9, 6.7],
    ],
)
def test_manual_keyframe_coverage_rejects_incomplete_or_boundary_only_inspection(
    keyframes,
):
    module = _load_module()
    with pytest.raises(module.EvidenceError, match="manual inspection"):
        module._validate_manual_keyframe_coverage(keyframes, duration_seconds=6.7)


def test_expected_capture_rejects_success_without_launching(tmp_path):
    module = _load_module()

    class SuccessfulRunner:
        @staticmethod
        def run_schedule(*_args, **_kwargs):
            return {"pass": True}

    schedule = SimpleNamespace(id=module.SCHEDULE_ID)
    executable = tmp_path / "tool"
    executable.write_bytes(b"tool")

    with pytest.raises(module.EvidenceError, match="unexpectedly completed"):
        module._capture_expected_exact(
            SuccessfulRunner(),
            schedule,
            demo=executable,
            output_root=tmp_path,
            ffmpeg=executable,
            ffprobe=executable,
            python=executable,
        )


def test_expected_capture_preserves_failure_for_validation(tmp_path):
    module = _load_module()

    class FailedRunner:
        @staticmethod
        def run_schedule(*_args, **_kwargs):
            raise ValueError(
                "exact-FBF fail-fast triggered at completed step 209: iteration_cap"
            )

    schedule = SimpleNamespace(id=module.SCHEDULE_ID)
    executable = tmp_path / "tool"
    executable.write_bytes(b"tool")

    error = module._capture_expected_exact(
        FailedRunner(),
        schedule,
        demo=executable,
        output_root=tmp_path,
        ffmpeg=executable,
        ffprobe=executable,
        python=executable,
    )
    assert "completed step 209: iteration_cap" in error


def test_verify_command_never_dispatches_capture(monkeypatch, capsys):
    module = _load_module()
    calls = []

    def fake_verify(*_args, **_kwargs):
        calls.append("verify")
        return {"pass": True}

    def reject_finalize(*_args, **_kwargs):
        pytest.fail("verify must not dispatch finalize")

    monkeypatch.setattr(module, "verify_bundle", fake_verify)
    monkeypatch.setattr(module, "finalize", reject_finalize)

    assert module.main(["verify"]) == 0
    assert calls == ["verify"]
    assert json.loads(capsys.readouterr().out)["pass"] is True
