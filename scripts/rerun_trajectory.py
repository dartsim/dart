#!/usr/bin/env python3
"""Log DART trajectory recorder output to an optional rerun.io recording."""

from __future__ import annotations

import argparse
import importlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import trajectory_record

MISSING_RERUN_MESSAGE = (
    "rerun-sdk not installed; pip install rerun-sdk or add it to the dev environment"
)


class MissingRerunError(RuntimeError):
    """Raised when the optional rerun SDK is not available."""


@dataclass(frozen=True)
class TrajectorySample:
    frame: int
    time: float
    body: str
    position: tuple[float, float, float]
    linear_velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]
    contact_count: int


@dataclass(frozen=True)
class ContactPoint:
    body_a: str
    body_b: str
    position: tuple[float, float, float]
    normal: tuple[float, float, float]
    depth: float


@dataclass(frozen=True)
class ContactEvent:
    frame: int
    time: float
    event: str
    count: int
    contacts: tuple[ContactPoint, ...]


def _import_rerun() -> Any:
    try:
        return importlib.import_module("rerun")
    except ImportError as exc:
        raise MissingRerunError(MISSING_RERUN_MESSAGE) from exc


def _as_float3(row: dict[str, str], prefix: str) -> tuple[float, float, float]:
    return (
        float(row[f"{prefix}_x"]),
        float(row[f"{prefix}_y"]),
        float(row[f"{prefix}_z"]),
    )


def parse_trajectory_text(text: str) -> list[TrajectorySample]:
    """Parse the TSV emitted by scripts/trajectory_record.py."""

    columns: list[str] = []
    samples: list[TrajectorySample] = []
    for line in text.splitlines():
        if line.startswith("# columns:"):
            columns = line.split(":", 1)[1].strip().split()
            continue
        if not line.strip() or line.startswith("#"):
            continue
        if not columns:
            raise ValueError("trajectory TSV is missing a '# columns:' header")
        parts = line.split()
        if len(parts) != len(columns):
            raise ValueError(
                f"trajectory row has {len(parts)} fields, expected {len(columns)}"
            )
        row = dict(zip(columns, parts))
        samples.append(
            TrajectorySample(
                frame=int(row["frame"]),
                time=float(row["time"]),
                body=row["body"],
                position=_as_float3(row, "pos"),
                linear_velocity=_as_float3(row, "lin"),
                angular_velocity=_as_float3(row, "ang"),
                contact_count=int(row["contact_count"]),
            )
        )
    if not samples:
        raise ValueError("trajectory TSV contains no samples")
    return samples


def _contact_float3(value: Any, field: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"contact {field} must contain exactly 3 values")
    return (float(value[0]), float(value[1]), float(value[2]))


def parse_contact_events_text(text: str) -> list[ContactEvent]:
    """Parse the JSONL emitted by scripts/trajectory_record.py --contacts."""

    events: list[ContactEvent] = []
    for line in text.splitlines():
        if not line.strip():
            continue
        payload = json.loads(line)
        contacts = []
        for contact in payload.get("contacts", []):
            contacts.append(
                ContactPoint(
                    body_a=str(contact["body_a"]),
                    body_b=str(contact["body_b"]),
                    position=_contact_float3(contact["position"], "position"),
                    normal=_contact_float3(contact["normal"], "normal"),
                    depth=float(contact["depth"]),
                )
            )
        events.append(
            ContactEvent(
                frame=int(payload["frame"]),
                time=float(payload["time"]),
                event=str(payload["event"]),
                count=int(payload["count"]),
                contacts=tuple(contacts),
            )
        )
    return events


def _group_samples(
    samples: Iterable[TrajectorySample],
) -> dict[int, list[TrajectorySample]]:
    frames: dict[int, list[TrajectorySample]] = {}
    for sample in samples:
        frames.setdefault(sample.frame, []).append(sample)
    for frame_samples in frames.values():
        frame_samples.sort(key=lambda item: item.body)
    return frames


def _group_contacts(events: Iterable[ContactEvent]) -> dict[int, list[ContactEvent]]:
    frames: dict[int, list[ContactEvent]] = {}
    for event in events:
        frames.setdefault(event.frame, []).append(event)
    return frames


def _configure_recording(
    rr: Any, *, application_id: str, spawn: bool, save: Path | None
) -> None:
    try:
        rr.init(application_id, spawn=spawn)
    except TypeError:
        rr.init(application_id)
        if spawn and hasattr(rr, "spawn"):
            rr.spawn()
    if save is not None:
        save.parent.mkdir(parents=True, exist_ok=True)
        rr.save(str(save))


def _set_time(rr: Any, *, frame: int, seconds: float) -> None:
    if hasattr(rr, "set_time_sequence"):
        rr.set_time_sequence("frame", frame)
    elif hasattr(rr, "set_time"):
        try:
            rr.set_time("frame", sequence=frame)
        except TypeError:
            pass

    if hasattr(rr, "set_time_seconds"):
        rr.set_time_seconds("sim_time", seconds)


def _points3d(
    rr: Any,
    points: list[tuple[float, float, float]],
    *,
    labels: list[str],
    colors: list[tuple[int, int, int]] | None = None,
    radii: float | None = None,
) -> Any:
    kwargs: dict[str, Any] = {"labels": labels}
    if colors is not None:
        kwargs["colors"] = colors
    if radii is not None:
        kwargs["radii"] = radii
    try:
        return rr.Points3D(points, **kwargs)
    except TypeError:
        try:
            return rr.Points3D(points, labels=labels)
        except TypeError:
            return rr.Points3D(points)


def _log_scalar(rr: Any, entity_path: str, value: float) -> None:
    scalar_type = getattr(rr, "Scalar", None) or getattr(rr, "Scalars", None)
    if scalar_type is None:
        return
    try:
        scalar = scalar_type(float(value))
    except TypeError:
        scalar = scalar_type([float(value)])
    rr.log(entity_path, scalar)


def _log_text(rr: Any, entity_path: str, text: str) -> None:
    text_log_type = getattr(rr, "TextLog", None)
    if text_log_type is None:
        return
    rr.log(entity_path, text_log_type(text))


def _log_contact_events(
    rr: Any, entity_prefix: str, events: Iterable[ContactEvent]
) -> None:
    for event in events:
        _set_time(rr, frame=event.frame, seconds=event.time)
        _log_scalar(rr, f"{entity_prefix}/metrics/contact_events", event.count)
        pairs = [f"{contact.body_a}/{contact.body_b}" for contact in event.contacts]
        _log_text(
            rr,
            f"{entity_prefix}/contacts/events",
            (
                f"frame={event.frame} time={event.time:.17e} "
                f"event={event.event} count={event.count} pairs={','.join(pairs)}"
            ),
        )
        if not event.contacts:
            continue
        points = [contact.position for contact in event.contacts]
        labels = [
            f"{contact.body_a}/{contact.body_b} depth={contact.depth:.3e}"
            for contact in event.contacts
        ]
        rr.log(
            f"{entity_prefix}/contacts/points",
            _points3d(
                rr,
                points,
                labels=labels,
                colors=[(255, 96, 48)] * len(points),
                radii=0.035,
            ),
        )


def log_recording(
    rr: Any,
    samples: list[TrajectorySample],
    *,
    contact_events: list[ContactEvent] | None = None,
    entity_prefix: str = "dart",
) -> int:
    """Log trajectory samples and optional contacts to the active rerun sink."""

    frames = _group_samples(samples)
    contacts_by_frame = _group_contacts(contact_events or [])
    for frame in sorted(frames):
        frame_samples = frames[frame]
        time = frame_samples[0].time
        _set_time(rr, frame=frame, seconds=time)
        points = [sample.position for sample in frame_samples]
        labels = [sample.body for sample in frame_samples]
        rr.log(
            f"{entity_prefix}/bodies/positions",
            _points3d(
                rr,
                points,
                labels=labels,
                colors=[(80, 160, 255)] * len(points),
                radii=0.04,
            ),
        )
        _log_scalar(
            rr,
            f"{entity_prefix}/metrics/contact_count",
            frame_samples[0].contact_count,
        )
        _log_text(
            rr,
            f"{entity_prefix}/trajectory/frames",
            (
                f"frame={frame} time={time:.17e} bodies={len(frame_samples)} "
                f"contact_count={frame_samples[0].contact_count}"
            ),
        )
        _log_contact_events(rr, entity_prefix, contacts_by_frame.get(frame, ()))

    extra_contact_frames = sorted(set(contacts_by_frame).difference(frames))
    for frame in extra_contact_frames:
        _log_contact_events(rr, entity_prefix, contacts_by_frame[frame])

    return len(frames)


def _load_contact_text(args: argparse.Namespace) -> str | None:
    if args.contact_jsonl is not None:
        return args.contact_jsonl.read_text(encoding="utf-8")
    if not args.contacts:
        return None
    if args.trajectory_tsv is not None:
        raise ValueError("--contacts with --trajectory-tsv requires --contact-jsonl")
    if args.steps is None:
        raise ValueError("--steps is required to record contact events")
    runner = trajectory_record.resolve_world_runner(
        scene=args.scene, factory=args.factory
    )
    return trajectory_record.record_contact_events(runner, args.steps)


def _load_trajectory_text(args: argparse.Namespace) -> str:
    if args.trajectory_tsv is not None:
        return args.trajectory_tsv.read_text(encoding="utf-8")
    if args.steps is None:
        raise ValueError("--steps is required unless --trajectory-tsv is used")
    runner = trajectory_record.resolve_world_runner(
        scene=args.scene, factory=args.factory
    )
    return trajectory_record.record_trajectory(runner, args.steps, args.body)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Log DART 7 trajectory recorder output to rerun.io for optional "
            "interactive or headless inspection."
        )
    )
    output = parser.add_mutually_exclusive_group(required=True)
    output.add_argument("--spawn", action="store_true", help="open the rerun viewer")
    output.add_argument("--save", type=Path, help="write a .rrd recording")

    source = parser.add_mutually_exclusive_group()
    source.add_argument("--scene")
    source.add_argument("--factory", help="importable world factory: module:callable")
    parser.add_argument(
        "--trajectory-tsv",
        type=Path,
        help="log an existing trajectory_record.py TSV instead of recording a scene",
    )
    parser.add_argument(
        "--contact-jsonl",
        type=Path,
        help="log an existing trajectory_record.py --contacts JSONL trace",
    )
    parser.add_argument(
        "--contacts",
        action="store_true",
        help="record and log contact events for the selected scene/factory",
    )
    parser.add_argument("--steps", type=int)
    parser.add_argument(
        "--body",
        action="append",
        default=[],
        help="body label/name to track while recording; repeat for several bodies",
    )
    parser.add_argument("--application-id", default="dart.trajectory")
    parser.add_argument("--entity-prefix", default="dart")
    return parser


def _flush_recording(rr: Any) -> None:
    for name in ("flush", "shutdown"):
        flush = getattr(rr, name, None)
        if callable(flush):
            flush()
            return


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    try:
        rr = _import_rerun()
    except MissingRerunError as exc:
        print(exc, file=sys.stderr)
        return 2

    try:
        trajectory_text = _load_trajectory_text(args)
        contact_text = _load_contact_text(args)
        samples = parse_trajectory_text(trajectory_text)
        contact_events = None
        if contact_text is not None:
            contact_events = parse_contact_events_text(contact_text)
        _configure_recording(
            rr,
            application_id=args.application_id,
            spawn=args.spawn,
            save=args.save,
        )
        frame_count = log_recording(
            rr,
            samples,
            contact_events=contact_events,
            entity_prefix=args.entity_prefix,
        )
        _flush_recording(rr)
    except (OSError, TypeError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
        print(f"rerun_trajectory.py: {exc}", file=sys.stderr)
        return 2

    if args.save is not None:
        print(f"wrote rerun recording: {args.save} ({frame_count} frames)")
    elif args.spawn:
        print(f"spawned rerun viewer ({frame_count} frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
