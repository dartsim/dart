from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional


@dataclass
class Recorder:
    """Simple JSONL recorder for scene states."""

    path: Optional[Path] = None
    meta: Dict[str, Any] = field(default_factory=dict)
    _fh: Optional[Any] = field(init=False, default=None)
    _start_time: float = field(init=False, default=0.0)
    _count: int = field(init=False, default=0)

    def start(self, path: str | Path, meta: Optional[Dict[str, Any]] = None) -> None:
        self.stop()
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("w", encoding="utf-8")
        self._start_time = time.time()
        self.meta = meta or {}
        self._count = 0
        header = {"type": "meta", "meta": self.meta, "started_at": self._start_time}
        self._fh.write(json.dumps(header) + "\n")

    def log(self, step: int, state: Dict[str, Any]) -> None:
        if self._fh is None:
            return
        payload = {
            "type": "state",
            "step": step,
            "timestamp": time.time(),
            "state": state,
        }
        self._fh.write(json.dumps(payload) + "\n")
        self._count += 1

    def stop(self) -> None:
        if self._fh:
            footer = {
                "type": "footer",
                "written": self._count,
                "ended_at": time.time(),
            }
            self._fh.write(json.dumps(footer) + "\n")
            self._fh.close()
        self._fh = None
        self._count = 0
