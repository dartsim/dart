from pathlib import Path

from demo_hub.core.recording import Recorder


def test_recorder_writes_jsonl(tmp_path: Path):
    rec = Recorder()
    out = tmp_path / "rec.jsonl"
    rec.start(out, {"scene": "dummy"})
    rec.log(0, {"foo": 1})
    rec.log(1, {"bar": 2})
    rec.stop()

    lines = out.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 4  # meta + 2 states + footer
    assert '"type": "meta"' in lines[0]
    assert '"type": "state"' in lines[1]
    assert '"type": "footer"' in lines[-1]
