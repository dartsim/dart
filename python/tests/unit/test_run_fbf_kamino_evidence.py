#!/usr/bin/env python3
# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the following "BSD-style" License:
#   Redistribution and use in source and binary forms, with or
#   without modification, are permitted provided that the following
#   conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.

import importlib.util
import math
import pathlib
import unittest

SCRIPT = pathlib.Path(__file__).parents[3] / "scripts" / "run_fbf_kamino_evidence.py"
SPEC = importlib.util.spec_from_file_location("run_fbf_kamino_evidence", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def row(**overrides):
    result = {
        "elapsed_ms": 5.0,
        "time_s": 1.0 / 60.0,
        "dt_s": 1.0 / 60.0,
        "contacts": 1,
        "finite_state": True,
        "x_m": 0.0,
        "y_m": 0.0,
        "z_m": MODULE.BACKSPIN_RADIUS,
        "vx_m_s": MODULE.BACKSPIN_ANALYTIC_VX,
        "vy_m_s": 0.0,
        "vz_m_s": 0.0,
        "wx_rad_s": 0.0,
        "wy_rad_s": MODULE.BACKSPIN_ANALYTIC_WY,
        "wz_rad_s": 0.0,
        "downhill_displacement_m": math.nan,
        "downhill_speed_m_s": math.nan,
        "backspin_slip_m_s": 0.0,
        "analytic_terminal_displacement_m": math.nan,
    }
    result.update(overrides)
    return result


class RunFbfKaminoEvidenceTest(unittest.TestCase):
    def test_specs_preserve_reconstructed_fixture_contract(self):
        backspin = MODULE.scenario_spec("backspin")
        slide = MODULE.scenario_spec("incline_mu_0_4")
        self.assertEqual(backspin["mass_kg"], 1.0)
        self.assertEqual(backspin["geometry"]["sphere_radius_m"], 0.25)
        self.assertEqual(backspin["initial_angular_velocity_rad_s"][1], -200.0)
        self.assertAlmostEqual(slide["geometry"]["incline_tan"], 0.5)
        self.assertAlmostEqual(
            slide["analytic_terminal_displacement_m"], 1.755, places=3
        )

    def test_backspin_summary_reports_timing_and_physical_pass(self):
        rows = [
            row(elapsed_ms=4.0, time_s=4.0 - 1.0 / 60.0),
            row(elapsed_ms=6.0, time_s=4.0),
        ]
        summary = MODULE.summarize_rows("backspin", rows)
        self.assertEqual(summary["mean_step_ms"], 5.0)
        self.assertEqual(summary["median_step_ms"], 5.0)
        self.assertTrue(summary["all_steps_within_dt"])
        self.assertTrue(summary["fixture_check_pass"])

    def test_truncated_smoke_does_not_claim_a_fixture_verdict(self):
        summary = MODULE.summarize_rows("backspin", [row()])
        self.assertFalse(summary["full_duration_reached"])
        self.assertIsNone(summary["fixture_check_pass"])

    def test_incline_verdict_distinguishes_stick_and_slide(self):
        stick = row(downhill_displacement_m=0.01, backspin_slip_m_s=math.nan)
        slide_spec = MODULE.scenario_spec("incline_mu_0_4")
        slide = row(
            downhill_displacement_m=slide_spec["analytic_terminal_displacement_m"],
            backspin_slip_m_s=math.nan,
            analytic_terminal_displacement_m=slide_spec[
                "analytic_terminal_displacement_m"
            ],
        )
        self.assertTrue(MODULE.fixture_verdict("incline_mu_0_5", stick)[0])
        self.assertTrue(MODULE.fixture_verdict("incline_mu_0_4", slide)[0])
        slide["downhill_displacement_m"] = 0.0
        self.assertFalse(MODULE.fixture_verdict("incline_mu_0_4", slide)[0])

    def test_report_refuses_apples_to_apples_claim(self):
        metadata = {
            "software": {"newton_version": "1.3.0", "warp_version": "1.15.0"},
            "device": {"name": "test GPU"},
        }
        summary = MODULE.summarize_rows("backspin", [row(time_s=4.0)])
        report = MODULE.render_report(metadata, [summary])
        self.assertIn("not an apples-to-apples paper reproduction", report)
        self.assertIn("raw.csv", report)

    def test_json_safe_replaces_non_finite_metrics(self):
        value = MODULE.json_safe({"missing": math.nan, "nested": [math.inf, 1.0]})
        self.assertEqual(value, {"missing": None, "nested": [None, 1.0]})

    def test_non_finite_numeric_cli_inputs_fail_before_optional_imports(self):
        for flag in ("--dt", "--tolerance"):
            for value in ("nan", "inf", "-inf"):
                with self.subTest(flag=flag, value=value):
                    with self.assertRaisesRegex(ValueError, "finite and positive"):
                        MODULE.main(["--output-dir", "unused", f"{flag}={value}"])

    def test_artifact_binding_records_size_and_digest(self):
        import hashlib
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "artifact"
            path.write_bytes(b"evidence")
            self.assertEqual(
                MODULE._artifact_binding(path),
                {
                    "bytes": len(b"evidence"),
                    "sha256": hashlib.sha256(b"evidence").hexdigest(),
                },
            )

    def test_output_guard_requires_force(self):
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory)
            (output / "raw.csv").write_text("existing", encoding="utf-8")
            with self.assertRaises(FileExistsError):
                MODULE._prepare_output(output, force=False)
            MODULE._prepare_output(output, force=True)
            self.assertFalse((output / "raw.csv").exists())

    def test_force_invalidates_complete_owned_set_and_preserves_unrelated(self):
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory)
            owned = (
                "metadata.json",
                "raw.csv",
                "summary.csv",
                "summary.json",
                "REPORT.md",
            )
            for name in owned:
                (output / name).write_text("stale", encoding="utf-8")
            unrelated = output / "notes.txt"
            unrelated.write_text("keep", encoding="utf-8")

            MODULE._prepare_output(output, force=True)

            self.assertFalse(any((output / name).exists() for name in owned))
            self.assertEqual(unrelated.read_text(encoding="utf-8"), "keep")


if __name__ == "__main__":
    unittest.main()
