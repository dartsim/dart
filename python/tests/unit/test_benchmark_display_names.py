import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "benchmark_display_names.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("benchmark_display_names", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_humanize_dart6_curated_surfaces():
    module = _load_module()

    assert (
        module.humanize_name("BM_Kinematics/10")
        == "Skel kinematics update corpus - 10 iterations"
    )
    assert (
        module.humanize_name("BM_Dynamics/100")
        == "Skel dynamics step corpus - 100 steps"
    )
    assert (
        module.humanize_name("BM_InverseDynamicsViaMassMatrix/40")
        == "Dense mass-matrix inverse dynamics - 40 links"
    )
    assert (
        module.humanize_name("BM_ContactInverseDynamicsBasis/8")
        == "Contact inverse dynamics basis sweep - 8 friction bases"
    )
    assert (
        module.humanize_name("BM_RunBoxes/4")
        == "Stacked boxes world step - 4 grid side"
    )
    assert (
        module.humanize_name("BM_ContactContainerActive/60/0/16")
        == "Contact container active step - 60 objects - 0 engine - 16 threads"
    )
    assert (
        module.humanize_name(
            "BM_ContactContainerDeactivation/60/0/16/iterations:1"
        )
        == "Contact container deactivation-enabled step - 60 objects - 0 engine - 16 threads"
    )


def test_family_mapping_and_generic_fallback():
    module = _load_module()

    assert (
        module.family_of("BM_ContactContainerActive/120/1/16")
        == "DART 6 contact and collision"
    )
    assert module.family_of("BM_UnmappedCase/4") == "Other benchmarks"
    assert module.humanize_name("BM_UnmappedCase/4") == "Unmapped Case - 4 arg0"
    assert (
        module.humanize_name("BM_UnmappedCase/4/iterations:1")
        == "Unmapped Case - 4 arg0"
    )
