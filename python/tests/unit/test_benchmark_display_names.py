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


def test_humanize_curated_core_surfaces():
    module = _load_module()
    assert (
        module.humanize_name("BM_WorldStepParallel/128/32")
        == "World step (parallel) · 128 parents · 32 children/parent"
    )
    assert (
        module.humanize_name("BM_RigidBodyStepSequential/1024")
        == "Rigid-body step (sequential) · 1024 bodies"
    )
    assert (
        module.humanize_name("BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10")
        == "Rigid-body batch (CPU baseline) · 1024 worlds · 128 bodies · 10 steps"
    )


def test_humanize_new_solver_surfaces():
    module = _load_module()
    assert (
        module.humanize_name("BM_RigidWorldStep_SequentialImpulse/2")
        == "Rigid world step (sequential impulse) · 2 boxes"
    )
    assert (
        module.humanize_name("BM_VbdWorldStepVbd/16")
        == "Deformable world step (VBD) · 16×16 grid"
    )
    assert (
        module.humanize_name("BM_DeformableFemBarStep/24")
        == "FEM bar step · 24 cells"
    )
    assert (
        module.humanize_name("BM_AvbdRigidFixedJointStep/8")
        == "AVBD fixed-joint step · 8 links"
    )


def test_humanize_generic_fallback_for_unmapped_names():
    module = _load_module()
    # Unknown base: drop BM_, split CamelCase, keep the raw arg.
    assert module.humanize_name("BM_SomeNewKernel/64") == "Some New Kernel · 64 arg0"
    # No args, no prefix.
    assert module.humanize_name("PlainName") == "Plain Name"


def test_family_grouping():
    module = _load_module()
    assert module.family_of("BM_WorldStepParallel/128/32") == module.FAMILY_CORE
    assert module.family_of("BM_RigidWorldStep_Ipc/4") == module.FAMILY_RIGID
    assert module.family_of("BM_VbdWorldStepDefault/8") == module.FAMILY_VBD
    assert module.family_of("BM_DeformableFemBarStep/2") == module.FAMILY_FEM
    assert module.family_of("BM_AvbdRigidFixedJointStep/1") == module.FAMILY_AVBD
    assert module.family_of("BM_Unmapped") == module.FAMILY_OTHER


def test_humanized_names_stay_unique_per_arg_row():
    """Each parameterized row must map to a distinct series key."""
    module = _load_module()
    raw = [
        "BM_RigidWorldStep_Ipc/1",
        "BM_RigidWorldStep_Ipc/2",
        "BM_RigidWorldStep_Ipc/4",
        "BM_VbdWorldStepVbd/8",
        "BM_VbdWorldStepVbd/16",
    ]
    titles = [module.humanize_name(name) for name in raw]
    assert len(set(titles)) == len(titles)
