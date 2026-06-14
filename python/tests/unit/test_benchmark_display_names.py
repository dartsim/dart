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
        module.humanize_name("BM_LcpCompare/Standard/Dantzig/12")
        == "LCP solver comparison · Standard surface · Dantzig solver · 12 rows"
    )
    assert (
        module.humanize_name("BM_LcpWorldContact/FrictionIndex/Pgs/4")
        == "LCP world contact · FrictionIndex surface · Pgs solver · 4 contacts"
    )
    assert (
        module.humanize_name("BM_LcpWorldBoxContact/FrictionIndex/NNCG/4")
        == "LCP world box contact · FrictionIndex surface · NNCG solver · 4 boxes"
    )
    assert (
        module.humanize_name("BM_LcpWorldBilliardsStep_BoxedLcp/4/1")
        == "LCP billiards world step · 4 pairs · 1 steps"
    )
    assert (
        module.humanize_name("BM_LcpWorldStackStep_BoxedLcp/4/200")
        == "LCP mass-ratio stack world step · 4 spheres · 200 steps"
    )
    assert (
        module.humanize_name("BM_LcpWorldCardPileStep_BoxedLcp/7/200")
        == "LCP card-pile world step · 7 cards · 200 steps"
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
        module.humanize_name("BM_AvbdEmptyWorldStep")
        == "AVBD empty baseline step"
    )
    assert (
        module.humanize_name("BM_AvbdRigidFixedJointStep/8")
        == "AVBD fixed-joint step · 8 links"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dMotorStep")
        == "AVBD demo2d motor step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dHangingRopeStep")
        == "AVBD demo2d hanging rope step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dFractureStep")
        == "AVBD demo2d fracture step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dDynamicFrictionStep")
        == "AVBD demo2d dynamic friction step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dFrictionCoefficientSweep/25")
        == "AVBD demo2d friction coefficient sweep · 25 max friction x10"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dStaticFrictionStep")
        == "AVBD demo2d static friction step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dPyramidStep")
        == "AVBD demo2d pyramid step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dCardsStep") == "AVBD demo2d cards step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dGroundStep")
        == "AVBD demo2d ground step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dStackStep")
        == "AVBD demo2d stack step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dStackRatioStep")
        == "AVBD demo2d stack ratio step"
    )
    assert module.humanize_name("BM_AvbdDemo2dRodStep") == "AVBD demo2d rod step"
    assert (
        module.humanize_name("BM_AvbdDemo2dSoftBodyStep")
        == "AVBD demo2d soft body step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dJointGridStep")
        == "AVBD demo2d joint grid step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dRopeStep") == "AVBD demo2d rope step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dHeavyRopeStep")
        == "AVBD demo2d heavy rope step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dSpringStep")
        == "AVBD demo2d spring step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo2dSpringRatioStep")
        == "AVBD demo2d spring ratio step"
    )
    assert module.humanize_name("BM_AvbdDemo2dNetStep") == "AVBD demo2d net step"
    assert (
        module.humanize_name("BM_AvbdDemo3dGroundStep")
        == "AVBD demo3d ground step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo3dDynamicFrictionStep")
        == "AVBD demo3d dynamic friction step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo3dStaticFrictionStep")
        == "AVBD demo3d static friction step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo3dPyramidStep")
        == "AVBD demo3d pyramid step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo3dRopeStep")
        == "AVBD demo3d rope step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo3dHeavyRopeStep")
        == "AVBD demo3d heavy rope step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo3dSpringStep")
        == "AVBD demo3d spring step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo3dSpringRatioStep")
        == "AVBD demo3d spring ratio step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo3dStackStep")
        == "AVBD demo3d stack step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo3dStackRatioStep")
        == "AVBD demo3d stack ratio step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo3dSoftBodyStep")
        == "AVBD demo3d soft body step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo3dBridgeStep")
        == "AVBD demo3d bridge step"
    )
    assert (
        module.humanize_name("BM_AvbdDemo3dBreakableStep")
        == "AVBD demo3d breakable step"
    )
    assert (
        module.humanize_name("BM_AvbdRigidBreakableJointStep/8")
        == "AVBD breakable-joint step · 8 breakable joints"
    )
    assert (
        module.humanize_name("BM_AvbdRigidSphericalBreakableJointStep/8")
        == "AVBD spherical breakable-joint step · 8 breakable joints"
    )
    assert (
        module.humanize_name("BM_AvbdRigidPrismaticMotorStep/8")
        == "AVBD prismatic-motor step · 8 motors"
    )
    assert (
        module.humanize_name("BM_AvbdArticulatedRevoluteMotorStep/8")
        == "AVBD articulated revolute-motor step · 8 motors"
    )
    assert (
        module.humanize_name("BM_AvbdArticulatedBreakableMotorStep/8")
        == "AVBD articulated breakable-motor step · 8 breakable motors"
    )
    assert (
        module.humanize_name("BM_AvbdArticulatedPrismaticMotorStep/8")
        == "AVBD articulated prismatic-motor step · 8 motors"
    )
    assert (
        module.humanize_name("BM_AvbdArticulatedPrismaticBreakableMotorStep/8")
        == "AVBD articulated prismatic breakable-motor step · 8 breakable motors"
    )
    assert (
        module.humanize_name("BM_AvbdArticulatedWorldPrismaticBreakableMotorStep/8")
        == "AVBD articulated world-prismatic breakable-motor step · 8 breakable motors"
    )
    assert (
        module.humanize_name("BM_AvbdArticulatedWorldRevoluteBreakableMotorStep/8")
        == "AVBD articulated world-revolute breakable-motor step · 8 breakable motors"
    )
    assert (
        module.humanize_name("BM_AvbdArticulatedBreakableJointStep/8")
        == "AVBD articulated breakable-joint step · 8 breakable joints"
    )
    assert (
        module.humanize_name("BM_AvbdArticulatedWorldSphericalBreakableJointStep/8")
        == (
            "AVBD articulated world-spherical breakable-joint step · "
            "8 breakable joints"
        )
    )
    assert (
        module.humanize_name("BM_AvbdArticulatedSphericalPairBreakableJointStep/8")
        == (
            "AVBD articulated spherical-pair breakable-joint step · "
            "8 breakable joints"
        )
    )
    assert (
        module.humanize_name("BM_AvbdArticulatedHighRatioChainStep")
        == "AVBD articulated high-ratio chain step"
    )
    assert (
        module.humanize_name("BM_AvbdPaperScaleHighRatioChainStep")
        == "AVBD paper-scale high-ratio chain step"
    )
    assert (
        module.humanize_name("BM_AvbdPaperScaleHighRatioChainIterationSweep/100")
        == "AVBD paper-scale high-ratio chain iteration sweep · "
        "100 max iterations"
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
    assert module.family_of("BM_LcpCompare/Standard/Dantzig/12") == module.FAMILY_LCP
    assert (
        module.family_of("BM_LcpWorldContact/FrictionIndex/Pgs/4")
        == module.FAMILY_LCP
    )
    assert (
        module.family_of("BM_LcpWorldBoxContact/FrictionIndex/NNCG/4")
        == module.FAMILY_LCP
    )
    assert (
        module.family_of("BM_LcpWorldBilliardsStep_BoxedLcp/4/1")
        == module.FAMILY_LCP
    )
    assert (
        module.family_of("BM_LcpWorldStackStep_BoxedLcp/4/200")
        == module.FAMILY_LCP
    )
    assert (
        module.family_of("BM_LcpWorldCardPileStep_BoxedLcp/4/200")
        == module.FAMILY_LCP
    )
    assert module.family_of("BM_VbdWorldStepDefault/8") == module.FAMILY_VBD
    assert module.family_of("BM_DeformableFemBarStep/2") == module.FAMILY_FEM
    assert module.family_of("BM_AvbdEmptyWorldStep") == module.FAMILY_AVBD
    assert (
        module.family_of("BM_AvbdPaperScaleHighRatioChainIterationSweep/100")
        == module.FAMILY_AVBD
    )
    assert module.family_of("BM_AvbdDemo2dMotorStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo2dHangingRopeStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo2dFractureStep") == module.FAMILY_AVBD
    assert (
        module.family_of("BM_AvbdDemo2dDynamicFrictionStep")
        == module.FAMILY_AVBD
    )
    assert (
        module.family_of("BM_AvbdDemo2dFrictionCoefficientSweep/25")
        == module.FAMILY_AVBD
    )
    assert (
        module.family_of("BM_AvbdDemo2dStaticFrictionStep")
        == module.FAMILY_AVBD
    )
    assert module.family_of("BM_AvbdDemo2dGroundStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo2dPyramidStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo2dCardsStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo2dStackStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo2dStackRatioStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo2dRodStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo2dJointGridStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo2dRopeStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo2dHeavyRopeStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo2dSpringStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo2dSpringRatioStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo2dNetStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo3dGroundStep") == module.FAMILY_AVBD
    assert (
        module.family_of("BM_AvbdDemo3dDynamicFrictionStep") == module.FAMILY_AVBD
    )
    assert module.family_of("BM_AvbdDemo3dStaticFrictionStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo3dPyramidStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo3dRopeStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo3dHeavyRopeStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo3dSpringStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo3dSpringRatioStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo3dStackStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo3dStackRatioStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo3dSoftBodyStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo3dBridgeStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdDemo3dBreakableStep") == module.FAMILY_AVBD
    assert module.family_of("BM_AvbdRigidFixedJointStep/1") == module.FAMILY_AVBD
    assert (
        module.family_of("BM_AvbdArticulatedRevoluteMotorStep/1")
        == module.FAMILY_AVBD
    )
    assert (
        module.family_of("BM_AvbdArticulatedBreakableMotorStep/1")
        == module.FAMILY_AVBD
    )
    assert (
        module.family_of("BM_AvbdArticulatedPrismaticBreakableMotorStep/1")
        == module.FAMILY_AVBD
    )
    assert (
        module.family_of("BM_AvbdArticulatedWorldPrismaticBreakableMotorStep/1")
        == module.FAMILY_AVBD
    )
    assert (
        module.family_of("BM_AvbdArticulatedWorldRevoluteBreakableMotorStep/1")
        == module.FAMILY_AVBD
    )
    assert (
        module.family_of("BM_AvbdArticulatedBreakableJointStep/1")
        == module.FAMILY_AVBD
    )
    assert (
        module.family_of("BM_AvbdRigidSphericalBreakableJointStep/1")
        == module.FAMILY_AVBD
    )
    assert (
        module.family_of("BM_AvbdArticulatedWorldSphericalBreakableJointStep/1")
        == module.FAMILY_AVBD
    )
    assert (
        module.family_of("BM_AvbdArticulatedSphericalPairBreakableJointStep/1")
        == module.FAMILY_AVBD
    )
    assert (
        module.family_of("BM_AvbdArticulatedHighRatioChainStep")
        == module.FAMILY_AVBD
    )
    assert (
        module.family_of("BM_AvbdPaperScaleHighRatioChainStep")
        == module.FAMILY_AVBD
    )
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
