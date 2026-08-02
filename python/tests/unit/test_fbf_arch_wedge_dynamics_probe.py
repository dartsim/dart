from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROBE_SOURCE = (
    ROOT / "tests/benchmark/integration/fbf_paper_arch_wedge_dynamics_probe.cpp"
)


def test_colored_bgs_is_enabled_on_the_installed_exact_solver():
    source = PROBE_SOURCE.read_text(encoding="utf-8")
    make_scene = source[
        source.index("Scene makeScene(const Options& options)") : source.index(
            "dart::constraint::ExactCoulombFbfConstraintSolver* getExactSolver"
        )
    ]

    install = make_scene.index("scene.world->setConstraintSolver(std::move(solver));")
    retrieve = make_scene.index("scene.world->getConstraintSolver()", install)
    enable = make_scene.index(
        "installed->setExactCoulombColoredBlockGaussSeidelEnabled(", retrieve
    )

    assert install < retrieve < enable
    assert "solver->setExactCoulombColoredBlockGaussSeidelEnabled(" not in make_scene
