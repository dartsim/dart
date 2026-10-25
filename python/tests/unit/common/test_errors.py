import dartpy as dart
import pytest

E = dart.errors

# dartpy._testing is compiled in with BUILD_TESTING and left out of wheels.
_testing = getattr(dart, "_testing", None)
needs_testing_hooks = pytest.mark.skipif(
    _testing is None, reason="dartpy was built without its testing hooks"
)


def test_errors_module_exports_only_world_busy_error():
    assert [name for name in dir(E) if not name.startswith("_")] == ["WorldBusyError"]


@pytest.mark.parametrize(
    "module, name",
    [
        (E, "WorldBusyError"),
        (dart.gui, "RenderingUnavailableError"),
        (getattr(dart, "viewer", None), "AssetsNotPrefetchedError"),
    ],
)
def test_runtime_conditions_have_one_builtin_base(module, name):
    if module is None:
        pytest.skip("dartpy was built without the viewer")
    error = getattr(module, name)
    assert error.__bases__ == (RuntimeError,)
    assert error.__module__ == module.__name__
    instance = error("message")
    assert instance.args == ("message",)
    assert str(instance) == "message"


def test_invalid_argument_raises_value_error():
    with pytest.raises(ValueError, match="exceeds the maximum") as info:
        dart.optimizer.Problem(2_000_000)
    assert type(info.value) is ValueError


@needs_testing_hooks
@pytest.mark.parametrize(
    "kind, builtin",
    [
        ("invalid_argument", ValueError),
        ("length_error", ValueError),
        ("domain_error", ValueError),
        ("range_error", ValueError),
        ("out_of_range", IndexError),
        ("overflow_error", OverflowError),
        ("runtime_error", RuntimeError),
        ("logic_error", RuntimeError),
        ("underflow_error", RuntimeError),
    ],
)
def test_standard_exceptions_use_nanobind_defaults(kind, builtin):
    with pytest.raises(builtin, match="^probe message$") as info:
        _testing.throw_(kind, "probe message")
    assert type(info.value) is builtin


@needs_testing_hooks
@pytest.mark.parametrize("kind, builtin", [("exception", RuntimeError), ("bad_alloc", MemoryError)])
def test_standard_exceptions_without_a_message_use_nanobind_defaults(kind, builtin):
    with pytest.raises(builtin) as info:
        _testing.throw_(kind)
    assert type(info.value) is builtin


@needs_testing_hooks
def test_world_busy_runtime_error_is_translated():
    with pytest.raises(E.WorldBusyError, match="^WorldBusyError: another operation is active$") as info:
        _testing.throw_("runtime_error", "WorldBusyError: another operation is active")
    assert type(info.value) is E.WorldBusyError


@needs_testing_hooks
@pytest.mark.parametrize(
    "kind, message, builtin",
    [
        ("runtime_error", "WorldBusyError", RuntimeError),
        ("runtime_error", "WorldBusyError:missing space", RuntimeError),
        ("runtime_error", "prefix WorldBusyError: busy", RuntimeError),
        ("runtime_error", "ModelFrozenError: use world.edit()", RuntimeError),
        ("logic_error", "WorldBusyError: busy", RuntimeError),
        ("invalid_argument", "WorldBusyError: busy", ValueError),
        ("out_of_range", "WorldBusyError: busy", IndexError),
        ("overflow_error", "WorldBusyError: busy", OverflowError),
        ("range_error", "WorldBusyError: busy", ValueError),
        ("underflow_error", "WorldBusyError: busy", RuntimeError),
    ],
)
def test_world_busy_translation_requires_runtime_error_and_exact_prefix(kind, message, builtin):
    with pytest.raises(builtin) as info:
        _testing.throw_(kind, message)
    assert type(info.value) is builtin
    assert str(info.value) == message


@needs_testing_hooks
def test_non_std_exceptions_pass_down_the_translator_chain():
    # nanobind has no catch-all, so an unhandled exception ends in SystemError.
    with pytest.raises(SystemError, match="could not be translated"):
        _testing.throw_("int")


def test_argument_conversion_raises_plain_type_error():
    with pytest.raises(TypeError) as info:
        dart.common.warn(42)
    assert type(info.value) is TypeError


def test_testing_hooks_stay_out_of_the_public_namespace():
    assert "_testing" not in [name for name in dir(dart) if not name.startswith("_")]
    assert not hasattr(dart.common, "_log_from_threads")
