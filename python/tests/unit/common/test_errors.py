import dartpy as dart
import pytest

E = dart.errors

# dartpy._testing is compiled in with BUILD_TESTING and left out of wheels.
_testing = getattr(dart, "_testing", None)
needs_testing_hooks = pytest.mark.skipif(
    _testing is None, reason="dartpy was built without its testing hooks"
)


@pytest.mark.parametrize(
    "error, builtin, branch",
    [
        (E.ValidationError, ValueError, E.DartError),
        (E.NotFoundError, KeyError, E.DartError),
        (E.OperationError, RuntimeError, E.DartError),
        (E.WorldBusyError, RuntimeError, E.OperationError),
        (E.ModelFrozenError, RuntimeError, E.OperationError),
        (E.CheckpointMismatchError, ValueError, E.ValidationError),
        (E.NameConflictError, ValueError, E.ValidationError),
    ],
)
def test_hierarchy(error, builtin, branch):
    assert issubclass(error, E.DartError)
    assert issubclass(error, branch)
    assert issubclass(error, builtin)
    assert error.__module__ == "dartpy.errors"


def test_branches_stay_apart():
    assert E.DartError.__module__ == "dartpy.errors"
    assert not issubclass(E.DartError, (ValueError, KeyError, RuntimeError))
    assert not issubclass(E.ValidationError, (RuntimeError, KeyError))
    assert not issubclass(E.OperationError, (ValueError, KeyError))
    assert not issubclass(E.NotFoundError, (ValueError, RuntimeError))


def test_renderer_errors_share_the_base():
    assert issubclass(dart.gui.RenderingUnavailableError, E.OperationError)
    if hasattr(dart, "viewer"):
        assert issubclass(dart.viewer.AssetsNotPrefetchedError, E.OperationError)


def test_invalid_argument_translates_to_validation_error():
    with pytest.raises(E.ValidationError, match="exceeds the maximum") as info:
        dart.optimizer.Problem(2_000_000)
    assert isinstance(info.value, ValueError)
    assert isinstance(info.value, E.DartError)
    with pytest.raises(ValueError):
        dart.optimizer.Problem(2_000_000)


def test_not_found_error_reads_as_its_message():
    message = "unknown link 'foo'; valid names: base, arm"
    error = E.NotFoundError(message)
    assert str(error) == message  # KeyError alone would quote it
    assert isinstance(error, KeyError)
    with pytest.raises(KeyError, match="valid names: base, arm"):
        raise error


@needs_testing_hooks
@pytest.mark.parametrize(
    "kind, expected",
    [
        ("runtime_error", E.OperationError),
        ("logic_error", E.OperationError),
        ("underflow_error", E.OperationError),
        ("exception", E.OperationError),
    ],
)
def test_other_std_exceptions_translate_to_operation_error(kind, expected):
    with pytest.raises(expected) as info:
        _testing.throw_(kind)
    assert isinstance(info.value, E.DartError)
    assert type(info.value) is expected


@needs_testing_hooks
@pytest.mark.parametrize(
    "kind, builtin",
    [
        ("out_of_range", IndexError),
        ("overflow_error", OverflowError),
        ("bad_alloc", MemoryError),
    ],
)
def test_deliberate_builtin_mappings_stay(kind, builtin):
    with pytest.raises(builtin) as info:
        _testing.throw_(kind)
    assert not isinstance(info.value, E.DartError)


@needs_testing_hooks
def test_non_std_exceptions_pass_down_the_translator_chain():
    # dartpy's translator leaves what it does not know to the next translator.
    # nanobind's has no catch-all, so the chain ends in SystemError rather than
    # in a silent return with no Python error set.
    with pytest.raises(SystemError, match="could not be translated") as info:
        _testing.throw_("int")
    assert not isinstance(info.value, E.DartError)


def test_argument_conversion_raises_plain_type_error():
    with pytest.raises(TypeError) as info:
        dart.common.warn(42)
    assert not isinstance(info.value, E.DartError)


def test_testing_hooks_stay_out_of_the_public_namespace():
    assert "_testing" not in [name for name in dir(dart) if not name.startswith("_")]
    assert not hasattr(dart.common, "_log_from_threads")
