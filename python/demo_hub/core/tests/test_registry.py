from demo_hub.core import build_default_registry


def test_registry_discovers_builtin_scene():
    registry = build_default_registry()
    assert "hello_world" in registry.scene_ids
    scene = registry.create("hello_world")
    assert scene.metadata.scene_id == "hello_world"

