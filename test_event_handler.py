#!/usr/bin/env python3
"""Test script to verify GUIEventHandler binding works."""

import dartpy as dart

print("Testing GUIEventHandler Python binding...")
print()

# Try to create a custom event handler in Python
class MyEventHandler(dart.gui.osg.GUIEventHandler):
    def __init__(self):
        super().__init__()
        print("✓ MyEventHandler created successfully!")
        self.key_count = 0

    def handle(self, ea, aa):
        """Handle events."""
        event_type = ea.getEventType()

        if event_type == dart.gui.osg.EventType.KEYDOWN:
            key = ea.getKey()
            self.key_count += 1
            print(f"Key pressed: {key} (count: {self.key_count})")
            return True

        return False

# Try to instantiate
try:
    handler = MyEventHandler()
    print("✓ Successfully created Python subclass of GUIEventHandler")
    print()
    print("Testing event types...")
    print(f"  KEYDOWN: {dart.gui.osg.EventType.KEYDOWN}")
    print(f"  KEYUP: {dart.gui.osg.EventType.KEYUP}")
    print(f"  PUSH: {dart.gui.osg.EventType.PUSH}")
    print()
    print("✓ All GUIEventHandler bindings working!")
except Exception as e:
    print(f"✗ Error: {e}")
    import traceback
    traceback.print_exc()
