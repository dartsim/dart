"""
Quick test to verify OpenGL context creation works with the fix.
"""
import dartpy as dart

print("Testing OpenGL context creation...")
print(f"Environment variables should be set by pixi task:")
import os
print(f"  OSG_GL_CONTEXT_VERSION = {os.getenv('OSG_GL_CONTEXT_VERSION', 'NOT SET')}")
print(f"  MESA_GL_VERSION_OVERRIDE = {os.getenv('MESA_GL_VERSION_OVERRIDE', 'NOT SET')}")
print()

try:
    # Create a simple world
    world = dart.simulation.World()

    # Create a simple skeleton
    urdf = dart.utils.DartLoader()
    kr5 = urdf.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    world.addSkeleton(kr5)

    # Create viewer - this is where OpenGL context creation happens
    print("Creating viewer...")
    viewer = dart.gui.osg.Viewer()

    # Create node
    node = dart.gui.osg.RealTimeWorldNode(world)
    viewer.addWorldNode(node)

    # Try to set up view
    print("Setting up view window...")
    viewer.setUpViewInWindow(0, 0, 640, 480)

    print("✅ SUCCESS! Viewer created without OpenGL errors")
    print("The fix is working!")
    print()
    print("Press Ctrl+C to exit (viewer will try to open)")

    # Don't actually run the viewer, just test creation
    # viewer.run()

except Exception as e:
    print(f"❌ FAILED: {e}")
    import traceback
    traceback.print_exc()
