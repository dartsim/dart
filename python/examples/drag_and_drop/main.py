import numpy as np
import dartpy as dart


def main():
    world = dart.simulation.World()

    tf = dart.math.Isometry3()

    tf.set_translation([4, -4, 0])
    frame = dart.gui.osg.InteractiveFrame(
        dart.dynamics.Frame.World(), "interactive frame", tf, 2
    )
    world.addSimpleFrame(frame)

    tf.set_translation([-4, 4, 0])
    draggable = dart.dynamics.SimpleFrame(frame, "draggable", tf)
    draggable.setShape(dart.dynamics.BoxShape([1, 1, 1]))
    draggable.getVisualAspect(True).setColor([0.9, 0, 0])
    world.addSimpleFrame(draggable)

    tf.set_translation([8, 0, 0])
    x_marker = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "X", tf)
    x_shape = dart.dynamics.BoxShape([0.2, 0.2, 0.2])
    x_marker.setShape(x_shape)
    x_marker.getVisualAspect(True).setColor([0.9, 0, 0])
    world.addSimpleFrame(x_marker)

    tf.set_translation([0, 8, 0])
    y_marker = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "Y", tf)
    y_shape = dart.dynamics.BoxShape([0.2, 0.2, 0.2])
    y_marker.setShape(y_shape)
    y_marker.getVisualAspect(True).setColor([0, 0.9, 0])
    world.addSimpleFrame(y_marker)

    tf.set_translation([0, 0, 8])
    z_marker = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "Z", tf)
    z_shape = dart.dynamics.BoxShape([0.2, 0.2, 0.2])
    z_marker.setShape(z_shape)
    z_marker.getVisualAspect(True).setColor([0, 0, 0.9])
    world.addSimpleFrame(z_marker)

    node = dart.gui.osg.WorldNode(world)

    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)
    viewer.enableDragAndDrop(frame)
    viewer.enableDragAndDrop(draggable)

    viewer.addInstructionText("\nCtrl + Left-click: Rotate the box\n")
    print(viewer.getInstructions())

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([20, 17, 17], [0, 0, 0], [0, 0, 1])
    viewer.run()


if __name__ == "__main__":
    main()
