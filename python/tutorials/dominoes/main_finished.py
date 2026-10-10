"""Dominoes: the completed cloning, collision, and robot-control lessons."""

import math

import dartpy as dart
import numpy as np

default_domino_height = 0.3
default_domino_width = 0.4 * default_domino_height
default_domino_depth = default_domino_width / 5.0
default_distance = default_domino_height / 2.0
default_angle = math.radians(20.0)
default_domino_density = 2.6e3  # kg/m^3
default_domino_mass = (
    default_domino_density
    * default_domino_height
    * default_domino_width
    * default_domino_depth
)
default_push_force = 8.0  # N
default_force_duration = 200  # simulation steps
default_push_duration = 1000  # simulation steps
default_playback_frame_step = 16
default_contact_force_scale = 0.1
default_endeffector_offset = 0.05


class Controller:
    def __init__(self, manipulator, domino):
        self.mManipulator = manipulator
        # Lesson 2b: hold the initial joint angles with a PD controller.
        self.mQDesired = manipulator.getPositions().copy()

        # Lesson 3a: aim the last body at the top of the first domino.
        self.mEndEffector = manipulator.getBodyNode(manipulator.getNumBodyNodes() - 1)
        self.mOffset = np.array([default_endeffector_offset, 0.0, 0.0])
        self.mTarget = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "target")
        target_offset = dart.math.Isometry3()
        target_offset.set_translation([0.0, 0.0, default_domino_height / 2.0])
        target_offset.set_rotation(
            self.mEndEffector.getTransform(domino.getBodyNode(0)).rotation()
        )
        self.mTarget.setTransform(target_offset, domino.getBodyNode(0))

        self.mKpPD = 200.0
        self.mKdPD = 20.0
        self.mKpOS = 5.0
        self.mKdOS = 0.01
        self.mForces = np.zeros(manipulator.getNumDofs())

    def setPDForces(self):
        """Lessons 2c/2d: stable PD with gravity and Coriolis compensation."""
        if self.mManipulator is None:
            return
        q = self.mManipulator.getPositions()
        dq = self.mManipulator.getVelocities()
        q_err = self.mQDesired - (q + dq * self.mManipulator.getTimeStep())
        dq_err = -dq
        mass = self.mManipulator.getMassMatrix()
        cg = self.mManipulator.getCoriolisAndGravityForces()
        self.mForces = mass @ (self.mKpPD * q_err + self.mKdPD * dq_err) + cg
        self.mManipulator.setForces(self.mForces)

    def setOperationalSpaceForces(self):
        """Lesson 3b: track the target while exerting a workspace push."""
        if self.mManipulator is None:
            return
        mass = self.mManipulator.getMassMatrix()
        jacobian = self.mEndEffector.getWorldJacobian(self.mOffset)
        pinv_j = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + 0.0025 * np.eye(6))
        derivative = self.mEndEffector.getJacobianClassicDeriv(self.mOffset)
        pinv_dj = derivative.T @ np.linalg.inv(
            derivative @ derivative.T + 0.0025 * np.eye(6)
        )

        error = np.zeros(6)
        end_tf = self.mEndEffector.getWorldTransform()
        error[3:] = self.mTarget.getWorldTransform().translation() - (
            end_tf.rotation() @ self.mOffset + end_tf.translation()
        )
        aa = dart.math.AngleAxis(
            self.mTarget.getTransform(self.mEndEffector).rotation()
        )
        error[:3] = aa.angle() * aa.axis()
        de = -self.mEndEffector.getSpatialVelocity(
            self.mOffset, self.mTarget, dart.dynamics.Frame.World()
        )
        cg = self.mManipulator.getCoriolisAndGravityForces()
        kp = self.mKpOS * np.eye(6)
        kd = self.mKdOS * np.eye(self.mManipulator.getNumDofs())
        f_desired = np.zeros(6)
        f_desired[3] = default_push_force
        dq = self.mManipulator.getVelocities()
        self.mForces = (
            mass @ (pinv_j @ kp @ de + pinv_dj @ kp @ error)
            - kd @ dq
            + kd @ pinv_j @ kp @ error
            + cg
            + jacobian.T @ f_desired
        )
        self.mManipulator.setForces(self.mForces)


class DominoEventHandler(dart.gui.osg.GUIEventHandler):
    def __init__(self, world, controller, viewer=None):
        super().__init__()
        self.mWorld = world
        self.mController = controller
        self.mViewer = viewer
        self.mFirstDomino = world.getSkeleton("domino")
        self.mFloor = world.getSkeleton("floor")
        self.mDominoes = []
        self.mAngles = []
        self.mTotalAngle = 0.0
        self.mHasEverRun = False
        self.mPlayingBack = False
        self.mShowContactForces = False
        self.mPlayFrame = 0
        self.mForceCountDown = 0
        self.mPushCountDown = 0
        self.mContactForceFrames = []
        self.mContactForceArrows = []

    def handle(self, ea, _aa):
        if ea.getEventType() != dart.gui.osg.GUIEventAdapter.KEYDOWN:
            return False
        key = ea.getKey()
        if key == ord("p"):
            self.togglePlayback()
            return True
        if key == ord("v"):
            self.toggleContactForces()
            return True

        if not self.mHasEverRun:
            if key in (ord("q"), ord("w"), ord("e")):
                angle = {
                    ord("q"): default_angle,
                    ord("w"): 0.0,
                    ord("e"): -default_angle,
                }
                self.attemptToCreateDomino(angle[key])
                return True
            if key == ord("d"):
                self.deleteLastDomino()
                return True
            if key == ord(" "):
                self.mHasEverRun = True
                self.stopPlayback()
                if self.mViewer is not None:
                    self.mViewer.simulate(True)
                return True
            return False

        if key == ord(" "):
            self.stopPlayback()
            return False
        if key == ord("f"):
            self.mForceCountDown = default_force_duration
            return True
        if key == ord("r"):
            self.mPushCountDown = default_push_duration
            return True
        return False

    def showPlaybackFrame(self):
        if not self.mPlayingBack:
            return
        recording = self.mWorld.getRecording()
        num_frames = recording.getNumFrames()
        if (
            num_frames == 0
            or recording.getNumSkeletons() != self.mWorld.getNumSkeletons()
        ):
            self.stopPlayback()
            return
        if any(
            recording.getNumDofs(i) != self.mWorld.getSkeleton(i).getNumDofs()
            for i in range(self.mWorld.getNumSkeletons())
        ):
            self.stopPlayback()
            return
        if self.mPlayFrame >= num_frames:
            self.mPlayFrame = 0
        for i in range(self.mWorld.getNumSkeletons()):
            self.mWorld.getSkeleton(i).setPositions(
                recording.getConfig(self.mPlayFrame, i)
            )
        self.updateRecordedContactForces(self.mPlayFrame)
        self.mPlayFrame += default_playback_frame_step

    def bakeFrame(self):
        self.mWorld.bake()

    def update(self):
        # Lesson 1d: push at the top of the domino for a fixed number of steps.
        if self.mForceCountDown > 0:
            self.mFirstDomino.getBodyNode(0).addExtForce(
                [default_push_force, 0.0, 0.0],
                [0.0, 0.0, default_domino_height / 2.0],
            )
            self.mForceCountDown -= 1
        if self.mPushCountDown > 0:
            self.mController.setOperationalSpaceForces()
            self.mPushCountDown -= 1
        else:
            self.mController.setPDForces()

    def updateContactForces(self):
        if not self.mShowContactForces:
            self.hideContactForces()
            return
        if self.mPlayingBack:
            return
        result = self.mWorld.getLastCollisionResult()
        count = result.getNumContacts()
        self.ensureContactForceVisuals(count)
        for i in range(count):
            contact = result.getContact(i)
            self.setContactForceVisual(
                i, contact.point, default_contact_force_scale * contact.force
            )
        self.hideContactForces(count)

    def attemptToCreateDomino(self, angle):
        """Lessons 1a/1b: clone and place a domino, accepting only a clear pose."""
        new_domino = self.mFirstDomino.clone()
        new_domino.setName("domino #" + str(len(self.mDominoes) + 1))
        last_domino = self.mDominoes[-1] if self.mDominoes else self.mFirstDomino
        dx = default_distance * np.array(
            [math.cos(self.mTotalAngle), math.sin(self.mTotalAngle), 0.0]
        )
        positions = last_domino.getPositions().copy()
        positions[3:] += dx
        positions[2] = self.mTotalAngle + angle
        new_domino.setPositions(positions)

        solver = self.mWorld.getConstraintSolver()
        collision_group = solver.getCollisionGroup()
        new_group = solver.getCollisionDetector().createCollisionGroup()
        new_group.addShapeFramesOf(new_domino)
        collision_group.removeShapeFramesOf(self.mFloor)
        try:
            domino_collision = collision_group.collide(new_group)
        finally:
            # Restore the floor even if the collision query fails.
            collision_group.addShapeFramesOf(self.mFloor)
        if domino_collision:
            print("The new domino would penetrate something. I will not add")
            print("it to the world. Remove some dominos with 'd' and try again")
            return
        self.mWorld.addSkeleton(new_domino)
        self.mAngles.append(angle)
        self.mDominoes.append(new_domino)
        self.mTotalAngle += angle

    def deleteLastDomino(self):
        """Lesson 1c: delete the last clone, retaining the original domino."""
        if self.mDominoes:
            self.mWorld.removeSkeleton(self.mDominoes.pop())
            self.mTotalAngle -= self.mAngles.pop()

    def togglePlayback(self):
        recording = self.mWorld.getRecording()
        if recording.getNumFrames() == 0:
            print("No recorded frames are available for replay.")
            return
        self.mPlayingBack = not self.mPlayingBack
        if self.mPlayingBack and self.mViewer is not None:
            self.mViewer.simulate(False)
        if self.mPlayingBack and self.mPlayFrame >= recording.getNumFrames():
            self.mPlayFrame = 0

    def toggleContactForces(self):
        self.mShowContactForces = not self.mShowContactForces
        if not self.mShowContactForces:
            self.hideContactForces()
            print("Contact force visualization disabled.")
            return
        print("Contact force visualization enabled.")
        if self.mPlayingBack:
            num_frames = self.mWorld.getRecording().getNumFrames()
            if num_frames == 0:
                self.hideContactForces()
                return
            if self.mPlayFrame >= num_frames:
                self.mPlayFrame = 0
            self.updateRecordedContactForces(self.mPlayFrame)
        else:
            self.updateContactForces()

    def stopPlayback(self):
        self.mPlayingBack = False

    def updateRecordedContactForces(self, frame):
        if not self.mShowContactForces:
            return
        recording = self.mWorld.getRecording()
        count = recording.getNumContacts(frame)
        self.ensureContactForceVisuals(count)
        for i in range(count):
            self.setContactForceVisual(
                i,
                recording.getContactPoint(frame, i),
                default_contact_force_scale * recording.getContactForce(frame, i),
            )
        self.hideContactForces(count)

    def ensureContactForceVisuals(self, count):
        while len(self.mContactForceFrames) < count:
            frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
            arrow = dart.dynamics.ArrowShape(
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.01],
                dart.dynamics.ArrowShapeProperties(0.002, 2.0, 0.15),
                [0.2, 0.2, 0.8, 1.0],
            )
            frame.setShape(arrow)
            frame.createVisualAspect().setHidden(True)
            self.mWorld.addSimpleFrame(frame)
            self.mContactForceFrames.append(frame)
            self.mContactForceArrows.append(arrow)

    def setContactForceVisual(self, index, point, force):
        visual = self.mContactForceFrames[index].getVisualAspect()
        if np.linalg.norm(force) < 1e-8:
            visual.setHidden(True)
            return
        visual.setHidden(False)
        self.mContactForceArrows[index].setPositions(point, np.asarray(point) + force)

    def hideContactForces(self, start=0):
        for frame in self.mContactForceFrames[start:]:
            frame.getVisualAspect().setHidden(True)


class CustomWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, handler):
        super().__init__(world)
        self.world = world
        self.handler = handler
        self.controller = handler.mController

    def customPreRefresh(self):
        self.handler.showPlaybackFrame()

    def customPreStep(self):
        self.handler.update()

    def customPostStep(self):
        self.handler.bakeFrame()
        self.handler.updateContactForces()


def createDomino():
    domino = dart.dynamics.Skeleton("domino")
    _, body = domino.createFreeJointAndBodyNodePair()
    box = dart.dynamics.BoxShape(
        [default_domino_depth, default_domino_width, default_domino_height]
    )
    shape = body.createShapeNode(box)
    shape.createVisualAspect()
    shape.createCollisionAspect()
    shape.createDynamicsAspect()
    inertia = dart.dynamics.Inertia()
    inertia.setMass(default_domino_mass)
    inertia.setMoment(box.computeInertia(default_domino_mass))
    body.setInertia(inertia)
    domino.getDof("Joint_pos_z").setPosition(default_domino_height / 2.0)
    return domino


def createFloor():
    floor = dart.dynamics.Skeleton("floor")
    _, body = floor.createWeldJointAndBodyNodePair()
    floor_width = 10.0
    floor_height = 0.01
    box = dart.dynamics.BoxShape([floor_width, floor_width, floor_height])
    shape = body.createShapeNode(box)
    shape.createVisualAspect().setColor([0.0, 0.0, 0.0])
    shape.createCollisionAspect()
    shape.createDynamicsAspect()
    tf = dart.math.Isometry3()
    tf.set_translation([0.0, 0.0, -floor_height / 2.0])
    body.getParentJoint().setTransformFromParentBodyNode(tf)
    return floor


def createManipulator():
    """Lesson 2a: load and position the six-axis KR5 manipulator."""
    loader = dart.utils.DartLoader()
    manipulator = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    if manipulator is None:
        raise RuntimeError("Could not load the KR5 manipulator sample URDF.")
    manipulator.setName("manipulator")
    tf = dart.math.Isometry3()
    tf.set_translation([-0.65, 0.0, 0.0])
    manipulator.getJoint(0).setTransformFromParentBodyNode(tf)
    manipulator.getDof(1).setPosition(math.radians(140.0))
    manipulator.getDof(2).setPosition(math.radians(-140.0))
    return manipulator


def build_scene():
    domino = createDomino()
    floor = createFloor()
    manipulator = createManipulator()
    world = dart.simulation.World()
    world.addSkeleton(domino)
    world.addSkeleton(floor)
    world.addSkeleton(manipulator)
    controller = Controller(manipulator, domino)
    handler = DominoEventHandler(world, controller)
    return CustomWorldNode(world, handler)


def main():
    node = build_scene()
    viewer = dart.gui.osg.Viewer()
    node.handler.mViewer = viewer
    viewer.addWorldNode(node)
    viewer.addEventHandler(node.handler)
    viewer.addInstructionText(
        "Before simulation has started, you can create new dominoes:\n"
        "'w': Create new domino angled forward\n"
        "'q': Create new domino angled to the left\n"
        "'e': Create new domino angled to the right\n"
        "'d': Delete the last domino that was created\n\n"
        "spacebar: Begin simulation (you can no longer create or remove dominoes)\n"
        "'p': replay simulation\n"
        "'f': Push the first domino with a disembodied force so that it falls over\n"
        "'r': Push the first domino with the manipulator so that it falls over\n"
        "'v': Turn contact force visualization on/off\n"
    )
    print(viewer.getInstructions())
    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([2.0, 1.0, 2.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0])
    viewer.run()


if __name__ == "__main__":
    main()
