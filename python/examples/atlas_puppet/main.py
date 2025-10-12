import dartpy as dart
import numpy as np


class RelaxedPosture(dart.optimizer.Function):
    def __init__(self, ideal_posture, lower, upper, weights, enforce_ideal=False):
        super(RelaxedPosture, self).__init__()
        self.enforce_ideal_posture = enforce_ideal
        self.ideal = ideal_posture
        self.lower = lower
        self.upper = upper
        self.weights = weights
        
        dofs = len(self.ideal)
        if len(self.lower) != dofs or len(self.weights) != dofs or len(self.upper) != dofs:
            print(f"[RelaxedPosture] Dimension mismatch:")
            print(f"  ideal:   {len(self.ideal)}")
            print(f"  lower:   {len(self.lower)}")
            print(f"  upper:   {len(self.upper)}")
            print(f"  weights: {len(self.weights)}")
        
        self.result_vector = np.zeros(dofs)
    
    def eval(self, x):
        self.compute_result_vector(x)
        return 0.5 * np.dot(self.result_vector, self.result_vector)
    
    def evalGradient(self, x, grad):
        self.compute_result_vector(x)
        grad[:] = 0.0
        smaller = min(len(self.result_vector), len(grad))
        for i in range(smaller):
            grad[i] = self.result_vector[i]
    
    def compute_result_vector(self, x):
        self.result_vector[:] = 0.0
        
        if self.enforce_ideal_posture:
            for i in range(len(x)):
                if len(self.ideal) <= i:
                    break
                self.result_vector[i] = self.weights[i] * (x[i] - self.ideal[i])
        else:
            for i in range(len(x)):
                if len(self.ideal) <= i:
                    break
                
                if x[i] < self.lower[i]:
                    self.result_vector[i] = self.weights[i] * (x[i] - self.lower[i])
                elif self.upper[i] < x[i]:
                    self.result_vector[i] = self.weights[i] * (x[i] - self.upper[i])


class TeleoperationWorld(dart.gui.osg.WorldNode):
    MOVE_Q = 0
    MOVE_W = 1
    MOVE_E = 2
    MOVE_A = 3
    MOVE_S = 4
    MOVE_D = 5
    MOVE_F = 6
    MOVE_Z = 7
    NUM_MOVE = 8
    
    def __init__(self, world, robot):
        super(TeleoperationWorld, self).__init__(world)
        self.atlas = robot
        self.iter = 0
        self.l_foot = robot.getEndEffector("l_foot")
        self.r_foot = robot.getEndEffector("r_foot")
        self.move_components = [False] * self.NUM_MOVE
        self.any_movement = False
    
    def setMovement(self, move_components):
        self.move_components = move_components
        self.any_movement = any(move_components)
    
    def customPreRefresh(self):
        if self.any_movement:
            old_tf = self.atlas.getBodyNode(0).getWorldTransform()
            new_tf = dart.math.Isometry3()
            
            forward = old_tf.rotation()[:, 0]
            forward[2] = 0.0
            if np.linalg.norm(forward) > 1e-10:
                forward = forward / np.linalg.norm(forward)
            else:
                forward[:] = 0.0
            
            left = old_tf.rotation()[:, 1]
            left[2] = 0.0
            if np.linalg.norm(left) > 1e-10:
                left = left / np.linalg.norm(left)
            else:
                left[:] = 0.0
            
            up = np.array([0.0, 0.0, 1.0])
            
            linear_step = 0.01
            elevation_step = 0.2 * linear_step
            rotational_step = 2.0 * np.pi / 180.0
            
            if self.move_components[self.MOVE_W]:
                new_tf.set_translation(new_tf.translation() + linear_step * forward)
            
            if self.move_components[self.MOVE_S]:
                new_tf.set_translation(new_tf.translation() - linear_step * forward)
            
            if self.move_components[self.MOVE_A]:
                new_tf.set_translation(new_tf.translation() + linear_step * left)
            
            if self.move_components[self.MOVE_D]:
                new_tf.set_translation(new_tf.translation() - linear_step * left)
            
            if self.move_components[self.MOVE_F]:
                new_tf.set_translation(new_tf.translation() + elevation_step * up)
            
            if self.move_components[self.MOVE_Z]:
                new_tf.set_translation(new_tf.translation() - elevation_step * up)
            
            if self.move_components[self.MOVE_Q]:
                rot_matrix = dart.math.eulerToMatrix([0, 0, rotational_step])
                new_tf.set_rotation(np.matmul(rot_matrix, new_tf.rotation()))
            
            if self.move_components[self.MOVE_E]:
                rot_matrix = dart.math.eulerToMatrix([0, 0, -rotational_step])
                new_tf.set_rotation(np.matmul(rot_matrix, new_tf.rotation()))
            
            new_tf.set_translation(new_tf.translation() + old_tf.translation())
            new_tf.set_rotation(np.matmul(new_tf.rotation(), old_tf.rotation()))
            
            positions = dart.dynamics.FreeJoint.convertToPositions(new_tf)
            self.atlas.getJoint(0).setPositions(positions)
        
        solved = self.atlas.getIK(True).solveAndApply(True)
        
        if not solved:
            self.iter += 1
        else:
            self.iter = 0
        
        if self.iter == 1000:
            print("Failing!")


class InputHandler(dart.gui.osg.GUIEventHandler):
    def __init__(self, viewer, teleop, atlas, world):
        super(InputHandler, self).__init__()
        self.viewer = viewer
        self.teleop = teleop
        self.atlas = atlas
        self.world = world
        self.initialize()
    
    def initialize(self):
        self.rest_config = self.atlas.getPositions()
        
        self.legs = []
        for i in range(self.atlas.getNumDofs()):
            dof_name = self.atlas.getDof(i).getName()
            if len(dof_name) > 6 and dof_name[1:6] == "_leg_":
                self.legs.append(self.atlas.getDof(i).getIndexInSkeleton())
        
        self.legs.append(self.atlas.getDof("rootJoint_rot_x").getIndexInSkeleton())
        self.legs.append(self.atlas.getDof("rootJoint_rot_y").getIndexInSkeleton())
        self.legs.append(self.atlas.getDof("rootJoint_pos_z").getIndexInSkeleton())
        
        self.default_bounds = []
        self.default_target_tf = []
        self.constraint_active = []
        self.end_effector_index = []
        
        for i in range(self.atlas.getNumEndEffectors()):
            ee = self.atlas.getEndEffector(i)
            ik = ee.getIK()
            if ik:
                self.default_bounds.append(ik.getErrorMethod().getBounds())
                self.default_target_tf.append(ik.getTarget().getRelativeTransform())
                self.constraint_active.append(False)
                self.end_effector_index.append(ee.getIndexInSkeleton())
        
        self.posture = self.atlas.getIK(True).getObjective()
        self.balance = self.atlas.getIK(True).getProblem().getEqConstraint(1)
        
        self.optimization_key = ord('r')
        self.move_components = [False] * TeleoperationWorld.NUM_MOVE
    
    def handle(self, ea, aa):
        if self.atlas is None:
            return False
        
        event_type = ea.getEventType()
        
        if event_type == dart.gui.osg.GUIEventAdapter.KEYDOWN:
            key = ea.getKey()
            
            if key == ord('p'):
                for i in range(self.atlas.getNumDofs()):
                    print(f"{self.atlas.getDof(i).getName()}: {self.atlas.getDof(i).getPosition()}")
                print("  -- -- -- -- -- ")
                return True
            
            if key == ord('t'):
                for i in range(self.atlas.getNumDofs()):
                    if i < 2 or 4 < i:
                        self.atlas.getDof(i).setPosition(self.rest_config[i])
                return True
            
            if ord('1') <= key <= ord('9'):
                index = key - ord('1')
                if index < len(self.constraint_active):
                    ee = self.atlas.getEndEffector(self.end_effector_index[index])
                    ik = ee.getIK()
                    if ik and self.constraint_active[index]:
                        self.constraint_active[index] = False
                        ik.getErrorMethod().setBounds(*self.default_bounds[index])
                        ik.getTarget().setRelativeTransform(self.default_target_tf[index])
                        self.world.removeSimpleFrame(ik.getTarget())
                    elif ik:
                        self.constraint_active[index] = True
                        ik.getErrorMethod().setBounds()
                        ik.getTarget().setTransform(ee.getTransform())
                        self.world.addSimpleFrame(ik.getTarget())
                return True
            
            if key == ord('x'):
                ee = self.atlas.getEndEffector("l_foot")
                ee.getSupport().setActive(not ee.getSupport().isActive())
                return True
            
            if key == ord('c'):
                ee = self.atlas.getEndEffector("r_foot")
                ee.getSupport().setActive(not ee.getSupport().isActive())
                return True
            
            if key == ord('w'):
                self.move_components[TeleoperationWorld.MOVE_W] = True
            elif key == ord('a'):
                self.move_components[TeleoperationWorld.MOVE_A] = True
            elif key == ord('s'):
                self.move_components[TeleoperationWorld.MOVE_S] = True
            elif key == ord('d'):
                self.move_components[TeleoperationWorld.MOVE_D] = True
            elif key == ord('q'):
                self.move_components[TeleoperationWorld.MOVE_Q] = True
            elif key == ord('e'):
                self.move_components[TeleoperationWorld.MOVE_E] = True
            elif key == ord('f'):
                self.move_components[TeleoperationWorld.MOVE_F] = True
            elif key == ord('z'):
                self.move_components[TeleoperationWorld.MOVE_Z] = True
            
            if key in [ord('w'), ord('a'), ord('s'), ord('d'), 
                      ord('q'), ord('e'), ord('f'), ord('z')]:
                self.teleop.setMovement(self.move_components)
                return True
            
            if self.optimization_key == key:
                if isinstance(self.posture, RelaxedPosture):
                    self.posture.enforce_ideal_posture = True
                
                if self.balance:
                    self.balance.setErrorMethod(
                        dart.constraint.BalanceConstraint.ErrorMethod.OPTIMIZE_BALANCE)
                
                return True
        
        if event_type == dart.gui.osg.GUIEventAdapter.KEYUP:
            key = ea.getKey()
            
            if key == self.optimization_key:
                if isinstance(self.posture, RelaxedPosture):
                    self.posture.enforce_ideal_posture = False
                
                if self.balance:
                    self.balance.setErrorMethod(
                        dart.constraint.BalanceConstraint.ErrorMethod.FROM_CENTROID)
                
                return True
            
            if key == ord('w'):
                self.move_components[TeleoperationWorld.MOVE_W] = False
            elif key == ord('a'):
                self.move_components[TeleoperationWorld.MOVE_A] = False
            elif key == ord('s'):
                self.move_components[TeleoperationWorld.MOVE_S] = False
            elif key == ord('d'):
                self.move_components[TeleoperationWorld.MOVE_D] = False
            elif key == ord('q'):
                self.move_components[TeleoperationWorld.MOVE_Q] = False
            elif key == ord('e'):
                self.move_components[TeleoperationWorld.MOVE_E] = False
            elif key == ord('f'):
                self.move_components[TeleoperationWorld.MOVE_F] = False
            elif key == ord('z'):
                self.move_components[TeleoperationWorld.MOVE_Z] = False
            
            if key in [ord('w'), ord('a'), ord('s'), ord('d'), 
                      ord('q'), ord('e'), ord('f'), ord('z')]:
                self.teleop.setMovement(self.move_components)
                return True
        
        return False


def create_ground():
    ground = dart.dynamics.Skeleton("ground")
    tf = dart.math.Isometry3()
    thickness = 0.01
    tf.set_translation([0, 0, -thickness / 2.0])
    
    joint_prop = dart.dynamics.WeldJoint.Properties()
    joint_prop.mT_ParentBodyToJoint = tf
    
    bn, joint = ground.createJointAndBodyNodePair(None, joint_prop, 
                                                   dart.dynamics.BodyNode.AspectProperties("ground_body"))
    
    ground_shape = dart.dynamics.BoxShape([10, 10, thickness])
    
    shape_node = bn.createShapeNodeWith(
        ground_shape,
        dart.dynamics.ShapeNode.AspectProperties(),
        dart.dynamics.VisualAspect(),
        dart.dynamics.CollisionAspect(),
        dart.dynamics.DynamicsAspect()
    )
    shape_node.getVisualAspect().setColor([0.2, 0.2, 1.0, 1.0])
    
    return ground


def create_atlas():
    urdf = dart.utils.DartLoader()
    atlas = urdf.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf")
    
    scale = 0.25
    box_shape = dart.dynamics.BoxShape([scale * 1.0, scale * 1.0, scale * 0.5])
    
    tf = dart.math.Isometry3()
    tf.set_translation([0.0, 0.0, 0.1])
    
    shape_node = atlas.getBodyNode(0).createShapeNodeWith(
        box_shape,
        dart.dynamics.ShapeNode.AspectProperties(),
        dart.dynamics.VisualAspect()
    )
    shape_node.getVisualAspect().setColor([0.0, 0.0, 0.0, 1.0])
    shape_node.setRelativeTransform(tf)
    
    return atlas


def setup_start_configuration(atlas):
    atlas.getDof("r_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("r_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("r_leg_aky").setPosition(-45.0 * np.pi / 180.0)
    
    atlas.getDof("l_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("l_leg_aky").setPosition(-45.0 * np.pi / 180.0)
    
    atlas.getDof("r_arm_shx").setPosition(65.0 * np.pi / 180.0)
    atlas.getDof("r_arm_ely").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("r_arm_elx").setPosition(-90.0 * np.pi / 180.0)
    atlas.getDof("r_arm_wry").setPosition(65.0 * np.pi / 180.0)
    
    atlas.getDof("l_arm_shx").setPosition(-65.0 * np.pi / 180.0)
    atlas.getDof("l_arm_ely").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("l_arm_elx").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("l_arm_wry").setPosition(65.0 * np.pi / 180.0)
    
    atlas.getDof("r_leg_kny").setPositionLowerLimit(10 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPositionLowerLimit(10 * np.pi / 180.0)


def setup_end_effectors(atlas):
    rootjoint_weights = 0.01 * np.ones(6)
    
    linear_bounds = np.full(3, np.inf)
    angular_bounds = np.full(3, np.inf)
    
    # Left hand
    tf_hand = dart.math.Isometry3()
    tf_hand.set_translation([0.0009, 0.1254, 0.012])
    rot_matrix = dart.math.eulerToMatrix([0, 0, 90.0 * np.pi / 180.0])
    tf_hand.set_rotation(rot_matrix)
    
    l_hand = atlas.getBodyNode("l_hand").createEndEffector("l_hand")
    l_hand.setDefaultRelativeTransform(tf_hand, True)
    
    lh_target = dart.gui.osg.InteractiveFrame(dart.dynamics.Frame.World(), "lh_target")
    
    l_hand.getIK(True).setTarget(lh_target)
    l_hand.getIK().useWholeBody()
    l_hand.getIK().getGradientMethod().setComponentWeights(rootjoint_weights)
    l_hand.getIK().getErrorMethod().setLinearBounds(-linear_bounds, linear_bounds)
    l_hand.getIK().getErrorMethod().setAngularBounds(-angular_bounds, angular_bounds)
    
    # Right hand
    tf_hand.set_translation([-0.0009, -0.1254, 0.012])
    tf_hand.set_rotation(rot_matrix.T)
    
    r_hand = atlas.getBodyNode("r_hand").createEndEffector("r_hand")
    r_hand.setDefaultRelativeTransform(tf_hand, True)
    
    rh_target = dart.gui.osg.InteractiveFrame(dart.dynamics.Frame.World(), "rh_target")
    
    r_hand.getIK(True).setTarget(rh_target)
    r_hand.getIK().useWholeBody()
    r_hand.getIK().getGradientMethod().setComponentWeights(rootjoint_weights)
    r_hand.getIK().getErrorMethod().setLinearBounds(-linear_bounds, linear_bounds)
    r_hand.getIK().getErrorMethod().setAngularBounds(-angular_bounds, angular_bounds)
    
    # Support geometry for feet
    sup_pos_x = 0.10 - 0.186
    sup_neg_x = -0.03 - 0.186
    sup_pos_y = 0.03
    sup_neg_y = -0.03
    support = [
        [sup_neg_x, sup_neg_y, 0.0],
        [sup_pos_x, sup_neg_y, 0.0],
        [sup_pos_x, sup_pos_y, 0.0],
        [sup_neg_x, sup_pos_y, 0.0]
    ]
    
    tf_foot = dart.math.Isometry3()
    tf_foot.set_translation([0.186, 0.0, -0.08])
    
    linear_bounds[2] = 1e-8
    angular_bounds[0] = 1e-8
    angular_bounds[1] = 1e-8
    
    # Left foot
    l_foot = atlas.getBodyNode("l_foot").createEndEffector("l_foot")
    l_foot.setRelativeTransform(tf_foot)
    
    lf_target = dart.gui.osg.InteractiveFrame(dart.dynamics.Frame.World(), "lf_target")
    
    l_foot.getIK(True).setTarget(lf_target)
    l_foot.getIK().setHierarchyLevel(1)
    l_foot.getIK().getErrorMethod().setLinearBounds(-linear_bounds, linear_bounds)
    l_foot.getIK().getErrorMethod().setAngularBounds(-angular_bounds, angular_bounds)
    
    l_foot.getSupport(True).setGeometry(support)
    l_foot.getSupport().setActive()
    
    # Right foot
    r_foot = atlas.getBodyNode("r_foot").createEndEffector("r_foot")
    r_foot.setRelativeTransform(tf_foot)
    
    rf_target = dart.gui.osg.InteractiveFrame(dart.dynamics.Frame.World(), "rf_target")
    
    r_foot.getIK(True).setTarget(rf_target)
    r_foot.getIK().setHierarchyLevel(1)
    r_foot.getIK().getErrorMethod().setLinearBounds(-linear_bounds, linear_bounds)
    r_foot.getIK().getErrorMethod().setAngularBounds(-angular_bounds, angular_bounds)
    
    r_foot.getSupport(True).setGeometry(support)
    r_foot.getSupport().setActive()
    
    height_change = -r_foot.getWorldTransform().translation()[2]
    atlas.getDof(5).setPosition(height_change)
    
    l_foot.getIK().getTarget().setTransform(l_foot.getTransform())
    r_foot.getIK().getTarget().setTransform(r_foot.getTransform())


def setup_whole_body_solver(atlas):
    solver = atlas.getIK(True).getSolver()
    if hasattr(solver, 'setNumMaxIterations'):
        solver.setNumMaxIterations(10)
    
    n_dofs = atlas.getNumDofs()
    
    default_weight = 0.01
    weights = default_weight * np.ones(n_dofs)
    weights[2] = 0.0
    weights[3] = 0.0
    weights[4] = 0.0
    
    weights[6] *= 0.2
    weights[7] *= 0.2
    weights[8] *= 0.2
    
    lower_posture = np.full(n_dofs, -np.inf)
    lower_posture[0] = -0.35
    lower_posture[1] = -0.35
    lower_posture[5] = 0.600
    
    lower_posture[6] = -0.1
    lower_posture[7] = -0.1
    lower_posture[8] = -0.1
    
    upper_posture = np.full(n_dofs, np.inf)
    upper_posture[0] = 0.35
    upper_posture[1] = 0.35
    upper_posture[5] = 0.885
    
    upper_posture[6] = 0.1
    upper_posture[7] = 0.1
    upper_posture[8] = 0.1
    
    objective = RelaxedPosture(atlas.getPositions(), lower_posture, upper_posture, weights)
    atlas.getIK().setObjective(objective)
    
    balance = dart.constraint.BalanceConstraint(atlas.getIK())
    atlas.getIK().getProblem().addEqConstraint(balance)
    
    balance.setErrorMethod(dart.constraint.BalanceConstraint.ErrorMethod.FROM_CENTROID)
    balance.setBalanceMethod(dart.constraint.BalanceConstraint.BalanceMethod.SHIFT_SUPPORT)


def enable_drag_and_drops(viewer, atlas):
    for i in range(atlas.getNumBodyNodes()):
        viewer.enableDragAndDrop(atlas.getBodyNode(i), False, False)
    
    for i in range(atlas.getNumEndEffectors()):
        ee = atlas.getEndEffector(i)
        if not ee.getIK():
            continue
        
        target = ee.getIK().getTarget()
        if hasattr(target, '__class__') and 'InteractiveFrame' in str(target.__class__):
            viewer.enableDragAndDrop(target)


def main():
    world = dart.simulation.World()
    
    atlas = create_atlas()
    world.addSkeleton(atlas)
    
    ground = create_ground()
    world.addSkeleton(ground)
    
    setup_start_configuration(atlas)
    setup_end_effectors(atlas)
    setup_whole_body_solver(atlas)
    
    node = TeleoperationWorld(world, atlas)
    
    viewer = dart.gui.osg.Viewer()
    
    viewer.allowSimulation(False)
    viewer.addWorldNode(node)
    
    viewer.addEventHandler(InputHandler(viewer, node, atlas, world))
    
    enable_drag_and_drops(viewer, atlas)
    
    display_elevation = 0.05
    viewer.addAttachment(dart.gui.osg.SupportPolygonVisual(atlas, display_elevation))
    
    print(viewer.getInstructions())
    print()
    print("Alt + Click:   Try to translate a body without changing its orientation")
    print("Ctrl + Click:  Try to rotate a body without changing its translation")
    print("Shift + Click: Move a body using only its parent joint")
    print("1 -> 4:        Toggle the interactive target of an EndEffector")
    print("W A S D:       Move the robot around the scene")
    print("Q E:           Rotate the robot counter-clockwise and clockwise")
    print("F Z:           Shift the robot's elevation up and down")
    print("X C:           Toggle support on the left and right foot")
    print("R:             Optimize the robot's posture")
    print("T:             Reset the robot to its relaxed posture")
    print()
    print("  Because this uses iterative Jacobian methods, the solver can get finicky,")
    print("  and the robot can get tangled up. Use 'R' and 'T' keys when the robot is")
    print("  in a messy configuration")
    print()
    print("  The green polygon is the support polygon of the robot, and the blue/red ball is")
    print("  the robot's center of mass. The green ball is the centroid of the polygon.")
    print()
    print("Note that this is purely kinematic. Physical simulation is not allowed in this app.")
    print()
    
    viewer.setUpViewInWindow(0, 0, 1280, 960)
    viewer.setCameraHomePosition(
        [5.34, 3.00, 2.41],
        [0.00, 0.00, 1.00],
        [-0.20, -0.08, 0.98]
    )
    
    viewer.setCameraManipulator(viewer.getCameraManipulator())
    
    viewer.run()


if __name__ == "__main__":
    main()