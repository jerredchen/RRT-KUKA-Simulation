from pydrake.multibody import plant
from pydrake.multibody.tree import Joint
from pydrake.multibody.tree import JointIndex
from pydrake.multibody.parsing import Parser
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.systems.framework import DiagramBuilder
from pydrake.common import FindResourceOrThrow
from pydrake.math import RigidTransform
import pydrake.solvers.mathematicalprogram as mp
from ompl import base as ob
from ompl import geometric as og
import lcm
from drake import lcmt_iiwa_status
from robotlocomotion.robot_plan_t import robot_plan_t
from bot_core import robot_state_t
import numpy as np

# If done in C++ might have trouble interfacing with OMPL
# Make sure to build LCM folder to update LCM types
# OMPL C++: #include .....h
# Define path for cmake file (make) (similar to bazel build)
# If you do in C++ must figure out linking and dependacies
# Might still want multibody in pydrake, possibly get end effector, use forward kinematics
# Going from end-effector to joint configuration is IK problem
# Going from joint to end-effector is forward kinematics problem
# plant.EvalBodyPoseInWorld() -> forward kinematics

lcmStatusChannel = "IIWA_STATUS"
lcmPlanChannel = "COMMITTED_ROBOT_PLAN"
NUM_JOINTS = 7
EE_X = 1.
EE_Y = 0.
EE_Z = 2.
EE_NAME = "iiwa_link_ee"

def Plan(joint_names, keyframes):
    num_time_steps = len(keyframes)

    path_plan = robot_plan_t()
    path_plan.utime = 0
    path_plan.robot_name = ""
    path_plan.matlab_data = []
    path_plan.num_states = num_time_steps
    for i in range(num_time_steps):
        path_plan.plan.append(robot_state_t())
        path_plan.plan_info.append(i)
        step = path_plan.plan[i]
        step.utime = i * int(1e6)
        step.num_joints = NUM_JOINTS
        for j in range(step.num_joints):
            step.joint_name.append(joint_names[j])
            step.joint_position.append(keyframes[i][j])
            step.joint_velocity.append(0)
            step.joint_effort.append(0)
    path_plan.num_grasp_transitions = 0
    path_plan.left_arm_control_type = path_plan.POSITION
    path_plan.right_arm_control_type = path_plan.NONE
    path_plan.left_leg_control_type = path_plan.NONE
    path_plan.right_leg_control_type = path_plan.NONE
    path_plan.num_bytes = 0
    return path_plan

status_count = 0
def handleStatus(channel, status):
    global status_count
    wp = path.getStates()
    # Create a list of joint names
    position_names = {}
    num_pos = multPlant.num_positions()
    for i in range(multPlant.num_joints()):
        joint = multPlant.get_joint(JointIndex(i))
        if joint.num_positions() == 0:
            continue
        position_names[joint.position_start()] = joint.name()   
    joint_names = [position_names[i] for i in range(len(position_names))]

    status_count += 1
    status_data = lcmt_iiwa_status.decode(status)
    iiwa_status = status_data
    iiwa_q = np.ndarray(shape=(NUM_JOINTS, 1))
    for i in range(status_data.num_joints):
        iiwa_q[i] = status_data.joint_position_measured[i]
    multPlant.SetPositions(context, iiwa_q)
    # curr_link_pose = multPlant.EvalBodyPoseInWorld(context, EE_NAME)
    # print(f"{EE_NAME} at: {curr_link_pose.translation().transpose()}")
    if status_count == 1:
        plan = Plan(joint_names, wp)
        if len(plan.plan) > 0:
            lc.publish(lcmPlanChannel, plan.encode())

# For some reason isStateValid in the validity checker callback not recognized inside a class
# Must keep each method outside of being used in an object
def isStateValid(state):
    # Implement collision avoidance here, avoid any obstacles in environments
    return True

path = None
def run():
    # Perform IK beforehand: get goal state in terms of 7D space from 3D Cartesian space
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(multPlant, diagram_context)
    lower_bound = np.array([EE_X - 0.01, EE_Y - 0.01, EE_Z - 0.01])
    upper_bound = np.array([EE_X + 0.01, EE_Y + 0.01, EE_Z + 0.01])
    EE_FRAME = multPlant.GetFrameByName("iiwa_link_ee_kuka")
    BASE_FRAME = multPlant.GetFrameByName("base")
    ik = InverseKinematics(multPlant, plant_context, with_joint_limits=True)
    prog = ik.get_mutable_prog()
    q = ik.q()
    ik.AddPositionConstraint(EE_FRAME, np.array([0., 0., 0.]), multPlant.world_frame(),lower_bound, upper_bound)
    result = mp.Solve(prog)
    q_val = result.GetSolution(q)

    global path
    # Created state space only of the joint limits for the joints
    space = ob.RealVectorStateSpace(7)
    bounds = ob.RealVectorBounds(7)
    bounds.setLow(-2.05948851735)
    bounds.setHigh(2.05948851735)
    bounds.setLow(0, -2.93215314335)
    bounds.setHigh(0, 2.93215314335)
    space.setBounds(bounds)
    # SimpleSetup solutionPath can iterate through states
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    # Start state
    start = ob.State(space)
    start[0] = start[1] = start[2] = start[3] = start[4] = start[5] = start[6] = 0
    # Goal state
    goal = ob.State(space)
    for i in range(len(q_val)):
        goal[i] = q_val[i]
    ss.setStartAndGoalStates(start, goal)
    si = ss.getSpaceInformation()
    planner = og.RRT(si)
    ss.setPlanner(planner)
    solved = ss.solve(20)
    if solved:
        print("Found Solution:")
        path = ss.getSolutionPath()
        # Segmentation fault if run() returns path and path is used in separate function 
        print(ss.getSolutionPath())

    else:
        print("No solution found.")
        return None
    lc.subscribe(lcmStatusChannel, handleStatus)

    while True:
        lc.handle()

if __name__ == "__main__":
    # Create a plant for the kuka iiwa arm
    builder = DiagramBuilder()
    multPlant, _ = plant.AddMultibodyPlantSceneGraph(builder, 0.001)
    urdf = "drake/manipulation/models/iiwa_description/urdf/iiwa14_no_collision.urdf"
    modelPath = FindResourceOrThrow(urdf)
    parser = Parser(multPlant)
    parser.AddModelFromFile(modelPath)
    multPlant.WeldFrames(multPlant.world_frame(), multPlant.GetFrameByName("base"))
    # Add the 
    multPlant.Finalize()
    context = multPlant.CreateDefaultContext()

    lc = lcm.LCM()
    iiwa_status = lcmt_iiwa_status()
    run()