#!/usr/bin/env python3Publisher

'''
############################################################
############################################################
Purpose: This node, Meca_Control, was created to integrate/test motion planning
with the real robot by communicating over topics and with services. It
can be used to control both robots--hence it doesn't take a namespace.
Currently, "user" code can be written in the run() method to use this
ros2 system and move the robots. In the future, it would be great to
create a shell command prompt for the user to control the robots or get
info quickly without writing code; also, a way to create a separate script
that interacts with these control functions would be ideal so this file does
not get messy. But for now, this is the state of the codebase.

Date Created: 04/26/2022
Developers: Jessica Myers, [add name here]
University of Illinois Urbana-Champaign

############################################################
############################################################
'''

import rclpy # python library for ros2. needed for every ros2 node
from rclpy.node import Node
import mecademicpy.robot as mdr
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from custom_interfaces.srv import GetMotionPlan, VisualizeMotionPlan, MoveJoints
from custom_interfaces.srv import GoToPose, MoveGripper, SetBlending, SetGripperForce, SetGripperVel, SetJointVel, SetJointAcc
from custom_interfaces.msg import RobotStatus, GripperStatus
import numpy as np
from functools import partial
import time
from meca_controller.meca_settings import ROBOT1, ROBOT2, ASSUMED_RESTING_STATE_CONFIG # ADJUST THESE GLOBAL CONSTANTS

class Meca_Control(Node):
    def __init__(self):
        super().__init__("meca_control")

        # Subscribe to keep up to date on current joint angles, pose, and status of the robot (may not need all these):
        self.joint_subscriber_robot1_ = self.create_subscription(JointState, f"{ROBOT1['namespace']}/MecademicRobot_joint_fb",
                                                                 self.update_robot1_joints_callback, 10)
        self.joint_subscriber_robot2_ = self.create_subscription(JointState, f"{ROBOT2['namespace']}/MecademicRobot_joint_fb",
                                                                 self.update_robot2_joints_callback, 10)
        self.pose_subscriber_robot1_ = self.create_subscription(JointState, f"{ROBOT1['namespace']}/MecademicRobot_pose_fb",
                                                                 self.update_robot1_pose_callback, 10)
        self.pose_subscriber_robot2_ = self.create_subscription(JointState, f"{ROBOT2['namespace']}/MecademicRobot_pose_fb",
                                                                 self.update_robot2_pose_callback, 10)
        self.gripper_subscriber_robot1_ = self.create_subscription(GripperStatus, f"{ROBOT1['namespace']}/MecademicRobot_gripper_status",
                                                                 self.update_robot1_gripper_status_callback, 10)
        self.gripper_subscriber_robot2_ = self.create_subscription(GripperStatus, f"{ROBOT2['namespace']}/MecademicRobot_gripper_status",
                                                                 self.update_robot2_gripper_status_callback, 10)
        self.status_subscriber_robot1_ = self.create_subscription(RobotStatus, f"{ROBOT1['namespace']}/MecademicRobot_status",
                                                                 self.update_robot1_status_callback, 10)
        self.status_subscriber_robot2_ = self.create_subscription(RobotStatus, f"{ROBOT2['namespace']}/MecademicRobot_status",
                                                                 self.update_robot2_status_callback, 10)

        # Initialize variables that will be set later:
        self.current_joints_robot1 = None
        self.current_joints_robot2 = None
        self.current_pose_robot1 = None
        self.current_pose_robot2 = None
        self.current_gripper_status_robot1 = None
        self.current_gripper_status_robot2 = None
        self.current_robot1_status = None
        self.current_robot2_status = None

    '''
    Purpose: Parts of this code rely upon getting information from topics published to by the meca_driver, such as current_joints.
             Errors will occur if the main code is executed before these are assigned (e.g. motion plan but not yet received the
             current joint states from the subscriber). This function will wait until the critical variables are set so that all
             the code runs as intended.

             -timeout_length: amount of time in seconds that should wait to receive data from both robots before beginning code execution;
                        after timeout period, just waits for data from one robot to begin because assumes you are only using 1 robot.

             TODO add to the critical init variables as code is built.
    '''
    def wait_for_initialization(self, timeout_length=2):
        time_start = time.time()
        while rclpy.ok():
            print('waiting to receive meca state data...')
            rclpy.spin_once(self) # spin once while we wait to avoid blocking any callbacks. (like the data we are waiting on)

            if (self.current_joints_robot1 is not None) and (self.current_joints_robot2 is not None):
                print('...received data for both robots, starting now.')
                return

            # If timeout period has elapsed, only check for data being received from one robot before starting:
            if ((time.time() - time_start) >= timeout_length) and ((self.current_joints_robot1 is not None) or (self.current_joints_robot2 is not None)):
                print('...received data for at least one robot, starting now because timeout has occurred waiting.\n')
                return

    '''
    Purpose: A space to write user code to control the robots. Example commands given here.
    
    TODO in the future, create a better way for the user to add code that doesn't involve editing this file.
    Currently unsure how to do this; moved older test code from this function into tests_meca_control.py,
    as a comment.

    NOTE: Everything involving the pinocchio collision detection / urdf / visualization / motion planning has joint
    angles in RADIANS. The mecademicpy interface uses DEGREES for the MoveJoints command.
    '''
    def run(self):
        # 0) For safety:
        self.set_joint_vel(ROBOT1['namespace'], 5)
        self.set_joint_vel(ROBOT2['namespace'], 5)
        self.set_blending(ROBOT1['namespace'], 0)
        self.set_blending(ROBOT2['namespace'], 0)

        # Here's an example of how to move the joints:
        # self.move_joints(np.array([5, 0, 0, 0, 0, 0]), ROBOT1['namespace')

        # ------------------------------------ Moving to the goal -----------------------------
        # 1) Define start and goal configurations:
        start_joint_angles = np.radians(self.get_current_joints_both_robots(ROBOT1['namespace'])) # len 12 array, both robots
        goal_joint_angles  = np.radians([-60, 9, 64, -12, -35, 0]) # len 6 array, one robot;
        # goal_joint_angles = np.radians(np.array([-5, 80.9, -7.3, 0, -74.6, 0])) # robots get close too close
        # goal_joint_angles = np.radians([-90, 60, 40, 3, -97, 0]) # wires in real life get too close

        # 2) Get motion plan to goal config:
        motion_plan1 = self.get_motion_plan(ROBOT1['namespace'], start_joint_angles, goal_joint_angles)

        # 3) Visualize motion plan to goal config:
        print('\nVisualizing motion plan ... ')
        self.visualize_motion_plan(ROBOT1['namespace'], start_joint_angles, motion_plan1)

        # ------------------------------------ Moving back to start -----------------------------
        # 4) Get "start" 2-robot config to give the motion planner, by concatenating the last config of motion plan with robot2's config
        motion_plan1_matrix = self.process_motion_plan_msg(motion_plan1) # Convert flattened motion plan to matrix form from msg
        end_joint_angles = np.concatenate([motion_plan1_matrix[-1, :], start_joint_angles[6:]]) # robot2 stays unmoving
        
        # 5) Get motion plan back to start    
        motion_plan2 = self.get_motion_plan(ROBOT1['namespace'], end_joint_angles, start_joint_angles[:6])
                
        # 6) Visualize motion plan back to start:
        print('\nVisualizing motion plan ... ')
        self.visualize_motion_plan(ROBOT1['namespace'], end_joint_angles, motion_plan2)
        
        # ------------------------------------ Real robot motion -----------------------------
        # self.execute_motion_plan(ROBOT1['namespace'], motion_plan1, error_tolerance=.1, timeout_length=60)
        # self.execute_motion_plan(ROBOT1['namespace'], motion_plan2, error_tolerance=.1, timeout_length=60)


    def update_robot1_joints_callback(self, joints):
        self.current_joints_robot1 = joints # JointState (has .position and .velocity fields)

    def update_robot2_joints_callback(self, joints):
        self.current_joints_robot2 = joints # JointState (has .position and .velocity fields)

    def update_robot1_pose_callback(self, pose):
        self.current_pose_robot1 = pose # Pose (has .position and .orientation fields)

    def update_robot2_pose_callback(self, pose):
        self.current_pose_robot2 = pose # Pose (has .position and .orientation fields)
    
    def update_robot1_gripper_status_callback(self, gripper_status):
        self.current_gripper_status_robot1 = gripper_status # custom msg type

    def update_robot2_gripper_status_callback(self, gripper_status):
        self.current_gripper_status_robot2 = gripper_status # custom msg type

    def update_robot1_status_callback(self, robot_status):
        self.current_robot1_status = robot_status # custom msg type

    def update_robot2_status_callback(self, robot_status):
        self.current_robot2_status = robot_status # custom msg type

    '''
    Purpose: used to determine from the namespace whether this is robot1 or robot2, for the purpose of getting the order right
             in the len 12 joint angle array sent to the motion planner and visualization.
    '''
    def determine_which_robot_from_namespace(self, namespace):
        if namespace == ROBOT1['namespace']:
            is_robot1 = True
        elif namespace == ROBOT2['namespace']:
            is_robot1 = False
        else:
            raise Exception(f"ERROR: Provided namespace must either be {ROBOT1['namespace']} or {ROBOT2['namespace']}.")
        return is_robot1


    '''
    Inputs: required: namespace of the robot for which you absolutely must know the current configuration in order to do motion
                      planning, because you are going to execute motion for that robot. Exception will be thrown if the current
                      joint angles are not being received for this robot.
                            - Ideally, you want to be receiving the joint angles for BOTH robots--a warning will be printed to
                              proceed with caution if the other robot's angles are not being received, but motion planning will
                              continue (at your own risk), using default resting config for the other robot for collision detection.

    Returns a np array of size 12 specifying the joint angles of the robots in the following order: [robot1, robot2]; this
    order will not change regardless of what the current namespace is. If the other robot's current joints are none,
    then the robot is not turned on and we are not currently receiving the joint angles. The default angles will be set to
    the robot's tpyical resting pose (feel free to change if necessary). Use motion planning at your own risk if you do not
    know the state of the other robot.

    '''
    def get_current_joints_both_robots(self, required):
        # 1) Figure out if this is robot1 or robot2:
        is_robot1 = self.determine_which_robot_from_namespace(required)

        # 2) Make sure receiving current joint angles from the required robot, print warning if this robot isn't required:
        if self.current_joints_robot1 is None:
            if is_robot1:
                raise Exception(f"ERROR in get_current_joints_both_robots: Not actively receiving current joint state from robot1; check if it is on and publishing.")
            else:
                self.get_logger().warn('Not actively receiving current joint state from robot1; check if it is on and publishing.' \
                            ' Use motion planning at your own risk. Defaulting to ASSUMED_RESTING_STATE_CONFIG for the robot1' \
                            ' during planning.\n\n')            
                robot1_joints = ASSUMED_RESTING_STATE_CONFIG
        else:
            robot1_joints = self.current_joints_robot1.position

        # 2.5) Repeat for other robot
        if self.current_joints_robot2 is None:
            if not is_robot1:
                raise Exception(f"ERROR in get_current_joints_both_robots: Not actively receiving current joint state from robot2; check if it is on and publishing.")
            else:
                self.get_logger().warn('Not actively receiving current joint state from robot2; check if it is on and publishing.' \
                            ' Use motion planning at your own risk. Defaulting to ASSUMED_RESTING_STATE_CONFIG for robot2' \
                            ' during planning.\n\n')            
                robot2_joints = ASSUMED_RESTING_STATE_CONFIG
        else:
            robot2_joints = self.current_joints_robot2.position

        # 3) Get len 12 joint angles - order always is [robot1, robot2]
        return np.concatenate([robot1_joints, robot2_joints])
    
    '''
    Service call

    Inputs:
    - desired_joint_angles (degrees): np.array of len 6.
    - robot_namespace: the namespace of the robot which you want to control. Should be /robot1 or /robot2
    - error_tolerance: (degrees) the joint angle tolerance within which the robot should reach before executing the next command
    - timeout_length: the max amount of time in seconds to wait on the robot to reach the desired position before returning failure

    Returns:
    - is_reached: boolean, whether the robot successfully reached the desired position.
    '''
    def move_joints(self, desired_joint_angles, robot_namespace, error_tolerance=.1, timeout_length=60):
        # 1) Set up client
        client = self.create_client(MoveJoints, f'{robot_namespace}/move_joints')
        while not client.wait_for_service(timeout_sec=1.0): # if still waiting on service for 1 sec print
            self.get_logger().warn("service not available, waiting again...")
        
        # 2) Formulate request
        request = MoveJoints.Request()  
        request.requested_joint_angles = desired_joint_angles.astype(np.float64).tolist()

        # 3) Make async call (don't block thread using synchronous call)
        future = client.call_async(request)
        
        # 4) Wait for the request to be processed, but keep spinning to avoid locking up the thread:
        rclpy.spin_until_future_complete(self, future) # note that we don't really care about the response (future.result()) itself.

        # 5) Wait for position to be reached, within a tolerance, and under the timeout constraints (to prevent infinite while loops):
        is_robot1 = self.determine_which_robot_from_namespace(robot_namespace)
        time_start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self) # spin once while we wait to avoid blocking any callbacks. (example: joint angle updates)
            
            if (time.time() - time_start) >= timeout_length:
                return False # was not reached to the desired precision within the specified amount of time
            
            # Break out when reaches desired joint angles:
            # if self.current_joints is not None:

            if self.has_reached_config(desired_joint_angles,
                                       self.current_joints_robot1.position if is_robot1 else self.current_joints_robot2.position,
                                       error_tolerance):
                self.get_logger().info(f"move_joints: Has reached desired joint angles position {desired_joint_angles}")
                return True # has reached the desired position
        
        return False # some undefined behavior occurred; should never reach here
    
    '''
    Service call

    Function inputs:
        - namespace (robot for which you want to control)
        - position (see pos below)
        - command: default "pos" (see command below); convenience functions open_gripper and close_gripper will use this
                   variable for making the service call.

    Request inputs:
        - command [String]: {"open", "close", "pos"}
        - pos [float]: gripper position in mm in range [0, 5.6]
    
    Response:
        - error [bool]: True if error occurred, False otherwise.
    '''
    def move_gripper(self, robot_namespace, position, command="pos"):
        # 1) Set up client
        client = self.create_client(MoveGripper, f'{robot_namespace}/move_gripper')
        while not client.wait_for_service(timeout_sec=1.0): # if still waiting on service for 1 sec print
            self.get_logger().warn("MoveGripper service not available, waiting again...")
        
        # 2) Formulate request
        request = MoveGripper.Request()  
        request.command = command
        request.pos = float(position)

        # 3) Make async call (don't block thread using synchronous call)
        future = client.call_async(request)
        
        # 4) Wait for the request to be processed, but keep spinning to avoid locking up the thread:
        rclpy.spin_until_future_complete(self, future) # note that we don't really care about the response (future.result()) itself.

        # 5) Process response:
        response = future.result()
        if response.error:
            raise Exception(f"ERROR in MoveGripper call: either incorrect command or position out of range. See driver for specific error message.")
        self.get_logger().info(f"MoveGripper for {robot_namespace} complete.")

    def open_gripper(self, robot_namespace):
        self.move_gripper(robot_namespace, 0, command="open")

    def close_gripper(self, robot_namespace):
        self.move_gripper(robot_namespace, 0, command="close")

    '''
    Purpose: Used to see whether the robot has reached its desired configuration, to a tolerance (in degrees).
    '''
    def has_reached_config(self, desired_joint_angles, current_joint_angles, error_tolerance=.1):
        # Take the AND operation across the list of booleans
        return np.all([self.within_tolerance(desired_joint, curr_joint, error_tolerance) 
                    for (desired_joint, curr_joint) in zip(desired_joint_angles, current_joint_angles)])
    
    # t - .0001 <= x <= t + .0001:
    def within_tolerance(self, desired_joint_pos, current_joint_pos, error_tolerance):
        return (((desired_joint_pos - error_tolerance) <= current_joint_pos) and 
                (current_joint_pos <= (desired_joint_pos + error_tolerance)))

    '''
    Service call

    Inputs: 
    -robot_namespace: the namespace of the robot for which you want to plan motion. Should be /robot1 or /robot2
                    - NOTE that theoretically, you could plan motion for both robots at the same time (goal_config of len 12),
                      though this has not been tested and may not be entirely implemented. Synchronized MoveJoints for the two arms
                      also is not yet in place, which is why I have left this out for the time being.
    -start_config: (in RADIANS) 1D len 12 np.array specifying current (or starting) robot configuration (joint angles) of both
                                robots for the purpose of collision detection [robot1, robot2]. One robot will remain stationary.
    -goal_config: (in RADIANS) 1D len 6 np.array specifying goal joint angles for the robot whose motion is being planned.

    Returns:
    -motion_plan: object of type MotionPlan.msg, see custom_interfaces for fields and format
    '''
    def get_motion_plan(self, robot_namespace, start_config, goal_config):
        # 1) Set up client
        client = self.create_client(GetMotionPlan, 'get_motion_plan')
        while not client.wait_for_service(timeout_sec=1.0): # if still waiting on service for 1 sec print
            self.get_logger().warn("GetMotionPlan service not available, waiting again...")
        
        # 2) Formulate request
        request = GetMotionPlan.Request()
        request.is_robot1 = self.determine_which_robot_from_namespace(robot_namespace)
        request.start_config = start_config.astype(np.float64).tolist()
        request.goal_config = goal_config.astype(np.float64).tolist()

        # 3) Make async call (don't block thread using synchronous call)
        future = client.call_async(request)

        # 4) Wait for the request to be processed, but keep spinning to avoid locking up the thread:
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()

                    if response.motion_plan.num_waypoints == 0:
                        raise Exception(f"ERROR: Motion planner ran out of time; Increase the max number of iterations or check your goal configuration.")

                    self.get_logger().info(f'Found a motion plan with {response.motion_plan.num_waypoints} waypoints.')
                    return response.motion_plan
                except Exception as e:
                    self.get_logger().error("Service call failed: %r" % (e,))

    '''
    Service call

    NOTE that this is for visualizing a motion plan, which currently is just for one robot in motion, with the other stationary.

    Inputs:
    -robot_namespace: the namespace of the robot for which you want to plan motion. Should be /robot1 or /robot2
    -two_arm_current_joint_angles: (in RADIANS) 1D len 12 np.array specifying current (or starting) robot configuration (joint angles) of both
                                    robots for the purpose of collision detection [robot1, robot2]. One robot will remain stationary.
    -motion_plan: MotionPlan object, output of get_motion_plan service call.
    '''
    def visualize_motion_plan(self, robot_namespace, two_arm_current_joint_angles, motion_plan):
        # 1) Set up client
        client = self.create_client(VisualizeMotionPlan, 'visualize_motion_plan')
        while not client.wait_for_service(timeout_sec=1.0): # if still waiting on service for 1 sec print
            self.get_logger().warn("VisualizeMotionPlan service not available, waiting again...")
        
        # 2) Formulate request
        request = VisualizeMotionPlan.Request()  
        request.motion_plan = motion_plan
        request.two_arm_current_joint_angles = two_arm_current_joint_angles.astype(np.float64).tolist()
        request.is_robot1 = self.determine_which_robot_from_namespace(robot_namespace)

        # 3) Make async call (don't block thread using synchronous call)
        future = client.call_async(request)

        # 4) Wait for the request to be processed, but keep spinning to avoid locking up the thread:
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result() # Don't really care about the response
                    self.get_logger().info(f'Finished call to visualize motion plan.')
                    return
                except Exception as e:
                    self.get_logger().error("Service call failed: %r" % (e,))

    '''
    Inputs:
    -robot_namespace: the namespace of the robot for which you want to plan motion. Should be /robot1 or /robot2
    -motion_plan: MotionPlan object, output of get_motion_plan service call.
    '''
    def execute_motion_plan(self, robot_namespace, motion_plan, error_tolerance=.1, timeout_length=60):
        # 0.5) Get data into usable format:
        num_waypoints = motion_plan.num_waypoints
        motion_plan_flattened = motion_plan.motion_plan_data
        motion_plan = np.array(motion_plan_flattened).reshape(num_waypoints, -1)

        print('\nexecuting motion plan...')
        for i in range(num_waypoints):
            # Convert to degrees!
            waypoint_q_i = np.degrees(motion_plan[i])
            self.get_logger().info(f'Moving joints to waypoint: {waypoint_q_i}')

            is_reached = self.move_joints(waypoint_q_i, robot_namespace, error_tolerance, timeout_length)
            if not is_reached:
                print('ERROR: a waypoint was not reached. returning.')
                return
        
        print('\... completed execution of motion plan')

    '''
    Converts MotionPlan.msg datatype (from service call) to a readable, usable form (unflattens the motion plan and
    forms a proper matrix of shape num_waypoints x num_joints)
    '''
    def process_motion_plan_msg(self, motion_plan_object):
        num_waypoints = motion_plan_object.num_waypoints
        motion_plan_flattened = motion_plan_object.motion_plan_data
        motion_plan = np.array(motion_plan_flattened).reshape(num_waypoints, -1)

        return motion_plan # shape: num_waypoints x num_joints
    
    '''
    Service call

    Request inputs:
        - blending [float] from 0 to 100
    
    Response:
        - error [bool]: True if error occurred, False otherwise.
    '''
    def set_blending(self, robot_namespace, blending):
        # 1) Set up client
        client = self.create_client(SetBlending, f'{robot_namespace}/set_blending')
        while not client.wait_for_service(timeout_sec=1.0): # if still waiting on service for 1 sec print
            self.get_logger().warn("SetBlending service not available, waiting again...")
        
        # 2) Formulate request
        request = SetBlending.Request()  
        request.blending = float(blending)

        # 3) Make async call (don't block thread using synchronous call)
        future = client.call_async(request)
        
        # 4) Wait for the request to be processed, but keep spinning to avoid locking up the thread:
        rclpy.spin_until_future_complete(self, future) # note that we don't really care about the response (future.result()) itself.

        # 5) Process response:
        response = future.result()
        if response.error:
            raise Exception(f"ERROR: SetBlending only accepts a float in range [0, 100]. You gave {blending}.")
        self.get_logger().info(f"SetBlending to {blending} for {robot_namespace} complete.")

    '''
    Service call

    Request inputs:
        - gripper_force: from 5 to 100, which is a percentage of the maximum force the MEGP 25E gripper can hold (40N).
    
    Response:
        - error [bool]: True if error occurred, False otherwise.
    '''
    def set_gripper_force(self, robot_namespace, gripper_force):
        # 1) Set up client
        client = self.create_client(SetGripperForce, f'{robot_namespace}/set_gripper_force')
        while not client.wait_for_service(timeout_sec=1.0): # if still waiting on service for 1 sec print
            self.get_logger().warn("SetGripperForce service not available, waiting again...")
        
        # 2) Formulate request
        request = SetGripperForce.Request()  
        request.gripper_force = float(gripper_force)

        # 3) Make async call (don't block thread using synchronous call)
        future = client.call_async(request)
        
        # 4) Wait for the request to be processed, but keep spinning to avoid locking up the thread:
        rclpy.spin_until_future_complete(self, future) # note that we don't really care about the response (future.result()) itself.

        # 5) Process response:
        response = future.result()
        if response.error:
            raise Exception(f"ERROR: SetGripperForce only accepts a float in range [5, 100]. You gave {gripper_force}.")
        self.get_logger().info(f"SetGripperForce to {gripper_force} for {robot_namespace} complete.")


    '''
    Service call

    Request inputs:
        - gripper_vel: from 5 to 100, which is a percentage of the maximum finger velocity of the MEGP 25E gripper (∼100 mm/s).
    
    Response:
        - error [bool]: True if error occurred, False otherwise.
    '''
    def set_gripper_vel(self, robot_namespace, gripper_vel):
        # 1) Set up client
        client = self.create_client(SetGripperVel, f'{robot_namespace}/set_gripper_vel')
        while not client.wait_for_service(timeout_sec=1.0): # if still waiting on service for 1 sec print
            self.get_logger().warn("SetGripperVel service not available, waiting again...")
        
        # 2) Formulate request
        request = SetGripperVel.Request()  
        request.gripper_vel = float(gripper_vel)

        # 3) Make async call (don't block thread using synchronous call)
        future = client.call_async(request)
        
        # 4) Wait for the request to be processed, but keep spinning to avoid locking up the thread:
        rclpy.spin_until_future_complete(self, future) # note that we don't really care about the response (future.result()) itself.

        # 5) Process response:
        response = future.result()
        if response.error:
            raise Exception(f"ERROR: SetGripperVel only accepts a float in range [5, 100]. You gave {gripper_vel}.")
        self.get_logger().info(f"SetGripperVel to {gripper_vel} for {robot_namespace} complete.")

    '''
    Service Call

    Request inputs:
        - joint_vel: from 0.001 to 100, which is a percentage of maximum joint velocities.
            - NOTE while you can specify the velocity as .001, I do not recommend it, as it was so slow I did not visually see
              movement. 1 works pretty well for moving slowly; I also do not recommend 100, as that is dangerously fast.
            - NOTE see the meca programming manual (https://cdn.mecademic.com/uploads/docs/meca500-r3-programming-manual-8-3.pdf)
              for more information; the velocities of the different joints are set proportionally as a function
              of their max speed.
    Response:
        - error [bool]: True if error occurred, False otherwise.
    '''
    def set_joint_vel(self, robot_namespace, joint_vel):
        # 1) Set up client
        client = self.create_client(SetJointVel, f'{robot_namespace}/set_joint_vel')
        while not client.wait_for_service(timeout_sec=1.0): # if still waiting on service for 1 sec print
            self.get_logger().warn("SetJointVel service not available, waiting again...")
        
        # 2) Formulate request
        request = SetJointVel.Request()  
        request.joint_vel = float(joint_vel)

        # 3) Make async call (don't block thread using synchronous call)
        future = client.call_async(request)
        
        # 4) Wait for the request to be processed, but keep spinning to avoid locking up the thread:
        rclpy.spin_until_future_complete(self, future) # note that we don't really care about the response (future.result()) itself.

        # 5) Process response:
        response = future.result()
        if response.error:
            raise Exception(f"ERROR: SetJointVel only accepts a float in range [.001, 100]. You gave {joint_vel}.")
        self.get_logger().info(f"SetJointVel to {joint_vel} for {robot_namespace} complete.")

    '''
    Service call

    Request inputs:
        - joint_acc: from 0.001 to 150, which is a percentage of maximum acceleration of the joints, ranging from 0.001% to 150%
    Response:
        - error [bool]: True if error occurred, False otherwise.
    '''
    def set_joint_acc(self, robot_namespace, joint_acc):
        # 1) Set up client
        client = self.create_client(SetJointAcc, f'{robot_namespace}/set_joint_acc')
        while not client.wait_for_service(timeout_sec=1.0): # if still waiting on service for 1 sec print
            self.get_logger().warn("SetJointAcc service not available, waiting again...")
        
        # 2) Formulate request
        request = SetJointAcc.Request()  
        request.joint_acc = float(joint_acc)

        # 3) Make async call (don't block thread using synchronous call)
        future = client.call_async(request)
        
        # 4) Wait for the request to be processed, but keep spinning to avoid locking up the thread:
        rclpy.spin_until_future_complete(self, future) # note that we don't really care about the response (future.result()) itself.

        # 5) Process response:
        response = future.result()
        if response.error:
            raise Exception(f"ERROR: SetJointAcc only accepts a float in range [.001, 150]. You gave {joint_acc}.")
        self.get_logger().info(f"SetJointAcc to {joint_acc} for {robot_namespace} complete.")


'''
Run this after you have started up the meca_driver node and are connected to the robot (and have started the motion planner):
ros2 run meca_controller meca_control
'''
def main(args=None):
    rclpy.init(args=args) # init ros2 communications

    node = Meca_Control() # create node
    
    try:
        node.wait_for_initialization() # wait for subscribers to receive critical data (current joint angles)
        node.run() # run user code
        rclpy.spin(node) # continues to run node indefinitely until cancel with CTRL-C
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.try_shutdown() # "Shutdown this context, if not already shutdown." shuts down ros2 communications and nodes.