#!/usr/bin/env python3Publisher

'''
############################################################
############################################################
The purpose of this Motion_Planner node is to receive start and goal
joint angles and send back a series of waypoints to achieve the path
collision-free. Visualization is handled as well.

Date Created: 04/29/2022
Developers: Jessica Myers, [add name here]
University of Illinois Urbana-Champaign

------------------------------------------------------------
Future TODOs:
- The urdf_model_path is hard-coded in here. Maybe add to meca_settings file
  or elsewhere in the future to have all modifications in one place.
- Upon ctrl c, close the visualization window.
- Add the option to not start the visualizer when launching the motion planner.
- Build upon 'preview' feature (visualization) of motion plan; maybe overlay over
  a separate real-time data visualization (ALSO TODO - subscribe and create real-time
  visualization of the robot moving!)
- Add argument/param to specify what motion planner to use (bidir-rrt? others?)
- Make bidir rrt inherit/extend from a Motion Planner interface; implement the required methods.
- Check to make sure collision pairs are efficient

############################################################
############################################################
'''

import rclpy # python library for ros2. needed for every ros2 node
from rclpy.node import Node

import time
import numpy as np
from .robot_model import RobotModel
from .bidirectional_rrt import BidirectionalRRT
from .shortcutting import Shortcutting

from custom_interfaces.srv import GetMotionPlan, VisualizeMotionPlan
from custom_interfaces.msg import MotionPlan  

from meca_controller.meca_settings import ROBOT1, ROBOT2 # to keep namespaces consistent across all the code

# inherit from node class of rclpy
class Motion_Planner(Node):
    '''
    inputs: 
        - 
    '''
    def __init__(self):
        super().__init__("motion_planner") # give node name, shown in rqt_graph
        # s.getcwd() is meca_ws/, so the urdf is at: '/src/resources/meca/meca.urdf'
        # Initialize robot collision/visualization model for collision detection and motion planning, from URDF:
        self.robot_model = RobotModel(urdf_model_path = './src/resources/meca/meca.urdf',
                        mesh_dir = './src/resources/meca/',
                        urdf_model_path_env = './src/resources/environment/environment.urdf',
                        mesh_dir_env = './src/resources/environment/',
                        namespace1 = ROBOT1['namespace'], namespace2 = ROBOT2['namespace'])
        

        self.robot_model.start_visualizer() # TODO launch separately as a separate ros node?

        self.motion_planner = BidirectionalRRT(self.robot_model)
        self.shortcutting_algorithm = Shortcutting(self.robot_model)

        # Set up services:
        self.srv = self.create_service(GetMotionPlan, 'get_motion_plan', self.plan_motion_callback)
        self.srv = self.create_service(VisualizeMotionPlan, 'visualize_motion_plan', self.visualize_motion_plan_callback)

    '''
    Plans motion for a single robot, while taking into account the pose of the stationary robot for collision detection.

    Request Inputs:
    - is_robot1 [Boolean] : the robot for which you want to plan motion 
                    - NOTE that theoretically, you could plan motion for both robots at the same time (goal_config of len 12),
                      though this has not been tested and may not be entirely implemented. Synchronized MoveJoints for the two arms
                      also is not yet in place, which is why I have left this out for the time being.
    - start_config: (in RADIANS) 1D len 12 np.array specifying current (or starting) robot configuration (joint angles) of both
                                robots for the purpose of collision detection [robot1, robot2]. One robot will remain stationary.
    - goal_config: (in RADIANS) 1D len 6 np.array specifying goal joint angles for the robot whose motion is being planned.

    Return Message:
    - motion_plan: object of type MotionPlan.msg, see custom_interfaces for fields and format

    '''
    def plan_motion_callback(self, request, response):
        # 0) Get request data into the proper format: (arrays are of Python array type, i.e. array('d', [3.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        two_arm_start_q = np.array(request.start_config.tolist()) # len 12 array of [robot1, robot2] joint angles
        goal_q = np.array(request.goal_config.tolist()) # len 6 array of joint angles for one robot

        # 0.5) Get len 6 array for the robot for which motion is being planned:
        start_q = two_arm_start_q[:6] if request.is_robot1 else two_arm_start_q[6:]

        # Be transparent:
        self.get_logger().info(f'Incoming request: plan motion from {start_q} to {goal_q} for {"robot1" if request.is_robot1 else "robot2"}')
        self.get_logger().info(f'Starting 2-arm configuration: {two_arm_start_q}')

        # 1) Set the robot_model's current len-12 configuration to ensure proper 2-robot collision detection.
        self.robot_model.reset_to_default_position()
        self.robot_model.set_current_q(two_arm_start_q) # order is always: [robot1, robot2]

        # 2) Generate Motion Plan (takes RADIANS)
        robot_namespace = ROBOT1['namespace'] if request.is_robot1 else ROBOT2['namespace']
        motion_plan, num_sampled_configs, num_nodes = self.motion_planner.generate_motion_plan(robot_namespace, start_q, goal_q)
        
        if motion_plan is None:
            self.get_logger().warn(f"ERROR: Motion planner ran out of time; Increase the max number of iterations or check your goal configuration. ")
            return response

        # 3) Apply shortcutting:
        shortcutted_plan = self.shortcutting_algorithm.apply_shortcutting(robot_namespace, motion_plan)

        # 3.5) for debugging-- visualize motion plan?
        # print(shortcutted_plan)
        self.get_logger().info(f'Found a motion plan with {len(shortcutted_plan)} waypoints.')
        self.get_logger().info('Motion plan:')
        for waypt in shortcutted_plan:
            self.get_logger().info(f'   {waypt}')
        

        # 4) Prepare data to send back:
        motion_plan = MotionPlan()
        motion_plan.num_joint_angles = 6
        motion_plan.num_waypoints = len(shortcutted_plan) # To reshape flattened array later, do: .reshape(num_waypoints, -1)
        motion_plan.motion_plan_data = np.array(shortcutted_plan).flatten().astype(np.float64).tolist()

        response.motion_plan = motion_plan
        return response
    
    '''
    NOTE: Assumes the visualization is already active.

    Request Inputs:
    - is_robot1 [Boolean] : the robot executing the motion plan
    - two_arm_current_joint_angles: (in RADIANS) 1D len 12 np.array specifying current (or starting) robot configuration (joint angles)
      of both robots [robot1, robot2]. One robot will remain stationary.
    - motion_plan: object of type MotionPlan.msg, see custom_interfaces for fields and format

    '''
    def visualize_motion_plan_callback(self, request, response):
        # 0) Get data from request:
        motion_plan_object = request.motion_plan
        two_arm_current_joint_angles = np.array(request.two_arm_current_joint_angles.tolist()) # len 12 array of [robot1, robot2] joint angles
        robot_namespace = ROBOT1['namespace'] if request.is_robot1 else ROBOT2['namespace']

        # 0.5) Get data into usable format:
        num_waypoints = motion_plan_object.num_waypoints
        motion_plan_flattened = motion_plan_object.motion_plan_data
        motion_plan = np.array(motion_plan_flattened).reshape(num_waypoints, -1)

        # 1) Set the robot_model's current len-12 configuration for proper visualization:
        self.robot_model.reset_to_default_position()
        self.robot_model.update_visualization(two_arm_current_joint_angles) # order is always: [robot1, robot2]

        # 2) Visualize motion plan:
        for i in range(num_waypoints - 1):
            q0 = motion_plan[i]
            q1 = motion_plan[i+1]
            
            # Compute the distance in joint space from q0 to q1
            d = np.linalg.norm(q1 - q0)

            # Iterate through points sampled at fixed resolution along the straight-line
            # path from q0 to q1 (FIXME: should use "exact" collision-checking and not have
            # to rely on a fixed resolution)
            for s in np.linspace(0, 1, int(np.ceil(d / 0.01))):

                # Get intermediate configuration
                q = (1 - s) * q0 + s * q1
                in_collision = self.robot_model.update_visualization(q, robot_namespace)

                # Stop if a collision was found, otherwise sleep for 1 ms before proceeding
                if in_collision:
                    print('COLLISION!')
                    break
                else:
                    time.sleep(0.001)
        
        return response


'''
Run: ros2 run meca_motion_planner motion_planner
'''
def main(args=None):
    rclpy.init(args=args) # init ros2 communications

    node = Motion_Planner() # create node

    try:
        rclpy.spin(node) # continues to run node indefinitely until cancel with CTRL-C
    except KeyboardInterrupt:
        print('disconnecting... ')

        # CTRL-C seems to already call shutdown; this is here as a safety:
        rclpy.try_shutdown() # shutdown communications and nodes, if not already shut down.
