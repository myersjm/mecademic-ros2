#!/usr/bin/env python3Publisher

'''
############################################################
############################################################
Purpose: This node establishes a connection to whichever robot is specified in the arguments;
It receives commands over service requests from other nodes (meca_control.py) and carries
them out by communicating with the robot using the mecademicpy library. It also publishes
the robot's state (status, joints, pose) over a topic, to which other nodes can subscribe
to learn the current joint angles, etc.

This code was adapted from the Mecademic ROS 1 driver node at the following repo:
https://github.com/Mecademic/ROS/tree/5c471e98a834b68503c93bd4c6a4719c32e3e491
but most of it has been modified or improved upon.

Date Created: 04/10/2022
Developers: Jessica Myers, Nithya Koneru, Max Hartman, Tim Bretl, [add name here]
University of Illinois Urbana-Champaign


############################################################
############################################################
'''

import rclpy # python library for ros2. needed for every ros2 node
from rclpy.node import Node
import mecademicpy.robot as mdr
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, UInt8MultiArray
from custom_interfaces.srv import MoveJoints, GoToPose, MoveGripper, SetBlending, SetGripperForce, SetGripperVel, SetJointVel, SetJointAcc
from custom_interfaces.msg import RobotStatus, GripperStatus
import numpy as np
import time
from meca_controller.meca_settings import ROBOT1, ROBOT2 # ADJUST THESE GLOBAL CONSTANTS

class Meca_Driver(Node):
    def __init__(self):
        super().__init__("meca_driver")
        self.namespace = self.get_namespace() # /robot1 or /robot2
        self.robot = mdr.Robot() # mecademic py api interface
        print('starting up...')

        # Connect to robot:
        if self.namespace == ROBOT1['namespace']:
            self.robot.Connect(address=ROBOT1['ip'])
        elif self.namespace == ROBOT2['namespace']:
            self.robot.Connect(address=ROBOT2['ip'])
        else:
            raise Exception(f"Provided namespace must either be {ROBOT1['namespace']} or {ROBOT2['namespace']}. Example usage: --ros-args -r __ns:={ROBOT1['namespace']}")

        # Start robot:
        self.robot.ActivateRobot()
        self.robot.Home()
        self.robot.SetRealTimeMonitoring('all')
        self.robot.WaitIdle(timeout=60) # wait for the robot to finish its initial movement
        print('ready.')

        # Set monitoring interval for real time data:
        self.MONITORING_INTERVAL = 0.001 # seconds TODO ADJUST THIS
        self.robot.SetMonitoringInterval(self.MONITORING_INTERVAL)
        DATA_LOGGING_TIME_INTERVAL = .001 # seconds TODO ADJUST THIS
        self.create_timer(DATA_LOGGING_TIME_INTERVAL, self.timed_data_logging_callback)

        # Init publishers to publish robot state: 
        # NOTE that the queue size (1) was from the prior drivercode--Adjust this?
        self.joint_publisher = self.create_publisher(JointState, "MecademicRobot_joint_fb", 1)
        self.pose_publisher = self.create_publisher(Pose, "MecademicRobot_pose_fb", 1)
        self.robot_status_publisher = self.create_publisher(RobotStatus, "MecademicRobot_status", 1)
        self.gripper_status_publisher = self.create_publisher(GripperStatus, "MecademicRobot_gripper_status", 1)

        # Set up services: NOTE that when this node is run with namespace provided, the service will be '{namespace}/move_joints/'
        self.srv_move_joints = self.create_service(MoveJoints, 'move_joints', self.move_joints_callback)
        self.srv_go_to_pose = self.create_service(GoToPose, 'go_to_pose', self.go_to_pose_callback)
        self.srv_move_gripper = self.create_service(MoveGripper, 'move_gripper', self.move_gripper_callback)

        self.srv_set_blending= self.create_service(SetBlending, 'set_blending', self.set_blending_callback)
        self.srv_set_gripper_force = self.create_service(SetGripperForce, 'set_gripper_force', self.set_gripper_force_callback)
        self.srv_set_gripper_vel = self.create_service(SetGripperVel, 'set_gripper_vel', self.set_gripper_vel_callback)
        self.srv_set_joint_vel = self.create_service(SetJointVel, 'set_joint_vel', self.set_joint_vel_callback)
        self.srv_set_joint_acc = self.create_service(SetJointAcc, 'set_joint_acc', self.set_joint_acc_callback)

        # Keep track of error state:
        self.is_in_error = False

    '''
    Purpose: Shuts down the robots, deactivating and closing the socket connection (disconnecting).
    '''
    def stop(self):
        try:
            self.robot.WaitIdle(60) 
            self.robot.DeactivateRobot()
            self.robot.Disconnect()
            print('stopped')
        except mdr.DisconnectError:
            self.stop_after_disconnect_error()

    '''
    Purpose: the DisconnectError is raised by Mecademicpy when an exception occurs, even if that exception is handled. In this
             node, CTRL-C is used to shut down the node and robots, so we do not want things to end without the robot being
             properly deactivated and disconnected. This function will reconnect for the purpose of properly shutting things down.
    '''
    def stop_after_disconnect_error(self):
        # 1) Reconnect so that we can properly shut down:
        if self.namespace == ROBOT1['namespace']:
            self.robot.Connect(address=ROBOT1['ip'])
        else: # is robot2
            self.robot.Connect(address=ROBOT2['ip'])
        
        # 2) Reset the error, if there is any, and resume robot motion if it is paused (it will be after a DisconnectError):
        self.robot.ResetError() # just in case
        self.robot.ResumeMotion()
        self.stop()

    '''
    If the robot is in an error state, this can be called to reset the error and resume motion.
    '''
    def handle_error(self):
        print('robot is in error state, correcting..')
        self.robot.ResetError()
        self.is_in_error = False
        print('...error state corrected.')
        self.robot.ResumeMotion()

    '''
    Retrieves live robot status, gripper status, joint state (position and velocity), and pose every 
    DATA_LOGGING_TIME_INTERVAL seconds and publishes the data to its corresponding topic.

    For more information about the robot status fields, see:
    https://github.com/Mecademic/mecademicpy/blob/main/mecademicpy/robot_classes.py#L892
    https://github.com/Mecademic/mecademicpy/blob/main/mecademicpy/robot_classes.py#L941

    For more info about the Pose (formerly RobotFeedback's cartesian field in the original driver), see
    the old MecademicDriver GitHub:
    https://github.com/Mecademic/python_driver/blob/master/MecademicRobot/RobotFeedback.py#L56
    "Cartesian coordinates, distances in mm, angles in degrees | [x,y,z,alpha,beta,gamma]"
    '''
    def timed_data_logging_callback(self):
        # 1) Get robot and gripper status:
        robot_status = self.robot.GetStatusRobot()
        gripper_status = self.robot.GetStatusGripper()
        
        # 2) Fill out msg objects:
        status = RobotStatus()
        status.activation_state = robot_status.activation_state
        status.brakes_engaged = robot_status.brakes_engaged
        status.end_of_block_status = robot_status.end_of_block_status
        status.error_status = robot_status.error_status
        status.estop_state = robot_status.estopState
        status.homing_state = robot_status.homing_state
        status.pause_motion_status = robot_status.pause_motion_status
        status.pstop2_state = robot_status.pstop2State
        status.recovery_mode = robot_status.recovery_mode
        status.simulation_mode = robot_status.simulation_mode

        grip_status = GripperStatus()
        grip_status.error_status = gripper_status.error_status
        grip_status.holding_part = gripper_status.holding_part
        grip_status.homing_state = gripper_status.homing_state
        grip_status.overload_error = gripper_status.overload_error
        grip_status.present = gripper_status.present
        grip_status.target_pos_reached = gripper_status.target_pos_reached

        # 3) Publish data:
        self.robot_status_publisher.publish(status)
        self.gripper_status_publisher.publish(grip_status)

        # 4) Keep track of whether the robot is in error:
        self.is_in_error = robot_status.error_status

        #########################################################

        # 1) Get robot joint and pose data:
        data = self.robot.GetRobotRtData(synchronous_update=True)

        # 2) Fill out joint state msg:
        joints_fb = JointState()
        # Joint angles in degrees:
        joints_fb.position = [data.rt_joint_pos.data[0],data.rt_joint_pos.data[1],data.rt_joint_pos.data[2],
                                data.rt_joint_pos.data[3],data.rt_joint_pos.data[4],data.rt_joint_pos.data[5]]
        # Joint velocity in degrees/sec:
        joints_fb.velocity = [data.rt_joint_vel.data[0],data.rt_joint_vel.data[1],data.rt_joint_vel.data[2],
                                data.rt_joint_vel.data[3],data.rt_joint_vel.data[4],data.rt_joint_vel.data[5]]
        
        # TODO Determine where to find joint effort info to finish out the JointState topic
        # TODO Figure out where to place rt_joint_torq

        # 3) Fill out the pose msg:
        pose_fb = Pose() # rt_cart_pos: Drive-measured end effector pose [x, y, z, alpha, beta, gamma]
        pose_fb.position.x = data.rt_cart_pos.data[0]
        pose_fb.position.y = data.rt_cart_pos.data[1]

        if len(data.rt_cart_pos.data) == 4: # Not sure why this is here -- from the prior code. Leaving in case it means something.
            pose_fb.orientation.x = data.rt_cart_pos.data[2]
            pose_fb.orientation.y = data.rt_cart_pos.data[3]
        else:
            pose_fb.position.z = data.rt_cart_pos.data[2]
            pose_fb.orientation.x = data.rt_cart_pos.data[3]
            pose_fb.orientation.y = data.rt_cart_pos.data[4]
            pose_fb.orientation.z = data.rt_cart_pos.data[5]
        # TODO The 'w' field in the pose orientation msg was not set in the prior code; unsure what this should be, if anything.

        # 4) Publish to topics:
        self.joint_publisher.publish(joints_fb)
        self.pose_publisher.publish(pose_fb)

    '''
    NOTE: formerly, I had written this as a blocking call, waiting on the robot to reach the desired position and return isReached
    if it got there before a timeout period. Blocking is bad, so I added spin once to the loop, but turns out you can't spin within
    a callback, as that is recursive spinning which is not supported with single-threaded nodes. The wait functionality has been
    moved to meca_control.py; as a result, there is no longer a return value associated with this callback, because MoveJoints
    does not return anything. It would be great to have a success value in the future, but for now, it returns the unfilled response
    and success is defined outside of here, in meca_control, when the joints meet the desired angles.

    Request inputs:
        -float64[] requested_joint_angles
    
    Response: None (besides whatever is passed in)
    '''
    def move_joints_callback(self, request, response):
        print('move joints callback')
        # 0) Get requested joint angles:
        desired_joint_angles = np.array(request.requested_joint_angles.tolist())
        self.get_logger().info(f'Moving joints: {desired_joint_angles}')
        
        # 1) Move Joints:
        self.robot.MoveJoints(*desired_joint_angles) # has no return value

        # 2) Return
        return response
    
    '''
    NOTE this has not yet been tested or incorporated with the motion planner. Use at your own risk.

    Request inputs:
        - Pose msg: position and orientation information
    
    Response: None
    '''
    def go_to_pose_callback(self, request, response):
        print('pose callback')
        # 1) Move Pose:
        if request.pose.position.z is not None:
            self.get_logger().info('Going to pose: ', [request.pose.position.x, request.pose.position.y, request.pose.position.z,
                                                        request.pose.orientation.x, request.pose.orientation.y, request.pose.orientation.z])
            self.robot.MovePose(request.pose.position.x, request.pose.position.y, request.pose.position.z,
                                request.pose.orientation.x, request.pose.orientation.y, request.pose.orientation.z)
        else:
            self.get_logger().info('Going to pose: ', [request.pose.position.x, request.pose.position.y,
                                                        request.pose.orientation.x, request.pose.orientation.y])
            self.robot.MovePose(request.pose.position.x, request.pose.position.y, request.pose.orientation.x, request.pose.orientation.y)

        # 2) Return
        return response

    '''
    Request inputs:
        - command [String]: {"open", "close", "pos"}
        - pos [float]: gripper position in mm in range [0, 5.6]
    
    Response:
        - error [bool]: True if error occurred, False otherwise.
    '''
    def move_gripper_callback(self, request, response):
        if request.command == "open":
            self.get_logger().info(f'Opening gripper')
            self.robot.GripperOpen()
        elif request.command == "close":
            self.get_logger().info(f'Closing gripper')
            self.robot.GripperClose()
        elif request.command == "pos":
            if (request.pos > 5.6) or (request.pos < 0):
                self.get_logger().warn(f"ERROR: MoveGripper pos only accepts a float in range [0, 5.6]")
                response.error = True
                return response
            self.get_logger().info(f'Moving gripper fingers to position {request.pos}')
            self.robot.MoveGripper(request.pos)
        else:
            self.get_logger().warn(f"ERROR: Invalid command given to MoveGripper service; only accepts 'open', 'close', or 'pos'.")
            response.error = True
            return response
        
        response.error = False
        return response

    '''
    NOTE I would print/log the blending after it has been set via GetBlending (available through the web interface),
    but this is for some reason not available to my knowledge in the mecademicpy library. In the future if we use a
    different communication protocol, this would be a nice feature / way to double check.

    Request inputs:
        - blending [float] from 0 to 100
    
    Response:
        - error [bool]: True if error occurred, False otherwise.
    '''
    def set_blending_callback(self, request, response):
        self.get_logger().info(f'Setting blending to: {request.blending}')
        
        if (request.blending > 100) or (request.blending < 0):
            self.get_logger().warn(f"ERROR: SetBlending only accepts a float in range [0, 100]")
            response.error = True
            return response

        self.robot.SetBlending(request.blending)
        response.error = False
        return response

    '''
    Request inputs:
        - gripper_force: from 5 to 100, which is a percentage of the maximum force the MEGP 25E gripper can hold (40N).
    
    Response:
        - error [bool]: True if error occurred, False otherwise.
    '''
    def set_gripper_force_callback(self, request, response):
        self.get_logger().info(f'Setting gripper force to: {request.gripper_force}')
        
        if (request.gripper_force > 100) or (request.gripper_force < 5):
            self.get_logger().warn(f"ERROR: SetGripperForce only accepts a float in range [5, 100]")
            response.error = True
            return response

        self.robot.SetGripperForce(request.gripper_force)
        response.error = False
        return response
    
    '''
    Request inputs:
        - gripper_vel: from 5 to 100, which is a percentage of the maximum finger velocity of the MEGP 25E gripper (∼100 mm/s).
    
    Response:
        - error [bool]: True if error occurred, False otherwise.
    '''
    def set_gripper_vel_callback(self, request, response):
        self.get_logger().info(f'Setting gripper velocity to: {request.gripper_vel}')
        
        if (request.gripper_vel > 100) or (request.gripper_vel < 5):
            self.get_logger().warn(f"ERROR: SetGripperVel only accepts a float in range [5, 100]")
            response.error = True
            return response

        self.robot.SetGripperVel(request.gripper_vel)
        response.error = False
        return response

    '''
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
    def set_joint_vel_callback(self, request, response):
        self.get_logger().info(f'Setting joint velocity to: {request.joint_vel}')
        
        if (request.joint_vel > 100) or (request.joint_vel < .001):
            self.get_logger().warn(f"ERROR: SetJointVel only accepts a float in range [.001, 100]")
            response.error = True
            return response

        self.robot.SetJointVel(request.joint_vel)
        response.error = False
        return response

    '''
    Request inputs:
        - joint_acc: from 0.001 to 150, which is a percentage of maximum acceleration of the joints, ranging from 0.001% to 150%
    Response:
        - error [bool]: True if error occurred, False otherwise.
    '''
    def set_joint_acc_callback(self, request, response):
        self.get_logger().info(f'Setting joint acceleration to: {request.joint_acc}')
        
        if (request.joint_acc > 150) or (request.joint_acc < .001):
            self.get_logger().warn(f"ERROR: SetJointAcc only accepts a float in range [.001, 100]")
            response.error = True
            return response

        self.robot.SetJointAcc(request.joint_acc)
        response.error = False
        return response


'''
Run this node in the following manner, passing in the name associated with the robot as the namespace (__ns):
ros2 run meca_controller meca_driver --ros-args -r __ns:=/robot1
ros2 run meca_controller meca_driver --ros-args -r __ns:=/robot2

Side note, in the future if you want additional parameters/arguments passed in you can do: -p param_name:=param_value
and access it inside the node class above by doing:
        self.declare_parameter('param_name', 'default value')
        print(self.get_parameter('param_name').value)

'''
def main(args=None):
    rclpy.init(args=args) # init ros2 communications

    node = Meca_Driver() # create node

    try:
        rclpy.spin(node) # continues to run node indefinitely until cancel with CTRL-C
    except KeyboardInterrupt:
        # automatically deactivates and disconnects robot once CTRL-C has been received.
        print('disconnecting... ')
        node.stop()
    except mdr.DisconnectError:
        """
        Sometimes CTRL-C triggers a DisconnectError from Mecademicpy before the KeyboardInterrupt exception is caught. This
        DisconnectError is directly because of the KeyboardInterrupt and causes the robot to disconnect, which we do not want, as
        we are trying to shut things down (deactivate). This function reconnects for the purpose of shutting things down completely
        so the robot isn't in a weird state:
        """
        print('disconnecting... ') # To avoid confusing the user, keeping print statements truthful to the broader goal of shutting down.
        node.stop_after_disconnect_error()
    
    # CTRL-C seems to already call shutdown; this is here as a safety:
    rclpy.try_shutdown() # shutdown communications and nodes, if not already shut down.
