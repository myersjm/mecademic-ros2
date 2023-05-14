"""
Tests run while developing the code. These were originally in meca_control.py's 'run' method.
Placing here until I find a better place for user code, and for in the future / for reference.

NOTE this file cannot be run, it simply houses some old test code.


###################################################################
####################### TEST Move Joints ##########################
###################################################################

self.move_joints(np.array([5, 0, 0, 0, 0, 0]), ROBOT1['namespace')

###################################################################
############# TEST Get Motion Plan and Visualize ##################
###################################################################
                
motion_plan = self.get_motion_plan(ROBOT1['namespace'],
                                    np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
                                    np.radians([-5, 80.9, -7.3, 0, -74.6, 0]))
        
        
print('\nVisualizing motion plan ... ')
self.visualize_motion_plan(ROBOT1['namespace'], motion_plan)

###################################################################
######## TEST Get Current Joint Angles of Both Robots #############
###################################################################

print(self.get_current_joints_both_robots(ROBOT1['namespace']))

###################################################################
####################### TEST Gripper ##############################
###################################################################

self.set_gripper_vel(ROBOT1['namespace'], 10)
time.sleep(1)
self.move_gripper(ROBOT1['namespace'], 3)
time.sleep(1)
self.close_gripper(ROBOT1['namespace'])
time.sleep(1)
self.open_gripper(ROBOT1['namespace'])
time.sleep(1)
self.close_gripper(ROBOT1['namespace'])
time.sleep(1)
self.set_gripper_vel(ROBOT1['namespace'], 100)
time.sleep(1)
self.open_gripper(ROBOT1['namespace'])
time.sleep(1)
self.close_gripper(ROBOT1['namespace'])
time.sleep(1)
self.open_gripper(ROBOT1['namespace'])
time.sleep(1)

self.close_gripper(ROBOT2['namespace'])
time.sleep(1)
self.open_gripper(ROBOT2['namespace'])
time.sleep(1)
self.set_gripper_vel(ROBOT2['namespace'], 10)
time.sleep(1)
self.close_gripper(ROBOT2['namespace'])
time.sleep(1)
self.open_gripper(ROBOT2['namespace'])

###################################################################
####################### TEST Set Commands #########################
###################################################################
# Testing new commands:
self.set_blending(ROBOT1['namespace'], 0)
self.set_gripper_force(ROBOT1['namespace'], 5)
self.set_gripper_vel(ROBOT1['namespace'], 10)
self.set_joint_acc(ROBOT1['namespace'], 2)
self.set_joint_vel(ROBOT1['namespace'], 10)
self.move_joints(np.array([0, 0, 0, 0, 0, 0]), ROBOT1['namespace'])

# These SHOULD error:
# self.set_gripper_vel(ROBOT1['namespace'], 0) # testing errors
# self.set_gripper_force(ROBOT1['namespace'], 101)
# self.set_gripper_vel(ROBOT1['namespace'], 101)
# self.set_joint_acc(ROBOT1['namespace'], 151)
# self.set_joint_vel(ROBOT1['namespace'], 151)

###################################################################
################ TEST No Motion Plan Found ########################
###################################################################
# This should throw an exception because the motion planner should
# run out of time (I forced this to happen by making the goal config
# not feasible / in collision):
start_joint_angles = np.radians([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
motion_plan1 = self.get_motion_plan(ROBOT2['namespace'],
                                    start_joint_angles,
                                    np.radians(np.array([90, 90, 90, 90, 90, 90])))


###################################################################
################ TEST Motion Planning #############################
###################################################################
# Plans and visualizes motion to a specific joint angle set, then 
# back to the start, and then executes on the real robots.
# Set speed low!

start_joint_angles = np.radians(self.get_current_joints_both_robots(ROBOT1['namespace']))
motion_plan1 = self.get_motion_plan(ROBOT1['namespace'],
                                    start_joint_angles,
                                    np.radians(np.array([-5, 80.9, -7.3, 0, -74.6, 0])))
        
print('\nVisualizing motion plan ... ')
self.visualize_motion_plan(ROBOT1['namespace'],
                           start_joint_angles,
                            motion_plan1)
        
motion_plan1_matrix = self.process_motion_plan_msg(motion_plan1)

end_joint_angles = np.concatenate([motion_plan1_matrix[-1, :], start_joint_angles[6:]]) # robot2 stays unmoving
        
motion_plan2 = self.get_motion_plan(ROBOT1['namespace'],
                                    end_joint_angles,
                                    start_joint_angles[:6])
        
        
print('\nVisualizing motion plan ... ')
self.visualize_motion_plan(ROBOT1['namespace'],
                             np.radians(self.get_current_joints_both_robots(ROBOT1['namespace'])),
                             motion_plan2)
        
# self.execute_motion_plan(ROBOT1['namespace', motion_plan1, error_tolerance=.1, timeout_length=60)
# self.execute_motion_plan(ROBOT1['namespace', motion_plan2, error_tolerance=.1, timeout_length=60)

###################################################################
# Same as above but on robot2:
# start_joint_angles = np.radians(self.get_current_joints_both_robots(ROBOT2['namespace']))
# motion_plan1 = self.get_motion_plan(ROBOT2['namespace'],
#                                    start_joint_angles,
#                                    np.radians(np.array([-5, 80.9, -7.3, 0, -74.6, 0])))


# print('\nVisualizing motion plan ... ')
# self.visualize_motion_plan(ROBOT2['namespace'],
#                             start_joint_angles,
#                             motion_plan1)

# motion_plan1_matrix = self.process_motion_plan_msg(motion_plan1)

# end_joint_angles = np.concatenate([start_joint_angles[:6], motion_plan1_matrix[-1, :]]) # robot1 stays unmoving

# motion_plan2 = self.get_motion_plan(ROBOT2['namespace'],
#                                    end_joint_angles,
#                                    start_joint_angles[6:])

# print('\nVisualizing motion plan ... ')
# self.visualize_motion_plan(ROBOT2['namespace'],
#                             np.radians(self.get_current_joints_both_robots(ROBOT2['namespace'])),
#                             motion_plan2)

# self.execute_motion_plan(ROBOT2['namespace'], motion_plan1, error_tolerance=.1, timeout_length=60)
# self.execute_motion_plan(ROBOT2['namespace'], motion_plan2, error_tolerance=.1, timeout_length=60)

###################################################################
################ TEST for deadlocks ###############################
###################################################################
# Test for deadlocks by making a service call to move a joint that takes a long time,
# and having a print statement in meca_driver's data publishing callback, so that we can 
# tell whether the data publishing callback gets delayed. if it just stops printing, then it is getting deadlocked
# until the blocking call to move joints finishes. 
self.move_joints(np.array([90, 0, 0, 0, 0, 0]), '/robot1', error_tolerance=.001, timeout_length=60)
self.move_joints(np.array([0, 0, 0, 0, 0, 0]), '/robot1', error_tolerance=.001, timeout_length=60)
print('done')

"""