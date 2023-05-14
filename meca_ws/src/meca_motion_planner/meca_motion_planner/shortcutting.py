'''
############################################################
############################################################
Purpose: This file houses the motion planning logic.

Date created: 03/06/2023 (originally), here 4/30/2023
Developers: Jessica Myers, [add name here]
University of Illinois Urbana-Champaign

############################################################
############################################################
'''

import numpy as np

class Shortcutting:
    '''
    Inputs: -robot model
    '''
    def __init__(self, robot_model):
        self.robot_model = robot_model

    '''
    Takes a list of configurations (waypoints) and creates a list of line segments (pairs of start waypt, end waypt).
    '''
    def get_list_of_segments(self, motion_plan):
        segment_list = []
        for i in range(1, len(motion_plan)):
            segment_list.append((motion_plan[i-1], motion_plan[i]))
        return segment_list

    '''
    This interpolation was taken from the straight-line-path collision check in the motion planning algorithm itself.
    '''
    def get_random_q_within_segment(self, q1, q2, resolution=.01):
        # Compute the distance in joint space from q1 to q2:
        d = np.linalg.norm(q1 - q2)
        
        discretized_linear_path = np.linspace(0, 1, int(np.ceil(d / resolution)))
        if len(discretized_linear_path) == 0:
            # print('Could not sample random q within segment, unless the resolution gets any smaller.')
            return False
        s = np.random.choice(discretized_linear_path)
        q = (1 - s)*q1 + s*q2 # intermediate config
        return q
        
    '''
    Goes through the list of q's, looks for the insert_after q, inserts q1, q2, then removes everything until insert_before.
    Creates a new list instead of removal though, for simplicity.

    [q_start, ... insert_after_q, ... insert_before_q, ... q_goal]       Before
    [q_start, ... insert_after_q, q1, q2, insert_before_q, ... q_goal]   After (deletes "..." portion)
    '''
    def revise_motion_plan(self, motion_plan, insert_after, insert_before, q1, q2):
        new_plan = []
        last = None # keep track of the last element
        encountered_initial = False
        wait_for_endpoint = False # once q1, q2 have been inserted, there may be waypoints that need to be skipped until
        for q in motion_plan:     # reaching the insert_before q.
            if (q == insert_after).all() and not encountered_initial:
                new_plan.append(q)
                new_plan.append(q1)
                new_plan.append(q2)
                wait_for_endpoint = True
                encountered_initial = True
                last = q2
                continue
            elif wait_for_endpoint:
                if (q == insert_before).all():
                    wait_for_endpoint = False
                    if not (last == q).all(): # Don't append duplicates
                        new_plan.append(q)
                # else, keep waiting...
            else:
                if not (last == q).all(): # Don't append duplicates
                    new_plan.append(q)
            last = q
        return new_plan
        
    '''
    Shortcut motion plan

                        Segment 1      Segment 2
    [(pair) (pair) .. (start_____) ... (____end) (pair) (pair)]
    [(pair) (pair) .. (start, random_q1) (random_q1, random_q2) (random_q2, end) (pair) (pair)]


    Question: won't this create tons more waypoints?

    '''
    def shortcut_path(self, robot_namespace, motion_plan):
        # 1) Get list of segments (tuples of waypoints describing the piecewise line segments of the motion plan):
        segment_list = self.get_list_of_segments(motion_plan)

        if len(segment_list) <= 1:
            return motion_plan # the rest of the code depends on there being at least 2 segments

        # 2) Randomly choose 2 segments:
        segment_ixs  = np.random.choice(np.arange(0,len(segment_list)), replace=False, size=2)
        segment_ixs.sort() # always just sorting 2 numbers, not a big deal in terms of runtime

        seg1 = segment_list[segment_ixs[0]]
        seg2 = segment_list[segment_ixs[1]]

        # 3) Randomly sample a configuration somewhere along the straight-line path of each line segment:
        q1 = self.get_random_q_within_segment(*seg1)
        q2 = self.get_random_q_within_segment(*seg2)
        
        if q1 is False or q2 is False:
            # Shortcut not made.
            # print('Shortcut not made.')
            return motion_plan

        # 4) Check for collisions in linear path between the two randomly sampled configurations:
        in_collision = self.check_for_straight_line_path_collision(q1, q2, robot_namespace)

        # 5) If there is no collision, modify motion plan, inserting this new segment, replacing what had been there before:
        if not in_collision:
            shortcutted_plan = self.revise_motion_plan(motion_plan, seg1[0], seg2[1], q1, q2)
            # Shortcut made.
            # print('Shortcut made.')
            return shortcutted_plan
        # Shortcut not made.
        # print('Shortcut not made.')
        return motion_plan

    # For debugging purposes:
    # def print_motion_plan(self, motion_plan):
    #     print([x[0] for x in motion_plan])
    #     print([f'|{b[0][0]}, {b[1][0]}|'for b in get_list_of_segments(motion_plan)])
        
    '''
    This calls the actual shortcut algorithm for a certain number of iterations.
    '''
    def apply_shortcutting(self, robot_namespace, motion_plan, num_iterations=50):
        for _ in range(num_iterations):
            motion_plan = self.shortcut_path(robot_namespace, motion_plan)
        return motion_plan
    
    '''
    Inputs: robot configurations q1 and q2; resolution of interpolation can be specified as well.
    '''
    def check_for_straight_line_path_collision(self, q1, q2, robot_namespace, resolution=.01):
        # Compute the distance in joint space from q1 to q2:
        d = np.linalg.norm(q1 - q2)

        for s in np.linspace(0, 1, int(np.ceil(d / resolution))):
            q = (1 - s)*q1 + s*q2 # intermediate config
            in_collision = self.robot_model.check_for_collision(q, robot_namespace)
            if in_collision:
                return True
        return False