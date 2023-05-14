'''
############################################################
############################################################
Purpose: This file houses the motion planning logic.

Date created: 03/06/2023 (originally), here 4/29/2023
Developers: Jessica Myers
(add your name here if you added something to this file!)

TODO
- make bidir rrt inherit/extend from Motion Planner interface. implement the required methods.
- In this code you will see self.robot_model.get_random_config_single_robot() -- currently motion planning is done for one robot
  at a time; update this to work with planning for two robots?
############################################################
############################################################
'''

import numpy as np

'''
Sample-Based Motion Planning: Bi-Directional RRT (Rapidly-Exploring Random Tree)
Description: Two trees that are acyclic graphs will be maintained and grown until they meet.
'''
class BidirectionalRRT:
    '''
    Inputs: -robot model
    '''
    def __init__(self, robot_model):
        self.robot_model = robot_model

    '''
    NOTE: this is currently written to plan motion for a SINGLE robot while the other is stationary (accepts len 6 joint angles);
    It does however respect collision detection for the other robot as long as you set the current joint angles for the robot_model
    to the current configuration before calling this. Theoretically this could be slightly modified to plan for both robots at the
    same time.

    Inputs: -robot_namespace: namespace of the robot for which you wish to plan motion (must match namespaces given to robot_model)
            -q_start: np.array containing start robot configuration (joint angles)
            -q_goal: np.array containing goal (end) robot configuration (joint angles)
            
    Returns: -motion_plan (List of np.arrays containing robot configurations)
             -num_sampled_configs and num_nodes in tree, purely for analysis of algorithm performance
    '''
    def generate_motion_plan(self, robot_namespace, q_start, q_goal, max_iterations=500):
        # Initialize variables for keeping track of sampling statistics for algorithmic performance:
        num_sampled_configs = 0
        num_nodes = 2 # for the start, goal nodes

        # 0) Initialize start and goal trees as nodes with parents == None
        start_root = Node(q_start)
        goal_root = Node(q_goal)

        max_free_space_sampling_attempts = 50 # should rarely need this many; just for robustness.
        for _ in range(max_iterations):
            # 1) Sample a collision-free configuration:
            sampled_q = None
            for _ in range(max_free_space_sampling_attempts):
                q = self.robot_model.get_random_config_single_robot() # TODO / NOTE this is currently for single robot motion planning
                num_sampled_configs+=1

                in_collision = self.robot_model.check_for_collision(q, robot_namespace)
                if not in_collision: 
                    sampled_q = q
                    break

            if sampled_q is None:
                # TODO add error handling, if this ever actually happens:
                print(f'ERROR: Could not sample a collision-free configuration (i.e. in free space) within {max_free_space_sampling_attempts} attempts')
            num_nodes+=1

            # 2a) Find the node (waypoint) in the start tree closest to sampled_q:
            nearest_node = self.find_nearest_point(start_root, sampled_q)

            # 2b) Check for straight-line path collisions:
            path_in_collision = self.check_for_straight_line_path_collision(nearest_node.config, sampled_q, robot_namespace)

            # 2c) Add to tree if no collisions:
            new_node = None
            if not path_in_collision:
                new_node = nearest_node.add_child(sampled_q)

            # 3a) Find node in goal tree closest to sampled_q:
            nearest_node = self.find_nearest_point(goal_root, sampled_q)

            # 3b) Check for straight-line path collisions:
            path_in_collision = self.check_for_straight_line_path_collision(nearest_node.config, sampled_q, robot_namespace)

            # 3c) If no collisions, either add to goal tree only, or connect the trees and be done:
            if not path_in_collision:
                if new_node is None:
                    nearest_node.add_child(sampled_q)
                else: # new_node has been connected to start tree, whole tree can be connected:
                    # Note--can't physically connect the trees unless post-process the goal tree and
                    # flip the order of the parent/children somehow. Don't have to connect them, just need to get
                    # the ordering.
                    # print('Tree connected!')
                    motion_plan_start = list(reversed(self.get_path_to_root(new_node))) # reverse list
                    motion_plan_end = self.get_path_to_root(nearest_node)
                    motion_plan = motion_plan_start + motion_plan_end
                    return motion_plan, num_sampled_configs, num_nodes
                
        print(f'Ran out of time, no motion plan found in {max_iterations} iterations.')
        return None, num_sampled_configs, num_nodes
                
    '''
    Input: root_node (Node), config (a np array of joint angles)
    Returns: Node

    Iterates through all the nodes in the tree, finding the closest node (robot config) to the given config.
    '''
    def find_nearest_point(self, root_node, config):
        all_nodes = root_node.depth_first_iterate([], [])
    #     print([n.config for n in all_nodes])

        min_dist = None
        nearest_node = None
        for node in all_nodes:
            dist = np.sum((node.config - config)**2)

            if min_dist is None or dist < min_dist:
                min_dist = dist
                nearest_node = node

        return nearest_node

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

    '''
    Follows a node's parents all the way back to the root, keeping track of the path as a python list.
    
    Inputs: -Node
    Returns: -path to root (List of robot configurations)
    '''
    def get_path_to_root(self, new_node):
        path = []
        current = new_node
        while current is not None:
            path.append(current.config)
            current = current.parent
        return path

    


'''
Nodes are connected to form a tree. Each node maintains info about its parent (only ever has max of 1 parent -- acyclic)
and its children (can have many) and the robot configuration at this waypoint.
'''
class Node:
    def __init__(self, config, parent=None):
        self.config = config # The robot configuration (joint angles) at this waypoint
        self.parent = parent
        self.children = []
    
    '''
    Inputs: child_config, the newly sampled robot configuration to be added to the tree.
    
    Within the parent node, the new *child* node is created and added to its children.
    The child node is returned in case a reference is desired.
    '''
    def add_child(self, child_config):
        child_node = Node(child_config, parent=self)
        self.children.append(child_node)
        return child_node
    
    '''
    Starting at the current node (self), iterate through the children in a depth-first manner, returning a list
    of all nodes in the tree (including self) in depth-first order. Also note, the preference/order with which children
    are depth-iterated upon was arbitrarily coded (did not intentionally treat the children in a particular order).
    
    Use case: used on the root node, this can return a list of all nodes in the whole tree in O(n) time. Any operation
    that is desired can then be used on this list (such as computing the distance between a sampled config and each
    node in the tree--this will be an O(n) operation anyway, can't quit early if you need to find the min dist, unless
    there is some other way of storing info somehow...).
    '''
    def depth_first_iterate(self, visited, to_visit_stack):
        visited.append(self)
        
        for child in self.children: # (still works if list is empty due to reaching the end of a depth-first path)
            to_visit_stack.append(child)
            
        # Recursive work: pop node from stack (LIFO--last in first out; depth first) if not empty:
        if len(to_visit_stack) == 0: # Base case
            return visited
        else:
            return to_visit_stack.pop().depth_first_iterate(visited, to_visit_stack)
        
