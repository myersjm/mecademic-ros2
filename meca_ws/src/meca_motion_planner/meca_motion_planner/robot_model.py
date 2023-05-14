'''
############################################################
############################################################
Purpose: This file acts as a wrapper for storing and updating the models/data
         required for collision detection and visualization. It is essentially
         a "robot model" that contains info for collision detection, and
         visualization.

Date created: 11/27/2022
Developers: Jessica Myers, Tim Bretl, Erika Jarosch, [add name here]
University of Illinois Urbana-Champaign

############################################################
############################################################
'''
import time
import numpy as np
import pinocchio as pin
import os
from pinocchio.visualize import MeshcatVisualizer
from pinocchio.utils import *

class RobotModel:
    '''
    Inputs: -urdf_model_path: (Default resources/meca/meca.urdf)
            -mesh_dir: directory containing meshes (Default resources/meca/)
            -include_environment: whether to include he environment/obstacles in collision detection and visualization.
            -namespace1 and namespace2: the namespaces of the two robots.
            -se3_transforms: used for merging pinocchio models (see GitHub readme for more info.) (is in meters)
    
    Working class variables: self.{model, collision_model, visual_model,
                                   data, collision_data, visual_data,
                                   vis}

   KNOW THAT: - joint angles are specified in radians
              - when giving a full 12-len (2-robot) configuration, pass it in as a flattened np array with robot1 (left robot) first.
                - the internal Pinocchio ordering of the robots is the opposite, but you do not need to worry about that as the user.
              - Starting the visualization will reset the current_q to the neutral configuration
    '''
    def __init__(self,
                 urdf_model_path = os.path.join('resources', 'meca', 'meca.urdf'),
                 mesh_dir = os.path.join('resources', 'meca'),
                 urdf_model_path_env = os.path.join('resources', os.path.join('environment', 'environment.urdf')),
                 mesh_dir_env = os.path.join('resources','environment'),
                 include_environment=True, namespace1 = 'robot1', namespace2 = 'robot2',
                 se3_transform_robots = pin.SE3(np.array([[np.cos(np.pi), -np.sin(np.pi), 0.],
                                                          [np.sin(np.pi), np.cos(np.pi), 0.],
                                                          [0., 0., 1.]]), np.array([0,0,0])),
                 se3_transform_env = pin.SE3(pin.SE3.Identity().rotation, np.array([-.0375,0,0]))):
        
        self.namespace1 = namespace1
        self.namespace2 = namespace2

        # 1) Create models for each robot, and the environment:
        model1, collision_model1, visual_model1 = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)
        model2, collision_model2, visual_model2 = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)
        model_env, collision_model_env, visual_model_env = pin.buildModelsFromUrdf(urdf_model_path_env, mesh_dir_env)
        
        # 2) Add namespace prefix to each robot's models and merge models:
        self.model, self.collision_model, self.visual_model = self.merge_models(model1, collision_model1, visual_model1,
                                                                                model2, collision_model2, visual_model2,
                                                                                se3_transform_robots, True, namespace1, namespace2)
                
        # 3) Add environment / obstacles (Merge environment urdf with robot model):
        if include_environment:
            self.model, self.collision_model, self.visual_model = self.merge_models(self.model, self.collision_model,
                                                                                    self.visual_model, model_env,
                                                                                    collision_model_env, visual_model_env,
                                                                                    se3_transform_env, False)
        # 4) Set up (Add/Remove) collision pairs:
        self.setup_collision_pairs()
        
        # 5) Load data (This **must** be done **after** you finish adding/removing collision pairs.) 
        #      update--yes, must be done after, but can do it before in addition and get no error.
        self.data, self.collision_data, self.visual_data = pin.createDatas(self.model, self.collision_model, self.visual_model)
        
        # 6) Init visualization variable to None so know if visualization has been started or not:
        self.vis = None
        
        # 7) Init current config (internally, is len-12 array of joint angles in radians, with right robot (robot2 )specified first):
        self._current_q = pin.neutral(self.model) # private variable to avoid confusion to the user; the user should not see this!
        
    '''
    Purpose: Keep track of the two-arm robot configuration so that if only one robot is specified at any time for a function
             (like collision checking), there is no need to remember or specify the unchanged/unmoving robot.

    Input: - q: a np.array of size 12 in radians to specify the full configuration of the entire 2-robot setup, with order: [robot1, robot2].
                - can also specify as len 6 array if give the robot namespace - mainly for internal uses..
           - visualize: whether to update the visualization (given it has been started); used internally to ensure
                                   visualization can be not used if necessary.

    NOTE: Stores the _current_q in the order [robot2, robot1] (reversed!) because due to the way Pinocchio merges models, the right
          robot arm will always be specified first in the configuration. Changing this would most likely introduce more hidden bugs
          than would simply changing how I store the configuration in this class.
    '''
    def set_current_q(self, q, robot_namespace=None, visualize=True):
        if robot_namespace is None and len(q) != 12:
            print("ERROR: q must be length 12 to specify both robots' configuration if no robot namespace is given.")
            return
        
        # Form the current q in the internal format, and set it:
        self._current_q = self._form_current_q(q, robot_namespace)

        # Convenience call in case the user forgets to update:
        if (self.vis is not None) and visualize:
            self.update_visualization()

    '''
    Helper for set_current_q which does everything except actually SET the current q. See set_current_q for info on inputs and
    functionality.

    Returns to-be _current_q, not set yet.
    '''
    def _form_current_q(self, q, robot_namespace=None):
        if robot_namespace is None and len(q) != 12:
            print("ERROR: q must be length 12 to specify both robots' configuration if no robot namespace is given.")
            return None
        elif robot_namespace is None:
            return np.concatenate([q[6:], q[:6]]) # reorder [robot1, robot2] -> [robot2, robot1] (internal Pinocchio representation)
        else:
            # 1) Determine internal ordering of robots in len-12 config with respect to namespace:
            first_ns_internally = self.get_namespace_of_first_robot_in_q() # should always be right robot (robot2)

            # 2) Fill in complete configuration of 2 robots with unused robot's current configuration:
            if robot_namespace == first_ns_internally:
                return np.concatenate([q, self._current_q[6:]])
            else:
                return np.concatenate([self._current_q[:6], q])

    '''
    Returns the user-side ordering of 12-len robot configuration (robot1, then robot2)
    '''
    def get_current_q(self):
        return np.concatenate([self._current_q[6:], self._current_q[:6]])
    
    '''
    Used for determining which robot's len 6 config comes first in the len 12 array, internally to pinocchio.
    (Note that this will always be the robot on the right, due to the way Pinocchio merges models, regardless
    of what namespace you give it -- so if you are following the robot1 (left), robot2 (right) convention, this
    function will always return robot2.)
    '''
    def get_namespace_of_first_robot_in_q(self):
        if self.namespace1 in self.model.names[1]:
            return self.namespace1
        elif self.namespace2 in self.model.names[1]:
            return self.namespace2
        else:
            raise Exception('ERROR: get_namespace_of_first_robot_in_q: neither namespace found in model; are you using namespaces correctly?')
        
    def add_namespace_prefix_to_models(self, model, collision_model, visual_model, namespace):
        # Rename geometry objects in collision model:
        for geom in collision_model.geometryObjects:
            geom.name = f'{namespace}/{geom.name}'

        # Rename geometry objects in visual model:
        for geom in visual_model.geometryObjects:
            geom.name = f'{namespace}/{geom.name}'

        # Rename frames in model:
        for f in model.frames:
            f.name = f'{namespace}/{f.name}'

        # Rename joints in model:
        for k in range(len(model.names)):
            model.names[k] = f'{namespace}/{model.names[k]}'

    '''
    Merge models, either for the purpose of having a two robots in a single model for collision detection and visualization
    purposes, or for adding the environment to the model.

    Inputs: 
    - model, collision model, and visual model for each robot (or pinocchio model) you want to merge;
    - se3_transform: pin.SE3 specifying the transform from the first model's frame to the second model.
        - i.e. in the case of merging urdfs of same robot arm to have 2 arms, rotate the 2nd arm 180 degrees.
    - merging_duplicate_urdfs: Boolean; True if merging models which both were created from the same urdf.
        - i.e. You are merging robot1 and robot2's models. used for error checking, to ensure namespaces are passed.
    - namespace (string, name for robot, used to prefix joint names etc.) for each robot;
    
    Returns: merged model, collision_model, visual_model, which contain both robots.

    Because the models were both made from the same urdf file, parts must be renamed (or prefaced with a prefix/namespace)
    in order to avoid errors. Also note that in C++ there is an appendGeometryModel function--this doesn't exist in the Python
    pinocchio for some reason, or else I would have appended the models, then appended the geometry models for collision and
    visual. appendModel does allow giving a pair of geometry models, but not 2 pairs (visual and collision) which is odd.
    To avoid appending models by hand, I do make use of the version of appendModel that takes model and geometry model pairs,
    but I call it twice and discard one of the combined models since I already did that.

    '''
    def merge_models(self, model1, collision_model1, visual_model1,
                     model2, collision_model2, visual_model2,
                     se3_transform, merging_duplicate_urdfs,
                     namespace1=None, namespace2=None):
        if merging_duplicate_urdfs: # i.e. making a model with 2 robot arms, given single urdf of one arm:
            if (namespace1 is None) or (namespace2 is None):
                print('ERROR: Missing namespace1 or namespace 2 arguments; When merging models which each use the same urdf, you need to preface model items with a namespace.')
                return
            self.add_namespace_prefix_to_models(model1, collision_model1, visual_model1, namespace1)
            self.add_namespace_prefix_to_models(model2, collision_model2, visual_model2, namespace2)

        # If anyone knows a simpler way, feel free to change this:
        model, visual_model = pin.appendModel(model1, model2, visual_model1, visual_model2, 0, se3_transform)
        model, collision_model = pin.appendModel(model1, model2, collision_model1, collision_model2, 0, se3_transform)

        return model, collision_model, visual_model
    
    '''
    Automated adding and removal of valid collision pairs.

    Assume there are no collisions in neutral (default) configuration. Add all collision pairs. go to neutral config.
    Remove all pairs where there is a collision at neutral config. NOTE: this is assuming you placed obstacles at correct
    locations (physically plausible--not intersecting the robot at neutral config. Clear of the robot. Otherwise, it will
    just be interpreted as like a table and it is attached and okay to be in collision. In that case, the pair will be 
    removed, but still will have the collision pair for , say, the end effector hitting table. just not the base.)

    NOTE: datas must be recreated after the collision pairs have been modified. I am creating datas before so that I can
    remove incorrect collisions.

    NOTE: Must have added the obstacles/env before calling this function.

    Redo collision pairs anytime after a geometry object / obstacle is added. 

    '''
    def setup_collision_pairs(self):
        # 1) Add "collision pairs" - this **must** be done before loading data, otherwise you get a kernel error. [TODO double check]
        self.collision_model.addAllCollisionPairs()

        # 2) Compute collisions at a neutral environment:
        self.data, self.collision_data, self.visual_data = pin.createDatas(self.model, self.collision_model, self.visual_model)
        q = pin.neutral(self.model) # [0,0,0,0,0,0] neutral config
        is_collision = pin.computeCollisions(
            self.model,
            self.data,
            self.collision_model,
            self.collision_data,
            q,
            False,
        )    

        # 3) Find all in pairs in collision at neutral config--these will be removed, as they correspond to attachments of
        #    the robot to itself or to its environment.
        collision_ixs_to_remove = []
        for k in range(len(self.collision_model.collisionPairs)):
            cr = self.collision_data.collisionResults[k]
            cp = self.collision_model.collisionPairs[k]
            if cr.isCollision(): # in collision
                collision_ixs_to_remove.append(k)

    #         print(
    #             f'collision pair {k:3d}:',
    #             cp.first,
    #             ',',
    #             cp.second,
    #             '- collision:',
    #             'Yes' if cr.isCollision() else 'No',
    #         )
    #     print(collision_ixs_to_remove)

        # Remove "useless" collision pairs (i.e., those pairs of rigid bodies that are always in collision regardless
        # of the configuration). It is **very important** to remove pairs from last to first
        # (i.e., from high to low index) because higher indices will change as you remove lower ones.
        for k in sorted(collision_ixs_to_remove, reverse=True):
            self.collision_model.removeCollisionPair(self.collision_model.collisionPairs[k])

    #     print('Number of collision pairs:', len(collision_model.collisionPairs))
    
    '''
    -display_collisions: (default False), whether to display the collision mesh

    NOTE: this resets the configuration to defaults, so if you must specify
    '''
    def start_visualizer(self, display_collisions=False):
        # Create the visualizer
        self.vis = MeshcatVisualizer(
            self.model,
            self.collision_model,
            self.visual_model
        )

        # Open the visualizer in a new browser window
        self.vis.initViewer(open=True)

        # Add robot to the visualizer
        self.vis.loadViewerModel()

        # Do not display the collision mesh
        self.vis.displayCollisions(display_collisions)

        # Do display the visual mesh (just for appearance)
        self.vis.displayVisuals(True)
        
        # Set robot to default position:
        self.reset_to_default_position()
        
    '''
    Purpose: Used to update the visualization AND sets the current configuration (_current_q) at the same time.

    Inputs: -q, a set of joint angles in radians, either:
                - a np.array of length 6 (the # of joints)
                - a np.array of length 12 (both robots) with order [robot1 (left robot), robot2 (right robot)]
                - None (update the visualization to reflect the internally stored _current_q - internal purposes only)
                **NOTE: the angles are in radians, contrary to the meca's robot.MoveJoints command, which is in degrees.** 
            -namespace: the namespace of the robot for which you are specifying the updated q. 
            -compute_collision: default True, whether to compute collisions

    Returns: -in_collision: boolean, whether a collision has occurred. False by default if no collisions were computed.

    NOTE: this is now written in terms of TWO robots--as long as you give the namespace for the robot whose q you are specifying,
          you can continue to call update_visualization with a len 6 single robot config, and the missing robot's info will automatically
          be filled in. However, you can also provide no namespace and the len 12 q specifying both robots. To know the order of which
          robot should be specified first, call self.get_namespace_of_first_robot_in_q().
    '''
    def update_visualization(self, q=None, namespace=None, compute_collisions=True):
        if self.vis is None:
            print(f'ERROR: Visualization has not been started yet. First start it using the start_visualizer function.')
            return
                
        if q is not None: # Set the current configuration; if q is None then don't need to because it is already set.
            self.set_current_q(q, namespace, visualize=False)

        # 4) Do forward kinematics at len 12 q
        pin.forwardKinematics(self.model, self.data, self._current_q)

        # 5) Update the pose of all rigid bodies (both for collision and display)
        pin.updateGeometryPlacements(self.model, self.data, self.collision_model, self.collision_data)
        pin.updateGeometryPlacements(self.model, self.data, self.visual_model, self.visual_data)

        # 6) Update the visualizer
        self.vis.display(self._current_q)

        # 7) Check for collisions:
        if compute_collisions:
            # Compute all collisions
            in_collision = pin.computeCollisions(self.model, self.data, self.collision_model, self.collision_data, self._current_q, False)
            return in_collision
        print('Collisions were not computed')
        return False # not in collision, by default.
    
    '''
    Purpose: Checks to see if a given robot configuration q (a np.array of joint angles in radians with either:
                    - length 6 and namespace provided specifying which robot
                    - length 12 and order [robot1 robot2] )
             will result in collision. Does not use visualization.
    
    Note: this is purely for hypothetically checking for a collision. If the q for both robots is not specified, the stored _current_q
    config will be used to check for the unspecified robot's config. Note that checking for a collision does not update the current_q
    with a new q for the specified robot.
    '''
    def check_for_collision(self, q, namespace=None):
        if namespace is None and len(q) != 12:
            print("ERROR: q must be length 12 to specify both robots' configuration if no robot namespace is given.")
            return
        
        # Form the current q in the internal format, but do not set it:
        two_arm_q = self._form_current_q(q, namespace)
        
        # 3) Do forward kinematics at q:
        pin.forwardKinematics(self.model, self.data, two_arm_q)

        # 4) Update the pose of all rigid bodies (just for collision--no need to use visuals for pure collision detection):
        pin.updateGeometryPlacements(self.model, self.data, self.collision_model, self.collision_data)
#         pin.updateGeometryPlacements(self.model, self.data, self.visual_model, self.visual_data)

        # 5) Check for collisions:
        in_collision = pin.computeCollisions(self.model, self.data, self.collision_model, self.collision_data, two_arm_q, False)
        return in_collision

    '''
    Sets the robot in the visualization to a neutral configuration. 
    For the meca robot, this is the configuration with all zero joint angles. (q = [0. 0. 0. 0. 0. 0.]), length 12 for two arms.
    '''
    def reset_to_default_position(self):
        # Get the set of joint angles that define the neutral configuration
        self._current_q = pin.neutral(self.model)
        
        # Update visualization:
        self.update_visualization()
        
    '''
    Convenience function, since pin.randomConfiguration(robot.model) now will return length 12 array, and sometimes you just want
    a length 6, like in motion planning for a single robot.
    '''
    def get_random_config_single_robot(self):
        return pin.randomConfiguration(self.model)[:6]
        
    '''
    Function to check for / print out collision pairs (not done / updated yet)
    '''
#     def collision_check(self):
# #         in_collision = pin.computeCollisions(self.model, self.data, self.collision_model, self.collision_data, q, False)

#     #     # Was there a collision?
#     #     print(f'The robot is in collision: {in_collision}')

#     #     # If so, which pairs were in collision?
#     #     if in_collision:
#     #         for k in range(len(collision_model.collisionPairs)):
#     #             cr = collision_data.collisionResults[k]
#     #             cp = collision_model.collisionPairs[k]
#     #             print(
#     #                 f'collision pair {k:3d}:',
#     #                 cp.first,
#     #                 ',',
#     #                 cp.second,
#     #                 '- collision:',
#     #                 'Yes' if cr.isCollision() else 'No',
#     #             )
#         pass