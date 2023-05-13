# mecademic-ros2

**Table of Contents**
- [Powering On/Off the Robots](#1-powering-the-robot-onoff)
- [Installation and Set Up](#2-installation-and-set-up)
- [Required Modifications to the Code](#3-required-modifications-to-the-code)
- [Commands to Prepare System (condensed)](#condensing-terminal-commands)
- [Commands to Run System](#4-commands-to-run-system)
- [System Architecture](#5-general-system-architecture)
- [Walkthroughs]
- [Guide to ROS2]
- [Useful Resources](#useful-resources)

## 1) Powering the Robot On/Off
### Powering on:
Flip the power switch, pull up on the E-Stop, and press the reset button:

https://github.com/myersjm/mecademic-ros2/assets/31910744/efcb0afe-1605-4d31-8d0e-35b5ce8c2f86

The robot should blink orange and red for about 30 seconds, then switch to blinking green. When it blinks green, the robot is ready to be connected to.

https://github.com/myersjm/mecademic-ros2/assets/31910744/80cb1eb8-4244-4a23-b004-1167c2a42a09

### Powering off:
To power off the robot, push the E-Stop button down and flip the power switch.

https://github.com/myersjm/mecademic-ros2/assets/31910744/94dc243e-3cfc-4e0b-84a6-8a709aba071e

## 2) Installation and Set Up
### Installing ROS2 Humble
Follow the instructions at [this link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for installing ROS2 Humble for Ubuntu 22.04. Note that we chose to install the desktop version (`sudo apt install ros-humble-desktop`). Also, [here is a helpful YouTube crash course video on ROS2 that we watched and learned a lot from](https://youtu.be/Gg25GfA456o).

### Creating a Virtual Environment (Required)
This is a one-time setup for making a virtual environment for running the ROS2 system. An early issue we ran into is importing and using 3rd-party libraries with ROS2. After exploring our options (see [this link](https://docs.ros.org/en/foxy/How-To-Guides/Using-Python-Packages.html)), we decided we didn't want to install packages directly onto the computer (wanted to maintain a self-contained environment for reproducibility), and rosdep and package.xml did not work with the mecademicpy library. We opted for using a virtual environment as a last option (note that we did try conda with ROS2, but found it to have issues).

Follow these instructions to set up the virtual environment and prepare the terminal for running code (Ubuntu 22.04):
1) In your terminal, navigate into `meca_ws`, the ROS2 workspace.
2) Make sure you have virtualenv installed (`pip install virtualenv`) *Note that we installed it within a conda environment for the sole purpose of creating the virtual environment in the next step; this was so we wouldn't have to do that on the desktop itself--but it does not matter where you install it.*
3) `virtualenv -p python3.10 ./mecavenv` **This creates the virtual environment, giving it the name mecavenv.** *Note that we ran into issues with ROS2 Humble having a different Python version (3.10) on the desktop than the virtual environment (defaulted to 3.11). We got errors about this, and it was a two-fold issue. First, the PYTHONPATH should have had paths to both the virtual environment's packages and the ROS distro's packages; second, there was the python version mismatch. We solved this by setting the python version for the virtual environment to be 3.10 as in the given command, and we found a solution at [this link](https://answers.ros.org/question/371083/how-to-use-python-virtual-environments-with-ros2/?answer=371551#post-id-371551) for altering the python path, which will be shown in a future step.*
4) If you used a conda env in step 3 to create the virtualenv, go ahead and `conda deactivate`. *Make sure there are no environments active (these are shown in parentheses to the left of the shell prompt)*
5) `source ./mecavenv/bin/activate` **Activate the virtual environment (should see `(mecavenv)` to the left of the shell prompt).**
6) `touch ./mecavenv/COLCON_IGNORE` *Ensures colcon doesn't try to build the venv (according to [this link](https://docs.ros.org/en/foxy/How-To-Guides/Using-Python-Packages.html)).*
7) `python3 -m pip install numpy scipy sympy matplotlib notebook ipywidgets pin meshcat mecademicpy catkin_pkg empy lark` **Install 3rd-party libraries in the virtual environment.** *Note, catkin_pkg, empy, and lark have been added because if you want to use cmake for creating custom services or messages, you need catkin_pkg, and for colcon-building the generated interfaces, you need empy and lark.* *Another important note: if you have used a conda environment before with pinocchio, you may have conda installed pinocchio--but with pip, it has a different install name, `python3 -m pip install pin`. This has been included in the command given.**
8) `source /opt/ros/humble/setup.bash`
9) `export PYTHONPATH=$PYTHONPATH:/home/jessicamyers/Jupyter/mecademic-ros2/meca_ws/mecavenv/lib/python3.10/site-packages` **NOTE: This must be updated for your personal full path to the virtual environment's Python site-packages folder (which should be the same after `mecademic-ros2`). Also, change the Python version if you are not using version 3.10 for whatever reason.** *This export command resolves the error mentioned in step 3, from [this link](https://answers.ros.org/question/371083/how-to-use-python-virtual-environments-with-ros2/?answer=371551#post-id-371551). You can run `echo $PYTHONPATH` to double check the paths.*
10) `export PYTHONPATH=$PYTHONPATH:/home/jessicamyers/Jupyter/meca-dev/meca_ws/mecavenv/lib/python3.10/site-packages/cmeel.prefix/lib/python3.10/site-packages` **This additional Python path export is required for getting pinocchio to be recognized. You will have to modify the beginning of this path to be specific to your file location.)**
11) Make sure you are in meca_ws, and then run `colcon build` which generates the build, install, and log folders. This should be run any time you modify the code.
12) `source install/setup.bash` This, too, will be run after you edit the code, since the previous step generates the install folder.
13) Run whatever you want to run now. We ran `ros2 run meca_controller meca_driver --ros-args -r __ns:=/robot1` (See commands below for updated commands).

### Condensing Terminal Commands
**Before you ros2 run any code**, you need to navigate to the `meca_ws` ROS2 workspace and run the terminal commands in steps 5-6, 8-10, followed by commands that should be run after you edit the code. This takes too long. I made a shell script called `meca_setup.sh` in the `meca_ws` directory that contains these commands--all you have to do is edit them, as you did in the section above, to fit your personal paths. After doing this, the only commands you ever will have to run are:

1) Make sure you are in the `meca_ws` directory
2) `source meca_setup.sh` **in every terminal you open**
3) `colcon build` **you only have to run this in one of the terminals you are using**
4) `source install/setup.bash` **in every terminal you open**

in that order. Note that after you edit the code, you will have to repeat steps 3 and 4.

### Colcon Build Errors
If you receive an error in colcon build that seems to be unrelated to your code, it could be an incorrect version of some package. I have had this error multiple times so I am documenting it here:
```
(mecavenv) jessicamyers@MightyMeca:~/Jupyter/mecademic-ros2/meca_ws$ colcon build
Starting >>> meca_controller
Starting >>> meca_motion_planner
--- stderr: meca_controller                                   
/home/jessicamyers/Jupyter/mecademic-ros2/meca_ws/mecavenv/lib/python3.10/site-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
  warnings.warn(
---
Finished <<< meca_controller [0.53s]
--- stderr: meca_motion_planner
/home/jessicamyers/Jupyter/mecademic-ros2/meca_ws/mecavenv/lib/python3.10/site-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
  warnings.warn(
---
Finished <<< meca_motion_planner [0.53s]

Summary: 2 packages finished [0.61s]
  2 packages had stderr output: meca_controller meca_motion_planner
```

To fix this, check what version of setuptools you have:
```
(mecavenv) jessicamyers@MightyMeca:~/Jupyter/mecademic-ros2/meca_ws$ python
Python 3.10.6 (main, Mar 10 2023, 10:55:28) [GCC 11.3.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import setuptools
>>> print(setuptools.__version__)
67.4.0
```

Per [this link](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/), you should downgrade setuptools using `pip install setuptools==58.2.0`. Now the build succeeds (though the error did not interfere with anything before, it was just a deprecation warning.)
```
(mecavenv) jessicamyers@MightyMeca:~/Jupyter/mecademic-ros2/meca_ws$ colcon build
Starting >>> meca_controller
Starting >>> meca_motion_planner
Finished <<< meca_controller [0.53s]                          
Finished <<< meca_motion_planner [0.53s]

Summary: 2 packages finished [0.61s]
```

## 3) Required Modifications to the Code
There are some edits that must be made to the code to enable the system to work for your robots.
1) In the **meca_settings.py** file (`meca_ws/src/meca_controller/meca_controller/meca_settings.py`), change the IP addresses for your robots. You can update the namespaces if you want, but that is not required. We chose `/robot1` to refer to the robot on the left when facing in front of the station, and `/robot2` to refer to the robot on the right. The resting state configuration does not have to be changed, as it most likely will never be used--it specifies the assumed resting position of the other robot if it is turned off and not publishing joint angle data when motion planning for the robot that is on. These all are made constants to try to eliminate hardcoding throughout the system. 
![image](https://github.com/myersjm/mecademic-ros2/assets/31910744/fc55b746-e315-49f0-8620-57709e684715)

2) In **robot_model.py** (`meca_ws/src/meca_motion_planner/meca_motion_planner/robot_model.py`), you will likely have to edit the robot extrinsic calibration (distance between the robot arms). This is for the purpose of motion planning, collision detection, and visualization. This can be done by editing the `se3_transform_robots` in the constructor, which is composed of a rotation and translation (in our case, the world axis is defined as the halfway point in between the two robots--this is something you likely also will have to change, in step 3 below--basically in this pinocchio model, the robot's URDF is loaded twice, and the models are combined using appendModel. To make them not on top of each other, you must transform the second robot to be on the right side, facing the other, by rotating about this world axis.):
![image](https://github.com/myersjm/mecademic-ros2/assets/31910744/a593651b-a636-4599-8c1d-64e5e8214ad5)

    Note that you will also need to edit the `se3_transform_env` for changing the placement of the environment with respect to the two-arm robot setup. This is       here because the environment URDF is loaded as its own pinocchio model, and it has to be merged with the two-arm model that was created. You will have to         edit the environment URDF too, seen in step 4 below.

3) In **meca.URDF** (`meca_ws/src/resources/meca/meca.urdf`), you can edit the world frame that is defined between the two robots:
![image](https://github.com/myersjm/mecademic-ros2/assets/31910744/5f0c9817-965b-4d09-9394-47d927519763)
      You may also need to edit the gripper and gripper finger links/joints in this URDF file if you are using different gripper fingers or a different gripper.

4) In **environment.URDF** (`meca_ws/src/resources/environment/environment.urdf`), you can edit the different obstacles in your robot's environment.

5) In the **meca_control.py** run() command, you can add your own code making use of the functions in meca_control.py to control the robots.
6) [Optional] In **meca_driver.py** (`meca_ws/src/meca_controller/meca_controller/meca_driver.py`), you may want to edit the rate at which data is obtained from the robot and published to topics:
![image](https://github.com/myersjm/mecademic-ros2/assets/31910744/f8f8d4a5-48e0-460e-bfc1-64fb60931fa3)

There could be other changes required, but these are all I can think of right now.

## 4) Commands to Run System
After running the terminal commands to prepare the system (See [Commands to Prepare System (condensed)](#condensing-terminal-commands) in the previous section), you can run the system using the following commands, each in a new terminal *(NOTE that if you are using different namespaces than `/robot1` and `/robot2`, you will have to modify commands 1 and 2)*:

1) `ros2 run meca_controller meca_driver --ros-args -r __ns:=/robot1` **launches robot1, establishing connection, services, and data topics**
2) `ros2 run meca_controller meca_driver --ros-args -r __ns:=/robot2` **launches robot2, establishing connection, services, and data topics**
3) `ros2 run meca_motion_planner motion_planner` **launches the motion planner, which opens a visualization window and establishes services**
4) `ros2 run meca_controller meca_control` **currently, this is where you will write code to interact with the robots**

## 5) General System Architecture
This is a rough depiction of the current system architecture.
![image](https://github.com/myersjm/mecademic-ros2/assets/31910744/897498ac-c959-4a29-9530-03a2717b2faf)

## 6) Important Things to Note About the Code
1) Joint angles sent to the **physical robot** should be in **degrees**.
2) Joint angles sent to the **visualization and motion planner** should be in **radians**. This is the format Pinocchio collision models require. 

## Useful resources

* https://github.com/Mecademic
* https://github.com/Mecademic/mecademicpy
* https://cdn.mecademic.com/uploads/docs/meca500-r3-programming-manual-8-4.pdf
* https://cdn.mecademic.com/uploads/docs/meca500-r3-user-manual-8-4.pdf
* https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/index.html
* https://github.com/rdeits/meshcat-python
* https://github.com/stack-of-tasks/pinocchio
* https://github.com/stack-of-tasks/pinocchio/blob/master/doc/pinocchio_cheat_sheet.pdf
* https://github.com/stack-of-tasks/pinocchio/tree/master/bindings/python
* https://github.com/stack-of-tasks/pinocchio/tree/master/bindings/python/pinocchio/visualize
* https://www.mathworks.com/help/robotics/?s_tid=srchbrcm
* https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html
