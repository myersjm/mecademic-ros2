# mecademic-ros2

**Table of Contents**
- [Powering On/Off the Robots](#1-powering-the-robot-onoff)
- [Installation and Set Up](#2-installation-and-set-up)
- [Commands to Prepare System (condensed)](#condensing-terminal-commands)
- [Commands to Run System]
- [Guide to ROS2]

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
9) `export PYTHONPATH=$PYTHONPATH:/home/jessicamyers/Jupyter/mecademic-ros2/meca_ws/mecavenv/lib/python3.10/site-packages` **NOTE: This must be updated for your personal full path to the virtual environment's Python site-packages folder (which should be the same after `mecademic-ros2`). Also, change the Python version if you are not using version 3.10 for whatever reason. This export command resolves the error mentioned in step 3, from [this link](https://answers.ros.org/question/371083/how-to-use-python-virtual-environments-with-ros2/?answer=371551#post-id-371551). You can run `echo $PYTHONPATH` to double check the paths.**
10) `export PYTHONPATH=$PYTHONPATH:/home/jessicamyers/Jupyter/meca-dev/meca_ws/mecavenv/lib/python3.10/site-packages/cmeel.prefix/lib/python3.10/site-packages` **This additional Python path export is required for getting pinocchio to be recognized. You will have to modify the beginning of this path to be specific to your file location.) **
11) Make sure you are in meca_ws, and then run `colcon build` which generates the build, install, and log folders. This should be run any time you modify the code.
12) `source install/setup.bash` This, too, will be run after you edit the code, since the previous step generates the install folder.
13) Run whatever you want to run now. We ran `ros2 run meca_controller meca_driver --ros-args -r __ns:=/robot1` (See commands below for updated commands).

### Condensing Terminal Commands
**Before you ros2 run any code**, you need to navigate to the `meca_ws` ROS2 workspace and run the terminal commands in steps 5-6, 8-10, followed by commands that should be run after you edit the code. This takes too long. I made a shell script called `meca_setup.sh` in the `meca_ws` directory that contains these commands--all you have to do is edit them, as you did in the section above, to fit your personal paths. After doing this, the only commands you ever will have to run are:

1) Make sure you are in the `meca_ws` directory
2) `source meca_setup.sh` **in every terminal you open**
3) `colcon build` **you only have to run this in one of the terminals you are using**
4) `source install/setup.bash` **in every terminal you open**

in that order.

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
