#!/bin/bash

# To run (in terminal), type: source meca_setup.sh

# Only thing left after this is to (after editing code):
# 1) colcon build
# 2) source install/setup.bash

source ./mecavenv/bin/activate

touch ./mecavenv/COLCON_IGNORE

source /opt/ros/humble/setup.bash

############ EDIT THESE AS NECESSARY: ################
# Full path to virtual env site-packages folder:
export PYTHONPATH=$PYTHONPATH:/home/jessicamyers/Jupyter/meca-dev/meca_ws/mecavenv/lib/python3.10/site-packages

# Path to cmeel.prefix site-packages folder (for pinocchio):
export PYTHONPATH=$PYTHONPATH:/home/jessicamyers/Jupyter/meca-dev/meca_ws/mecavenv/lib/python3.10/site-packages/cmeel.prefix/lib/python3.10/site-packages
