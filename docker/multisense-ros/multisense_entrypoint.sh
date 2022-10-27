#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

# Setup the ROS environment.
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Setup the Multisense workspace.
#cd /home/robotlab/dev/multisense_ws/src && catkin_init_workspace .
cd /home/robotlab/dev/multisense_ws && catkin_make
source "/home/robotlab/dev/multisense_ws/devel/setup.bash"

# Takes any command line arguments passed to ros_entrypoint.sh and execs them as a command.
# The intention is basically "Do everything in this .sh script, then in the same shell
# run the command the user passes in on the command line".
# Taken from https://stackoverflow.com/a/39082923/1070333
exec "$@"
