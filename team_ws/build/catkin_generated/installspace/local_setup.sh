#!/usr/bin/env sh
# generated from catkin/cmake/template/local_setup.sh.in

# since this file is sourced either use the provided _CATKIN_SETUP_DIR
# or fall back to the destination set at configure time
<<<<<<< HEAD
: ${_CATKIN_SETUP_DIR:=/home/leonardonavacastellanos/Documents/Tec/OctavoSemestre/Reto/TE3003B_Integration_of_Robotics_and_Intelligent_Systems/team_ws/install}
=======
: ${_CATKIN_SETUP_DIR:=/home/raul/TE3003B_Integration_of_Robotics_and_Intelligent_Systems/team_ws/install}
>>>>>>> c8dd5f9352ebabc349fb7a5d17739ee853746a5e
CATKIN_SETUP_UTIL_ARGS="--extend --local"
. "$_CATKIN_SETUP_DIR/setup.sh"
unset CATKIN_SETUP_UTIL_ARGS
