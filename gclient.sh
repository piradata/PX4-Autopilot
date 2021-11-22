#!/bin/bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
GAZEBO_MASTER_URI=http://127.0.0.1:11345 gzclient
