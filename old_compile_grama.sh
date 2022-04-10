#!/bin/bash

# sudo make no_sim=1 px4_sitl_default gazebo &

# source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

# roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/navio.world

make px4_sitl_default gazebo_iris PX4_SITL_WORLD:=$(pwd)/Tools/sitl_gazebo/worlds/gramado.world
