#!/bin/bash

make px4_sitl_default gazebo_iris PX4_SITL_WORLD:=$(pwd)/Tools/sitl_gazebo/worlds/not_so_empty.world

# sudo make px4_sitl_default gazebo_iris PX4_SITL_WORLD:=$(pwd)/Tools/sitl_gazebo/worlds/uneven.world

# sudo make px4_sitl_default gazebo_iris PX4_SITL_WORLD:=$(pwd)/Tools/sitl_gazebo/worlds/iris.world

# sudo make px4_sitl_default gazebo_iris PX4_SITL_WORLD:=$(pwd)/Tools/sitl_gazebo/worlds/baylands.world
