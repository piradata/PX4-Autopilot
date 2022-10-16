#!/bin/bash

if [ $# -eq 0 ]
then
  echo "EEROR!: No arguments supplied!"
else
  if test -f "$HOME/src/bags/$@.bag"
  then
    echo "EEROR!: File $@.bag already exists on '$HOME/src/bags/'!"
  else
    make px4_sitl_default gazebo_iris &
    RUNNING_PX4_GAZEBO_PID=$!
    sleep 6

    roslaunch mavros px4.launch fcu_url:='udp://:14550@127.0.0.1:14555' &
    RUNNING_ROSCORE_PID=$!
    sleep 3

    echo "Recording BAG $@"
    rosbag record -a -o "$HOME/src/bags/" -O "$@" &
    RUNNING_ROSBAG_PID=$!
    rosrun wpg v6_fuzzy_vel_smc_py3.py
    sleep 1
    echo ""
    echo "Killing rosbag process with PID $RUNNING_ROSBAG_PID..."
    kill -sSIGINT $RUNNING_ROSBAG_PID
    echo "Rosbag process killed!"

    sleep 1

    echo ""
    echo "Killing px4_gazebo process with PID $RUNNING_PX4_GAZEBO_PID..."
    kill -sSIGINT $RUNNING_PX4_GAZEBO_PID
    echo "Px4_gazebo process killed!"

    sleep 1

    echo ""
    echo "Killing roscore process with PID $RUNNING_ROSCORE_PID..."
    kill -sSIGINT $RUNNING_ROSCORE_PID
    echo "Roscore process killed!"
  fi
fi

