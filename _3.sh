#!/bin/bash

if [ $# -eq 0 ]
then
  echo "EEROR!: No arguments supplied!"
else
  if test -f "$@.bag"
  then
    echo "EEROR!: File $@.bag already exists!"
  else
    echo "Recording BAG $@"
    rosbag record -a -O "$@" &
    RUNNING_ROSBAG_PID=$!
    rosrun wpg v6_fuzzy_vel_smc_py3.py
    sleep 1
    echo ""
    echo "Killing rosbag process with PID $RUNNING_ROSBAG_PID..."
    kill -sSIGINT $RUNNING_ROSBAG_PID
    echo "Rosbag process killed!"
  fi
fi

