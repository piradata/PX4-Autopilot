#!/bin/bash
./connect.sh &> /dev/null &
sleep 2
./gclient.sh &> /dev/null &


# ./connect.sh
# ./gclient.sh
# rosbag record -a -O PID_test
# ./run.sh
