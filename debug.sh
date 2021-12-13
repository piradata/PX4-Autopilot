#!/bin/bash
./connect.sh &> /dev/null &
sleep 2
./gclient.sh &> /dev/null &
rosbag record -a &> /dev/null &
./run.sh
