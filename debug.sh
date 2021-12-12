#!/bin/bash
./connect.sh &
sleep 1
./gclient.sh &
./run.sh
