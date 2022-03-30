#!/bin/bash

# make px4_sitl gazebo_typhoon_h480
make px4_sitl gazebo_iris

# roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
./connect.sh &> /dev/null &
sleep 2

# rosrun wpg v6_fuzzy_vel_smc_py3.py

python3 ~/src/Firmware/Tools/sitl_gazebo/missoes/cria_rota.py
python3 ~/src/Firmware/Tools/sitl_gazebo/missoes/executa_missao_EKF.py
python3 ~/src/Firmware/Tools/sitl_gazebo/databank/YOLO/opencv_mavros.py
