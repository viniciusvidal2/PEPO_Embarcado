#!/bin/bash

# Ir para a pasta do workpace
cd /home/cap/pepo_ws
source /opt/ros/melodic/setup.bash

# Compilar dependencias menores
catkin_make --pkg cv_camera dynamixel_workbench_controllers dynamixel_workbench_msgs led_control livox_ros_driver gps_uart

# Dormir um pouco para esperar iniciar
sleep 1s

# Compilar pacote de comunicacao
catkin_make -j1 --pkg communication

# Dormir um pouco para esperar iniciar
sleep 1s

# Compilar pacote de comunicacao
catkin_make -j1 --pkg pepo_obj

# Dormir um pouco para esperar iniciar
sleep 1s

# Compilar pacote de comunicacao
catkin_make -j1 --pkg pepo_space

# Dormir um pouco para esperar iniciar
sleep 1s

# Compilar pacote de comunicacao
catkin_make -j1 --pkg fog

# Dormir um pouco para esperar iniciar
sleep 1s

# Pacote do rosbridge e rosauth para ligar no mobile
catkin_make --pkg rosbridge_server rosbridge_msgs rosbridge_library rosbridge_suite rosapi rosauth

# Dormir um pouco para esperar iniciar
sleep 1s

# Pacote do loam livox para horizon
catkin_make -j1 --pkg loam_horizon
