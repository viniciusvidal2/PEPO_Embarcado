#!/bin/bash

# Ir para a pasta do workpace
cd /home/pepo/pepo_ws

# Compilar dependencias menores
catkin_make --pkg cv_camera dynamixel_workbench_controllers dynamixel_workbench_msgs led_control livox_ros_driver

# Dormir um pouco para esperar iniciar
sleep 3s

# Compilar pacote de comunicacao
catkin_make --pkg communication

# Dormir um pouco para esperar iniciar
sleep 3s

# Compilar pacote de comunicacao
catkin_make --pkg pepo_obj

# Dormir um pouco para esperar iniciar
sleep 3s

# Compilar pacote de comunicacao
catkin_make --pkg pepo_space
