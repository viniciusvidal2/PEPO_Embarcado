#!/bin/bash

# Dormir um pouco para esperar iniciar
sleep 7s

# Liberar o toin la da usb
echo 12 | sudo -S chmod a+rw /dev/ttyUSB0

# Setar os parametros pra capturar
v4l2-ctl --set-ctrl=exposure_auto=1
v4l2-ctl --set-ctrl=white_balance_temperature_auto=0
v4l2-ctl --set-ctrl=exposure_absolute=8
v4l2-ctl --set-ctrl=brightness=230
v4l2-ctl --set-ctrl=backlight_compensation=0
v4l2-ctl --set-ctrl=saturation=250
v4l2-ctl --set-ctrl=white_balance_temperature=4000

# Importar dependencias de ROS - workspaces
source /opt/ros/melodic/setup.bash
source /home/pepo/pepo_ws/devel/setup.bash
export ROS_IP=192.168.0.101
export ROS_MASTER_URI=http://192.168.0.101:11311

# Chamar servidor rosbridge de inicio
roslaunch rosbridge_server rosbridge_websocket.launch