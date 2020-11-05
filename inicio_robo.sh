#!/bin/bash

# Dormir um pouco para esperar iniciar
sleep 5s

# Mudar a pasta para a que contem o driver compilado
cd /home/cap/rtl8188fu

# Chamar o modprobe
echo 12 | sudo -S modprobe cfg80211
# Chamar o insmod
echo 12 | sudo -S insmod rtl8188fu.ko

# Liberar o toin la da usb
echo 12 | sudo -S chmod a+rw /dev/ttyUSB0

# Setar os parametros pra camera em ambiente aberto
v4l2-ctl --set-ctrl=exposure_auto=1
v4l2-ctl --set-ctrl=white_balance_temperature_auto=0
v4l2-ctl --set-ctrl=exposure_absolute=8
v4l2-ctl --set-ctrl=brightness=230
v4l2-ctl --set-ctrl=backlight_compensation=0
v4l2-ctl --set-ctrl=saturation=250
v4l2-ctl --set-ctrl=white_balance_temperature=4000

# Importar dependencias de ROS - workspaces
source /opt/ros/melodic/setup.bash
source /home/cap/pepo_ws/devel/setup.bash
export ROS_IP=192.168.0.101
export ROS_MASTER_URI=http://192.168.0.101:11311

# Chamar servidor rosbridge de inicio
roslaunch rosbridge_server rosbridge_websocket.launch
sleep 5s

rosparam set exposure_auto 1
rosparam set white_balance_temperature_auto 0
rosparam set exposure_absolute 30
rosparam set brightness 500
rosparam set backlight_compensation 0
rosparam set saturation 250
rosparam set white_balance_temperature 4000

# Chamar o servidor flask para o aplicativo
python3 /home/cap/pepo_ws/src/PEPO_Embarcado/app.py
