#!/bin/bash

# Dormir um pouco para esperar iniciar
sleep 7s

# Liberar o toin la da usb
echo 12 | sudo -S chmod a+rw /dev/ttyUSB0

# Setar os parametros pra capturar
v4l2-ctl --set-ctrl=exposure_auto=1
v4l2-ctl --set-ctrl=exposure_absolute=0
v4l2-ctl --set-ctrl=brightness=90
v4l2-ctl --set-ctrl=backlight_compensation=0
