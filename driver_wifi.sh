#!/bin/bash

# Dormir um pouco para esperar iniciar
sleep 10s

# Mudar a pasta para a que contem o driver compilado
cd /home/cap/rtl8188fu

# Chamar o modprobe
echo 12 | sudo -S modprobe cfg80211
# Chamar o insmod
echo 12 | sudo -S insmod rtl8188fu.ko
