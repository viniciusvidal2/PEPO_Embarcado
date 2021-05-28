#!/bin/bash
# Atualizar a data e hora da jetson para nao ter problemas com compilacao
date_hour=$(date "+%Y-%m-%d %H:%M:%S")
set_date_command="echo 12 | sudo -S timedatectl set-time \"$date_hour\""
sshpass -p 12 ssh cap@192.168.0.102 'echo 12 | sudo -S timedatectl set-ntp 0'
sshpass -p 12 ssh cap@192.168.0.102 $set_date_command
# Transferir codigo de PC para pepo, sincronizando e atualizando
sshpass -p 12 rsync -avz -e 'ssh' $1 cap@192.168.0.102:/home/cap/pepo_ws/src/PEPO_Embarcado/
# Setar como executavel o no de gps em python
sshpass -p 12 ssh cap@192.168.0.102 'echo 12 | sudo -S chmod +x /home/cap/pepo_ws/src/PEPO_Embarcado/gps_uart/scripts/gps.py'
# Compilar tudo
sshpass -p 12 ssh cap@192.168.0.102 'cd /home/cap/pepo_ws/src/PEPO_Embarcado && ./compilar_ws.sh'
