#!/bin/bash

# Transferir codigo de PC para pepo, sincronizando e atualizando
sshpass -p 12 rsync -avz -e 'ssh' $1 cap@192.168.0.102:/home/cap/pepo_ws/src/PEPO_Embarcado/
sshpass -p 12 ssh cap@192.168.0.102 'cd /home/cap/pepo_ws/src/PEPO_Embarcado && ./compilar_ws.sh'
