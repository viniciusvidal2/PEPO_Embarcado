#!/bin/bash

# Transferir codigo de vinicius para pepo, sincronizando e atualizando
rsync -avz -e 'ssh' /home/vinicius/pepo_embarcado_ws/src/PEPO_Embarcado/ pepo@192.168.0.101:/home/pepo/pepo_ws/src/PEPO_Embarcado/

# Entrar no pepo mandando compilar de uma vez
ssh pepo@192.168.0.101 'cd /home/pepo/pepo_ws/src/PEPO_Embarcado && ./compilar_ws.sh'
