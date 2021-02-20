#!/bin/bash

# Transferir codigo de vinicius para pepo, sincronizando e atualizando
rsync -avz -e 'ssh' /home/vinicius/pepo_embarcado_ws/src/PEPO_Embarcado/ cap@192.168.0.102:/home/cap/pepo_ws/src/PEPO_Embarcado/
