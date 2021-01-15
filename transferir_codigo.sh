#!/bin/bash

# Transferir codigo de vinicius para pepo, sincronizando e atualizando
rsync -avz -e 'ssh' /home/vinicius/pepo_embarcado_ws/src/PEPO_Embarcado/ pepo@192.168.0.101:/home/cap/pepo_ws/src/PEPO_Embarcado/
