#!/usr/bin/python3
import sys, time
from time import sleep
import Jetson.GPIO as gpio

# Iniciando tudo
channel = 13 # pino de saida que funcionou na Jetson
gpio.setmode(gpio.BOARD) # Usando numeracao raiz da placa
gpio.setwarnings(False) # parar com chatices
gpio.setup(channel, gpio.OUT) # vai funcionar como saida


# Piscando LED rapido V vezes
V = 20
for v in range(1, V):
    gpio.output(channel, gpio.HIGH)
    sleep(0.3)
    gpio.output(channel, gpio.LOW)
    sleep(0.3)
    gpio.output(channel, gpio.LOW )

# Deixando o led totalmente aceso
gpio.output(channel, gpio.HIGH)
gpio.output(channel, gpio.HIGH)
gpio.output(channel, gpio.HIGH)

