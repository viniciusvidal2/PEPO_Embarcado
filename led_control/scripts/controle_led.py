#!/usr/bin/env python3
import rospy
import sys
from time import sleep
import Jetson.GPIO as gpio
from std_msgs.msg import Bool
from led_control.srv import LED

# Variavel que observa o LED
estado_led = 1

def callbackLED(comando):
    # Variaveis globais
    global estado_led
    estado_led = comando.led

    return True

def controle():
    global estado_led

    rospy.init_node('controle_led', anonymous=False, disable_signals=False)
    rospy.loginfo("Iniciando no de controle do LED ...")

    # Iniciando canal de saida GPIO da Jetson
    channel = 13 # pino de saida que funcionou na Jetson
    gpio.setmode(gpio.BOARD) # Usando numeracao raiz da placa
    gpio.setwarnings(False) # parar com chatices
    gpio.setup(channel, gpio.OUT) # vai funcionar como saida
    
    # Servidor para controle do LED
    l = rospy.Service('/controle_led', LED, callbackLED)

    taxa = 10
    rospy.loginfo("Rodando controle do LED a %d Hz...", taxa)
    r = rospy.Rate(taxa)
    while not rospy.is_shutdown():
        # Variar segundo comando do LED
        if estado_led == 1: # continuo
            gpio.output(channel, gpio.HIGH)
            gpio.output(channel, gpio.HIGH)
            gpio.output(channel, gpio.HIGH)
        if estado_led == 2: # pisca rapido
            gpio.output(channel, gpio.HIGH)
            sleep(0.3)
            gpio.output(channel, gpio.LOW)
            sleep(0.3)
            gpio.output(channel, gpio.LOW )
        if estado_led == 3: # pisca lento
            gpio.output(channel, gpio.HIGH)
            sleep(1)
            gpio.output(channel, gpio.LOW)
            sleep(1)
            gpio.output(channel, gpio.LOW )

        # Rodar o ciclo ros
        r.sleep()

    rospy.spin()
    rospy.loginfo("Finalizando no ...")
    
if __name__ == '__main__':
    controle()
