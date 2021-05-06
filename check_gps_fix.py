#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import time
import serial
import adafruit_gps

def check():
    # Serial Port
    uart = serial.Serial("/dev/ttyTHS1", baudrate=9600, timeout=10)

    # Create a GPS module instance.
    gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial

    # Turn on the basic GGA and RMC info (what you typically want)
    gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")

    # Set update rate to once a second (1hz) which is what you typically want.
    gps.send_command(b"PMTK220,200")

    # Acquire from GPS
    for i in range(15):
        time.sleep(0.2)
        gps.update()

    # Check fix
    fix = 'yes' if gps.has_3d_fix else 'no'

    uart.close()

    return fix
