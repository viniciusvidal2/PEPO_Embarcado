#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import numpy as np
import time
import serial
import adafruit_gps

# Serial Port
uart = serial.Serial("/dev/ttyTHS1", baudrate=9600, timeout=10)

# Create a GPS module instance.
gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial

##############################
def gps_publish():
    global gps, uart
    ###### SETUP #########
    pub_gps = rospy.Publisher('/gps', NavSatFix, queue_size=10)
    rospy.init_node('gps_node', anonymous=False)
    rate = rospy.Rate(10) # 20hz
    # Main loop runs forever printing the location, etc. every second.
    last_print = time.monotonic()
    while not rospy.is_shutdown():
        try:
            gps.update()
        except:
            rospy.logwarn('Bad message reading. Lets discard and restart the serial connection.')
            # Reset serial connection and gps entity. Start reading again
            uart.close()
            time.sleep(1)
            uart = serial.Serial("/dev/ttyTHS1", baudrate=9600, timeout=10)
            gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial
            gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
            gps.send_command(b"PMTK220,1000")
            
        # Every second print out current location details if there's a fix.
        current = time.monotonic()
        if current - last_print >= 1.0:
            last_print = current
            # GPS message
            msg = NavSatFix()
            msg.header.frame_id = 'map'
            msg.header.stamp = rospy.Time.now()
            msg.status.status  = 1
            msg.status.service = 1
            if gps.has_3d_fix:
                msg.latitude  = gps.latitude
                msg.longitude = gps.longitude
                msg.altitude  = gps.altitude_m
                msg.status.status = 2
            else:
                msg.latitude  = 0
                msg.longitude = 0
                msg.altitude  = 0

            # Publish message
            print(f'Last GPS status: {msg.status.status:d}.')
            pub_gps.publish(msg)
        rate.sleep()

    # If connection is down, close the serial port
    uart.close()

##############################
if __name__ == '__main__':
    
    try:
        # Turn on the basic GGA and RMC info (what you typically want)
        gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")

        # Set update rate to once a second (1hz) which is what you typically want.
        gps.send_command(b"PMTK220,1000")

        # Run the GPS publish routine
        gps_publish()

    except rospy.ROSInterruptException:
        pass
