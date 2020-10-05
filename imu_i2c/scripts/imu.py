#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

import time
import smbus
import math
 
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

print ("Reading Data from Gyroscope and Accelerometer")

roll  = 0
pitch = 0
mroll = 0
mpitch= 0
##############################
def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass
##############################
def talker():
         
    ###### SETUP #########
    pub_imu = rospy.Publisher('imu', Float32MultiArray, queue_size = 1)
    rospy.init_node('imu_node', anonymous=False)
    rate = rospy.Rate(20) # 20hz
   
    mroll  = 0
    mpitch = 0   
   
    ######## LOOP  ########
    while not rospy.is_shutdown():
        ax = read_raw_data(ACCEL_XOUT_H)
        ay = read_raw_data(ACCEL_YOUT_H)
        az = read_raw_data(ACCEL_ZOUT_H)

        AX = 0.000061128340*ax - 0.0000012617380*ay + 8.45974200e-8*az - 0.021714000
        AY = 0.000001114320*ax + 0.0000607818500*ay - 3.80779400e-8*az + 0.008420878
        AZ = 0.000000266423*ax - 0.0000005166751*ay + 0.00006071884*az + 0.097948000
        roll  = np.arctan2(AY, AZ)
        pitch = np.arctan(-AX / (AY * np.sin(roll) + AZ * np.cos(roll)))

        mroll  = 0.9*mroll  + 0.1*roll
        mpitch = 0.9*mpitch + 0.1*pitch

        headdata = Float32MultiArray(data=[180.0/np.pi*mroll,180.0/np.pi*mpitch])
        pub_imu.publish(headdata)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
