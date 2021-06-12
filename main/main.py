#!/usr/bin/env python3

# Script permettant de récuperer la position et l'angle 

from math import *
import sys
import serial
from marvelmind import MarvelmindHedge

HEDGEID = 54
#GAIN_K = 0.66
GAIN_K = 0.2
#GAIN_K = 0.066
CAPTOR_DISTANCE = 150
WHEEL_RADIUS = 0.08
l = 0.5
L = -0.1

def main():

    target_x = 5
    target_y = 0

    ser2 = serial.Serial('/dev/ttyS0', 115200, timeout=1) # Arduino <-> RPI
    ser2.flush()

    hedge = setHedge()
    while 1:
        try:
            if ser2.in_waiting > 0:
                # Récuperation de l'angle
                angle  = ser2.readline().decode('utf-8').rstrip()

                # Angle en radian
                angle = float(angle) * pi / 180

                # Récuperation de la position
                #hedge.dataEvent.wait(1)
                #hedge.dataEvent.clear()

                #if (hedge.positionUpdated):
                position =  getPosition(hedge)
                while position[0] != HEDGEID:
                    position = getPosition(hedge)
                result = position[1:4] + [angle]
                print('X:{} Y:{} θ:{}'.format(result[0], result[1],  result[3]))
                
                pos_x = float(result[0])
                pos_y = float(result[1])

                ux,uy = correction(pos_x, pos_y, target_x, target_y)
                linear_speed, rotation_speed = rotation_matrix(ux, uy, float(angle))
                motor_speed_left, motor_speed_right = command(linear_speed, rotation_speed)

                print('motor_speed_left:{} motor_speed_right:{}'.format(motor_speed_left*1000, motor_speed_right*1000))

                serialReturn = str(motor_speed_left*1000) + "/" + str(motor_speed_right*1000) + "\n"
               # serialReturn = "100/0\n"
                ser2.write(bytes(serialReturn, 'UTF-8'))

                    
                

        except KeyboardInterrupt:
            hedge.stop()  # stop and close serial port
            sys.exit()

def getPosition(hedge):
    return hedge.position()


def setHedge():
    hedge = MarvelmindHedge(tty="/dev/ttyACM0", adr=None, debug=False)
    hedge.start()
    return hedge


####
# Bloc 2
# speed : v°
# rotation_speed : theta°
###
def command(linear_speed, rotation_speed): 
    v1 = linear_speed - rotation_speed * WHEEL_RADIUS
    v2 = linear_speed + rotation_speed * WHEEL_RADIUS
    return v1, v2

####
# Bloc 1.2
# speed : v°
# rotation_speed : theta°
###
def rotation_matrix(ux,uy,theta):
    linear_speed = ux * cos(theta) + uy*sin(theta)
    rotation_speed = -(ux * sin(theta)/L) + (uy*cos(theta)/L)
    return linear_speed,rotation_speed

####
# Bloc 1.1
# speed : v°
# rotation_speed : theta°
###
def correction(pos_x, pos_y, target_x, target_y):
    ux = GAIN_K * (target_x - pos_x)
    uy = GAIN_K * (target_y - pos_y)    
    return ux, uy

main()
