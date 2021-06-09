#!/usr/bin/env python3

# Script permettant de récuperer la position et l'angle 

from math import *
import sys
import serial
from marvelmind import MarvelmindHedge

HEDGEID = 54
GAIN_K = 0.66
CAPTOR_DISTANCE = 100 # 10cm
WHEEL_RAY = 35 # 3,5 cm

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

                if (hedge.positionUpdated):
                    position =  getPosition(hedge)
                    while position[0] != HEDGEID:
                        position = getPosition(hedge)
                    result = position[1:4] + [angle]
                    print('X:{} Y:{} Z:{} θ:{}'.format(result[0], result[1], result[2], result[3]))

               
                    speed, rotation_speed = position_rot_speed(float(result[0]), float(result[1]), target_x, target_y, float(angle))
                    motor_speed_left, motor_speed_right = commande(speed, rotation_speed)

                    print('motor_speed_left:{} motor_speed_right:{}'.format(motor_speed_left, motor_speed_right))

                    serialReturn = motor_speed_left + "/" + motor_speed_right + "\n"
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
# Bloc 1 (cf. CR)
# pos_x : Position en x
# pos_y : Position en y
# target_x : x souhaité
# target_y : y souhaité
# theta : angle en rad
####
def position_rot_speed(pos_x, pos_y, target_x, target_y, theta):
    ux = GAIN_K * (target_x - pos_x)
    uy = GAIN_K * (target_y - pos_y)
    rinvx = CAPTOR_DISTANCE * cos(theta) + CAPTOR_DISTANCE * sin(theta)
    rinvy = -sin(theta) + cos(theta)
    
    return rinvx*ux, rinvy*uy

####
# Bloc 2
# speed : v°
# rotation_speed : theta°
####
def commande(speed, rotation_speed) :
    motor_speed_left = speed + rotation_speed * WHEEL_RAY
    motor_speed_right = speed - rotation_speed * WHEEL_RAY
    return motor_speed_left, motor_speed_right


main()
