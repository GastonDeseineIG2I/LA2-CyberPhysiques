from math import *
import serial

GAIN_K = 0.66
CAPTOR_DISTANCE = 100 # 10cm
WHEEL_RAY = 35 # 3,5 cm

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


def main():
    while True:
        


    ser = serial.Serial('/dev/ttyS0',115200, timeout=1)
    ser.flush()
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)