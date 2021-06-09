#!/usr/bin/env python3

# Script permettant de récuperer la position et l'angle 

from time import sleep
import sys
import serial
import time
from marvelmind import MarvelmindHedge

HEDGEID = 54

def main():
    ser2 = serial.Serial2('/dev/ttyS0', 115200, timeout=1)
    ser2.flush()

    ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
    ser.flush()

    hedge = setHedge()
    while 1:
        try:
            if ser.in_waiting > 0:
                # Récuperation de l'angle
                angle  = ser.readline().decode('utf-8').rstrip()

                # Récuperation de la position
                hedge.dataEvent.wait(1)
                hedge.dataEvent.clear()

                if (hedge.positionUpdated):
                    position =  getPosition(hedge)
                    while position[0] != HEDGEID:
                        position = getPosition(hedge)
                    result = position[1:4] + [angle]
                    print('X:{} Y:{} Z:{} θ:{}'.format(result[0], result[1], result[2], result[3]))

               
                    ser.write(b"2/-2\n")
                    line = ser.readline().decode('utf-8').rstrip()
                    time.sleep(1)
                

        except KeyboardInterrupt:
            hedge.stop()  # stop and close serial port
            sys.exit()

def getPosition(hedge):
    return hedge.position()


def setHedge():
    hedge = MarvelmindHedge(tty="/dev/ttyACM0", adr=None, debug=False)
    hedge.start()
    return hedge

main()
