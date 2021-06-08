#!/usr/bin/env python
from marvelmind import MarvelmindHedge
from time import sleep
import sys

def main():
    hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=None, debug=False) # create MarvelmindHedge thread
    
    if (len(sys.argv)>1):
        hedge.tty= sys.argv[1]
    
    hedge.start() # start thread
    while True:
        try:
            hedge.dataEvent.wait(1)
            hedge.dataEvent.clear()

            if (hedge.positionUpdated):
                hedge.print_position()
                position =  hedge.position()
                
                x = position[0]
                y = position[1]
                z = position[2]
                timestamp = position[3]
                

        except KeyboardInterrupt:
            hedge.stop()  # stop and close serial port
            sys.exit()
main()
