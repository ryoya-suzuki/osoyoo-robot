from __future__ import division
import time
import RPi.GPIO as GPIO
lf1,lf2,lf3,lf4,lf5=0,0,0,0,0
#line follow module port define
lleft = 5
lsensor = 6
msensor = 13
rsensor = 19
rright = 26
#sensors init
lf1,lf2,lf3,lf4,lf5,lf=0,0,0,0,0,0

#Initialise GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(lsensor,GPIO.IN)
GPIO.setup(msensor,GPIO.IN)
GPIO.setup(rsensor,GPIO.IN)
GPIO.setup(lleft,GPIO.IN)
GPIO.setup(rright,GPIO.IN)
# Read tracking senbsors's data
def read_sensors():
    global lf1,lf2,lf3,lf4,lf5,lf
    lf1 = GPIO.input(lleft)
    lf2 = GPIO.input(lsensor)
    lf3 = GPIO.input(msensor)
    lf4 = GPIO.input(rsensor)
    lf5 = GPIO.input(rright)

def destroy():
    GPIO.cleanup()
   
def main():
    global lf1,lf2,lf3,lf4,lf5,lf
    while True:
        read_sensors()
        lf=str(lf1)+str(lf2)+str(lf3)+str(lf4)+str(lf5)
        print(lf)
 
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
	#robot car stop
        destroy()
