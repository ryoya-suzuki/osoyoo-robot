from __future__ import division
import time
import csv
import datetime
#import the PCA9685 module.
import osoyoo_PCA9685
import RPi.GPIO as GPIO
#L298N port define
ena = 8
enb = 13
in1 = 9
in2 = 10
in3 = 11
in4 = 12
lf1,lf2,lf3,lf4,lf5=0,0,0,0,0
#from left to right ,three tracking sensors are connected to BCM17,BCM27 and BCM22
lleft = 5
lsensor = 6
msensor = 13
rsensor = 19
rright = 26
high_speed=3000
mid_speed=1800
low_speed=1200
# Initialise the PCA9685 using the default address (0x40).
pwm = osoyoo_PCA9685.PCA9685()
# Set frequency to 60hz.
pwm.set_pwm_freq(60)
#Initialise GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(lsensor,GPIO.IN)
GPIO.setup(msensor,GPIO.IN)
GPIO.setup(rsensor,GPIO.IN)
GPIO.setup(lleft,GPIO.IN)
GPIO.setup(rright,GPIO.IN)
#Set Current Time
dt_now = datetime.datetime.now()
# Read tracking senbsors's data
def read_sensors():
    global lf1,lf2,lf3,lf4,lf5
    lf1 = GPIO.input(lleft)
    lf2 = GPIO.input(lsensor)
    lf3 = GPIO.input(msensor)
    lf4 = GPIO.input(rsensor)
    lf5 = GPIO.input(rright)
#Set motor speed
def set_speed(lspeed,rspeed):
    pwm.set_pwm(ena,0,lspeed)
    pwm.set_pwm(enb,0,rspeed)

#Robot car forward
def go_forward():
    pwm.set_pwm(in1,0,4095)   #IN1
    pwm.set_pwm(in2,0,0)      #IN2
    
    pwm.set_pwm(in3,0,4095)   #IN3
    pwm.set_pwm(in4,0,0)      #IN4
#Robot car backwards
def go_back():
    pwm.set_pwm(in1,0,0)      #IN1
    pwm.set_pwm(in2,0,4095)   #IN2
    
    pwm.set_pwm(in3,0,0)      #IN3
    pwm.set_pwm(in4,0,4095)   #IN4

#Robot car turn left
def turn_left():
    pwm.set_pwm(in1,0,4095)   #IN1
    pwm.set_pwm(in2,0,0)      #IN2
    
    pwm.set_pwm(in3,0,0)      #IN3
    pwm.set_pwm(in4,0,4095)   #IN4

#Robot turn right
def turn_right():
    pwm.set_pwm(in1,0,0)      #IN1
    pwm.set_pwm(in2,0,4095)   #IN2
    
    pwm.set_pwm(in3,0,4095)   #IN3
    pwm.set_pwm(in4,0,0)      #IN4

#Robot stop move
def stop():
	set_speed(0,0)
	
#Reset PCA9685's all channels    
def destroy():
	pwm.set_all_pwm(0,0)

#Robot car moves along the black line
def tracking():
    while True:
        read_sensors()
        set_speed(0,0)
        lf=str(lf1)+str(lf2)+str(lf3)+str(lf4)+str(lf5)
        print(lf)
        csvlist.append(str(lf1)+','+str(lf2)+','+str(lf3)+','+str(lf4)+','+str(lf5))
        if(lf=='00100' or lf=='01110' or lf=='01010' or lf=='00000'):
            set_speed(high_speed,high_speed)
            go_forward()
            continue            
        if(lf=='00010' or lf=='00110'  or lf=='00101'):
            set_speed(low_speed,low_speed)
            turn_right()
            continue
        if(lf=='00001' or lf=='00111' or lf=='00011'):
            set_speed(mid_speed,mid_speed)
            turn_right()
            continue
   
        if(lf=='01000' or lf=='01100'  or lf=='10100'):
       	    set_speed(low_speed,low_speed)
            turn_left()
            continue
        if(lf=='10000' or lf=='11100' or lf=='11000'): 
            set_speed(mid_speed,mid_speed)
            turn_left()
            continue
        if(lf=='11111'):
            stop()
if __name__ == '__main__':
    try:
	#set log file
        log_name = './log/log_' + dt_now.strftime('%Y%m%d%H%M%S') + '.csv'
        f = open(log_name, 'w')
        writer = csv.writer(f,delimiter='\n')
        csvlist = []
    #start to line follow 
        tracking()
    except KeyboardInterrupt:
	#robot car stop
        destroy()
    #save log
        writer.writerow(csvlist)
        f.close()

