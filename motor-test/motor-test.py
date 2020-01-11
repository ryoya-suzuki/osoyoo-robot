from __future__ import division
import time
#import the PCA9685 module.
import osoyoo_PCA9685
import RPi.GPIO as GPIO
speed1 = 2000  #forward/backward speed
speed2 = 1000  #turn speed
ena = 8
enb = 13
in1 = 9
in2 = 10
in3 = 11
in4 = 12
# Initialise the PCA9685 using the default address (0x40).
pwm = osoyoo_PCA9685.PCA9685()
# Set frequency to 60hz.
pwm.set_pwm_freq(60)
#Set motor speed. speed max is 4095,min is 0 
def set_speed(lspeed,rspeed):
    pwm.set_pwm(ena,0,lspeed)
    pwm.set_pwm(enb,0,rspeed)

#Robot car forward
def go_forward():
    set_speed(speed1,speed1)
    
    pwm.set_pwm(in1,0,4095)   #IN1
    pwm.set_pwm(in2,0,0)      #IN2
    
    pwm.set_pwm(in3,0,4095)   #IN3
    pwm.set_pwm(in4,0,0)      #IN4

#Robot car backwards
def go_back():
    set_speed(speed1,speed1)
    
    pwm.set_pwm(in1,0,0)      #IN1
    pwm.set_pwm(in2,0,4095)   #IN2
    
    pwm.set_pwm(in3,0,0)      #IN3
    pwm.set_pwm(in4,0,4095)   #IN4

#Robot car turn left
def turn_left():
    set_speed(speed2,speed2)
    
    pwm.set_pwm(in1,0,4095)   #IN1
    pwm.set_pwm(in2,0,0)      #IN2
    
    pwm.set_pwm(in3,0,0)      #IN3
    pwm.set_pwm(in4,0,4095)   #IN4

#Robot turn right
def turn_right():
    set_speed(speed2,speed2)
    
    pwm.set_pwm(in1,0,0)      #IN1
    pwm.set_pwm(in2,0,4095)   #IN2
    
    pwm.set_pwm(in3,0,4095)   #IN3
    pwm.set_pwm(in4,0,0)      #IN4

#Robot stop move
def stop():
    set_speed(0,0)

def test():
    #forward-->2s
    go_forward()
    time.sleep(2)
    #backward-->2s
    go_back()
    time.sleep(2)
    #turn left-->2s
    turn_left()
    time.sleep(2)
    #turn right-->2s
    turn_right()
    time.sleep(2)
    #stop
    stop()
    
#Reset PCA9685's all channels    
def destroy():
    pwm.set_all_pwm(0,0)
if __name__ == '__main__':
    try:
	#forward-->backward-->turn left-->turn right-->stop
        test()
    except KeyboardInterrupt:
	#robot car stop
        destroy()
