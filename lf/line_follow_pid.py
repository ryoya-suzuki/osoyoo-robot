from __future__ import division
import time
import csv
import datetime
import math
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
r_speed=0
l_speed=0
max_speed=2500
min_speed=1200
pwm_move = 4095
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
#Set line weight
W_LINE = [-36, -17, 0, 17, 36]
#error line pos
error_line = 0
counter_sensor = 0
#Initial time
time_start = time.time()
time_elapsed = 0
#Parameter for pid
k_p = 80
#Side flag
f_left=0
f_right=1
#PWM? constrain
pwm_def = 2000
pwm_max = 4095
pwm_min = 0

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
    pwm.set_pwm(in1,0,pwm_move)   #IN1
    pwm.set_pwm(in2,0,0)      #IN2
    
    pwm.set_pwm(in3,0,pwm_move)   #IN3
    pwm.set_pwm(in4,0,0)      #IN4
#Robot car backwards
def go_back():
    pwm.set_pwm(in1,0,0)      #IN1
    pwm.set_pwm(in2,0,pwm_move)   #IN2
    
    pwm.set_pwm(in3,0,0)      #IN3
    pwm.set_pwm(in4,0,pwm_move)   #IN4

#Robot car turn left
def turn_left():
    pwm.set_pwm(in1,0,pwm_move)   #IN1
    pwm.set_pwm(in2,0,0)      #IN2
    
    pwm.set_pwm(in3,0,0)      #IN3
    pwm.set_pwm(in4,0,pwm_move)   #IN4

#Robot turn right
def turn_right():
    pwm.set_pwm(in1,0,0)      #IN1
    pwm.set_pwm(in2,0,pwm_move)   #IN2
    
    pwm.set_pwm(in3,0,pwm_move)   #IN3
    pwm.set_pwm(in4,0,0)      #IN4

#Robot stop move
def stop():
	set_speed(0,0)
	
#Reset PCA9685's all channels    
def destroy():
	pwm.set_all_pwm(0,0)

#Calculate line position
def get_error_line():
    global counter_sensor, error_line
    error_line = lf1*W_LINE[0] + lf2*W_LINE[1] + lf3*W_LINE[2] + lf4*W_LINE[3] + lf5*W_LINE[4]
    counter_sensor = lf1 + lf2 + lf3 + lf4 + lf5
    if(counter_sensor!=0):
	    error_line /= counter_sensor
    if(counter_sensor==0):
        error_line = 0

def constrain_limit(val, min_val, max_val):
    if(val < min_val): return min_val
    if(val > max_val): return max_val
    return(val)
	
def set_speed_error(error_line):
    abs_error = abs(error_line)
    vel = max_speed - abs_error/W_LINE[4]*(max_speed-min_speed)
    vel = int(vel)
    print('vel: ',vel)
    set_speed(vel,vel)

#Robot car moves along the black line
def tracking():
    while True:
        read_sensors()
        set_speed(0,0)
        get_error_line()
        lf=str(lf1)+str(lf2)+str(lf3)+str(lf4)+str(lf5)
        time_elapsed = time_start - time.time()
        print(error_line)
        time_elapsed = math.floor(time_elapsed*1000000)/1000000 #[usec]
        csvlist.append(str(time_elapsed)+','+str(lf1)+','+str(lf2)+','+str(lf3)+','+str(lf4)+','+str(lf5)+','+str(error_line))
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

#Robot car moves along the black line with variable speed
def tracking_speed():
    while True:
        read_sensors()
        set_speed(0,0)
        get_error_line()
        lf=str(lf1)+str(lf2)+str(lf3)+str(lf4)+str(lf5)
        time_elapsed = time_start - time.time()
        print(error_line)
        time_elapsed = math.floor(time_elapsed*1000000)/1000000 #[usec]
        csvlist.append(str(time_elapsed)+','+str(lf1)+','+str(lf2)+','+str(lf3)+','+str(lf4)+','+str(lf5)+','+str(error_line))
        if(lf=='00100' or lf=='01110' or lf=='01010' or lf=='00000'):
            set_speed_error(error_line)
            go_forward()
            continue            
        if(lf=='00010' or lf=='00110'  or lf=='00101'):
            set_speed_error(error_line)
            turn_right()
            continue
        if(lf=='00001' or lf=='00111' or lf=='00011'):
            set_speed_error(error_line)
            turn_right()
            continue
   
        if(lf=='01000' or lf=='01100'  or lf=='10100'):
       	    set_speed_error(error_line)
            turn_left()
            continue
        if(lf=='10000' or lf=='11100' or lf=='11000'): 
            set_speed_error(error_line)
            turn_left()
            continue
        if(lf=='11111'):
            stop()

def calc_pid(error_line, f_side):
    if(f_side==f_left):
        error_line = -error_line
        p=k_p*error_line
        #i=k_i*
        #d=k_d*
    if(f_side==f_right):
        p=k_p*error_line
        #i=k_i*
        #d=k_d*
    val_pid = p
    val_pid = int(pwm_def + val_pid)
    return(constrain_limit(val_pid, pwm_min, pwm_max))

#Robot car moves along the black line by pid control
def tracking_pid():
    while True:
        read_sensors()
        get_error_line()
        time_elapsed = time_start - time.time()
        time_elapsed = math.floor(time_elapsed*1000000)/1000000 #[usec]
        csvlist.append(str(time_elapsed)+','+str(lf1)+','+str(lf2)+','+str(lf3)+','+str(lf4)+','+str(lf5)+','+str(error_line))
        print(error_line)

        if(counter_sensor > 0):
            set_speed_error(error_line)
            val_left = calc_pid(error_line,f_left)
            val_right = calc_pid(error_line,f_right)
            print('left:'+str(val_left)+'right'+str(val_right))
            pwm.set_pwm(in1,0,val_left)   #IN1
            pwm.set_pwm(in2,0,0)          #IN2
            pwm.set_pwm(in3,0,val_right)   #IN3
            pwm.set_pwm(in4,0,0)          #IN4
        if(counter_sensor==0 or counter_sensor==5):
            stop()
            
if __name__ == '__main__':
    try:
	#set log file
        log_name = './log/log_' + dt_now.strftime('%Y%m%d%H%M%S') + '.csv'
        f = open(log_name, 'w')
        writer = csv.writer(f,delimiter='\n')
        csvlist = []
        csvlist.append('time[usec],lf1,lf2,lf3,lf4,lf5,error')
        time_start = time.time()
    #start to line follow 
        tracking_pid()
    except KeyboardInterrupt:
	#robot car stop
        destroy()
    #save log
        writer.writerow(csvlist)
        f.close()

