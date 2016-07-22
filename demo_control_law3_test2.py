#!/usr/bin/python

import os
__author__ = "yingqi"
__date__ = "summer 2015"
import numpy as np
from scipy import linalg as LA
import scipy.io
from time import sleep
from random import choice
import threading
import time
import socket
import math
import sys
import ConfigParser
from ev3dev import *
from ast import literal_eval
import Queue

####----------- robot number --(change here for each vehicle)-------#####

####  by default the vechicle num 1 get information from virtual leader
Robot = 'Robot1'
mat_file = 'test_law3_test2.mat'
#### global nRob(change here for each vehicle)
nRob = 1

#### geometrical formation parameter(change here for each vehicle)
Px = 0
Py = 0.2
Ptheta = 0

# adjacent matrix of comunication topology, also defined in 'topologie.cfg'
Topologie = np.mat([[1,1,1,1,1,1],
                    [1,1,1,1,1,1],
                    [1,1,1,1,1,1],
                    [1,1,1,1,1,1],
                    [1,0,0,0,0,0],
                    [0,0,1,0,0,0]])
####----------- global paramater initialisition --(change here for each vehicle)------------###############

#### motors and touch-senser initializations
lmotor = large_motor(OUTPUT_A); assert lmotor.connected
rmotor = large_motor(OUTPUT_D); assert rmotor.connected
ts     = touch_sensor()
motors = [lmotor, rmotor]

#### motors dutycycle
Angular_Speed = 0
Line_Speed = 0
D1 = 0
D2 = 0

#### position initialisition
Theta = 0
X = 0
Y = 0
Theta_last = 0
X_last = 0
Y_last = 0

#### control parameter initialision
beta = 0.01
gamma = 0.06
k0 = 2
b = 1
# if nRob == 1:
#     b = 1
# else:
#     b = 0
P=1
I=0.2
D=1

#### the 4 important control index
u1 = 0
u2 = 0
z1 = 0
z2 = 0
z3 = 0

#### mechanical parameter initialision
r = 0.023  #### radius for wheel
L = 0.12   #### distance for two wheel

#### virtual leader information ---(it's only used for vheicle num 1)
# time = 0
u10 = 0.1

#### kalman-filter parameter
N = 1
T1 = 0
T2 = 0
T1_last =0
T2_last = 0
kalman_gain_L = 0
kalman_gain_R = 0
P_error_L = 1
P_error_R = 1
P_minus_L = 0
P_minus_R = 0
Q = 1e-3   #### Q is incentive noise
R = 1e-3   #### R is detecting noise
#### motor driver function
#### basic drive for one wheel
def drive(m, p):
    if m.connected:
       m.run_direct(duty_cycle_sp=p)

#### run function with PI control during action

####stop function
def stop():
    for m in motors:
        if m.connected:
            m.stop()

#### touch sensor to start
def touch():
    return ts.connected and ts.value()

############ command law part  #########################
def calculate_vitrualleader(t):
#### calculate vitrual leader position information
    print 'calculate_virtualleader'
    global k0, u10
    w = u10
    x = 2 * math.sin(w*t)
    y = -1*math.cos(w*t)
    print 'virtual leader position x:'+str(x)
    print 'virtual leader position y:'+str(y)
    z10 = w*t
    print 'virtual leader position theta:'+str(z10/math.pi)+'pi'
    z30 = x*math.sin(w*t) - y*math.cos(w*t)
    z20 = x*math.cos(w*t)+y*math.sin(w*t)+k0*z30*np.sign(u10)
    return z10,z20

def calculate_controlindex():
#### use local info list to calculate control law index U1 U2
    print 'calculate_controlindex'
    global z1,z2,z3,u1,u2,Theta,X,Y,Px,Py,u10,b,nRob
    sum1 = 0
    sum2 = 0
    dz20 = 0
    z1 = Theta
    z3 = (X-Px)*math.sin(Theta)-(Y-Py)*math.cos(Theta)
    z2 = (X-Px)*math.cos(Theta)+(Y-Py)*math.sin(Theta)+k0*z3*np.sign(u1)
    print 'localZ1:'+str(z1)
    print 'localZ2:'+str(z2)
    print 'localZ3:'+str(z3)

    for i in list_info:
        if(i==0):
            continue
        sum1 += Topologie[nRob-1,i-1]*pow((z1-list_info[i].z1),Topologie[nRob-1,i-1])
    for i in list_info:
        if(i==0):
            continue
        sum2 += Topologie[nRob-1,i-1]*pow((z2-list_info[i].z2),Topologie[nRob-1,i-1])
    u1 = u10 -0.06*(sum1+b*(z1-list_info[0].z1))-0.03*np.sign(sum1+b*(z1-list_info[0].z1))
    u2 = -0.06*(sum2+b*(z2-list_info[0].z2))-0.03*np.sign(sum2+b*(z2-list_info[0].z2))-k0*math.fabs(u1)*z2   ##### need to know how dz20
    print 'u1:'+str(u1)
    print 'u2:'+str(u2)
    print 'beta u1:'+str(-beta*sum1)
    print 'beta u2:'+str(- beta*sum2)
    print 'gamma u1:'+str(- gamma*b*(z1-list_info[0].z1))
    print 'gamma u2:'+str(- gamma*b*(z2-list_info[0].z2))
    print 'k0 u2:'+str(-k0*math.fabs(u1)*z2)
    return u1,u2

def calculate_vehiclestate(u1,u2):
#### use U1,U2,two index to caculate vehicle stat
    global z3,Angular_Speed,Line_Speed
    print('calculate_vechiclestate')
    Angular_Speed= u1
    Line_Speed = u2+(1+math.pow(k0,2))*u1*z3
    w = Angular_Speed
    v = Line_Speed
    return w,v

def calculate_wheelspeed(w,v):
#### use w,v,two index to caculate vehicle wheel stat
    print('calculate_wheelspeed')
    Rotation_Speed_L = (v-w*L/2)/r
    Rotation_Speed_R = (v+w*L/2)/r
    w1 = Rotation_Speed_L
    w2 = Rotation_Speed_R
    return w1,w2

def update_position(m1,m2):
#### update position
#### here we use kalman filter
    print 'updateposition'
    global Angular_Speed,Line_Speed,L,r
    global X,Y,Theta,X_last,Y_last,Theta_last
    global kalman_gain_L,kalman_gain_R,T1_last,T2_last,T1,T2,t_last
    global P_minus_L,P_minus_R,P_error_L,P_error_R,Q,R
    T1_sample = m1.position
    T2_sample = m2.position
    # time update
    t = time.time()
    time_run = t - t_last
    #1)X(k|k-1) = AX(k-1|k-1) + BU(k) + W(k),A=1,BU(k) = 0
    T1 = (Line_Speed*time_run - Angular_Speed*time_run*L/2)/r*180/np.pi
    T2 = (Line_Speed*time_run + Angular_Speed*time_run*L/2)/r*180/np.pi
    #2)P(k|k-1) = AP(k-1|k-1)A' + Q(k) ,A=1  Q is incentive noise
    P_minus_L = P_error_L + Q
    P_minus_R = P_error_R + Q
    # measurement update
    #3)Kg(k)=P(k|k-1)H'/[HP(k|k-1)H' + R],H=1  R is detecting noise
    kalman_gain_L = P_minus_L/(P_minus_L + R)
    kalman_gain_R = P_minus_R/(P_minus_R + R)
    #4)X(k|k) = X(k|k-1) + Kg(k)[Z(k) - HX(k|k-1)], H=1
    T1 = kalman_gain_L*T1_sample+(1-kalman_gain_L)*T1_last
    T1_last = T1
    T2 = kalman_gain_R*T2_sample+(1-kalman_gain_R)*T2_last
    T2_last = T2
    #5)P(k|k) = (1 - Kg(k)H)P(k|k-1), H=1
    P_error_L = (1 - kalman_gain_L)*P_minus_L
    P_error_R = (1 - kalman_gain_R)*P_error_R
    Theta = Theta_last+((T2-T1)/360)*(r/L)*2*math.pi
    Theta_last = Theta
    Theta_Degree = Theta*180/math.pi
    X = X_last+((T1+T2)*math.pi/360)*r*math.cos(Theta)
    Y = Y_last+((T1+T2)*math.pi/360)*r*math.sin(Theta)
    X_last = X
    Y_last = Y
    print 'T1:'+str(T1)
    print 'T2:'+str(T2)
    print 'position x:'+str(X)
    print 'position y: '+str(Y)
    print  'theta :'+str(Theta/math.pi)+'pi'
    return T1,T2

def calculate_runduty(T1,T2,wL,wR):
    #### calculate duty cycle use T1,T2 1:left 2:right
    #### here in this algo we has integrated PI for duty cycle calculate
    global t_last,D1,D2,p
    e1_R = wR-T2/(360*(time.time()-t_last))*math.pi*2
    e1_L = wL-T1/(360*(time.time()-t_last))*math.pi*2
    if error_queue_L.qsize()<5:
        error_queue_L.put(e1_L)
    if error_queue_R.qsize()<5:
        error_queue_R.put(e1_R)
    if error_queue_L.qsize()>=5:
        error_queue_L.put(e1_L)
        error_queue_L.get()
    if error_queue_R.qsize()>=5:
        error_queue_R.put(e1_R)
        error_queue_R.get()
    in_el_R = 0
    in_el_L = 0
    mycopy = []
    while True:
        if not error_queue_L.empty():
            elem = error_queue_L.get(block=False)
            mycopy.append(elem)
        else:
            break
    for elem in mycopy:
        in_el_L+=elem
        error_queue_L.put(elem)
    mycopy[:] = []
    while True:
        if not error_queue_R.empty():
            elem = error_queue_R.get(block=False)
            mycopy.append(elem)
        else:
            break
    for elem in mycopy:
        in_el_R+=elem
        error_queue_R.put(elem)

    D1=D1+P*e1_L+I/5*in_el_L
    D2=D2+P*e1_R+I/5*in_el_R
    D1=int(D1)
    D2=int(D2)
    return D1,D2

def append_info(m):
    #### append information procedure to .mat file
    mat_time.append(m[0])
    mat_z10.append(m[1])
    mat_z20.append(m[2])
    mat_z1.append(m[3])
    mat_z2.append(m[4])
    mat_z3.append(m[5])
    mat_X.append(m[6])
    mat_Y.append(m[7])
    mat_Theta.append(m[8])

####--------------- local info list -----------##############
list_info = {}

####--------------- information list sent to matlab ---############
mat_info ={}
mat_time = []
mat_z10 = []
mat_z20 = []
mat_z1 = []
mat_z2 = []
mat_z3 = []
mat_X = []
mat_Y = []
mat_Theta = []

####----------------error queue in PID -----------################
error_queue_L = Queue.Queue() #### error queue for intergral of error in PID
error_queue_R = Queue.Queue()

############ net connecting part ########################
####-------------- info class --------------###############
class Info:
    def __init__(self, from_num, z1, z2):
      self.from_num = from_num
      self.z1 = z1
      self.z2 = z2

    def print_info(self):
        print 'get info'+'z1='+self.z1+'z2='+self.z2+'message from' + self.from_num

#####-------------client part---------------################
#Main thread to send information to other vehicle'URL in topologie configuration
class Transmit(threading.Thread):
    def run(self):
    # Sending data to the server
        global adr
        val_ancien_capteur = touch()
        print('Transmit started')
        global s # connection
        global threadRunning
        global delta_t
        while(threadRunning):
            if(touch()):
                global ifsend, stat_me
                ifsend = True
            if ifsend:
                #print info
                for i in range(0,len(adr)):
                    send_info = [nRob,z1,z2]
                    s.sendto(str(send_info),(adr[i],PORT))
            time.sleep(delta_t)
        print('Transmit stopped - threadRunning')
        return

####-----------server part--------------#############
#Main thread to receive information from whatever other vehicle
class Receive(threading.Thread):
    def run(self):
        # Receiving data from the server
        print('Receive started')
        global s # connection
        global threadRunning
        global t0,t_last
        t0 = time.time()
        i = 0
        num_tour = 0  #### num for i th tour
        x =[]
        while(threadRunning):
            # Use 1 second timeout on receive
            print "reception..."
            print '############################################################'
            try:
                s.settimeout(0.05)
                data, addr = s.recvfrom(1024)
                if(len(data) > 0):
                    global ifsend
                    ifsend = True
                if ifsend:# ifsend as a global var show transmit global begin
                    if (len(data)>0):
                        print 'Message get from[' + addr[0] + ':' + str(addr[1]) + '] - '+ data
                        #### save vitrual leader info(only num 1 know virtual leader)
                        z10,z20 = calculate_vitrualleader(time.time()-t0)
                        print 'runtime'+str(time.time()-t0)
                        list_info[0] = Info(0,z10,z20)
                        #### save received info (from_num.z1,z2) into a local global dictionnary
                        message = literal_eval(data)
                        i = message[0]
                        info = Info(message[0], message[1], message[2])
                        info_key=i
                        list_info[info_key]=info
                        if(0 in list_info):
                            print 'show virtual leader'
                            print list_info[0].from_num
                            print list_info[0].z1
                            print list_info[0].z2
                        if(1 in list_info):
                            print 'show num 1'
                            print list_info[1].from_num
                            print list_info[1].z1
                            print list_info[1].z2
                        if(2 in list_info):
                            print 'show num 2'
                            print list_info[2].from_num
                            print list_info[2].z1
                            print list_info[2].z2

                        ####calculate U1 U2  (ps:for vechicle1 and other vechicle ,the format is different)
                        u1,u2=calculate_controlindex()
                        ####calculate angular_speed and velocity
                        w,v = calculate_vehiclestate(u1,u2)
                        ####calculate vechicle wheel running state
                        w1,w2=calculate_wheelspeed(w,v)
                        ####update position
                        T1,T2=update_position(motors[0],motors[1])
                        ####reset motors
                        motors[0].reset() #### left
                        motors[1].reset() #### right
                        #### run motors change w to duty cycle
                        D1,D2 = calculate_runduty(T1,T2,w1,w2)
                        print 'left duty cyclr:'+str(D1)
                        print 'right duty cycle:' +str(D2)
                        x=np.array([time.time()-t0,z10,z20,z1,z2,z3,X,Y,Theta])
                        print x[:]
                        append_info(x)
                        print 'now is '+str(num_tour)+'tour'
                        num_tour=num_tour+1
                        if(D1>-100 and D1<100):
                            drive(motors[0],D1)
                        elif(D1<=-100):
                            drive(motors[0],-100)
                        elif(D1>=100):
                            drive(motors[0],100)
                        if(D2>-100 and D2<100):
                            drive(motors[1],D2)
                        elif(D2<=-100):
                            drive(motors[1],-100)
                        elif(D2>=100):
                            drive(motors[1],100)
                else: # Connection closed by server
                    threadRunning = False

                global delta_t
                time.sleep(delta_t)

            #### if we cant receive a info from any robot now,we wait for 3 s and react directly
            except socket.timeout:
                print 'READ failed. timeout + No Receive This Time'
                #### save vitrual leader info
                z10,z20 = calculate_vitrualleader(time.time()-t0)
                print 'runtime'+str(time.time()-t0)
                list_info[0] = Info(0,z10,z20)
                if(0 in list_info):
                    print 'show virtual leader'
                    print list_info[0].from_num
                    print list_info[0].z1
                    print list_info[0].z2
                ####calculate U1 U2  (ps:for vechicle1 and other vechicle ,the format is different)
                u1,u2=calculate_controlindex()
                ####calculate angular_speed and velocity
                w,v = calculate_vehiclestate(u1,u2)
                ####calculate vechicle wheel running state
                w1,w2=calculate_wheelspeed(w,v)
                ####update position
                T1,T2=update_position(motors[0],motors[1])
                ####reset motors
                motors[0].reset()
                motors[1].reset()
                 #### run motors change w to duty cycle
                D1,D2 = calculate_runduty(T1,T2,w1,w2)
                print 'left duty cyclr:'+str(D1)
                print 'right duty cycle:' +str(D2)
                x=np.array([time.time()-t0,z10,z20,z1,z2,z3,X,Y,Theta])
                print x[:]
                append_info(x)
                print 'now is '+str(num_tour)+'tour'
                num_tour=num_tour+1
                if(D1>-100 and D1<100):
                    drive(motors[0],D1)
                elif(D1<=-100):
                    drive(motors[0],-100)
                elif(D1>=100):
                    drive(motors[0],100)
                if(D2>-100 and D2<100):
                    drive(motors[1],D2)
                elif(D2<=-100):
                    drive(motors[1],-100)
                elif(D2>=100):
                    drive(motors[1],100)

                # global delta_t
                # time.sleep(delta_t)

            print '1 tour reception'
            t_last = time.time()
        print('Receive stopped - threadRunning')
        return

# #############------------ start running here---------###############
# ###############------------------ mian threading  ------------------#############
HOST = ''
PORT = 5005

threadRunning = False
ifsend = False
delta_t = 0.03
t0 = 0
t_last = 0

s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
s.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
s.bind(('',5005))

os.system('clear')
print('Connection with server established')

# recuperation des adresses dans le fichier cfg
config = ConfigParser.RawConfigParser()
config.read('topologie.cfg')
list = config.options(Robot)
adr=['' for i in range(len(list))]
for i in range(0,len(list)):
  adr[i] = config.get(Robot, list[i])
print adr
print 'initial position x:'+str(Px)
print 'initial position y:'+str(Py)
X = Px
Y = Py
X_last = Px
Y_last = Py
Theta = Ptheta
Theta_last = Ptheta
####reset motors
motors[0].reset()
motors[1].reset()

try:
  print(34 * '-')
  print(Robot)
  print("        M A I N - M E N U")
  print(' Press CTRL+C to close connection')
  print(34 * '-')
  # Create instance of class
  threadRunning = True
  transmit = Transmit()
  receive = Receive()
  # Start class
  transmit.start()
  receive.start()
  while(threadRunning):
    time.sleep(delta_t)

except KeyboardInterrupt: # Stop program when CTRL+C is pressed
  threadRunning = False
  print('Main stopped')
  print(34 * '-')
  print(34 * '-')
  print(34 * '-')
  mat_info['time']=mat_time
  mat_info['z10']=mat_z10
  mat_info['z20']=mat_z20
  mat_info['z1']=mat_z1
  mat_info['z2']=mat_z2
  mat_info['z3']=mat_z3
  mat_info['X']=mat_X
  mat_info['Y']=mat_Y
  mat_info['Theta']=mat_Theta
  print len(mat_info)
  scipy.io.savemat(mat_file,mat_info)
  s.sendto('Force Close Thread RECEIVE',('localhost',PORT))
  time.sleep(2)
  s.close()

finally:
  s.close()