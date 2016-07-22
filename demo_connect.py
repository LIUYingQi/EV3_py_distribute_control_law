
#!/usr/bin/python
# coding: utf-8
import os
__author__ = "yingqi"
__date__ = "3/3/2016"
import socket
import math
import sys
import numpy as np
from thread import *
import threading
import time
import socket
import ConfigParser
from ev3dev import *

####----------- robot number --(change here for each vehicle)-------#####
####  by default the vechicle num 1 get information from virtual leader
Robot = 'Robot1'

####----------- global paramater initialisition --(change here for each vehicle)------------###############
#### position initialisition
Theta = 0
X = 0
Y = 0

Theta_last = 0
X_last = 0
Y_last = -0.9
#### control parameter initialision
beta = 0.01
gamma = 0.06
k0 = 2
P=5
I=10
D=1
e1_R = 0
e2_R = 0
e3_R = 0
e1_L = 0
e2_L = 0
e3_L = 0
#### mechanical parameter initialision
r = 0.0216  #### radius for wheel
L = 0.087   #### distance for two wheel
#### geometrical formation parameter
Px = 0
Py = 0.2
#### virtual leader information ---(it's only used for vheicle num 1)
time = 0
z20 = k0
u10 = 0.1
#### pre-filter parameter
N = 1
T1_last = np.zeros((1,N))
T2_last = np.zeros((1,N))
T1_M_last = 0
T2_M_last = 0
Threshold = 150
T1_invalid = 0
T2_invalid = 0
Lowpass = 0.5
####-----------motors and touch-senser initializations--------------###############
global lmotor
global rmotor
lmotor = large_motor(OUTPUT_A); assert lmotor.connected
rmotor = large_motor(OUTPUT_D); assert rmotor.connected
ts     = touch_sensor()
####-----------touch sensor function touch()-----########
def touch():
    return ts.connected and ts.value()

####-----------server part--------------#############
#Main thread to receive information from whatever other vehicle
class Receive(threading.Thread):
	def run(self):
		# Receiving data from the server
		print('Receive started as server')
		global s # connection
		global threadRunning
        print (threadRunning)
        if(threadRunning):
            print "thread running"
        else:
            print "thread not running"
        while(threadRunning):
            print 'reception...'
            try:
                ########  reception and reaction alogorithme
                data, addr = s.recvfrom(1024)
                print 'get info from'+addr
                if(len(data) > 0):
                    global ifsend
                    ifsend = True
                if ifsend:
                    print "here is the reaction"
            except:
                print "Error: socket error for server"
                time.sleep(delta_t)
		print('Receive stopped - threadRunning')
###############################################################################


#####-------------client part---------------################
#Main thread to send information to other vehicle'URL in topologie configuration
class Transmit(threading.Thread):
    def run(self):
    # Sending data to the server
        print('Transmit started')
        global s # connection
        global threadRunning
        global delta_t
        if(threadRunning):
            print "thread running "
        else:
            print "thread not running"
        while(threadRunning):
            print 'Message start send '
            if(touch()):                ####  use a touch sensor for vechicle num 1 to start a transmision
                global ifsend, stat_me
                ifsend = True
                stat_me[0] = 1
                delta_t = 0.5
            if ifsend:
                try:
                    #####  transmision message to a URL , URL define in topologie text
                    message="hello joris"
                    for i in range(0,len(adr)):
                        print 'envoi a '+ adr[i]
                        s.sendto(str(message),(adr[i],PORT))
                    print "ok send it to host"
                except:
                    print "Error: socket error for client"
                    time.sleep(delta_t)
        print('Transmit stopped - threadRunning')
###########################################################################


##########------------algorithme part------------------###################
def Receive_action():
    T1_sample = lmotor.position
    T2_sample = rmotor.position

    T1 = Lowpass*T1_sample+(1-Lowpass)*T1_last
    T1_last = T1
    T1 = Lowpass*T1_sample+(1-Lowpass)*T2_last
    T2_last = T2

    Theta = Theta_last+((T1-T2)/360)*(r/L)*2*math.pi
    Theta_Degree = Theta*180/math.pi
    X = X_last+((T1+T2)*math.pi/360)*r*math.cos(Theta)
    Y = Y_last+((T1+T2)*math.pi/360)*r*math.sin(Theta)

    z11 = Theta
    u1 = u10-gamma*(z11-z20)
    Z31 = (X-Px)
##########################################################################


#######-------------  main programme --------------------------- ###############
os.system('clear')
print('this programme is to test the connection between two EV3')

#### initialsation for topologie configure
config = ConfigParser.RawConfigParser()
config.read('topologie.cfg')
list = config.options(Robot)
global adr
adr=['' for i in range(len(list))]
for i in range(0,len(list)):
  adr[i] = config.get(Robot, list[i])
print adr

#### parameter in main programme that influence control
delta_t = 1;
threadRunning=True
#### Socket parameter
HOST = ''
PORT = 5005

#### Global main programme parameter
global threadRunning # Used to stop threads
threadRunning = False
global s

global ifsend
ifsend = False

#############------------ start running here---------###############
############-------------creating socket ------------################
try:
    s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    s.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
    s.bind((HOST,PORT))
except socket.error , msg:
    print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()
print 'Socket bind complete'
#Start listening on socket
s.listen(10)
print 'Socket now listening'
print('Connection with server established')

###############------------------ mian threading  ------------------##################
try:
  print(34 * '-')
  print("DEMO FOR DISTRIBUTE CONTROL LAW")
  print(' Press CTRL+C to close connection')
  print(34 * '-')
  # Create instance of class
  # global threadRunning # Used to stop threads
  threadRunning = True
  transmit = Transmit()
  receive = Receive()
  # Start class
  transmit.start()
  receive.start()
  while(threadRunning):
    time.sleep(delta_t)

except KeyboardInterrupt: # Stop program when CTRL+C is pressed
  print('Main stopped')
  threadRunning = False
  s.sendto('Force Close Thread RECEIVE',('localhost',PORT))
  time.sleep(2)
  s.close()

finally:
  s.close()