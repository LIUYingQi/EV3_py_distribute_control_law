
#!/usr/bin/python
# coding: utf-8
import os
__author__ = "yongqi"
__date__ = "3/3/2016"

import threading
import time
import socket
import sys
import ConfigParser
from ev3dev import *

#############################################
motors = [
        motor(OUTPUT_A),
        motor(OUTPUT_B),
        motor(OUTPUT_C),
        motor(OUTPUT_D)
        ]

port = 0
ts   = touch_sensor()

def drive(m, p):
    if m.connected:
        m.run_forever(speed_regulation_enabled='off', duty_cycle_sp=p)

def stop(m):
    if m.connected:
        m.stop()

def done():
    return ts.connected and ts.value()

###############################################################

class Receive(threading.Thread):
	def run(self):
		# Receiving data from the server
		print('Receive started')
		global s # connection
		global threadRunning
		while(threadRunning):
			# Use 1 second timeout on receive
			print "reception..."
			try:
                               # s.settimeout(10)
				data, addr = s.recvfrom(1024)
				if(len(data) == 0): # Connection closed by server
					threadRunning = False
				else:
					print 'Message get'
					if data == 'GOGOGO' :
						print 'go'
						drive(motor(OUTPUT_A), 20)
					elif data == 'DEMITOUR' :
						print 'demi'
					else:
						drive(motor(OUTPUT_A), 0)
						print 'test'
				time.sleep(0.01)
			except socket.timeout:
				print 'READ failed. timeout+ Receive stopped - threadRunning'
				return
		print('Receive stopped - threadRunning')
		return

# Running main program
HOST = ''
PORT = 5005

global threadRunning # Used to stop threads
threadRunning = False
global s

s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
s.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
s.bind(('',5005))

os.system('clear')
print('Connection with server established')
try:
	print(34 * '-')
	print("        M A I N - M E N U")
	print(' Press CTRL+C to close connection')
	print(34 * '-')
	# Create instance of class
	threadRunning = True
	receive = Receive()
	# Start class
	receive.start()
	while(threadRunning):
		time.sleep(0.1)

except KeyboardInterrupt: # Stop program when CTRL+C is pressed
	print('Main stopped')
	threadRunning = False
	s.sendto('Force Close Thread RECEIVE',('localhost',PORT))
	time.sleep(2)
	s.close()

finally:
	s.close()