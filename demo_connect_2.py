
#!/usr/bin/python
# coding: utf-8
import os
__author__ = "yingqi"
__date__ = "3/3/2016"
import socket
import sys
from thread import *
import threading
import time
import socket
from ev3dev import *

####
print('this programme is to test the connection between two EV3')


######          param for server
HOST = ''   # Symbolic name meaning all available interfaces
PORT = 8888 # Arbitrary non-privileged port
######          param for client
host = '192.168.23.2';
port = 8888;
delta_t = 1
global threadRunning # Used to stop threads
threadRunning = False
#Creat Socket here
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print 'Socket created'
#Bind socket to local host and port
try:
    s.bind((HOST, PORT))
except socket.error , msg:
    print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()
print 'Socket bind complete'

#Start listening on socket
s.listen(10)
print 'Socket now listening'

####-----------server part--------------#############
#Function for handling connections. This will be used to create threads
def clientthread(conn):
    #Sending message to connected client
    conn.send('Welcome to the server. Listening\n') #send only takes string
    data = conn.recv(1024)
    reply = 'OK.get..'+data
    print(reply)
    conn.send(reply)
        #came out of loop
    conn.close()

#Main thread to receive information from whatever other vehicle
class Receive(threading.Thread):
	def run(self):
		# Receiving data from the server
		print('Receive started')
		global s # connection
		global threadRunning
        while(threadRunning):
            print 'reception...'
            try:
                conn, addr = s.accept()
                start_new_thread(clientthread ,(conn,))

            except:
                print "Error: socket error for server"

            time.sleep(delta_t)
        print('Receive stopped - threadRunning')


#####-------------client part---------------################
class Transmit(threading.Thread):
    def run(self):
    # Sending data to the server
        print('Transmit started')
        global s # connection
        global threadRunning
        global delta_t
        while(threadRunning):
			print 'Message start send '
			try:
				message="hello joris"
				s.sendto(message,(host,port))
                # print 'ok send it to'+host
    			# conn, addr = s.accept()
    			# data = conn.recv(1024)
    			# print data
			except:
   				print "Error: socket error for client"
				time.sleep(delta_t)
        print('Transmit stopped - threadRunning')


#######  main programme  ############
os.system('clear')
print('Connection with server established')
try:
  print(34 * '-')
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
  print('Main stopped')
  threadRunning = False
  s.sendto('Force Close Thread RECEIVE',('localhost',PORT))
  time.sleep(2)
  s.close()

finally:
  s.close()