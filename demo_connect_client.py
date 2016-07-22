#Socket client example in python

import socket   #for sockets
import sys  #for exit
import thread
import threading
import time

#create an INET, STREAMing socket
#Main thread to send information to other vehicle'URL in topologie configuration
class Transmit(threading.Thread):
    def run(self):
    # Sending data to the server
        print('Transmit started')
        global s # connection
        global threadRunning
        global delta_t
        while(threadRunning):
            if(1):
                global ifsend, stat_me
                ifsend = True
                # stat_me[0] = 1
                delta_t = 0.5
            if ifsend:
                #print info
                addr = '192.168.1.105'
                PORT = 5005
                print 'envoi a '+ addr
                z1 = 13.5
                z2 = 12.6
                send_info = [1,z1,z2]
                s.sendto(str(send_info),(addr,PORT))

            time.sleep(delta_t)
        print('Transmit stopped - threadRunning')
        return


threadRunning = False
ifsend = False
delta_t = 0.5

s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
s.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
s.bind(('',5005))

print('Connection with server established')


try:
  print(34 * '-')
  print("        M A I N - M E N U")
  print(' Press CTRL+C to close connection')
  print(34 * '-')
  # Create instance of class
  threadRunning = True
  # global sl_old
  # sl_old = lmotor.position
  # global sr_old
  # sr_old = rmotor.position

  transmit = Transmit()
  # Start class
  transmit.start()
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
