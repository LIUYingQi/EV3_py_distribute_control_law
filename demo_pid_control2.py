#!/usr/bin/python
print(
"""This demo allows to control motors attached to each of the output ports with the
brick buttons. This could be very helpful for initial tests of your robot's
functionality.

object:mesure precision for cercle move
"""
)

from time import sleep
from ev3dev import *
import time
from math import *

lcd = LCD()
motors = [
        motor(OUTPUT_A),
        motor(OUTPUT_B),
        motor(OUTPUT_C),
        motor(OUTPUT_D)
        ]
port = 0

print '*********************************************************'
print '***********************test for line********************'
print 'initial parameters'
print '*********************************************************'
print 'porlarity'
print motors[0].polarity
print 'position'
print motors[0].position
print 'speed_regulation_enabled'
print motors[0].speed_regulation_enabled
print 'speed_regulation_p'
print motors[0].speed_regulation_p
print 'speed_regulation_i'
print motors[0].speed_regulation_i
print 'speed_regulation_d'
print motors[0].speed_regulation_d
print 'duty_cycle'
print motors[0].duty_cycle
print 'stop_command'
print motors[0].stop_command
print 'time_sp'
print motors[0].time_sp
print 'ramp_up_sp'
print motors[0].ramp_up_sp
print 'ramp_down_sp'
print motors[0].ramp_down_sp
print 'speed_sp'
print motors[0].speed_sp
print '******************   test   begin    ************************'

ki=0.5;
pi=0.02;

motors[0].reset();
motors[3].reset();

def drive(m, p):
    if m.connected:
       m.run_direct(duty_cycle_sp=p)

def stop(m):
    if m.connected:
        m.stop()

#ov1 ov2
#objet viteese 1 and 2
def test_cercle(m1,m2,ov1,ov2):
    ####   use PI control here
    ####   use state control
    for x in range(0,10):
        print m1.position;
        print m2.position;
        print m1.speed
        print m2.speed
        v1=m1.speed
        v2=m2.speed
        p1=m1.position
        p2=m2.position
        u1=ki*(ov1*p2/ov2-p1) + (ov1/8.8)+pi*(ov1*v2/ov2-v1)+5
        u2=ki*(p1-ov1*p2/ov2)+(ov2/8.8)+pi*(v1-ov1*v2/ov2)+5
        u1=int(u1)
        u2=int(u2)
        print u1
        print u2
        drive(m1,u1)
        drive(m2,u2)
        # time.sleep(0.001)

print (time.strftime("%H:%M:%S"))
test_cercle(motors[0],motors[3],400,400)
print (time.strftime("%H:%M:%S"))
stop(motors[0])
stop(motors[3])
