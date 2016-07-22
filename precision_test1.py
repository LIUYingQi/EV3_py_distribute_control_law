#!/usr/bin/python

print(
"""This demo allows to control motors attached to each of the output ports with the
brick buttons. This could be very helpful for initial tests of your robot's
functionality.

object:mesure precision for line move
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

def drive(m, p):
    if m.connected:
       m.run_direct(duty_cycle_sp=p)
def stop(m):
    if m.connected:
        m.stop()

k=0.5;

motors[0].reset();
motors[3].reset();

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

for x in range(0,100):
    print motors[0].position;
    print motors[3].position;
    print motors[0].speed
    print motors[3].speed
    v0=motors[0].speed
    v3=motors[3].speed
    p0=motors[0].position
    p3=motors[3].position
    u0=k*(p3-p0)+50
    u3=k*(p0-p3)+50
    print u0
    print u3
    u0=int(u0)
    u3=int(u3)
    print u0
    print u3
    drive(motors[0],u0)
    drive(motors[3],u3)
    time.sleep(0.1)

stop(motors[0])
stop(motors[3])
