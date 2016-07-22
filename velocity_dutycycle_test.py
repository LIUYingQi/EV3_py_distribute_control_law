#!/usr/bin/python

print(
"""This demo allows to control motors attached to each of the output ports with the
brick buttons. This could be very helpful for initial tests of your robot's
functionality.

object:mesure relation for velocity and duty_cycle
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

for x in range(0,100):
        print motors[0].position;
        print motors[3].position;
        print motors[0].duty_cycle;
        print motors[3].duty_cycle;
        print motors[0].speed
        print motors[3].speed
        drive(motors[0],x)
        drive(motors[3],x)
        time.sleep(0.5)
motors[0].reset()
motors[3].reset()
print motors[0].position;
print motors[3].position;
drive(motors[0],1000)
drive(motors[3],1000)
stop(motors[0])
stop(motors[3])
