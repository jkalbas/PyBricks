#!/usr/bin/env pybricks-micropython
# Joseph Kalbas, 730414620
# Andre Javan, 730210739
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
import math
#import numpy as np
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
RIGHT_CORRECTION = 0.98
MOTOR_CORRECTION = 1.02
# Create your objects here.
ev3 = EV3Brick()
mA = Motor(Port.A)
mB = Motor(Port.B)
ultra = UltrasonicSensor(Port.S3)
t1 = TouchSensor(Port.S1)
t2 = TouchSensor(Port.S2)
"""pose = np.zeros(3)
# Write your program here.
def getPose():
    return pose
def setPose(x,y,th):
    pose = np.array([x,y,th])
def updatePose(dx,dy,dth=0):
    pose += [dx,dy,dth]"""
def stop():
    mA.brake()
    mB.brake()
#for motors not pose
def angleReset():
    mA.reset_angle(0)
    mB.reset_angle(0)
def align():
    angA = mA.angle()
    angB = mB.angle()
    if(angA > angB):
        mB.run_target(100,angA)
    else:
        mA.run_target(100,angB)
    #TODO update pose?
def moveStraight(speed):
    mA.run(speed)
    mB.run(speed)
    #figure out way to do this with pose update

def moveDistance(speed,cm):
    degrees = (cm/2.8)*(180/math.pi) * MOTOR_CORRECTION
    #th = getPose()[2]
    mA.run_angle(speed*RIGHT_CORRECTION,degrees*RIGHT_CORRECTION, wait=False)  #for wheel radius 2.8cm
    mB.run_angle(speed,degrees)
    stop()
    #updatePose(cm*math.cos(th),cm*math.sin(th))
def moveDistanceByTime(speed, cm):
    #th = getPose()[2]
    time = cm/2.8*180/math.pi/speed
    mA.run_time(speed,time)
    mB.run_time(speed,time)
    #updatePose(cm*math.cos(th),cm*math.sin(th))
def beep(time):
    ev3.speaker.beep(400,time)
def correctUltra(distance):
    return distance * 1 - 60

#MOVE FORWARD
while(len(ev3.buttons.pressed())<1):
    pass
moveDistance(500,120)
beep(300)

#ULTRASONIC
while(len(ev3.buttons.pressed())<1):
    pass
while(correctUltra(ultra.distance())>540):
    mA.run(500)
    mB.run(500)
stop()
beep(300)

#TOUCH
while(len(ev3.buttons.pressed())<1):
    pass
while t1.pressed() == False and t2.pressed() == False:
    mA.run(500)
    mB.run(500)
stop()
moveDistance(500,-50)
beep(300)