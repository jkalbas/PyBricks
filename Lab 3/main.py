#!/usr/bin/env pybricks-micropython
# Joseph Kalbas, 730414620
# Andre Javan, 730210739
from cmath import pi
import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
import math

#import numpy as np
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

#sensors report distance in mm
#give distance to travel in CM
DIST_CORRECT = .5
W_CORRECT = 1.1
SET = 250
R = 2.8
degToRad = math.pi/180
OUT_MAX = .65
IN_MAX = .25
L = 14 #between inner edge of wheels outer edge or middle?
MAX_SPEED = 1560
# Write your program here.
class Car:
    def __init__(self,target=0):
        self.pose = Pose()
        self.ev3 = EV3Brick()
        self.mL = Motor(Port.A)
        self.mR = Motor(Port.D)
        self.ultra = UltrasonicSensor(Port.S3)
        self.tR = TouchSensor(Port.S4)
        self.tL = TouchSensor(Port.S2)
        self.error = Error(target,self.ultra)
        self.distance = 0
    #differential drive
    #v in cm/s
    #time in ms
    
    #distance things
    def getDistance(self):
        return self.distance
    def setDistance(self,d):
        self.distance = d
    def dFrom(self,tp):
        cp = self.pose.getPose()
        return math.sqrt((tp[0]-cp[0])**2+(tp[1]-cp[1])**2)
    def inRect(self,tp, xtol, ytol):
        cp = self.pose.getPose()
        return abs(cp[0]-tp[0])<xtol and abs(cp[1]-tp[1])<ytol 
    def updateDistanceH(self,dx,dy):
        self.distance += math.sqrt(dx**2+dy**2)*DIST_CORRECT
    def updateDistanceV(self,v,dt):
        self.distance += v*dt/1000*DIST_CORRECT
    def updateDistanceDD(self,vR,vL,dt):
        if(vR==vL):
            self.distance += vR*dt/1000
        else:
            v = vTot(vR,vL)
            w = (vR-vL)/L
            wdt = w*dt*W_CORRECT/1000
            rICC = L*(vL+vR)/2/(vR-vL)
            xICC = -rICC*math.sin(self.pose.th)
            yICC = +rICC*math.cos(self.pose.th)
            dx = -xICC*math.cos(wdt)+yICC*math.sin(wdt)+xICC
            dy = -xICC*math.sin(wdt)-yICC*math.cos(wdt)+yICC
            self.updateDistanceH(dx,dy)
    #movement
    def moveFastWithBias(self,speed,bias,dt=50):
        #take percentage of max angular velocity
        wR = MAX_SPEED*speed
        wL = wR
        #turn
        if(bias>=0): #going left or straight
            #limit overadjustment
            wL = max(wL-bias,wL*OUT_MAX)
        else: #turning right
            if(-bias>wR):
                bias = -wR
            wR = max(wR+bias,wR*IN_MAX)
        #run motors
        self.mR.run(wR)
        self.mL.run(wL)
        #update things
        vR = vFromW(wR)
        vL = vFromW(wL)
        self.pose.updatePoseDD(vR,vL,dt)
        #self.updateDistanceDD(vR,vL,dt)
        self.updateDistanceV(vTot(vR,vL),dt)
        wait(dt)
    def brake(self):
        self.mR.brake()
        self.mL.brake()
    #other utilities
    def waitForPress(self):
        while(len(self.ev3.buttons.pressed())<1):
            pass
    def beep(self,t=250):
        self.ev3.speaker.beep(400,t)
    def getTouched(self):
        return self.tR.pressed()+self.tL.pressed()
    def pullOut(self):
        self.mL.run(-360)
        self.mR.run(-360)
        wait(900)
        self.brake()
        wait(50)
        #update farther to adjust for wheelspin
        self.pose.updatePoseDD(vFromW(-360),vFromW(-360),1000)
        self.mL.run(360)
        self.mR.run(-360)
        wait(625)
        self.brake()
        self.pose.updatePoseDD(vFromW(-360),vFromW(360),625)
    def turnToTheta(self, target):
        curr = self.pose.getPose()[2] % (2*pi)
        dth = curr - target
        self.turnAngle(dth)
    def turnAngle(self, angle):
        vl = angle*L/2
        wl = vl/R/degToRad
        self.mL.run(wl)
        self.mR.run(-wl)
        wait(1000)
        self.brake()
        self.pose.updatePoseDD(-vl,vl,1000)
    def turnToPoint(self,x,y):
        p = self.pose.getPose()
        self.turnAngle(math.atan((p[1]-y)/(p[0]-x)))


#pose things
class Pose:
    def __init__(self,x=0,y=0,th=0):
        self.x = x
        self.y = y
        self.th = th
    def getPose(self):
        return [self.x,self.y,self.th]
    def setPose(self,x,y,th):
        self.x = x
        self.y = y
        self.th = th# % (2*math.pi)
    def updatePose(self,dx=0,dy=0,dth=0):
        self.x+=dx
        self.y+=dy
        self.th=(dth+self.th) % (2*math.pi)
    def updatePoseDD(self,vR,vL,dt):
        if(vR==vL):
            self.updatePose(dX(vR,self.th,dt),dY(vR,self.th,dt))
        else:
            v = vTot(vR,vL)
            w = (vR-vL)/L
            wdt = w*dt*W_CORRECT/1000
            rICC = L*(vL+vR)/2/(vR-vL)
            xICC = self.x-rICC*math.sin(self.th)
            yICC = self.y+rICC*math.cos(self.th)
            xNew = (self.x-xICC)*math.cos(wdt)-(self.y-yICC)*math.sin(wdt)+xICC
            yNew = (self.x-xICC)*math.sin(wdt)+(self.y-yICC)*math.cos(wdt)+yICC
            self.setPose(xNew,yNew,self.th+wdt)
class Error:
    def __init__(self,target,ultra):
        self.error = [0,0,0]
        self.totalError = 0
        self.target = target
        self.ultra = ultra
        self.out = target
    def ultraPID(self,kP,kI,kD,dt):
        self.error[0] = self.ultra.distance()-self.target
        de = (self.error[0] - self.error[1])/dt
        self.error[1] = self.error[0]
        self.totalError += self.error[0]*dt/1000
        self.out = kP*self.error[0]+kI*self.totalError+kD*de
        return self.out
    #finite difference version
    def ultraPID2(self,kP,kI,kD,dt):
        d = dt/1000
        a0 = kP+kI*d+kD/d
        a1 = -kP - 2*kD/dt
        a2 = kD/dt
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.error[0] = self.ultra.distance() - self.target
        self.out += a0 * self.error[0] + a1 * self.error[1] + a2 * self.error[2]
        return self.out
def vFromW(w):
    return R*w*degToRad
def vTot(vR,vL):
    return (vR+vL)/2
def dTheta(vR,vL,dt):
    return (vR-vL)/L*dt/1000
def dX(v,th,dt):
    return v*math.cos(th)*dt/1000
def dY(v,th,dt):
    return v*math.sin(th)*dt/1000
#maybe use to give our sensor a smoother response curve
#probably use like speed*sigmoid(distance)-speed/2 for nice bias
def sigmoid(x):
    return 1/(1+math.exp(-x))
def sigmoidAdjusted(s,x):
    s1=s*MAX_SPEED
    return sigmoid(x)*s1-.5*s1
#TESTING
#-----------------
#~~~~~~~~~~~~~~~~~
#initializing
#poses = []
car = Car(target=SET)
car.pose.setPose(200,50,pi/2)
car.waitForPress()
car.setDistance(0)
"""print(car.pose.getPose())
car.mL.run(360)
car.mR.run(-360)
wait(1000)
car.brake()
print(vFromW(-360))
car.pose.updatePoseDD(vFromW(-360),vFromW(360),1000)
print(car.pose.getPose())"""
d = DataLog("x","y","theta")
while(not car.getTouched()):
    car.moveFastWithBias(.3,0)
    x,y,th = car.pose.getPose()
    d.log(x,y,th)
car.brake()
hitpoint = car.pose.getPose()
wait(50)
# car.mL.run(-100)
# car.mR.run(-1300)
# wait(700)
# car.brake()
car.pullOut()
x,y,th = car.pose.getPose()
d.log(x,y,th)
dt = 100
dist_from_start = car.getDistance()
speed = .3
car.setDistance(0)

while(not car.inRect(hitpoint,5,50) or car.getDistance()<65):  
    car.moveFastWithBias(speed,car.error.ultraPID(7,0,0,dt),dt)
    if(car.getTouched()):
        car.pullOut()
    x,y,th = car.pose.getPose()
    d.log(x,y,th)

car.brake()
#car.beep(1000)
#car.turnToTheta(pi)
# car.mL.run(-1300, wait=True)
# car.mR.run(-100)
car.mL.run(360)
car.mR.run(-360)
wait(550)
car.brake()
car.pose.updatePoseDD(vFromW(-360),vFromW(360),550)
# car.turnToPoint(200,50)
# wait(200)
car.setDistance(0)
while(car.getDistance() < dist_from_start):
    car.moveFastWithBias(.5,0)
    x,y,th = car.pose.getPose()
    d.log(x,y,th)
car.brake()
car.beep(2000)

