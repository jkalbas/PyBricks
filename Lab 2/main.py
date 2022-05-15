from threading import main_thread
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
import math
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()
mA = Motor(Port.A)
mB = Motor(Port.B)
ultra = UltrasonicSensor(Port.S3)
t1 = TouchSensor(Port.S1)
t2 = TouchSensor(Port.S2)

MOTOR_CORRECTION = 1
min_thresh = 15
max_thresh = 55
total_angle_thresh = ((200 /2.8) *180/math.pi) * MOTOR_CORRECTION

def turnRight(angle): #this just goes straight
    mA.run_angle(100, angle, wait=False)
    mB.run_angle(100, angle)

while(len(ev3.buttons.pressed())<1):
    pass

while t1.pressed() == False and t2.pressed() == False:
    mA.run(500)
    mB.run(500)

mA.brake()
mB.brake()

#check whether both wheels turn correctly. Maybe add wait=True?
turnRight(-90)

mA.reset_angle(0)
mB.reset_angle(0)

#check if sensor is too close or too far
while ((mA.angle() + mB.angle() / 2 ) < total_angle_thresh):
    while ultra.distance() > min_thresh and ultra.distance() < max_thresh:
        mA.run(500)
        mB.run(500)
    
    if ultra.distance() < min_thresh:
        #turn right
        hi = 0
    else:
        #turn left
        hi = 0

ev3.speaker.beep(400,300)
