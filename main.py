#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from threading import Thread


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Initialising the brick
ev3 = EV3Brick()

#This program requires LEGO EV3 MicroPython v2.0 or higher.
#Click "Open user guide" on the EV3 extension tab for more information.
#Initialising motor at ports a+b
#r is right, l is left

motor_r = Motor(Port.A)
motor_l = Motor(Port.B)
motor_pickupWheels = Motor(Port.C)
motor_v = Motor(Port.D)
gyro = GyroSensor(Port.S1)


#Drivebase has LEFT motor first!
qdpi = DriveBase(motor_l,motor_r,wheel_diameter=32.5,axle_track=204.5)
#larvefod diameter 32.5

##### testing thingamajig######

#variables

#angle value gets updated by some other section so we know how many degrees it has to turn
angle = 45
#timerot value gets updated with how much time it turns to that angle(?)
timerot = 1000
#basespeed is the basespeed dumbass 
basespeed = 150
#drive will define if it drives
drive = 1

def qdpidrive(drive):

    while drive==1: 
       #gyro correction along with driving. 
        correction = (0 - gyro.angle())*1
        qdpi.drive(150,correction)
        break


def qdpirotate(angle):
    
    qdpi.drive(0,angle)
    

def qdpiopen():

    motor_v.run(100)
    wait(1000)
    motor_v.stop()
   

##run rotate program until correctly placed

motor_pickupWheels.run(500)
#gyro reset angle!! this has to run first
gyro.reset_angle(0)

qdpidrive(drive=1)
wait(10000)
qdpidrive(drive=0)
qdpirotate(angle=45)
wait(1000)
qdpirotate(angle=0)
wait(1000)
qdpiopen()
qdpipause()


brick.sound.beeps(2)

